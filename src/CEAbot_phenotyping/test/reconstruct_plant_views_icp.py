#!/usr/bin/env python3
"""Refine saved TF camera poses with ICP and merge each plant's views.

The saved base_link -> camera transform is always applied first.  ICP therefore
only estimates a small correction for TF/calibration/timing error; it is not
asked to register two clouds from scratch.
"""

import argparse
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree


def load_metadata(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def pose_matrix(meta):
    names = ("actual_x", "actual_y", "actual_z", "actual_qx", "actual_qy", "actual_qz", "actual_qw")
    missing = [name for name in names if name not in meta]
    if missing:
        raise ValueError("missing pose fields: " + ", ".join(missing))

    x, y, z, w = np.asarray(
        [meta["actual_qx"], meta["actual_qy"], meta["actual_qz"], meta["actual_qw"]],
        dtype=np.float64,
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-length pose quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = [meta["actual_x"], meta["actual_y"], meta["actual_z"]]
    return result


def planning_pose_matrix(meta):
    planning_meta = dict(meta)
    for name in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        planning_meta[f"actual_{name}"] = meta[f"planning_actual_{name}"]
    return pose_matrix(planning_meta)


def transform_xyz(xyz, transform):
    return xyz @ transform[:3, :3].T + transform[:3, 3]


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0 or len(points) == 0:
        return points
    cells = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(indices)]


def cap_points(points, maximum, seed):
    if maximum <= 0 or len(points) <= maximum:
        return points
    rng = np.random.default_rng(seed)
    return points[np.sort(rng.choice(len(points), maximum, replace=False))]


def load_cloud(view_dir, maximum, seed, minimum_depth, maximum_depth):
    path = view_dir / "cloud_xyzrgb.npy"
    points = np.load(path)
    if points.ndim != 2 or points.shape[1] < 6:
        raise ValueError(f"{path} must be an Nx6 array")
    points = points[:, :6].astype(np.float64, copy=False)
    points = points[np.all(np.isfinite(points[:, :3]), axis=1)]
    points = points[
        (points[:, 2] >= minimum_depth) & (points[:, 2] <= maximum_depth)
    ]
    return cap_points(points, maximum, seed)


def plant_geometry(scan_dir, plant_name, first_meta):
    """Return the detected plant target in base_link and its horizontal radius."""
    scan_meta = load_metadata(scan_dir / "metadata.yaml")
    plant_id = int(plant_name.removeprefix("plant_"))
    plant = next(
        (entry for entry in scan_meta.get("plants", []) if int(entry["plant_id"]) == plant_id),
        None,
    )
    if plant is None:
        raise ValueError(f"{scan_dir / 'metadata.yaml'} has no entry for {plant_name}")

    target_command = np.array(
        [
            plant["target"]["x_mm"] / 1000.0,
            plant["target"]["y_mm"] / 1000.0,
            plant["target"]["z_mm"] / 1000.0,
            1.0,
        ]
    )
    # Both camera poses were sampled at the cloud timestamp, so this recovers
    # base_link <- command_frame without requiring a live TF tree.
    base_from_command = pose_matrix(first_meta) @ np.linalg.inv(
        planning_pose_matrix(first_meta)
    )
    target_base = (base_from_command @ target_command)[:3]
    return target_base, float(plant["radius_mm"]) / 1000.0


def crop_around_plant(points, center, radius, below, above):
    horizontal_distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    keep = (
        (horizontal_distance <= radius)
        & (points[:, 2] >= center[2] - below)
        & (points[:, 2] <= center[2] + above)
    )
    return points[keep]


def rgb_pixel_xyz_samples(view_dir, minimum_depth, maximum_depth):
    """Recover sparse pixel-to-XYZ samples using RGB values unique in the image."""
    image = cv2.imread(str(view_dir / "color.png"), cv2.IMREAD_COLOR)
    cloud = np.load(view_dir / "cloud_xyzrgb.npy")
    image_rgb = (
        (image[:, :, 2].astype(np.uint32) << 16)
        | (image[:, :, 1].astype(np.uint32) << 8)
        | image[:, :, 0].astype(np.uint32)
    )
    colors, counts = np.unique(image_rgb, return_counts=True)
    unique_colors = colors[counts == 1]
    flat_colors = image_rgb.ravel()
    order = np.argsort(flat_colors)
    sorted_colors = flat_colors[order]
    cloud_colors = (
        (cloud[:, 3].astype(np.uint32) << 16)
        | (cloud[:, 4].astype(np.uint32) << 8)
        | cloud[:, 5].astype(np.uint32)
    )
    usable = (
        np.isin(cloud_colors, unique_colors)
        & (cloud[:, 2] >= minimum_depth)
        & (cloud[:, 2] <= maximum_depth)
    )
    cloud_rows = np.flatnonzero(usable)
    locations = np.searchsorted(sorted_colors, cloud_colors[usable])
    pixels = order[locations]
    pixel_xy = np.column_stack(
        (pixels % image.shape[1], pixels // image.shape[1])
    ).astype(np.float64)
    return image, pixel_xy, cloud[cloud_rows, :3].astype(np.float64)


def lift_keypoints(keypoints, pixel_xy, xyz, maximum_pixel_distance=4.0):
    query = np.array([keypoint.pt for keypoint in keypoints])
    distances, indices = cKDTree(pixel_xy).query(query, k=1)
    lifted = xyz[indices].copy()
    lifted[distances > maximum_pixel_distance] = np.nan
    return lifted


def coarse_rgbd_transform(source_data, target_data, seed):
    """Estimate target_camera <- source_camera from SIFT matches lifted to 3D."""
    source_image, source_pixels, source_xyz = source_data
    target_image, target_pixels, target_xyz = target_data
    sift = cv2.SIFT_create(nfeatures=6000)
    source_keys, source_descriptors = sift.detectAndCompute(
        cv2.cvtColor(source_image, cv2.COLOR_BGR2GRAY), None
    )
    target_keys, target_descriptors = sift.detectAndCompute(
        cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY), None
    )
    if source_descriptors is None or target_descriptors is None:
        raise ValueError("SIFT found no descriptors")

    matches = cv2.BFMatcher().knnMatch(source_descriptors, target_descriptors, k=2)
    matches = [first for first, second in matches if first.distance < 0.70 * second.distance]
    source = lift_keypoints(
        [source_keys[match.queryIdx] for match in matches], source_pixels, source_xyz
    )
    target = lift_keypoints(
        [target_keys[match.trainIdx] for match in matches], target_pixels, target_xyz
    )
    valid = np.all(np.isfinite(source), axis=1) & np.all(np.isfinite(target), axis=1)
    source, target = source[valid], target[valid]
    if len(source) < 12:
        raise ValueError(f"only {len(source)} lifted SIFT matches")

    rng = np.random.default_rng(seed)
    best_inliers = np.zeros(len(source), dtype=bool)
    for _ in range(3000):
        sample = rng.choice(len(source), 3, replace=False)
        candidate = rigid_transform(source[sample], target[sample])
        errors = np.linalg.norm(transform_xyz(source, candidate) - target, axis=1)
        inliers = errors < 0.015
        if inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    if best_inliers.sum() < 15:
        raise ValueError(f"only {best_inliers.sum()} RGB-D RANSAC inliers")

    result = rigid_transform(source[best_inliers], target[best_inliers])
    errors = np.linalg.norm(transform_xyz(source, result) - target, axis=1)
    inliers = errors < 0.015
    result = rigid_transform(source[inliers], target[inliers])
    rmse = float(
        np.sqrt(np.mean(np.square(np.linalg.norm(transform_xyz(source[inliers], result) - target[inliers], axis=1))))
    )
    return result, int(inliers.sum()), rmse


def build_initial_poses(view_dirs, seed, minimum_depth, maximum_depth):
    """Chain adjacent RGB-D registrations, anchored by the first saved TF pose."""
    data = {
        view: rgb_pixel_xyz_samples(view, minimum_depth, maximum_depth)
        for view in view_dirs
    }
    saved_poses = {
        view: pose_matrix(load_metadata(view / "meta.yaml")) for view in view_dirs
    }
    poses = {view_dirs[0]: saved_poses[view_dirs[0]]}
    diagnostics = {view_dirs[0]: {"method": "saved_tf_reference"}}

    for index, view in enumerate(view_dirs[1:], start=1):
        # Top views can differ strongly from the last ring view. Try the last
        # view and the first ring view, then retain the stronger 3D solution.
        candidates = [view_dirs[index - 1]]
        if view.name == "top":
            candidates.extend([view_dirs[0], view_dirs[len(view_dirs) // 2]])
        solutions = []
        for target_view in dict.fromkeys(candidates):
            if target_view == view or target_view not in poses:
                continue
            try:
                relative, inliers, rmse = coarse_rgbd_transform(
                    data[view], data[target_view], seed + index
                )
                solutions.append((inliers, -rmse, target_view, relative, rmse))
            except Exception:
                pass

        if solutions:
            inliers, _, target_view, relative, rmse = max(solutions)
            poses[view] = poses[target_view] @ relative
            diagnostics[view] = {
                "method": "rgbd_sift_ransac",
                "target_view": target_view.name,
                "inliers": inliers,
                "rmse_m": rmse,
            }
            print(
                f"  {view.name}: coarse RGB-D -> {target_view.name}, "
                f"{inliers} inliers, RMSE {rmse * 1000:.2f} mm"
            )
        else:
            previous = view_dirs[index - 1]
            relative_tf = np.linalg.inv(saved_poses[previous]) @ saved_poses[view]
            poses[view] = poses[previous] @ relative_tf
            diagnostics[view] = {"method": "saved_tf_fallback"}
            print(f"  {view.name}: coarse RGB-D failed, using relative TF")

    # If a missing view creates a gap that cannot be matched forward, rebuild
    # the affected tail backward from the 330° -> 0° ring closure.
    ring_views = [view for view in view_dirs if view.name != "top"]
    fallback_indices = [
        index
        for index, view in enumerate(ring_views)
        if diagnostics[view]["method"] == "saved_tf_fallback"
    ]
    if fallback_indices and len(ring_views) > 2:
        first_bad = fallback_indices[0]
        backward_poses = {}
        try:
            last = ring_views[-1]
            closure, inliers, rmse = coarse_rgbd_transform(
                data[last], data[ring_views[0]], seed + 1000
            )
            backward_poses[last] = poses[ring_views[0]] @ closure
            backward_info = {
                last: {
                    "method": "rgbd_sift_ransac_backward",
                    "target_view": ring_views[0].name,
                    "inliers": inliers,
                    "rmse_m": rmse,
                }
            }
            for index in range(len(ring_views) - 2, first_bad - 1, -1):
                view = ring_views[index]
                target_view = ring_views[index + 1]
                relative, inliers, rmse = coarse_rgbd_transform(
                    data[view], data[target_view], seed + 1000 + index
                )
                backward_poses[view] = backward_poses[target_view] @ relative
                backward_info[view] = {
                    "method": "rgbd_sift_ransac_backward",
                    "target_view": target_view.name,
                    "inliers": inliers,
                    "rmse_m": rmse,
                }
            for view, pose in backward_poses.items():
                poses[view] = pose
                diagnostics[view] = backward_info[view]
            print(
                f"  rebuilt {ring_views[first_bad].name}..{ring_views[-1].name} "
                "backward from ring closure"
            )
        except Exception as exc:
            print(f"  backward ring recovery failed: {exc}")
    return poses, saved_poses, diagnostics


def rigid_transform(source, target):
    """Return the proper rigid transform mapping paired source to target."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = target_center - rotation @ source_center
    return result


def rotation_degrees(transform):
    cosine = np.clip((np.trace(transform[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def icp(source, target, max_distance, iterations, trim_fraction, tolerance):
    """Trimmed point-to-point ICP; inputs and output are in the TF/world frame."""
    tree = cKDTree(target)
    moved = source.copy()
    total = np.eye(4)
    previous_rmse = float("inf")
    stats = {"pairs": 0, "rmse": float("inf"), "iterations": 0}

    for iteration in range(iterations):
        distances, indices = tree.query(moved, k=1, workers=-1)
        valid_mask = np.isfinite(distances) & (distances <= max_distance)
        # Keep only reciprocal nearest neighbours. This prevents a large patch
        # of repetitive foliage from collapsing onto one unrelated leaf.
        moved_tree = cKDTree(moved)
        _, reverse_indices = moved_tree.query(target, k=1, workers=-1)
        source_indices = np.arange(len(moved))
        valid_mask &= reverse_indices[indices] == source_indices
        valid = np.flatnonzero(valid_mask)
        if len(valid) < 30:
            break

        keep_count = max(30, int(len(valid) * trim_fraction))
        if keep_count < len(valid):
            valid = valid[np.argpartition(distances[valid], keep_count - 1)[:keep_count]]

        delta = rigid_transform(moved[valid], target[indices[valid]])
        moved = transform_xyz(moved, delta)
        total = delta @ total
        rmse = float(np.sqrt(np.mean(np.square(distances[valid]))))
        stats = {"pairs": int(len(valid)), "rmse": rmse, "iterations": iteration + 1}

        if abs(previous_rmse - rmse) < tolerance:
            break
        previous_rmse = rmse

    return total, stats


def save_ply(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices = np.empty(
        len(points),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")],
    )
    for index, name in enumerate(("x", "y", "z")):
        vertices[name] = points[:, index].astype(np.float32)
    for index, name in enumerate(("red", "green", "blue"), start=3):
        vertices[name] = np.clip(points[:, index], 0, 255).astype(np.uint8)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        vertices.tofile(stream)


def view_sort_key(path):
    match = re.match(r"view_(\d+)_", path.name)
    # Register the ring in capture order, then add the top view.
    return (0, int(match.group(1))) if match else (1, path.name)


def align_plant(plant_dir, output_dir, args):
    view_dirs = sorted(
        [path for path in plant_dir.iterdir() if path.is_dir() and (path / "cloud_xyzrgb.npy").exists()],
        key=view_sort_key,
    )
    if not view_dirs:
        print(f"{plant_dir.name}: no point clouds found")
        return

    first_meta = load_metadata(view_dirs[0] / "meta.yaml")
    plant_center, detected_radius = plant_geometry(
        plant_dir.parent, plant_dir.name, first_meta
    )
    crop_radius = detected_radius + args.crop_margin
    if args.coarse_rgbd:
        initial_poses, saved_poses, coarse_diagnostics = build_initial_poses(
            view_dirs, args.seed, args.min_depth, args.max_depth
        )
    else:
        saved_poses = {
            view: pose_matrix(load_metadata(view / "meta.yaml")) for view in view_dirs
        }
        initial_poses = saved_poses
        coarse_diagnostics = {
            view: {"method": "saved_tf"} for view in view_dirs
        }
    aligned = []
    transforms = {}
    print(
        f"\n{plant_dir.name}: {len(view_dirs)} views, center "
        f"{np.round(plant_center, 4).tolist()}, "
        f"crop radius {crop_radius * 1000:.0f} mm"
    )

    for view_index, view_dir in enumerate(view_dirs):
        meta = load_metadata(view_dir / "meta.yaml")
        if meta.get("frame_id") != meta.get("pose_child_frame"):
            raise ValueError(
                f"{view_dir}: cloud frame {meta.get('frame_id')!r} does not match "
                f"pose child frame {meta.get('pose_child_frame')!r}"
            )

        saved_tf_pose = saved_poses[view_dir]
        tf_pose = initial_poses[view_dir]
        cloud = load_cloud(
            view_dir,
            args.max_points_per_view,
            args.seed + view_index,
            args.min_depth,
            args.max_depth,
        )
        tf_cloud = cloud.copy()
        tf_cloud[:, :3] = transform_xyz(tf_cloud[:, :3], tf_pose)
        unfiltered_count = len(tf_cloud)
        tf_cloud = crop_around_plant(
            tf_cloud,
            plant_center,
            crop_radius,
            args.crop_below,
            args.crop_above,
        )
        if len(tf_cloud) < 100:
            raise ValueError(
                f"{view_dir}: crop retained only {len(tf_cloud)} of "
                f"{unfiltered_count} points"
            )
        # Keep a camera-frame copy so the final refined pose is applied once.
        cloud = tf_cloud.copy()
        cloud[:, :3] = transform_xyz(tf_cloud[:, :3], np.linalg.inv(tf_pose))
        correction = np.eye(4)
        stage_stats = []

        if aligned:
            target = voxel_downsample(np.vstack(aligned), args.icp_voxel)
            target_xyz = cap_points(target[:, :3], args.max_icp_points, args.seed)
            source_xyz = voxel_downsample(tf_cloud, args.icp_voxel)[:, :3]
            source_xyz = cap_points(source_xyz, args.max_icp_points, args.seed + view_index)

            for distance in args.max_correspondence:
                delta, stats = icp(
                    transform_xyz(source_xyz, correction),
                    target_xyz,
                    distance,
                    args.iterations,
                    args.trim_fraction,
                    args.tolerance,
                )
                correction = delta @ correction
                stats["max_correspondence_m"] = distance
                stage_stats.append(stats)

            translation = float(
                np.linalg.norm(
                    transform_xyz(plant_center[None, :], correction)[0]
                    - plant_center
                )
            )
            angle = rotation_degrees(correction)
            if translation > args.max_correction_translation or angle > args.max_correction_rotation:
                print(
                    f"  {view_dir.name}: rejected ICP correction "
                    f"({translation * 1000:.1f} mm, {angle:.2f} deg)"
                )
                correction = np.eye(4)
            else:
                final = stage_stats[-1]
                print(
                    f"  {view_dir.name}: crop {len(tf_cloud)}/{unfiltered_count}, "
                    f"{final['pairs']} pairs, "
                    f"RMSE {final['rmse'] * 1000:.2f} mm, correction "
                    f"{translation * 1000:.1f} mm / {angle:.2f} deg"
                )
        else:
            print(
                f"  {view_dir.name}: fixed reference view, "
                f"crop {len(tf_cloud)}/{unfiltered_count}"
            )

        refined_pose = correction @ tf_pose
        tf_cloud[:, :3] = transform_xyz(cloud[:, :3], refined_pose)
        aligned.append(tf_cloud)
        transforms[view_dir.name] = {
            "saved_tf_pose": saved_tf_pose.tolist(),
            "coarse_pose": tf_pose.tolist(),
            "coarse_registration": coarse_diagnostics[view_dir],
            "icp_correction": correction.tolist(),
            "refined_pose": refined_pose.tolist(),
            "crop_points": len(tf_cloud),
            "stages": stage_stats,
        }

        if args.save_debug_views:
            save_ply(output_dir / "debug_views" / plant_dir.name / f"{view_dir.name}_icp.ply", tf_cloud)

    merged = voxel_downsample(np.vstack(aligned), args.output_voxel)
    output_path = output_dir / f"{plant_dir.name}_merged_icp.ply"
    save_ply(output_path, merged)
    with (output_dir / f"{plant_dir.name}_icp_transforms.json").open("w", encoding="utf-8") as stream:
        json.dump(transforms, stream, indent=2)
    print(f"  saved {output_path} ({len(merged)} points)")


def main():
    parser = argparse.ArgumentParser(
        description="Merge plant clouds using saved TF poses refined by trimmed, multi-scale ICP."
    )
    parser.add_argument("scan_dir", type=Path)
    parser.add_argument("--plant", default="", help="Only process one plant, e.g. plant_01")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--icp-voxel", type=float, default=0.006, help="ICP downsampling voxel in metres")
    parser.add_argument("--output-voxel", type=float, default=0.001, help="Merged output voxel in metres")
    parser.add_argument(
        "--max-correspondence",
        type=float,
        nargs="+",
        default=[0.03, 0.015, 0.008],
        help="Coarse-to-fine correspondence distances in metres",
    )
    parser.add_argument("--iterations", type=int, default=25, help="Maximum ICP iterations per scale")
    parser.add_argument("--trim-fraction", type=float, default=0.7, help="Fraction of closest matches retained")
    parser.add_argument("--tolerance", type=float, default=1e-6, help="ICP RMSE convergence tolerance in metres")
    parser.add_argument("--max-correction-translation", type=float, default=0.08, help="Reject larger ICP shifts (m)")
    parser.add_argument("--max-correction-rotation", type=float, default=12.0, help="Reject larger ICP rotations (deg)")
    parser.add_argument(
        "--max-points-per-view",
        type=int,
        default=0,
        help="Point cap before output; 0 preserves every depth-filtered point",
    )
    parser.add_argument("--max-icp-points", type=int, default=80000)
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.15,
        help="Discard camera-frame points closer than this before registration (m)",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=0.70,
        help="Discard camera-frame points farther than this before registration (m)",
    )
    parser.add_argument(
        "--crop-margin",
        type=float,
        default=0.05,
        help="Horizontal margin added to the detected plant radius (m)",
    )
    parser.add_argument(
        "--crop-below",
        type=float,
        default=0.12,
        help="Keep this far below the detected plant target (m)",
    )
    parser.add_argument(
        "--crop-above",
        type=float,
        default=0.30,
        help="Keep this far above the detected plant target (m)",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--coarse-rgbd",
        action="store_true",
        help="Use adjacent-view SIFT+3D RANSAC before ICP instead of TF alone",
    )
    parser.add_argument("--save-debug-views", action="store_true")
    args = parser.parse_args()

    scan_dir = args.scan_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser() if args.output_dir else scan_dir / "reconstruction_icp"
    output_dir.mkdir(parents=True, exist_ok=True)
    plants = [scan_dir / args.plant] if args.plant else sorted(scan_dir.glob("plant_*"))
    plants = [plant for plant in plants if plant.is_dir()]
    if not plants:
        raise RuntimeError(f"No plant directories found in {scan_dir}")
    if not 0 < args.trim_fraction <= 1:
        raise ValueError("--trim-fraction must be in (0, 1]")

    for plant in plants:
        align_plant(plant, output_dir, args)


if __name__ == "__main__":
    main()
