"""Optional registration backends for reconstruct_plant_views.py.

All input clouds are already in base_link. A returned correction left-multiplies
the saved camera pose. Each method starts from the same unmodified input views.
Open3D is imported only when registration is requested.
"""

import argparse
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import yaml


METHODS = ("robust_icp", "colored_icp")


@dataclass
class RegistrationView:
    path: Path
    points: np.ndarray  # Full depth-filtered XYZRGB cloud in base_link.
    pose: np.ndarray  # base_link <- camera.
    color_index: int = 0


def read_rgbd(view_dir):
    """Read aligned RGB-D on the pinhole pixel grid used by reconstruction.

    This preserves the existing projection convention; it does not rectify raw
    images or register an unaligned depth image to the color camera.
    """
    required = [view_dir / name for name in ("depth.npy", "color.png", "meta.yaml")]
    missing = [path.name for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing required files: " + ", ".join(missing))
    with required[2].open(encoding="utf-8") as stream:
        meta = yaml.safe_load(stream) or {}
    info = meta.get("color_camera_info", {})
    k = info.get("k") if isinstance(info, dict) else None
    if not isinstance(k, list) or len(k) != 9:
        raise ValueError("color_camera_info.k must contain 9 values")
    intrinsics = np.asarray([k[0], k[4], k[2], k[5]], dtype=float)
    if not np.all(np.isfinite(intrinsics)) or np.any(intrinsics[:2] <= 0):
        raise ValueError("invalid color camera intrinsics")
    scale = float(meta.get("depth_scale_m_per_unit", 0.001))
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("invalid depth_scale_m_per_unit")
    depth = np.load(required[0], allow_pickle=False)
    color = cv2.imread(str(required[1]), cv2.IMREAD_COLOR)
    if depth.ndim != 2 or color is None or color.shape[:2] != depth.shape:
        raise ValueError("depth must be HxW and color.png must have the same size")
    return depth.astype(np.float32) * np.float32(scale), color, intrinsics


def add_registration_arguments(parser):
    group = parser.add_argument_group("registration methods (independent comparison runs)")
    group.add_argument("--all-methods", action="store_true", help="Enable robust and colored ICP; individual --no-* switches override this")
    for method in METHODS:
        group.add_argument("--" + method.replace("_", "-"), action=argparse.BooleanOptionalAction,
                           default=None, help=f"Enable/disable a separate {method} reconstruction (default: off)")
    group.add_argument("--baseline", action=argparse.BooleanOptionalAction, default=True,
                       help="Also save the original pose-only reconstruction (default: on)")
    group.add_argument("--run-name", help="Comparison subfolder name; defaults to a unique UTC timestamp")
    group.add_argument("--plant", help="Process only this plant directory, e.g. plant_02")
    group.add_argument("--save-debug-views", action="store_true", help="Save each corrected view as a separate PLY")
    group.add_argument("--pairing", choices=("adjacent", "all"), default="adjacent",
                       help="Adjacent side views plus top-to-side pairs, or all pairs")
    group.add_argument("--max-pair-angle", type=float, default=80.0,
                       help="Maximum side-view angular gap for adjacent pairing, degrees")
    group.add_argument("--registration-margin", type=float, default=0.03,
                       help="Extra crop padding during registration, metres; final crop is unchanged")
    group.add_argument("--pose-graph", action=argparse.BooleanOptionalAction, default=True,
                       help="Jointly optimize accepted pairwise constraints (default: on)")
    group = parser.add_argument_group("ICP parameters (metres unless stated)")
    group.add_argument("--icp-voxels", type=float, nargs="+", default=[0.008, 0.004, 0.002])
    group.add_argument("--icp-distances", type=float, nargs="+", default=[0.03, 0.015, 0.008],
                       help="Maximum correspondence distance at each ICP scale; same length as --icp-voxels")
    group.add_argument("--icp-iterations", type=int, default=40, help="Maximum iterations per scale")
    group.add_argument("--normal-radius-factor", type=float, default=2.5, help="Normal search radius / voxel size")
    group.add_argument("--normal-max-nn", type=int, default=30)
    group.add_argument("--robust-kernel", choices=("huber", "tukey"), default="huber")
    group.add_argument("--robust-scale", type=float, default=0.01, help="Robust loss scale, metres")
    group.add_argument("--colored-geometric-weight", type=float, default=0.968,
                       help="Colored ICP geometry weight in [0,1]; color weight is 1 minus this")

    group = parser.add_argument_group("pair acceptance and pose correction limits")
    group.add_argument("--min-fitness", type=float, default=0.20,
                       help="Minimum matched-point fraction in BOTH directions")
    group.add_argument("--min-correspondences", type=int, default=30)
    group.add_argument("--max-rmse", type=float, default=0.008, help="Maximum inlier RMSE in BOTH directions, metres")
    group.add_argument("--max-correction-translation", type=float, default=0.05,
                       help="Maximum displacement of the plant center from the saved pose, metres")
    group.add_argument("--max-correction-rotation", type=float, default=12.0, help="Maximum correction angle, degrees")



def validate_registration_arguments(parser, args):
    args.methods = [name for name in METHODS
                    if (args.all_methods if getattr(args, name) is None else getattr(args, name))]
    if not args.methods and not args.baseline:
        parser.error("enable at least one registration method or --baseline")
    if args.run_name and (args.run_name in {".", ".."} or Path(args.run_name).name != args.run_name):
        parser.error("--run-name must be a single directory name")
    if args.plant and (not args.plant.startswith("plant_") or Path(args.plant).name != args.plant):
        parser.error("--plant must be a directory name such as plant_02")
    positive = ("icp_iterations", "normal_radius_factor", "normal_max_nn", "robust_scale",
                "min_correspondences", "max_rmse", "max_correction_translation",
                "max_correction_rotation")
    for name in positive:
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("registration_margin", "voxel_size", "min_depth", "crop_margin", "crop_below", "crop_above"):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and nonnegative")
    if not np.isfinite(args.max_depth) or args.max_depth <= args.min_depth:
        parser.error("--max-depth must be finite and greater than --min-depth")
    for name in ("min_fitness", "colored_geometric_weight"):
        if not 0 <= getattr(args, name) <= 1:
            parser.error(f"--{name.replace('_', '-')} must be in [0,1]")
    if not 0 < args.max_pair_angle <= 180 or not 0 < args.max_correction_rotation <= 180:
        parser.error("angular limits must be in (0,180] degrees")
    if args.normal_max_nn < 3 or args.min_correspondences < 3:
        parser.error("normal neighbors and correspondences must each be at least 3")
    if len(args.icp_voxels) != len(args.icp_distances):
        parser.error("--icp-voxels and --icp-distances must have the same length")
    for name in ("icp_voxels", "icp_distances"):
        values = np.asarray(getattr(args, name))
        if not np.all(np.isfinite(values)) or np.any(values <= 0) or np.any(np.diff(values) > 0):
            parser.error(f"--{name.replace('_', '-')} must contain positive, coarse-to-fine values")


@lru_cache(maxsize=1)
def require_open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "Registration requires Open3D. Install with: python3 -m pip install "
            "'open3d>=0.19,<0.20'. "
            "Pose-only reconstruction does not require Open3D."
        ) from exc
    return o3d


def transform_xyz(xyz, transform):
    return xyz @ transform[:3, :3].T + transform[:3, 3]


def crop_mask(xyz, center, radius, below, above):
    return ((np.linalg.norm(xyz[:, :2] - center[:2], axis=1) <= radius)
            & (xyz[:, 2] >= center[2] - below) & (xyz[:, 2] <= center[2] + above))


def candidate_pairs(views, pairing, max_angle):
    """Only connect actual neighboring angles across acceptable circular gaps."""
    if pairing == "all":
        return [(i, j) for i in range(len(views)) for j in range(i + 1, len(views))]
    sides = sorted((float(v.path.name.rsplit("_", 1)[1][:-3]) % 360, i)
                   for i, v in enumerate(views) if v.path.name != "top")
    pairs = set()
    if len(sides) > 1:
        for (a, i), (b, j) in zip(sides, sides[1:] + sides[:1]):
            if abs((a - b + 180) % 360 - 180) <= max_angle:
                pairs.add(tuple(sorted((i, j))))
    for i, view in enumerate(views):
        if view.path.name == "top":
            pairs.update(tuple(sorted((i, j))) for _, j in sides)
    return sorted(pairs)


def correction_size(transform, center):
    if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-5)):
        raise ValueError("registration returned a non-rigid or non-finite transform")
    shift = np.linalg.norm(transform_xyz(center[None, :], transform)[0] - center)
    angle = np.degrees(np.arccos(np.clip((np.trace(transform[:3, :3]) - 1) / 2, -1, 1)))
    return float(shift), float(angle)


def check_correction(transform, center, args):
    shift, angle = correction_size(transform, center)
    if shift > args.max_correction_translation or angle > args.max_correction_rotation:
        raise ValueError(f"correction exceeds limits: {shift * 1000:.1f} mm / {angle:.2f} deg")


def make_cloud(points, voxel, camera_position, args):
    o3d = require_open3d()
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points[:, :3])
    cloud.colors = o3d.utility.Vector3dVector(np.clip(points[:, 3:6] / 255.0, 0, 1))
    cloud = cloud.voxel_down_sample(voxel)
    if len(cloud.points) < args.min_correspondences:
        raise ValueError(f"only {len(cloud.points)} registration points at voxel {voxel:g} m")
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel * args.normal_radius_factor, max_nn=args.normal_max_nn))
    cloud.orient_normals_towards_camera_location(camera_position)
    return cloud


def refine_icp(source_scales, target_scales, initial, method, args):
    reg = require_open3d().pipelines.registration
    transform = initial.copy()
    for source, target, distance in zip(source_scales, target_scales, args.icp_distances):
        criteria = reg.ICPConvergenceCriteria(max_iteration=args.icp_iterations)
        if method == "colored_icp":
            result = reg.registration_colored_icp(
                source, target, distance, transform,
                reg.TransformationEstimationForColoredICP(args.colored_geometric_weight), criteria)
        else:
            loss = (reg.HuberLoss if args.robust_kernel == "huber" else reg.TukeyLoss)(args.robust_scale)
            result = reg.registration_icp(source, target, distance, transform,
                                          reg.TransformationEstimationPointToPlane(loss), criteria)
        if len(result.correspondence_set) < args.min_correspondences:
            raise ValueError("too few ICP correspondences")
        transform = result.transformation.copy()
    return transform


def registration_metrics(source, target, transform, distance):
    reg = require_open3d().pipelines.registration
    forward = reg.evaluate_registration(source, target, distance, transform)
    reverse = reg.evaluate_registration(target, source, distance, np.linalg.inv(transform))
    return {"fitness": min(float(forward.fitness), float(reverse.fitness)),
            "rmse_m": max(float(forward.inlier_rmse), float(reverse.inlier_rmse)),
            "correspondences": min(len(forward.correspondence_set), len(reverse.correspondence_set)),
            "forward_fitness": float(forward.fitness), "reverse_fitness": float(reverse.fitness)}


def solve_pose_graph(views, edges, center, args):
    """Anchor each connected component to a saved pose; isolated views stay put.

    Maximum-weight spanning trees initialize consistent corrections. Optional
    Open3D optimization adds the remaining edges as uncertain loop constraints.
    A component exceeding correction limits is reverted as a whole.
    """
    o3d = require_open3d()
    reg = o3d.pipelines.registration
    corrections = [np.eye(4) for _ in views]
    remaining = set(range(len(views)))
    components = []
    while remaining:
        # Prefer a side view as anchor rather than a possibly weak top view.
        root = min(remaining, key=lambda i: (views[i].path.name == "top", i))
        visited, tree_edges = {root}, set()
        while True:
            frontier = [(edge["weight"], k, edge) for k, edge in enumerate(edges)
                        if (edge["source"] in visited) != (edge["target"] in visited)]
            if not frontier:
                break
            _, k, edge = max(frontier, key=lambda item: (item[0], -item[1]))
            i, j, transform = edge["source"], edge["target"], edge["transform"]
            if i in visited:
                corrections[j] = corrections[i] @ np.linalg.inv(transform)
                visited.add(j)
            else:
                corrections[i] = corrections[j] @ transform
                visited.add(i)
            tree_edges.add(k)
        nodes = sorted(visited)
        remaining -= visited
        status = "saved_pose_only" if len(nodes) == 1 else "spanning_tree"
        reason = None
        if len(nodes) > 1:
            try:
                if args.pose_graph:
                    index = {node: i for i, node in enumerate(nodes)}
                    graph = reg.PoseGraph()
                    for node in nodes:
                        graph.nodes.append(reg.PoseGraphNode(corrections[node]))
                    for k, edge in enumerate(edges):
                        if edge["source"] in visited and edge["target"] in visited:
                            graph.edges.append(reg.PoseGraphEdge(
                                index[edge["source"]], index[edge["target"]], edge["transform"],
                                edge["information"], uncertain=k not in tree_edges))
                    reg.global_optimization(
                        graph, reg.GlobalOptimizationLevenbergMarquardt(),
                        reg.GlobalOptimizationConvergenceCriteria(),
                        reg.GlobalOptimizationOption(max_correspondence_distance=args.icp_distances[-1],
                                                     reference_node=index[root]))
                    # Preserve the component's saved base_link anchor exactly.
                    anchor_inverse = np.linalg.inv(graph.nodes[index[root]].pose)
                    for node in nodes:
                        corrections[node] = anchor_inverse @ graph.nodes[index[node]].pose
                    status = "pose_graph"
                for node in nodes:
                    check_correction(corrections[node], center, args)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                for node in nodes:
                    corrections[node] = np.eye(4)
                status, reason = "rejected_component_saved_poses", str(exc)
        components.append({"views": [views[i].path.name for i in nodes],
                           "anchor": views[root].path.name, "status": status, "reason": reason})
    return corrections, components


def align_views(views, method, center, radius, args):
    if method not in METHODS:
        raise ValueError(f"unsupported registration method: {method}")
    o3d = require_open3d()
    reg = o3d.pipelines.registration
    cache = {}
    pairs, edges = [], []

    def scales(index):
        if index not in cache:
            view = views[index]
            keep = crop_mask(view.points, center, radius + args.registration_margin,
                             args.crop_below + args.registration_margin,
                             args.crop_above + args.registration_margin)
            points = view.points[keep]
            cache[index] = [make_cloud(points, voxel, view.pose[:3, 3], args) for voxel in args.icp_voxels]
        return cache[index]

    for i, j in candidate_pairs(views, args.pairing, args.max_pair_angle):
        info = {"source": views[i].path.name, "target": views[j].path.name, "accepted": False}
        try:
            source, target = scales(i), scales(j)
            info["before"] = registration_metrics(source[-1], target[-1], np.eye(4), args.icp_distances[-1])
            transform = refine_icp(source, target, np.eye(4), method, args)
            check_correction(transform, center, args)
            after = registration_metrics(source[-1], target[-1], transform, args.icp_distances[-1])
            info["after"] = after
            if after["correspondences"] < args.min_correspondences or after["fitness"] < args.min_fitness:
                raise ValueError("insufficient overlap after registration")
            if after["rmse_m"] > args.max_rmse:
                raise ValueError("registration RMSE exceeds limit")
            information = reg.get_information_matrix_from_point_clouds(
                source[-1], target[-1], args.icp_distances[-1], transform)
            edges.append({"source": i, "target": j, "transform": transform, "information": information,
                          "weight": after["fitness"] / max(after["rmse_m"], 1e-6)})
            info.update(accepted=True, transform=transform.tolist())
            print(f"    {info['source']} -> {info['target']}: fitness {after['fitness']:.3f}, RMSE {after['rmse_m'] * 1000:.2f} mm", flush=True)
        except (ValueError, RuntimeError, OSError, cv2.error, np.linalg.LinAlgError) as exc:
            info["reason"] = str(exc)
            print(f"    {info['source']} -> {info['target']}: rejected ({exc})", flush=True)
        pairs.append(info)

    corrections, components = solve_pose_graph(views, edges, center, args)
    for component in components:
        if component["status"] in {"saved_pose_only", "rejected_component_saved_poses"}:
            print(f"    {component['status']}: {', '.join(component['views'])}", flush=True)
    # Measure the final graph solution as well as individual pairwise fits.
    for info in pairs:
        if info["accepted"]:
            i = next(k for k, v in enumerate(views) if v.path.name == info["source"])
            j = next(k for k, v in enumerate(views) if v.path.name == info["target"])
            relative = np.linalg.inv(corrections[j]) @ corrections[i]
            info["final"] = registration_metrics(scales(i)[-1], scales(j)[-1], relative, args.icp_distances[-1])
    return corrections, {"method": method, "open3d_version": o3d.__version__, "pairs": pairs,
                         "accepted_pairs": len(edges), "components": components}
