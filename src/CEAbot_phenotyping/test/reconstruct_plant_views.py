#!/usr/bin/env python3

from pathlib import Path
import argparse
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


VIEW_COLORS = np.array(
    [
        [255, 0, 0],
        [0, 180, 0],
        [0, 80, 255],
        [255, 180, 0],
        [220, 0, 220],
        [0, 200, 200],
    ],
    dtype=np.float64,
)


def parse_meta_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as metadata_file:
        return yaml.safe_load(metadata_file) or {}


def pose_to_matrix(meta: dict) -> np.ndarray:
    required = [
        "actual_x",
        "actual_y",
        "actual_z",
        "actual_qx",
        "actual_qy",
        "actual_qz",
        "actual_qw",
    ]
    missing = [key for key in required if key not in meta]
    if missing:
        raise ValueError(f"missing pose fields: {missing}")

    translation = np.array(
        [
            float(meta["actual_x"]),
            float(meta["actual_y"]),
            float(meta["actual_z"]),
        ],
        dtype=np.float64,
    )
    quat_xyzw = np.array(
        [
            float(meta["actual_qx"]),
            float(meta["actual_qy"]),
            float(meta["actual_qz"]),
            float(meta["actual_qw"]),
        ],
        dtype=np.float64,
    )

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    transform[:3, 3] = translation
    return transform


def transform_points(points_xyz: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points_xyz @ transform[:3, :3].T + transform[:3, 3]


def limit_points(points: np.ndarray, max_points: int, seed: int = 7) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points

    rng = np.random.default_rng(seed)
    indices = rng.choice(len(points), size=max_points, replace=False)
    indices.sort()
    return points[indices]


def voxel_downsample_xyzrgb(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(points) == 0:
        return points

    voxels = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, keep_indices = np.unique(voxels, axis=0, return_index=True)
    keep_indices.sort()
    return points[keep_indices]


def load_cloud_xyzrgb(view_dir: Path, max_points_per_view: int) -> np.ndarray:
    cloud_path = view_dir / "cloud_xyzrgb.npy"
    if not cloud_path.exists():
        raise FileNotFoundError(f"missing {cloud_path}")

    cloud = np.load(cloud_path)
    if cloud.ndim != 2 or cloud.shape[1] < 6:
        raise ValueError(f"{cloud_path} must have shape Nx6 or larger")

    cloud = cloud[:, :6].astype(np.float64, copy=False)
    valid = np.isfinite(cloud[:, 0]) & np.isfinite(cloud[:, 1]) & np.isfinite(cloud[:, 2])
    cloud = cloud[valid]
    return limit_points(cloud, max_points_per_view)


def save_binary_ply(path: Path, points: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)

    ply = np.empty(
        len(points),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    ply["x"] = points[:, 0].astype(np.float32)
    ply["y"] = points[:, 1].astype(np.float32)
    ply["z"] = points[:, 2].astype(np.float32)
    ply["red"] = np.clip(points[:, 3], 0, 255).astype(np.uint8)
    ply["green"] = np.clip(points[:, 4], 0, 255).astype(np.uint8)
    ply["blue"] = np.clip(points[:, 5], 0, 255).astype(np.uint8)

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(ply)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        ply.tofile(f)


def recolor_cloud(points: np.ndarray, color_index: int) -> np.ndarray:
    recolored = points.copy()
    recolored[:, 3:6] = VIEW_COLORS[color_index % len(VIEW_COLORS)]
    return recolored


def reconstruct_plant(plant_dir: Path,output_dir: Path,max_points_per_view: int,voxel_size: float,save_debug_views: bool,color_by_view: bool,):
    view_dirs = sorted(path for path in plant_dir.iterdir() if path.is_dir())
    transformed_views = []

    print(f"\n{plant_dir.name}: found {len(view_dirs)} view folders")

    for view_index, view_dir in enumerate(view_dirs):
        meta_path = view_dir / "meta.yaml"
        if not meta_path.exists():
            print(f"  skip {view_dir.name}: missing meta.yaml")
            continue

        try:
            meta = parse_meta_yaml(meta_path)
            pose_child_frame = meta.get("actual_pose_child_frame", "unknown")

            if pose_child_frame != "camera_color_optical_frame":
                raise ValueError(
                    "unsupported actual_pose_child_frame "
                    f"{pose_child_frame!r}; expected camera_color_optical_frame"
                )

            transform_world_camera = pose_to_matrix(meta)

            cloud = load_cloud_xyzrgb(view_dir, max_points_per_view)
            cloud[:, :3] = transform_points(cloud[:, :3], transform_world_camera)
            if color_by_view:
                cloud = recolor_cloud(cloud, view_index)
            transformed_views.append(cloud)

            if save_debug_views:
                debug_path = output_dir / "debug_views" / plant_dir.name / f"{view_dir.name}_transformed.ply"
                save_binary_ply(debug_path, cloud)

            print(f"  loaded {view_dir.name}: {len(cloud)} points")
        except Exception as exc:
            print(f"  skip {view_dir.name}: {exc}")

    if not transformed_views:
        print(f"  no usable views for {plant_dir.name}")
        return

    merged = np.vstack(transformed_views)
    before_downsample = len(merged)
    merged = voxel_downsample_xyzrgb(merged, voxel_size)

    output_path = output_dir / f"{plant_dir.name}_merged.ply"
    save_binary_ply(output_path, merged)

    print(
        f"  saved {output_path} "
        f"({before_downsample} -> {len(merged)} points, voxel={voxel_size} m)"
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Merge plant view point clouds using saved camera poses. "
            "Metadata must store zed2i_left_camera_frame_optical -> "
            "camera_color_optical_frame."
        )
    )
    parser.add_argument("scan_dir", type=Path, help="Folder containing plant_XX/view folders")
    parser.add_argument("--plant", default="", help="Only reconstruct one plant, e.g. plant_01")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory for merged PLY files")
    parser.add_argument("--voxel-size", type=float, default=0.003, help="Voxel size in meters. Use 0 to disable.")
    parser.add_argument("--save-debug-views", action="store_true", help="Save each transformed view as its own PLY.")
    parser.add_argument("--color-by-view", action="store_true", help="Override RGB so each view has a unique debug color.")
    parser.add_argument(
        "--max-points-per-view",
        type=int,
        default=250000,
        help="Random point cap per view before merging. Use 0 for all points.",
    )
    args = parser.parse_args()

    scan_dir = args.scan_dir.expanduser()
    output_dir = args.output_dir.expanduser() if args.output_dir else scan_dir / "reconstruction"

    if args.plant:
        plant_dirs = [scan_dir / args.plant]
    else:
        plant_dirs = sorted(path for path in scan_dir.glob("plant_*") if path.is_dir())

    if not plant_dirs:
        raise RuntimeError(f"No plant folders found in {scan_dir}")

    for plant_dir in plant_dirs:
        reconstruct_plant(
            plant_dir,
            output_dir,
            args.max_points_per_view,
            args.voxel_size,
            args.save_debug_views,
            args.color_by_view,
        )


if __name__ == "__main__":
    main()
