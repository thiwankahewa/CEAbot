#!/usr/bin/env python3

from pathlib import Path
import argparse
from datetime import datetime, timezone
import json
import re
import numpy as np
import yaml

from CEAbot_phenotyping.test.reconstruction.plant_view_registration import (
    RegistrationView,
    add_registration_arguments,
    align_views,
    correction_size,
    read_rgbd,
    require_open3d,
    validate_registration_arguments,
)


VIEW_COLORS = np.array(
    [
        [255, 0, 0],
        [0, 255, 0],
        [0, 100, 255],
        [255, 200, 0],
        [255, 0, 255],
        [0, 255, 255],
        [255, 100, 0],
        [140, 0, 255],
        [170, 255, 0],
        [255, 0, 120],
        [0, 160, 120],
        [160, 80, 0],
        [100, 180, 255],
        [255, 160, 200],
        [180, 180, 180],
    ],
    dtype=np.float64,
)

EXPECTED_POSE_FRAME = "base_link"
EXPECTED_POSE_CHILD_FRAME = "gemini336_color_optical_frame"
LEGACY_POSE_CHILD_FRAME = "camera_color_optical_frame"
EXPECTED_CLOUD_FRAME = "gemini336_color_optical_frame"
SCAN_TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"
SIDE_VIEW_PATTERN = re.compile(r"^view_\d+_(-?\d+(?:\.\d+)?)deg$")


def parse_meta_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as metadata_file:
        return yaml.safe_load(metadata_file) or {}


def pose_to_matrix(meta: dict) -> np.ndarray:
    required = ["actual_x","actual_y","actual_z","actual_qx","actual_qy","actual_qz","actual_qw",]
    missing = [key for key in required if key not in meta]
    if missing:
        raise ValueError(f"missing pose fields: {missing}")

    translation = np.array([float(meta["actual_x"]), float(meta["actual_y"]), float(meta["actual_z"]),], dtype=np.float64,)
    quat_xyzw = np.array([ float(meta["actual_qx"]), float(meta["actual_qy"]), float(meta["actual_qz"]), float(meta["actual_qw"]),], dtype=np.float64,)
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quat_xyzw)):
        raise ValueError("actual pose contains non-finite values")

    quat_norm = np.linalg.norm(quat_xyzw)
    if quat_norm < 1e-12:
        raise ValueError("actual pose quaternion has zero length")

    x, y, z, w = quat_xyzw / quat_norm
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    transform[:3, 3] = translation
    return transform


def transform_points(points_xyz: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points_xyz @ transform[:3, :3].T + transform[:3, 3]

def crop_world_points(points, center, radius, below, above):
    horizontal_distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    keep = ((horizontal_distance <= radius) & (points[:, 2] >= center[2] - below) & (points[:, 2] <= center[2] + above))
    return points[keep]

def voxel_downsample_xyzrgb(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(points) == 0:
        return points

    voxels = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, keep_indices = np.unique(voxels, axis=0, return_index=True)
    keep_indices.sort()
    return points[keep_indices]


def create_rgbd_cloud(view_dir: Path) -> np.ndarray:
    """Create camera-frame XYZRGB points in memory from depth and color data."""
    depth_m, color_bgr, (fx, fy, cx, cy) = read_rgbd(view_dir)
    rows, columns = np.nonzero(np.isfinite(depth_m) & (depth_m > 0.0))
    z = depth_m[rows, columns]
    cloud = np.empty((len(z), 6), dtype=np.float32)
    cloud[:, 0] = (columns.astype(np.float32) - cx) * z / fx
    cloud[:, 1] = (rows.astype(np.float32) - cy) * z / fy
    cloud[:, 2] = z
    cloud[:, 3:6] = color_bgr[rows, columns, ::-1]
    return cloud


def load_cloud_xyzrgb(view_dir: Path,minimum_depth: float,maximum_depth: float,cloud_source: str,) -> np.ndarray:
    if cloud_source == "original":
        cloud_path = view_dir / "cloud_xyzrgb.npy"
        if not cloud_path.exists():
            raise FileNotFoundError(f"missing {cloud_path}")
        cloud = np.load(cloud_path, allow_pickle=False)
        if cloud.ndim != 2 or cloud.shape[1] < 6:
            raise ValueError(f"{cloud_path} must have shape Nx6 or larger")
    else:
        cloud = create_rgbd_cloud(view_dir)

    cloud = cloud[:, :6].astype(np.float64, copy=False)
    valid = np.isfinite(cloud[:, 0]) & np.isfinite(cloud[:, 1]) & np.isfinite(cloud[:, 2])
    cloud = cloud[valid]
    cloud = cloud[(cloud[:, 2] >= minimum_depth) & (cloud[:, 2] <= maximum_depth)]
    return cloud


def get_plant_crop(scan_dir: Path, plant_dir: Path):
    scan_meta = parse_meta_yaml(scan_dir / "metadata.yaml")
    plant_id = int(plant_dir.name.removeprefix("plant_"))
    plant = next((item for item in scan_meta.get("plants", []) if int(item["plant_id"]) == plant_id),None,)
    if plant is None:
        raise ValueError(f"metadata.yaml has no target for {plant_dir.name}")

    target_base = plant.get("target_base")
    if target_base is None:
        raise ValueError(f"metadata.yaml has no target_base for {plant_dir.name}; ""capture a new scan with the updated plant_view_scanner")
    if target_base.get("frame_id") != EXPECTED_POSE_FRAME:
        raise ValueError(f"unsupported target_base frame {target_base.get('frame_id')!r}; "f"expected {EXPECTED_POSE_FRAME!r}")
    center = np.array([target_base["x_m"], target_base["y_m"], target_base["z_m"]],dtype=np.float64,)
    radius = float(plant["radius_mm"]) / 1000.0
    return center, radius


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


def circular_angle_distance(first: float, second: float) -> float:
    """Return the shortest distance between two angles in degrees."""
    return abs((first - second + 180.0) % 360.0 - 180.0)


def select_view_dirs(plant_dir: Path, number_of_side_views: int | None) -> list[Path]:
    """Select evenly spaced side views and always include the top view."""
    all_view_dirs = sorted(path for path in plant_dir.iterdir() if path.is_dir())
    top_views = [path for path in all_view_dirs if path.name == "top"]
    side_views = []

    for path in all_view_dirs:
        match = SIDE_VIEW_PATTERN.match(path.name)
        if match:
            side_views.append((float(match.group(1)) % 360.0, path))

    if number_of_side_views is None or number_of_side_views >= len(side_views):
        selected_sides = [path for _, path in sorted(side_views)]
    elif number_of_side_views == 0:
        selected_sides = []
    else:
        remaining = list(side_views)
        anchor_angle, anchor_path = min(remaining, key=lambda item: (circular_angle_distance(item[0], 0.0), item[0], item[1].name),)
        selected_sides = [anchor_path]
        remaining.remove((anchor_angle, anchor_path))
        targets = [(anchor_angle + index * 360.0 / number_of_side_views) % 360.0 for index in range(1, number_of_side_views)]
        for target in targets:
            angle, path = min(remaining, key=lambda item: (circular_angle_distance(item[0], target), item[0], item[1].name))
            selected_sides.append(path)
            remaining.remove((angle, path))

    return top_views + selected_sides

def reconstruct_plant(scan_dir: Path,plant_dir: Path,output_dir: Path,voxel_size: float,color_by_view: bool,minimum_depth: float,maximum_depth: float,crop_margin: float,crop_below: float,crop_above: float,number_of_side_views: int | None,cloud_source: str,registration_args=None):
    view_dirs = select_view_dirs(plant_dir, number_of_side_views)
    views = []
    skipped_views = []

    first_meta_path = next((view / "meta.yaml" for view in view_dirs if (view / "meta.yaml").exists()),None,)
    if first_meta_path is None:
        print(f"{plant_dir.name}: no view metadata found")
        return

    plant_center, detected_radius = get_plant_crop(scan_dir, plant_dir)
    crop_radius = detected_radius + crop_margin
    print(f"\n{plant_dir.name}: using {len(view_dirs)} views: " + ", ".join(view.name for view in view_dirs))
    if not any(view.name == "top" for view in view_dirs):
        print("  warning: top view is unavailable for this plant")

    for view_index, view_dir in enumerate(view_dirs):
        meta_path = view_dir / "meta.yaml"
        if not meta_path.exists():
            print(f"  skip {view_dir.name}: missing meta.yaml")
            skipped_views.append({"view": view_dir.name, "reason": "missing meta.yaml"})
            continue

        try:
            meta = parse_meta_yaml(meta_path)
            pose_frame = meta.get("pose_frame", "unknown")
            pose_child_frame = meta.get("pose_child_frame", "unknown")
            cloud_frame = meta.get("camera_frame", meta.get("frame_id", "unknown"))

            if pose_frame not in {EXPECTED_POSE_FRAME}:
                raise ValueError(f"unsupported pose_frame {pose_frame!r}; expected {EXPECTED_POSE_FRAME}")
            if pose_child_frame not in {EXPECTED_POSE_CHILD_FRAME,LEGACY_POSE_CHILD_FRAME,}:
                raise ValueError(f"unsupported pose_child_frame {pose_child_frame!r}; "f"expected {EXPECTED_POSE_CHILD_FRAME} "f"(or legacy {LEGACY_POSE_CHILD_FRAME})")
            if cloud_frame != EXPECTED_CLOUD_FRAME:
                raise ValueError(f"unsupported point-cloud frame {cloud_frame!r}; "f"expected {EXPECTED_CLOUD_FRAME}")

            transform_world_camera = pose_to_matrix(meta)

            cloud = load_cloud_xyzrgb(view_dir,minimum_depth,maximum_depth,cloud_source,)
            cloud[:, :3] = transform_points(cloud[:, :3], transform_world_camera)
            if len(cloud) == 0:
                raise ValueError("no valid points in the requested depth range")
            views.append(RegistrationView(view_dir, cloud, transform_world_camera, view_index))

        except Exception as exc:
            print(f"  skip {view_dir.name}: {exc}")
            skipped_views.append({"view": view_dir.name, "reason": str(exc)})

    if not views:
        print(f"  no usable views for {plant_dir.name}")
        return []

    methods = registration_args.methods if registration_args else []
    baseline = registration_args.baseline if registration_args else True
    comparison = bool(methods or (registration_args and registration_args.run_name))
    saved_paths = []
    for method in (["baseline"] if baseline else []) + methods:
        method_dir = output_dir / method if comparison else output_dir
        print(f"  {method}: {method_dir}", flush=True)
        if method == "baseline":
            corrections = [np.eye(4) for _ in views]
            diagnostics = {"method": method, "pairs": [], "accepted_pairs": 0, "components": []}
        else:
            corrections, diagnostics = align_views(views, method, plant_center, crop_radius, registration_args)
        transformed_views, poses = [], {}
        for view, correction in zip(views, corrections):
            # Work from the full cloud so final cropping cannot discard points
            # merely because their initial pose was slightly wrong.
            cloud = view.points.copy()
            cloud[:, :3] = transform_points(cloud[:, :3], correction)
            cloud = crop_world_points(cloud, plant_center, crop_radius, crop_below, crop_above)
            if color_by_view:
                cloud = recolor_cloud(cloud, view.color_index)
            transformed_views.append(cloud)
            shift, angle = correction_size(correction, plant_center)
            poses[view.path.name] = {"saved_pose": view.pose.tolist(), "correction": correction.tolist(),
                                    "refined_pose": (correction @ view.pose).tolist(), "points": len(cloud),
                                    "center_displacement_m": shift, "rotation_deg": angle}
            if registration_args and registration_args.save_debug_views:
                save_binary_ply(method_dir / "debug_views" / plant_dir.name / f"{view.path.name}.ply", cloud)
        merged = np.vstack(transformed_views)
        before_downsample = len(merged)
        merged = voxel_downsample_xyzrgb(merged, voxel_size)
        output_suffix = "_merged.ply" if cloud_source == "original" else "_merged_rgbd.ply"
        output_path = method_dir / f"{plant_dir.name}{output_suffix}"
        save_binary_ply(output_path, merged)
        diagnostics.update(scan=str(scan_dir.resolve()), plant=plant_dir.name, poses=poses,
                           skipped_views=skipped_views, points_before_downsample=before_downsample,
                           output_points=len(merged), evaluation_distance_m=(registration_args.icp_distances[-1]
                           if registration_args else None))
        if registration_args:
            diagnostics["parameters"] = vars(registration_args)
        write_json(method_dir / f"{plant_dir.name}_registration.json", diagnostics)
        print(f"  saved {output_path} ({before_downsample} -> {len(merged)} points)", flush=True)
        saved_paths.append(output_path)
    return saved_paths


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, default=str, allow_nan=False)
        stream.write("\n")


def build_parser():
    parser = argparse.ArgumentParser(description="Reconstruct plant views with saved poses, robust ICP, or colored ICP.")
    parser.add_argument("input_dir", type=Path, help="A scan folder, or a parent folder containing timestamped scans")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output root; comparison runs use <root>/<run>/<method>/")
    parser.add_argument("--views", type=int, default=None, help="Number of evenly spaced side views to use; the top view is always added when available")
    parser.add_argument("--start-time", type=parse_cli_timestamp, default=None, help="Inclusive start: YYYYMMDD_HHMMSS or YYYY-MM-DD[ HH:MM[:SS]]")
    parser.add_argument("--end-time", type=parse_cli_timestamp, default=None, help="Inclusive end: YYYYMMDD_HHMMSS or YYYY-MM-DD[ HH:MM[:SS]]")
    parser.add_argument("--voxel-size", type=float, default=0, help="Voxel size in meters. Use 0 to disable.")
    parser.add_argument("--color-by-view", action="store_true", help="Override RGB so each view has a unique debug color.")
    parser.add_argument("--cloud-source",choices=("original", "rgbd"),default="rgbd",help="Input per view: RGB-D converted in memory (default), or legacy cloud_xyzrgb.npy",)
    parser.add_argument("--min-depth", type=float, default=0.15, help="Minimum camera-frame depth in metres.")
    parser.add_argument("--max-depth", type=float, default=0.70, help="Maximum camera-frame depth in metres.")
    parser.add_argument("--crop-margin",type=float,default=0.05,help="Horizontal margin added to detected plant radius in metres.",)
    parser.add_argument("--crop-below", type=float, default=0.12)
    parser.add_argument("--crop-above", type=float, default=0.30)
    add_registration_arguments(parser)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_registration_arguments(parser, args)

    if args.views is not None and args.views < 0:
        parser.error("--views must be zero or greater")
    if args.start_time and args.end_time and args.start_time > args.end_time:
        parser.error("--start-time must not be later than --end-time")

    input_dir = args.input_dir.expanduser()
    if not input_dir.is_dir():
        parser.error(f"input directory does not exist: {input_dir}")

    scan_dirs = find_scan_dirs(input_dir, args.start_time, args.end_time)
    if not scan_dirs:
        raise RuntimeError(f"No scan folders found in {input_dir} for the requested date/time range")

    if args.methods:
        try:
            require_open3d()
        except RuntimeError as exc:
            parser.error(str(exc))
    comparison = bool(args.methods or args.run_name)
    run_name = args.run_name or datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fUTC")

    multiple_scans = len(scan_dirs) > 1 or scan_dirs[0] != input_dir
    output_root = args.output_dir.expanduser() if args.output_dir else None
    for scan_dir in scan_dirs:
        print(f"\nProcessing scan: {scan_dir.name}")
        if output_root is None:
            output_dir = scan_dir / "reconstruction"
        elif multiple_scans:
            output_dir = output_root / scan_dir.name
        else:
            output_dir = output_root

        if comparison:
            output_dir = output_dir / run_name
            try:
                output_dir.mkdir(parents=True, exist_ok=False)
            except FileExistsError:
                parser.error(f"comparison run already exists: {output_dir}; choose a different --run-name")
            write_json(output_dir / "run_config.json", {"scan": str(scan_dir.resolve()),
                       "created_utc": datetime.now(timezone.utc).isoformat(), "parameters": vars(args)})

        plant_dirs = sorted(path for path in scan_dir.glob("plant_*") if path.is_dir())
        if args.plant:
            plant_dirs = [path for path in plant_dirs if path.name == args.plant]
            if not plant_dirs:
                parser.error(f"{scan_dir} does not contain {args.plant}")
        for plant_dir in plant_dirs:
            reconstruct_plant(scan_dir,plant_dir,output_dir,args.voxel_size,args.color_by_view,args.min_depth,args.max_depth,args.crop_margin,args.crop_below,args.crop_above,args.views,args.cloud_source,args)

def find_scan_dirs(input_dir: Path, start_time: datetime | None, end_time: datetime | None) -> list[Path]:
    """Find one explicit scan or timestamped scans immediately below a root."""
    if any(path.is_dir() for path in input_dir.glob("plant_*")):
        candidates = [input_dir]
    else:
        candidates = [path for path in input_dir.iterdir() if path.is_dir()]

    selected = []
    for candidate in candidates:
        timestamp = scan_timestamp(candidate)
        if timestamp is None:
            continue
        if start_time is not None and timestamp < start_time:
            continue
        if end_time is not None and timestamp > end_time:
            continue
        if any(path.is_dir() for path in candidate.glob("plant_*")):
            selected.append(candidate)
    return sorted(selected, key=lambda path: (scan_timestamp(path), path.name))

def scan_timestamp(scan_dir: Path) -> datetime | None:
    metadata_path = scan_dir / "metadata.yaml"
    if metadata_path.exists():
        try:
            value = parse_meta_yaml(metadata_path).get("top_scan_timestamp")
            if value:
                return datetime.strptime(str(value), SCAN_TIMESTAMP_FORMAT)
        except (OSError, ValueError, TypeError):
            pass

    match = re.search(r"(\d{8}_\d{6})$", scan_dir.name)
    if match:
        return datetime.strptime(match.group(1), SCAN_TIMESTAMP_FORMAT)
    return None

def parse_cli_timestamp(value: str) -> datetime:
    """Accept scan timestamps and common ISO-style date/time values."""
    normalized = value.strip().replace("T", " ")
    formats = ( SCAN_TIMESTAMP_FORMAT, "%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d",)
    for timestamp_format in formats:
        try:
            return datetime.strptime(normalized, timestamp_format)
        except ValueError:
            pass
    raise argparse.ArgumentTypeError(f"invalid date/time {value!r}; use YYYYMMDD_HHMMSS or YYYY-MM-DD[ HH:MM[:SS]]")


if __name__ == "__main__":
    main()
