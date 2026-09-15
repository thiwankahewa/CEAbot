#!/usr/bin/env python3
"""Measure natural vertical plant height using local soil and explicit soil history.

Reads original RGB-D views, optionally applying saved registration poses. It does
not depend on Open3D or on the cropped reconstruction PLY. See PLANT_HEIGHT.md.
"""

import argparse
import csv
from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, QhullError, cKDTree

from CEAbot_phenotyping.test.reconstruction.plant_view_registration import read_rgbd
from CEAbot_phenotyping.test.reconstruction.reconstruct_plant_views import (
    EXPECTED_CLOUD_FRAME, EXPECTED_POSE_CHILD_FRAME, LEGACY_POSE_CHILD_FRAME,
    parse_meta_yaml, pose_to_matrix, save_binary_ply, scan_timestamp, transform_points,
)


@dataclass
class Settings:
    soil_inner_radius_m: float = 0.008
    soil_outer_radius_m: float = 0.045
    soil_grid_m: float = 0.002
    soil_residual_m: float = 0.004
    soil_min_cells: int = 60
    soil_min_inlier_fraction: float = 0.55
    soil_min_sectors: int = 5  # Of eight angular sectors around the stem.
    soil_max_nearest_m: float = 0.025
    soil_max_slope_deg: float = 25.0
    soil_min_hull_area_m2: float = 0.0005
    roi_margin_m: float = 0.030
    roi_min_radius_m: float = 0.060
    roi_below_target_m: float = 0.150
    roi_above_target_m: float = 0.300
    min_depth_m: float = 0.050
    max_depth_m: float = 0.800
    voxel_m: float = 0.001
    support_radius_m: float = 0.003
    support_min_neighbors: int = 3  # Excludes the query point itself.
    component_radius_m: float = 0.005
    component_min_seed_voxels: int = 8
    plant_above_soil_m: float = 0.006
    history_max_spread_m: float = 0.010
    history_max_stem_shift_m: float = 0.030
    top_agreement_m: float = 0.008


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def validate_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError("pose/reference transforms must be finite, rigid 4x4 matrices")
    return matrix


def vertical_basis(up):
    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
        raise ValueError("up must contain three finite values with nonzero length")
    up = up / np.linalg.norm(up)
    x = np.eye(3)[np.argmin(np.abs(up))]
    x = x - np.dot(x, up) * up
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(up, x), up])


def validate_settings(settings):
    for field in fields(settings):
        value = getattr(settings, field.name)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{field.name} must be finite and positive")
        if field.type is int and (isinstance(value, bool) or not isinstance(value, int)):
            raise ValueError(f"{field.name} must be an integer")
    if not 0 < settings.soil_min_inlier_fraction <= 1:
        raise ValueError("soil_min_inlier_fraction must be in (0, 1]")
    if not 1 <= settings.soil_min_sectors <= 8 or settings.soil_min_cells < 3:
        raise ValueError("soil needs at least 3 cells and 1..8 sectors")
    if not settings.soil_inner_radius_m < settings.soil_outer_radius_m:
        raise ValueError("soil_inner_radius_m must be smaller than soil_outer_radius_m")
    if settings.min_depth_m >= settings.max_depth_m or settings.soil_max_slope_deg >= 90:
        raise ValueError("invalid depth limits or soil slope")


def voxel_indices(xyz, size):
    """Keep one original point per voxel; do not move a measured tip by averaging."""
    return np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_index=True)[1]


def color_candidates(rgb):
    hsv = cv2.cvtColor(np.asarray(rgb, np.uint8).reshape(-1, 1, 3), cv2.COLOR_RGB2HSV)[:, 0]
    h, s, v = hsv.T
    vegetation = (h >= 20) & (h <= 100) & (s >= 25) & (v >= 20)
    # Brown substrate is a candidate, not a sufficient soil classification.
    soil = (h <= 30) & (s >= 35) & (v >= 20) & (v <= 245) & ~vegetation
    return vegetation, soil


def load_mask(root, scan, plant, view, kind, shape):
    if root is None:
        return None
    path = root / scan.name / plant / view / f"{kind}.png"
    if not path.exists():
        return None
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None or mask.shape != shape:
        raise ValueError(f"{path}: expected a readable mask of shape {shape}")
    return mask > 0


def connected_registration(result):
    """Only fuse one connected registration component into a measurement cloud."""
    components = result.get("components", [])
    if not components:
        raise ValueError("registration has no connected-component diagnostics; use --registration none")
    component = max(components, key=lambda item: (len(item["views"]), "top" in item["views"]))
    names = set(component["views"])
    if not names or not names.issubset(result.get("poses", {})):
        raise ValueError("registration component references missing poses")
    return {**result, "poses": {key: value for key, value in result["poses"].items() if key in names},
            "measurement_component": component,
            "excluded_disconnected_views": sorted(set(result["poses"]) - names)}


def registration_for(scan, plant, mode, run):
    if mode == "none":
        return None, None
    if run:
        matches = [scan / "reconstruction" / run / mode / f"{plant}_registration.json"]
        if not matches[0].is_file():
            raise ValueError(f"missing requested registration: {matches[0]}")
    else:
        matches = sorted((scan / "reconstruction").glob(f"**/{mode}/{plant}_registration.json"))
    if not matches:
        return None, None
    path = matches[-1]
    result = json.loads(path.read_text())
    if result.get("plant") != plant or Path(result.get("scan", "")).name != scan.name:
        raise ValueError(f"registration metadata does not match {scan.name}/{plant}")
    return connected_registration(result), path


def load_points(scan, plant, center, basis, reference, radius, settings, registration, mask_root,
                unregistered_views="single"):
    chunks, seeds, soils, ids, pixels, views, skipped = [], [], [], [], [], [], []
    available = sorted(view for view in (scan / plant).iterdir()
                       if view.is_dir() and (view / "meta.yaml").is_file())
    if registration is None and unregistered_views == "single" and available:
        available = [next((view for view in available if view.name == "top"), available[0])]
    for view in available:
        if not view.is_dir() or not (view / "meta.yaml").is_file():
            continue
        # Preserve the exact selected view set from a requested reconstruction.
        if registration is not None and view.name not in registration.get("poses", {}):
            continue
        try:
            meta = parse_meta_yaml(view / "meta.yaml")
            if (meta.get("pose_frame") != "base_link"
                    or meta.get("pose_child_frame") not in {EXPECTED_POSE_CHILD_FRAME, LEGACY_POSE_CHILD_FRAME}
                    or meta.get("camera_frame", meta.get("frame_id")) != EXPECTED_CLOUD_FRAME):
                raise ValueError("unsupported camera/pose frame")
            depth, bgr, (fx, fy, cx, cy) = read_rgbd(view)
            pose = pose_to_matrix(meta) if registration is None else validate_transform(
                registration["poses"][view.name]["refined_pose"])
            valid = np.isfinite(depth) & (depth >= settings.min_depth_m) & (depth <= settings.max_depth_m)
            rows, cols = np.nonzero(valid)
            z = depth[rows, cols]
            xyz = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))
            xyz = transform_points(xyz, reference @ pose) @ basis
            keep = ((np.linalg.norm(xyz[:, :2] - center[:2], axis=1) <= radius)
                    & (xyz[:, 2] >= center[2] - settings.roi_below_target_m)
                    & (xyz[:, 2] <= center[2] + settings.roi_above_target_m))
            rows, cols, xyz = rows[keep], cols[keep], xyz[keep]
            rgb = bgr[rows, cols, ::-1]
            vegetation, soil = color_candidates(rgb)
            plant_mask = load_mask(mask_root, scan, plant, view.name, "plant", depth.shape)
            soil_mask = load_mask(mask_root, scan, plant, view.name, "soil", depth.shape)
            if plant_mask is not None:
                vegetation = plant_mask[rows, cols]
            if soil_mask is not None:
                soil = soil_mask[rows, cols]
            soil &= ~vegetation
            chunks.append(np.column_stack((xyz, rgb)))
            seeds.append(vegetation)
            soils.append(soil)
            ids.append(np.full(len(xyz), len(views), dtype=np.int32))
            pixels.append(np.column_stack((rows, cols)))
            views.append({"name": view.name, "path": str(view), "shape": list(depth.shape),
                          "points": len(xyz), "plant_mask": plant_mask is not None,
                          "soil_mask": soil_mask is not None})
        except (ValueError, OSError, KeyError) as exc:
            skipped.append({"view": view.name, "reason": str(exc)})
    if not chunks or sum(map(len, chunks)) == 0:
        raise ValueError(f"no usable RGB-D points; skipped views: {skipped}")
    return (np.vstack(chunks), np.concatenate(seeds), np.concatenate(soils),
            np.concatenate(ids), np.vstack(pixels), views, skipped)


def fit_local_soil(points, center_xy, settings):
    """RANSAC local soil surface with coverage checks; returns global plane h=ax+by+c."""
    rejected = {"accepted": False, "reason": "too_few_soil_cells", "candidate_points": len(points)}
    if len(points) < settings.soil_min_cells:
        return rejected
    # Equal horizontal area weighting prevents one dense view dominating the fit.
    cell = np.floor(points[:, :2] / settings.soil_grid_m).astype(np.int64)
    _, inverse = np.unique(cell, axis=0, return_inverse=True)
    order = np.argsort(inverse)
    groups = np.split(order, np.flatnonzero(np.diff(inverse[order])) + 1)
    grid = np.asarray([np.median(points[g], axis=0) for g in groups])
    rejected["candidate_cells"] = len(grid)
    if len(grid) < settings.soil_min_cells:
        return rejected
    xy = grid[:, :2] - center_xy
    design = np.column_stack((xy, np.ones(len(xy))))
    best = np.zeros(len(grid), dtype=bool)
    rng = np.random.default_rng(7)
    max_slope = np.tan(np.deg2rad(settings.soil_max_slope_deg))
    for _ in range(300):
        sample = rng.choice(len(grid), 3, replace=False)
        if abs(np.linalg.det(design[sample])) < 1e-8:
            continue
        coefficients = np.linalg.solve(design[sample], grid[sample, 2])
        if np.linalg.norm(coefficients[:2]) > max_slope:
            continue
        inliers = np.abs(design @ coefficients - grid[:, 2]) <= settings.soil_residual_m
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 3:
        return {**rejected, "reason": "no_near_horizontal_surface"}
    for _ in range(3):
        coefficients = np.linalg.lstsq(design[best], grid[best, 2], rcond=None)[0]
        best = np.abs(design @ coefficients - grid[:, 2]) <= settings.soil_residual_m
        if best.sum() < 3:
            return {**rejected, "reason": "unstable_surface"}
    coefficients = np.linalg.lstsq(design[best], grid[best, 2], rcond=None)[0]
    inlier_xy = xy[best]
    sectors = len(np.unique(np.floor((np.arctan2(inlier_xy[:, 1], inlier_xy[:, 0]) + np.pi)
                                     / (2 * np.pi) * 8).astype(int) % 8))
    try:
        hull = ConvexHull(inlier_xy)
        enclosed = bool(np.all(hull.equations[:, -1] <= 1e-8))
        area = float(hull.volume)
    except QhullError:
        enclosed, area = False, 0.0
    residuals = grid[best, 2] - design[best] @ coefficients
    nearest = float(np.min(np.linalg.norm(inlier_xy, axis=1)))
    checks = {
        "too_few_soil_inliers": best.sum() >= settings.soil_min_cells,
        "low_soil_inlier_fraction": best.mean() >= settings.soil_min_inlier_fraction,
        "soil_only_on_one_side": sectors >= settings.soil_min_sectors and enclosed,
        "soil_too_far_from_stem": nearest <= settings.soil_max_nearest_m,
        "soil_area_too_small": area >= settings.soil_min_hull_area_m2,
        "soil_slope_too_large": np.linalg.norm(coefficients[:2]) <= max_slope,
    }
    global_coefficients = coefficients.copy()
    global_coefficients[2] -= coefficients[:2] @ center_xy
    reasons = [reason for reason, passed in checks.items() if not passed]
    return {"accepted": not reasons, "reason": ",".join(reasons) if reasons else "sufficient_local_soil",
            "candidate_points": len(points), "candidate_cells": len(grid), "inlier_cells": int(best.sum()),
            "inlier_fraction": float(best.mean()), "sectors": sectors, "stem_inside_hull": enclosed,
            "hull_area_m2": area, "nearest_soil_m": nearest,
            "roughness_m": float(np.sqrt(np.mean(residuals ** 2))),
            "plane": global_coefficients.tolist(), "soil_elevation_m": float(coefficients[2]),
            "stem_xy_m": np.asarray(center_xy).tolist()}


def historical_soil(history, timestamp, history_key, reference_id, up, center_xy, settings):
    """Average accepted prior days, never prior fallback results or future data."""
    if not history_key or not reference_id:
        return {"accepted": False, "reason": "history_identity_or_reference_not_configured"}
    candidates = []
    for observation in history:
        if (observation.get("history_key") != history_key or observation.get("reference_id") != reference_id
                or not observation.get("accepted") or observation.get("source") != "current_scan"
                or observation["timestamp"] >= timestamp
                or np.asarray(observation.get("up", [])).shape != (3,)
                or not np.allclose(observation["up"], up)):
            continue
        if np.linalg.norm(np.asarray(observation["stem_xy_m"]) - center_xy) > settings.history_max_stem_shift_m:
            continue
        candidates.append(observation)
    if not candidates:
        return {"accepted": False, "reason": "no_comparable_prior_soil"}
    # One vote per date, irrespective of point count or repeated imaging that day.
    days = {}
    for observation in candidates:
        days.setdefault(observation["timestamp"][:10], []).append(observation)
    dates = sorted(days)
    planes = np.asarray([np.median([o["plane"] for o in days[date]], axis=0) for date in dates])
    heights = planes @ np.r_[center_xy, 1.0]
    median = np.median(heights)
    mad = 1.4826 * np.median(np.abs(heights - median))
    keep = np.abs(heights - median) <= max(3 * mad, 0.003)
    spread = float(np.ptp(heights[keep]))
    if spread > settings.history_max_spread_m:
        return {"accepted": False, "reason": "historical_soil_levels_disagree", "spread_m": spread}
    plane = np.mean(planes[keep], axis=0)
    used_dates = [date for date, selected in zip(dates, keep) if selected]
    return {"accepted": True, "reason": "accepted_prior_daily_mean", "plane": plane.tolist(),
            "soil_elevation_m": float(plane @ np.r_[center_xy, 1.0]),
            "days": len(used_dates), "dates": used_dates, "spread_m": spread,
            "between_day_sd_m": float(np.std(heights[keep], ddof=1)) if keep.sum() > 1 else None,
            "observations": [o["observation_id"] for date in used_dates for o in days[date]],
            "rejected_dates": [date for date, selected in zip(dates, keep) if not selected]}


def supported_mask(xyz, settings):
    if len(xyz) <= settings.support_min_neighbors:
        return np.zeros(len(xyz), dtype=bool)
    distances, _ = cKDTree(xyz).query(xyz, k=settings.support_min_neighbors + 1)
    return distances[:, -1] <= settings.support_radius_m


def select_plant(points, seeds, plane, center, settings):
    """Select supported components with plant seeds; include connected organs of any color."""
    xyz = points[:, :3]
    elevation_above_surface = xyz[:, 2] - np.column_stack((xyz[:, :2], np.ones(len(xyz)))) @ plane
    candidates = elevation_above_surface > settings.plant_above_soil_m
    indices = np.flatnonzero(candidates)
    indices = indices[voxel_indices(xyz[indices], settings.voxel_m)]
    supported = supported_mask(xyz[indices], settings)
    indices = indices[supported]
    result = {"above_soil_points": int(candidates.sum()), "supported_voxels": len(indices)}
    if not len(indices):
        return indices, {**result, "reason": "no_supported_plant_points"}
    distances, neighbors = cKDTree(xyz[indices]).query(
        xyz[indices], k=min(16, len(indices)), distance_upper_bound=settings.component_radius_m)
    if neighbors.ndim == 1:
        neighbors, distances = neighbors[:, None], distances[:, None]
    row = np.broadcast_to(np.arange(len(indices))[:, None], neighbors.shape)
    valid = np.isfinite(distances)
    graph = coo_matrix((np.ones(valid.sum()), (row[valid], neighbors[valid])), shape=(len(indices), len(indices)))
    count, labels = connected_components(graph, directed=False)
    seed_counts = np.bincount(labels, weights=seeds[indices], minlength=count)
    # All sufficiently seeded components in the isolated plant ROI are retained.
    # Thus missing stem depth does not automatically remove a disconnected leaf.
    selected = seed_counts[labels] >= settings.component_min_seed_voxels
    return indices[selected], {**result, "components": count,
                               "selected_components": int((seed_counts >= settings.component_min_seed_voxels).sum()),
                               "plant_voxels": int(selected.sum()),
                               "reason": "supported_seeded_components" if selected.any() else "no_plant_seeded_components"}


def validate_history(history):
    if not isinstance(history, list):
        raise ValueError("history observations must be a list")
    seen = set()
    for observation in history:
        identifier = observation["observation_id"]
        if identifier in seen:
            raise ValueError(f"duplicate history observation: {identifier}")
        seen.add(identifier)
        datetime.fromisoformat(observation["timestamp"])
        for name, shape in [("plane", (3,)), ("stem_xy_m", (2,)), ("up", (3,))]:
            value = np.asarray(observation[name], dtype=float)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"invalid historical {name}: {identifier}")
        if not np.isclose(np.linalg.norm(observation["up"]), 1):
            raise ValueError(f"historical up must be normalized: {identifier}")


def write_validation_plot(directory, results):
    matched = [r for r in results if r.get("error_mm") is not None]
    if not matched:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    manual = np.asarray([r["manual_height_mm"] for r in matched])
    measured = np.asarray([r["height_mm"] for r in matched])
    fig, axis = plt.subplots(figsize=(7, 6))
    for location in sorted({r["location"] for r in matched}):
        mask = np.array([r["location"] == location for r in matched])
        axis.scatter(manual[mask], measured[mask], label=location)
    for x, y, r in zip(manual, measured, matched):
        axis.annotate(f"P{r['plant_id']}", (x, y), xytext=(5, 4), textcoords="offset points", fontsize=8)
    bounds = [min(manual.min(), measured.min()) - 3, max(manual.max(), measured.max()) + 3]
    axis.plot(bounds, bounds, "--", color="gray", label="Equal height")
    axis.set(xlim=bounds, ylim=bounds, xlabel="Manual height (mm)", ylabel="Estimated height (mm)",
             title=f"Initial comparison: n={len(matched)}, MAE={np.abs(measured-manual).mean():.2f} mm")
    axis.set_aspect("equal")
    axis.legend()
    fig.tight_layout()
    fig.savefig(directory / "manual_comparison.png", dpi=160)
    plt.close(fig)


def write_artifacts(directory, points, seed, soil_candidates, plant_indices, views, view_ids,
                    pixels, center, soil, top_index, settings):
    # All saved PLYs are in the measurement frame: Z is physical up.
    display = points.copy()
    display[:, 3:] *= 0.35
    display[soil_candidates, 3:] = [185, 125, 65]
    display[plant_indices, 3:] = [40, 220, 80]
    if top_index is not None:
        tip_distance = np.linalg.norm(points[:, :3] - points[top_index, :3], axis=1)
        display[tip_distance < settings.support_radius_m, 3:] = [255, 30, 230]
    keep = voxel_indices(points[:, :3], settings.voxel_m)
    save_binary_ply(directory / "classified.ply", display[keep])
    if len(plant_indices):
        save_binary_ply(directory / "plant_cleaned.ply", points[plant_indices])
    if soil_candidates.any():
        soil_points = points[soil_candidates]
        save_binary_ply(directory / "soil_candidates.ply", soil_points[voxel_indices(soil_points[:, :3], settings.voxel_m)])
    if top_index is not None and soil is not None:
        soil_h = soil["soil_elevation_m"]
        top = points[top_index, :3]
        line = np.column_stack((np.full(100, top[0]), np.full(100, top[1]), np.linspace(soil_h, top[2], 100)))
        save_binary_ply(directory / "height_line.ply", np.column_stack((line, np.tile([255, 0, 255], (100, 1)))))
    plant_mask = np.zeros(len(points), dtype=bool)
    plant_mask[plant_indices] = True
    for view_id, view in enumerate(views):
        bgr = cv2.imread(str(Path(view["path"]) / "color.png"))
        subset = view_ids == view_id
        for mask, color in [(soil_candidates, (65, 125, 185)), (plant_mask, (40, 220, 40))]:
            rc = pixels[subset & mask]
            bgr[rc[:, 0], rc[:, 1]] = color
        if top_index is not None and view_ids[top_index] == view_id:
            r, c = pixels[top_index]
            cv2.circle(bgr, (int(c), int(r)), 10, (255, 0, 255), 2)
        cv2.imwrite(str(directory / f"{view['name']}_overlay.png"), bgr)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    sample = np.arange(0, len(points), max(1, len(points) // 15000))
    for axis, dim in zip(axes, [0, 1]):
        axis.scatter((points[sample, dim] - center[dim]) * 1000, points[sample, 2] * 1000,
                     c=display[sample, 3:] / 255, s=0.5)
        if soil is not None:
            axis.axhline(soil["soil_elevation_m"] * 1000, color="saddlebrown", label="Soil at stem")
        if top_index is not None:
            top = points[top_index]
            axis.scatter([(top[dim] - center[dim]) * 1000], [top[2] * 1000], c="magenta", s=30)
        axis.set_xlabel(f"{'XY'[dim]} relative to stem (mm)")
        axis.set_ylabel("Elevation along physical up (mm)")
        axis.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(directory / "height_preview.png", dpi=150)
    plt.close(fig)


def measure(scan, plant_record, config, settings, args, history):
    plant = f"plant_{int(plant_record['plant_id']):02d}"
    history[:] = [o for o in history if o["observation_id"] != f"{scan.name}/{plant}"]
    timestamp = scan_timestamp(scan).isoformat()
    directory = args.output_dir / scan.name / plant
    directory.mkdir(parents=True, exist_ok=True)
    target = plant_record.get("target_base", {})
    if target.get("frame_id") != "base_link":
        raise ValueError("scan needs target_base in base_link")
    reference = validate_transform(config.get("base_to_reference", np.eye(4)))
    basis = vertical_basis(args.up)
    center_base = np.asarray([target[f"{dim}_m"] for dim in "xyz"], dtype=float)
    if "stem_xy_base_m" in config:
        stem = np.asarray(config["stem_xy_base_m"], dtype=float)
        if stem.shape != (2,) or not np.isfinite(stem).all():
            raise ValueError("stem_xy_base_m must contain two finite coordinates")
        center_base[:2] = stem
    center = transform_points(center_base[None], reference)[0] @ basis
    radius = float(config.get("roi_radius_m", max(settings.roi_min_radius_m,
                                                  float(plant_record["radius_mm"]) / 1000 + settings.roi_margin_m)))
    if not np.isfinite(radius) or radius <= settings.soil_outer_radius_m:
        raise ValueError("roi_radius_m must be finite and exceed soil_outer_radius_m")
    registration, registration_path = registration_for(scan, plant, args.registration, args.registration_run)
    points, seed, soil_color, view_ids, pixels, views, skipped = load_points(
        scan, plant, center, basis, reference, radius, settings, registration, args.mask_root,
        args.unregistered_views)
    distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    # Remove the immediate neighborhood of vegetation from soil candidates.
    soil_candidates = soil_color & (distance >= settings.soil_inner_radius_m) & (distance <= settings.soil_outer_radius_m)
    if seed.any() and soil_candidates.any():
        seed_distance = cKDTree(points[seed, :3]).query(points[soil_candidates, :3], k=1)[0]
        soil_candidates[np.flatnonzero(soil_candidates)[seed_distance < 0.003]] = False
    current = fit_local_soil(points[soil_candidates, :3], center[:2], settings)
    prior = historical_soil(history, timestamp, config.get("history_key"), config.get("reference_id"),
                            basis[:, 2].tolist(), center[:2], settings)
    soil, soil_source = (current, "current_scan") if current["accepted"] else (
        (prior, "historical_daily_mean") if prior["accepted"] else (None, "unavailable"))
    warnings = []
    if "stem_xy_base_m" not in config:
        warnings.append("stem_position_uses_scan_target_proxy")
    if not all(view["plant_mask"] for view in views):
        warnings.append("automatic_plant_seeds_review_non_green_or_disconnected_organs")
    if registration is None:
        warnings.append("saved_camera_poses_only_no_registration")
    elif registration.get("excluded_disconnected_views"):
        warnings.append("excluded_disconnected_registration_views")
    if len(views) == 1:
        warnings.append("single_view_visible_height_verify_top_not_occluded")
    if registration is None and len(views) > 1:
        warnings.append("unregistered_view_fusion_requires_review_not_eligible_for_soil_history")
    if skipped:
        warnings.append("some_views_skipped")
    if current["accepted"] and prior["accepted"]:
        if abs(current["soil_elevation_m"] - prior["soil_elevation_m"]) > settings.history_max_spread_m:
            warnings.append("current_soil_disagrees_with_history_check_drift_or_soil_change")
    result = {"scan": scan.name, "plant_id": int(plant_record["plant_id"]), "timestamp": timestamp,
              "location": parse_meta_yaml(scan / "metadata.yaml").get("location"),
              "status": "unmeasurable", "height_mm": None, "height_p99_mm": None, "height_p995_mm": None,
              "soil_source": soil_source, "current_soil": current, "historical_soil": prior,
              "history_key": config.get("history_key"), "reference_id": config.get("reference_id"),
              "measurement_basis_columns": basis.tolist(), "base_to_reference": reference.tolist(),
              "stem_target_measurement_m": center.tolist(), "roi_radius_m": radius,
              "registration": str(registration_path) if registration_path else None,
              "excluded_disconnected_views": registration.get("excluded_disconnected_views", []) if registration else [],
              "views": views, "skipped_views": skipped, "warnings": warnings}
    plant_indices, top_index = np.array([], dtype=int), None
    if soil is not None:
        plant_indices, plant_info = select_plant(points, seed, np.asarray(soil["plane"]), center, settings)
        result["plant_selection"] = plant_info
        if len(plant_indices):
            elevations = points[plant_indices, 2]
            top_index = int(plant_indices[np.argmax(elevations)])
            top = points[top_index, :3]
            soil_h = soil["soil_elevation_m"]
            per_view = {}
            # Per-view maxima retain provenance before global voxel deduplication.
            for view_id, view in enumerate(views):
                own = (view_ids == view_id) & seed
                own_indices = np.flatnonzero(own)
                own_indices = own_indices[voxel_indices(points[own_indices, :3], settings.voxel_m)]
                own_indices = own_indices[supported_mask(points[own_indices, :3], settings)]
                if len(own_indices):
                    per_view[view["name"]] = float((points[own_indices, 2].max() - soil_h) * 1000)
            view_spread = float(np.ptp(list(per_view.values()))) if len(per_view) > 1 else None
            if view_spread is not None and view_spread > settings.top_agreement_m * 1000:
                warnings.append("per_view_tops_disagree_check_occlusion_motion_and_registration")
            if (radius - distance[top_index] < 0.005
                    or center[2] + settings.roi_above_target_m - top[2] < 0.005):
                warnings.append("top_near_crop_boundary_enlarge_roi")
            result.update(status="estimated", height_mm=float((top[2] - soil_h) * 1000),
                          height_p99_mm=float((np.percentile(elevations, 99) - soil_h) * 1000),
                          height_p995_mm=float((np.percentile(elevations, 99.5) - soil_h) * 1000),
                          soil_elevation_m=soil_h, top_point_measurement_m=top.tolist(),
                          top_view=views[view_ids[top_index]]["name"],
                          top_pixel_rc=pixels[top_index].tolist(), per_view_seed_height_mm=per_view,
                          per_view_seed_height_spread_mm=view_spread)
            if result["height_mm"] <= 0:
                result.update(status="unmeasurable", height_mm=None)
                warnings.append("nonpositive_height_check_soil_and_vertical_direction")
    if current["accepted"]:
        observation = {**current, "source": "current_scan", "timestamp": timestamp,
                       "observation_id": f"{scan.name}/{plant}", "history_key": config.get("history_key"),
                       "reference_id": config.get("reference_id"), "up": basis[:, 2].tolist(),
                       "settings": asdict(settings), "registration": result["registration"]}
        if registration is None and len(views) > 1:
            observation.update(accepted=False, reason="unregistered_multiview_fusion")
        # Reruns replace the same observation rather than increasing its weight.
        history[:] = [o for o in history if o["observation_id"] != observation["observation_id"]]
        history.append(observation)
    if not args.no_artifacts:
        write_artifacts(directory, points, seed, soil_candidates, plant_indices, views, view_ids,
                        pixels, center, soil, top_index, settings)
    write_json(directory / "measurement.json", result)
    return result


def read_manual(path, unit):
    if path is None:
        return {}
    result = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            resolved_unit = row.get("unit") or unit
            if resolved_unit not in {"mm", "cm"}:
                raise ValueError("manual CSV needs unit=mm/cm or --manual-unit mm/cm")
            date = datetime.strptime(row["date"], "%Y-%m-%d").date().isoformat()
            key = (date, row["location"], int(row["plant_id"]))
            height = float(row["height"]) * (10 if resolved_unit == "cm" else 1)
            if key in result or not np.isfinite(height) or height <= 0:
                raise ValueError(f"duplicate or invalid manual measurement: {key}")
            result[key] = height
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="One scan or a directory of timestamped scans")
    parser.add_argument("--date", help="Only process YYYY-MM-DD; prior observations can come from --history")
    parser.add_argument("--start-date", help="Experiment start YYYY-MM-DD; exclude earlier scans AND soil history (overrides config)")
    parser.add_argument("--locations", nargs="+", help="For example b1_r16 b1_r17")
    parser.add_argument("--plant", type=int)
    parser.add_argument("--output-dir", type=Path, required=True, help="New output directory; existing directories are refused")
    parser.add_argument("--config", type=Path, help="JSON settings, stem positions, history identities and reference transforms")
    parser.add_argument("--history", type=Path, help="Previously saved soil_history.json")
    parser.add_argument("--mask-root", type=Path, help="Optional per-view plant.png / soil.png masks; see documentation")
    parser.add_argument("--registration", choices=["none", "robust_icp", "colored_icp", "rgbd_features", "fpfh"], default="robust_icp")
    parser.add_argument("--registration-run", help="Exact reconstruction run name; default uses latest name per plant")
    parser.add_argument("--unregistered-views", choices=["single", "all"], default="single",
                        help="Without registration use top (or first available) view; all enables flagged diagnostic fusion")
    parser.add_argument("--up", nargs=3, type=float, default=[0, 0, -1], help="Physical up in the reference frame (default 0 0 -1 for this robot)")
    parser.add_argument("--manual-csv", type=Path)
    parser.add_argument("--manual-unit", choices=["mm", "cm"], help="Unit for CSV rows with blank units")
    parser.add_argument("--no-artifacts", action="store_true", help="Skip PLY, overlay and preview generation")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text()) if args.config else {}
        settings = Settings(**config.get("settings", {}))
        validate_settings(settings)
        vertical_basis(args.up)
        if args.date:
            datetime.strptime(args.date, "%Y-%m-%d")
        start_value = args.start_date or config.get("experiment_start_date")
        start_date = datetime.strptime(start_value, "%Y-%m-%d").date() if start_value else None
        if start_date and args.date and datetime.strptime(args.date, "%Y-%m-%d").date() < start_date:
            raise ValueError("--date is before the experiment start date")
        manual = read_manual(args.manual_csv, args.manual_unit)
        history = json.loads(args.history.read_text())["observations"] if args.history else []
        validate_history(history)
        if start_date:
            history = [o for o in history if datetime.fromisoformat(o["timestamp"]).date() >= start_date]
        if not args.input_dir.is_dir():
            raise ValueError(f"input directory does not exist: {args.input_dir}")
        candidates = [args.input_dir] if (args.input_dir / "metadata.yaml").exists() else list(args.input_dir.iterdir())
        scans = []
        for scan in candidates:
            timestamp = scan_timestamp(scan) if scan.is_dir() else None
            if timestamp is None or not (scan / "metadata.yaml").is_file():
                continue
            if start_date and timestamp.date() < start_date:
                continue
            if args.date and timestamp.date().isoformat() != args.date:
                continue
            if args.locations and parse_meta_yaml(scan / "metadata.yaml").get("location") not in args.locations:
                continue
            scans.append(scan)
        if not scans:
            raise ValueError("no scans match the requested date/locations")
        args.output_dir.mkdir(parents=True, exist_ok=False)
    except (ValueError, OSError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    write_json(args.output_dir / "run_config.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "settings": asdict(settings), "config": config,
        "experiment_start_date": start_date.isoformat() if start_date else None,
        "arguments": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "measurement": "vertical distance from local soil at stem to highest supported plant organ"})
    results = []
    for scan in sorted(scans, key=lambda p: (scan_timestamp(p), p.name)):
        meta = parse_meta_yaml(scan / "metadata.yaml")
        location_config = config.get("locations", {}).get(meta.get("location"), {})
        scan_config = config.get("scans", {}).get(scan.name, {})
        for plant in meta.get("plants", []):
            if args.plant is not None and plant["plant_id"] != args.plant:
                continue
            plant_config = {**config.get("defaults", {}),
                            **{k: v for k, v in location_config.items() if k != "plants"},
                            **location_config.get("plants", {}).get(str(plant["plant_id"]), {}),
                            **{k: v for k, v in scan_config.items() if k != "plants"},
                            **scan_config.get("plants", {}).get(str(plant["plant_id"]), {})}
            try:
                result = measure(scan, plant, plant_config, settings, args, history)
            except (ValueError, OSError, KeyError) as exc:
                result = {"scan": scan.name, "plant_id": plant["plant_id"], "timestamp": scan_timestamp(scan).isoformat(),
                          "location": meta.get("location"), "status": "failed", "error": str(exc), "height_mm": None}
                write_json(args.output_dir / scan.name / f"plant_{plant['plant_id']:02d}" / "measurement.json", result)
            key = (result["timestamp"][:10], result["location"], result["plant_id"])
            result["manual_height_mm"] = manual.get(key)
            result["error_mm"] = (result["height_mm"] - manual[key]
                                  if key in manual and result.get("height_mm") is not None else None)
            write_json(args.output_dir / scan.name / f"plant_{plant['plant_id']:02d}" / "measurement.json", result)
            results.append(result)
            print(f"{scan.name}/plant_{plant['plant_id']:02d}: {result['status']}, "
                  f"height_mm={result.get('height_mm')}, soil={result.get('soil_source')}", flush=True)
    columns = ["scan", "location", "plant_id", "timestamp", "status", "height_mm", "height_p99_mm", "height_p995_mm",
               "soil_source", "soil_elevation_m", "manual_height_mm", "error_mm", "warnings", "error"]
    with (args.output_dir / "heights.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({**r, "warnings": ";".join(r.get("warnings", []))} for r in results)
    errors = np.asarray([r["error_mm"] for r in results if r["error_mm"] is not None])
    matched = { (r["timestamp"][:10], r["location"], r["plant_id"]) for r in results if r["error_mm"] is not None }
    validation = {"matched_estimates": len(errors), "manual_records": len(manual),
                  "unmatched_manual": [list(key) for key in manual if key not in matched],
                  "mae_mm": float(np.abs(errors).mean()) if len(errors) else None,
                  "rmse_mm": float(np.sqrt((errors ** 2).mean())) if len(errors) else None,
                  "bias_mm": float(errors.mean()) if len(errors) else None}
    write_json(args.output_dir / "results.json", {"measurements": results, "validation": validation})
    write_json(args.output_dir / "soil_history.json", {"version": 1, "observations": history})
    if not args.no_artifacts:
        write_validation_plot(args.output_dir, results)
    print(f"Saved {len(results)} measurements to {args.output_dir}")
    if not results or any(r["status"] == "failed" for r in results):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
