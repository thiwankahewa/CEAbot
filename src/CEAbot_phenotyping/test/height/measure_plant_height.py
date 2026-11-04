#!/usr/bin/env python3

import argparse
import csv
from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
from functools import lru_cache
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, QhullError, cKDTree

# Resolve sibling scripts independently of the working directory or ROS install.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "reconstruction"))

from plant_view_registration import read_rgbd
from reconstruct_plant_views import (EXPECTED_POSE_FRAME,EXPECTED_CLOUD_FRAME, EXPECTED_POSE_CHILD_FRAME, parse_meta_yaml, save_binary_ply, scan_timestamp, transform_points,)


# Fixed inverted arm mounting: (x, y, z) -> (x, -y, -z), with physical up along -Z.
MEASUREMENT_BASIS = np.array([ [1.0, 0.0, 0.0],[0.0, -1.0, 0.0],[0.0, 0.0, -1.0],])

# ===========================================================================================================

@dataclass
class SegmentationSettings:
    weights: str = "/home/thiwa/plant_dataset/soil_plant_indivdual/training_runs/v2/weights/best.pt"
    confidence: float = 0.25
    imgsz: int = 640


@lru_cache(maxsize=2)
def segmentation_model(weights):
    """Load local weights once for all views/plants in the run."""

    path = Path(weights).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"segmentation weights do not exist: {path}")
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise ValueError("YOLO segmentation requires ultralytics; install it in this Python environment") from exc
    model = YOLO(str(path))
    if model.task != "segment" or model.names != {0: "soil", 1: "plant"}:
        raise ValueError("segmentation model must have class 0=soil and class 1=plant")
    return model


def prepare_segmentation():
    settings = SegmentationSettings()
    if isinstance(settings.imgsz, bool) or not isinstance(settings.imgsz, int) or settings.imgsz <= 0:
        raise ValueError("segmentation.imgsz must be a positive integer")
    settings.weights = str(Path(settings.weights).expanduser().resolve())
    segmentation_model(settings.weights)
    return settings

# ===========================================================================================================

@dataclass
class Settings:
    ''' settings are in a class to easy to organize'''
    
    soil_radius_m: float = 0.100
    soil_grid_m: float = 0.002
    soil_residual_m: float = 0.004
    soil_min_cells: int = 60
    soil_min_inlier_fraction: float = 0.55  # value between 0 and 1, inclusive
    soil_min_sectors: int = 5  # Of eight angular sectors around the stem.
    soil_max_nearest_m: float = 0.025
    soil_max_slope_deg: float = 25.0
    soil_min_hull_area_m2: float = 0.0005
    roi_margin_m: float = 0.050 # plant radius + margin for the ROI, to avoid losing points at the edge of the plant
    roi_below_target_m: float = 0.250
    roi_above_target_m: float = 0.040
    min_depth_m: float = 0.050
    max_depth_m: float = 0.500
    voxel_m: float = 0.001
    support_radius_m: float = 0.003
    support_min_neighbors: int = 3  # Excludes the query point itself.
    component_radius_m: float = 0.005
    component_min_seed_voxels: int = 8
    plant_above_soil_m: float = 0.006
    history_max_spread_m: float = 0.010
    top_agreement_m: float = 0.008
    canopy_grid_m: float = 0.002  # Occupied horizontal projection cells for canopy area.

def validate_settings(settings):
    allow_zero = {"roi_margin_m", "roi_below_target_m", "roi_above_target_m", "min_depth_m",
                  "soil_min_inlier_fraction", "soil_max_slope_deg", "soil_min_hull_area_m2",
                  "plant_above_soil_m", "history_max_spread_m", "top_agreement_m"}
    for field in fields(settings):
        value = getattr(settings, field.name)
        if not np.isfinite(value) or value < 0 or (value == 0 and field.name not in allow_zero):
            bound = "nonnegative" if field.name in allow_zero else "positive"
            raise ValueError(f"{field.name} must be finite and {bound}")
    for name in ("soil_min_cells", "soil_min_sectors", "support_min_neighbors", "component_min_seed_voxels"):
        value = getattr(settings, name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an integer")
    if settings.soil_min_cells < 3 or settings.soil_min_sectors > 8 or settings.soil_min_inlier_fraction > 1:
        raise ValueError("invalid soil cell count, sector count or inlier fraction")
    if settings.min_depth_m >= settings.max_depth_m or settings.soil_max_slope_deg >= 90:
        raise ValueError("invalid depth limits or soil slope")

# ===========================================================================================================

def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def voxel_indices(xyz, size):
    """Keep one original point per voxel; do not move a measured tip by averaging."""
    return np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_index=True)[1]

# ===========================================================================================================

def connected_registration(result):
    """Only fuse one connected registration component into a measurement cloud."""
    components = result.get("components", [])
    if not components:
        raise ValueError("registration has no connected-component diagnostics; regenerate the robust ICP reconstruction")
    component = max(components, key=lambda item: (len(item["views"]), "top" in item["views"]))
    names = set(component["views"])
    return {**result, "poses": {key: value for key, value in result["poses"].items() if key in names},
            "measurement_component": component,
            "excluded_disconnected_views": sorted(set(result["poses"]) - names)}


def registration_for(scan, plant):
    ''' Return the most recent robust ICP registration and path for that json for a scan/plant. '''

    matches = sorted((scan / "reconstruction").glob(f"**/robust_icp/{plant}_registration.json"))
    if not matches:
        raise ValueError(f"No saved robust ICP registration for {scan.name}/{plant}; run reconstruction first")
    path = matches[-1]
    result = json.loads(path.read_text())
    if result.get("plant") != plant or Path(result.get("scan", "")).name != scan.name:
        raise ValueError(f"registration metadata does not match {scan.name}/{plant}")
    return connected_registration(result), path

# ===========================================================================================================

def save_segmented_images(directory, view_name, bgr, plant_mask, soil_mask):
    """Save one full-image overlay per view; soil wins overlap."""
    
    directory.mkdir(parents=True, exist_ok=True)
    plant_mask = plant_mask & ~soil_mask
    overlay = bgr.copy()
    for mask, color in [(soil_mask, (65, 125, 185)), (plant_mask, (40, 220, 40))]:
        overlay[mask] = np.rint(0.5 * bgr[mask] + 0.5 * np.asarray(color)).astype(np.uint8)
    path = directory / f"{view_name}_seg.png"
    cv2.imwrite(str(path), overlay)

def segment_view(bgr, segmentation=None):
    """Return HxW boolean plant/soil masks for an original BGR image."""
    
    result = segmentation_model(segmentation.weights).predict(
        source=bgr, conf=segmentation.confidence, imgsz=segmentation.imgsz,
        retina_masks=True, classes=[0, 1], agnostic_nms=False,
        save=False, verbose=False,)[0]
    plant = np.zeros(bgr.shape[:2], dtype=bool)
    soil = np.zeros_like(plant)
    if result.masks is not None:
        masks = result.masks.data.cpu().numpy() > 0.5
        classes = result.boxes.cls.cpu().numpy().astype(int)
        if masks.shape != (len(classes), *bgr.shape[:2]):
            raise ValueError("segmentation masks must match the original RGB-D image size")
        soil = masks[classes == 0].any(axis=0)
        plant = masks[classes == 1].any(axis=0)
    plant &= ~soil
    return plant, soil

def load_points(scan, plant, center, basis, radius, settings, registration, segmentation=None, segmentation_output_dir=None):
    chunks, seeds, soils, ids, pixels, views, skipped = [], [], [], [], [], [], []
    available = sorted(view for view in (scan / plant).iterdir() if view.is_dir() and (view / "meta.yaml").is_file())
    for view in available:
        # Preserve the exact selected view set from a requested reconstruction.
        if view.name not in registration["poses"]:
            continue
        try:
            meta = parse_meta_yaml(view / "meta.yaml")
            if (meta.get("pose_frame") != EXPECTED_POSE_FRAME or meta.get("pose_child_frame") not in {EXPECTED_POSE_CHILD_FRAME} or meta.get("camera_frame") != EXPECTED_CLOUD_FRAME):
                raise ValueError("unsupported camera/pose frame")
            depth, bgr, (fx, fy, cx, cy) = read_rgbd(view)
            pose = np.asarray(registration["poses"][view.name]["refined_pose"], dtype=float)
            valid = np.isfinite(depth) & (depth >= settings.min_depth_m) & (depth <= settings.max_depth_m)
            rows, cols = np.nonzero(valid)
            z = depth[rows, cols]
            xyz = np.column_stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z))
            xyz = transform_points(xyz, pose) @ basis
            keep = ((np.linalg.norm(xyz[:, :2] - center[:2], axis=1) <= radius)
                    & (xyz[:, 2] >= center[2] - settings.roi_below_target_m)
                    & (xyz[:, 2] <= center[2] + settings.roi_above_target_m))
            rows, cols, xyz = rows[keep], cols[keep], xyz[keep]
            rgb = bgr[rows, cols, ::-1]

            plant_mask, soil_mask = segment_view(bgr, segmentation)
            if segmentation_output_dir is not None:
                save_segmented_images(segmentation_output_dir, view.name, bgr, plant_mask, soil_mask)
            vegetation = plant_mask[rows, cols]
            soil = soil_mask[rows, cols]
            vegetation &= ~soil
            chunks.append(np.column_stack((xyz, rgb)))
            seeds.append(vegetation)
            soils.append(soil)
            ids.append(np.full(len(xyz), len(views), dtype=np.int32))
            pixels.append(np.column_stack((rows, cols)))
            views.append({"name": view.name, "path": str(view)})
        except (ValueError, OSError, KeyError) as exc:
            skipped.append({"view": view.name, "reason": str(exc)})
    if not chunks or sum(map(len, chunks)) == 0:
        raise ValueError(f"no usable RGB-D points; skipped views: {skipped}")
    return (np.vstack(chunks), np.concatenate(seeds), np.concatenate(soils), np.concatenate(ids), np.vstack(pixels), views, skipped)

# ===========================================================================================================

def exclude_nearby_plant_per_view(points, seed, soil_candidates, view_ids):
    """Exclude soil within 3 mm of plant seeds from the same camera view only."""
    keep = soil_candidates.copy()
    for view_id in np.unique(view_ids):
        own_seed = seed & (view_ids == view_id)
        own_soil_indices = np.flatnonzero(keep & (view_ids == view_id))
        if not own_seed.any() or not len(own_soil_indices):
            continue
        distances = cKDTree(points[own_seed, :3]).query(points[own_soil_indices, :3], k=1)[0]
        keep[own_soil_indices[distances < 0.003]] = False
    return keep


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


def historical_soil(history, timestamp, history_key, up, center_xy, settings):
    """Reuse prior stem soil elevations, assuming unchanged pot elevation/tilt."""
    if not history_key:
        return {"accepted": False, "reason": "history_identity_not_available"}
    candidates = []
    for observation in history:
        if (observation.get("history_key") != history_key or observation.get("frame_id") != "base_link"
                or not observation.get("accepted") or observation.get("source") != "current_scan"
                or observation["timestamp"] >= timestamp
                or np.asarray(observation.get("up", [])).shape != (3,)
                or not np.allclose(observation["up"], up)):
            continue
        candidates.append(observation)
    if not candidates:
        return {"accepted": False, "reason": "no_comparable_prior_soil"}
    # One vote per date, irrespective of point count or repeated imaging that day.
    days = {}
    for observation in candidates:
        days.setdefault(observation["timestamp"][:10], []).append(observation)
    dates = sorted(days)
    daily_planes = []
    for date in dates:
        moved_planes = []
        for observation in days[date]:
            plane = np.asarray(observation["plane"], dtype=float).copy()
            # Translate the old local surface horizontally with the stem proxy.
            # This preserves its stem soil elevation; it is not a vertical or
            # rotational correction and does not estimate actual pot motion.
            plane[2] += plane[:2] @ (np.asarray(observation["stem_xy_m"]) - center_xy)
            moved_planes.append(plane)
        daily_plane = np.median(moved_planes, axis=0)
        daily_height = np.median(np.asarray(moved_planes) @ np.r_[center_xy, 1.0])
        daily_plane[2] = daily_height - daily_plane[:2] @ center_xy
        daily_planes.append(daily_plane)
    planes = np.asarray(daily_planes)
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
            "position_assumption": "unchanged_pot_elevation_and_tilt",
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


def select_plant(points, seeds, plane, center, settings, soil_mask=None):
    """Filter plant points; YOLO masks exclude unlabeled background before voxelization."""
    xyz = points[:, :3]
    elevation_above_surface = xyz[:, 2] - np.column_stack((xyz[:, :2], np.ones(len(xyz)))) @ plane
    candidates = elevation_above_surface > settings.plant_above_soil_m
    candidates &= seeds
    if soil_mask is not None:
        # Component expansion must not add soil-labeled points back to the plant.
        candidates &= ~soil_mask
    indices = np.flatnonzero(candidates)
    indices = indices[voxel_indices(xyz[indices], settings.voxel_m)]
    supported = supported_mask(xyz[indices], settings)
    indices = indices[supported]
    result = {"above_soil_points": int(candidates.sum()), "supported_voxels": len(indices),}
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



def measure_canopy(points, center, radius, settings):
    """Describe the observed cleaned plant cloud projected onto physical XY.

    Span is the convex-hull diameter. Perpendicular width is a projection extent,
    not a stem-crossing chord or the minimum Feret diameter. Grid area counts
    occupied cells once, including overlapping views, without filling gaps.
    """
    result = {"canopy_status": "unmeasurable", "canopy_max_span_mm": None,
              "canopy_perpendicular_width_mm": None, "canopy_projected_area_mm2": None,
              "canopy_hull_area_mm2": None, "canopy_grid_mm": settings.canopy_grid_m * 1000,
              "canopy_near_crop_boundary": False,
              "canopy_details": {"reason": "insufficient_plant_points"}}
    xyz = np.asarray(points)[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xy = np.unique(xyz[:, :2], axis=0)
    if len(xy) < 3:
        return result
    # Centering improves numerical conditioning without changing metric geometry.
    origin = xy.mean(axis=0)
    try:
        hull = ConvexHull(xy - origin)
    except QhullError:
        result["canopy_details"]["reason"] = "degenerate_horizontal_footprint"
        return result
    boundary = xy[hull.vertices]
    best_squared, endpoints = -1.0, None
    for point in boundary:
        squared = np.sum((boundary - point) ** 2, axis=1)
        other = int(np.argmax(squared))
        if squared[other] > best_squared:
            best_squared = float(squared[other])
            endpoints = np.array([point, boundary[other]])
    span = np.sqrt(best_squared)
    direction = (endpoints[1] - endpoints[0]) / span
    perpendicular = np.array([-direction[1], direction[0]])
    projection = (xy - origin) @ perpendicular
    low, high = float(projection.min()), float(projection.max())
    grid_origin = xy.min(axis=0)
    cells = np.unique(np.floor((xy - grid_origin) / settings.canopy_grid_m + 1e-9).astype(np.int64), axis=0)
    margins = {
        "horizontal_m": float(radius - np.linalg.norm(xyz[:, :2] - center[:2], axis=1).max()),
        "upper_m": float(center[2] + settings.roi_above_target_m - xyz[:, 2].max()),
        "lower_m": float(xyz[:, 2].min() - (center[2] - settings.roi_below_target_m)),
    }
    near_crop = any(value < 0.005 for value in margins.values())
    result.update(
        canopy_status="estimated", canopy_max_span_mm=float(span * 1000),
        canopy_perpendicular_width_mm=float((high - low) * 1000),
        canopy_projected_area_mm2=float(len(cells) * (settings.canopy_grid_m * 1000) ** 2),
        canopy_hull_area_mm2=float(hull.volume * 1e6),
        canopy_near_crop_boundary=near_crop,
        canopy_details={"reason": "observed_cleaned_plant_projection", "source": "plant_cleaned_points",
                        "projection": "physical_horizontal_xy", "area_method": "occupied_grid_cells_no_gap_filling",
                        "point_count": len(xyz), "occupied_cells": len(cells),
                        "grid_origin_xy_m": grid_origin.tolist(), "hull_xy_m": boundary.tolist(),
                        "max_span_endpoints_xy_m": endpoints.tolist(),
                        "max_span_direction_xy": direction.tolist(),
                        "perpendicular_direction_xy": perpendicular.tolist(),
                        "perpendicular_line_endpoints_xy_m": (origin + np.array([low, high])[:, None] * perpendicular).tolist(),
                        "crop_margins": margins})
    return result


def write_canopy_preview(directory, points, center, radius, canopy):
    """Plot measured spans and the hull on a metric horizontal projection."""
    if canopy["canopy_status"] != "estimated":
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, axis = plt.subplots(figsize=(7, 7))
    xy = (points[:, :2] - center[:2]) * 1000
    axis.scatter(xy[:, 0], xy[:, 1], s=2, c="forestgreen", label="Retained plant points")
    details = canopy["canopy_details"]
    hull = (np.asarray(details["hull_xy_m"]) - center[:2]) * 1000
    hull = np.vstack((hull, hull[0]))
    axis.plot(hull[:, 0], hull[:, 1], color="gray", linestyle="--", label="Convex hull")
    for key, color, label, value in [
        ("max_span_endpoints_xy_m", "darkorange", "Maximum span", canopy["canopy_max_span_mm"]),
        ("perpendicular_line_endpoints_xy_m", "royalblue", "Perpendicular width", canopy["canopy_perpendicular_width_mm"]),
    ]:
        line = (np.asarray(details[key]) - center[:2]) * 1000
        axis.plot(line[:, 0], line[:, 1], color=color, linewidth=2, marker="o", markersize=3,
                  label=f"{label}: {value:.1f} mm")
    axis.add_patch(Circle((0, 0), radius * 1000, fill=False, color="red", linestyle=":", label="Horizontal crop"))
    axis.plot(0, 0, "+", color="black", label="Stem proxy")
    title = f"Observed canopy | grid area: {canopy['canopy_projected_area_mm2']:.0f} mm²"
    if canopy["canopy_near_crop_boundary"]:
        title += "\nNear crop boundary: check coverage"
    axis.set(xlabel="X relative to stem (mm)", ylabel="Y relative to stem (mm)", title=title)
    axis.set_aspect("equal", adjustable="box")
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(directory / "canopy_preview.png", dpi=150)
    plt.close(fig)


def write_artifacts(directory, points, soil_candidates, plant_indices, center, soil, top_index, settings):
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

def plant_history_key(experiment, location, plant_id):
    """Identify the same plant across scans within an experiment."""
    if experiment is None:
        return None
    return f"{experiment}/{location}/plant_{int(plant_id):02d}"


def soil_observation_from_measurement(measurement):
    """Build an in-memory history entry from the directly measured soil."""
    current = measurement.get("current_soil")
    if not current or not current.get("accepted"):
        return None
    return {**current, "source": "current_scan",
            "timestamp": measurement["timestamp"],
            "observation_id": f"{measurement['scan']}/plant_{int(measurement['plant_id']):02d}",
            "history_key": measurement["history_key"],
            "frame_id": measurement["frame_id"],
            "up": [row[2] for row in measurement["measurement_basis_columns"]],
            "settings": measurement.get("settings", {}),
            "registration": measurement["registration"],
            "segmentation": measurement["segmentation"]}


def discover_soil_history(scan, metadata, experiment_start, processed_scans):
    """Load each earlier same-row scan's own plant observations from its latest run."""

    if experiment_start is None:
        return []
    location = metadata.get("location")
    current_time = scan_timestamp(scan)
    plants = {f"plant_{int(plant['plant_id']):02d}": f"{experiment_start}/{location}/plant_{int(plant['plant_id']):02d}"
              for plant in metadata.get("plants", [])}
    observations = {}
    for previous in sorted(scan.parent.iterdir()):  # loop through all scans in the same experiment parent folder
        if not previous.is_dir() or previous in processed_scans or not (previous / "metadata.yaml").is_file():
            continue
        source = previous
        try:
            timestamp = scan_timestamp(previous)
            if timestamp is None or timestamp >= current_time or timestamp.date() < experiment_start:
                continue
            if parse_meta_yaml(previous / "metadata.yaml").get("location") != location:
                continue
            for plant, expected_key in plants.items():
                files = list((previous / "phenotyping").glob(f"*/{plant}/measurement.json"))
                if not files:
                    continue
                source = max(files, key=lambda path: (path.parent.parent.name, str(path)))
                try:
                    observation = soil_observation_from_measurement(json.loads(source.read_text()))
                    if not observation:
                        continue
                    if (observation.get("history_key") != expected_key
                            or observation["observation_id"] != f"{previous.name}/{plant}"
                            or not experiment_start <= datetime.fromisoformat(observation["timestamp"]).date()
                            or datetime.fromisoformat(observation["timestamp"]) >= current_time):
                        continue
                    observations[observation["observation_id"]] = observation
                except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
                    print(f"  warning: skipping soil history {source}: {exc}")
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            print(f"  warning: skipping soil history {source}: {exc}")
    return list(observations.values())


def measure(scan, plant_record, settings, args, history, output_dir, segmentation):
    plant = f"plant_{int(plant_record['plant_id']):02d}"
    location = parse_meta_yaml(scan / "metadata.yaml").get("location")
    history_key = plant_history_key(args.experiment_start_date, location, plant_record["plant_id"])
    history[:] = [o for o in history if o["observation_id"] != f"{scan.name}/{plant}"]  # remove any prior observation for this plant in this scan
    timestamp = scan_timestamp(scan).isoformat()
    directory = output_dir / plant
    directory.mkdir(parents=True, exist_ok=True)
    target = plant_record.get("target_base", {})
    if target.get("frame_id") != EXPECTED_POSE_FRAME:
        raise ValueError("scan needs target_base in base_link")
    center_base = np.asarray([target[f"{dim}_m"] for dim in "xyz"], dtype=float)
    basis = MEASUREMENT_BASIS
    center = center_base @ basis
    plant_radius = float(plant_record["radius_mm"]) / 1000
    if not np.isfinite(plant_radius) or plant_radius <= 0:
        raise ValueError("plant radius_mm must be finite and positive")
    # Small plants still need the full configured soil disk inside the crop.
    radius = max(plant_radius + settings.roi_margin_m, settings.soil_radius_m)
    registration, registration_path = registration_for(scan, plant)

    points, seed, soil_color, view_ids, pixels, views, skipped = load_points(
        scan, plant, center, basis, radius, settings, registration, segmentation, segmentation_output_dir=directory / "segmentation" )
    
    distance = np.linalg.norm(points[:, :2] - center[:2], axis=1)
    soil_candidates = soil_color & (distance <= settings.soil_radius_m)
    soil_candidates = exclude_nearby_plant_per_view(points, seed, soil_candidates, view_ids)
    # estimates the soil surface near the stem from the current scan
    current = fit_local_soil(points[soil_candidates, :3], center[:2], settings)
    current["plant_exclusion"] = {"scope": "same_view", "distance_m": 0.003}
    # estimates the soil reference using valid earlier measurements of the same plant
    prior = historical_soil(history, timestamp, history_key, basis[:, 2].tolist(), center[:2], settings)
    soil, soil_source = (current, "current_scan") if current["accepted"] else (
        (prior, "historical_daily_mean") if prior["accepted"] else (None, "unavailable"))
    warnings = ["stem_position_uses_scan_target_proxy"]
    if soil_source == "historical_daily_mean":
        warnings.append("historical_soil_assumes_unchanged_pot_elevation_and_tilt")
    if registration.get("excluded_disconnected_views"):
        warnings.append("excluded_disconnected_registration_views")
    if len(views) == 1:
        warnings.append("single_view_visible_height_verify_top_not_occluded")
    if skipped:
        warnings.append("some_views_skipped")
    if current["accepted"] and prior["accepted"]:
        if abs(current["soil_elevation_m"] - prior["soil_elevation_m"]) > settings.history_max_spread_m:
            warnings.append("current_soil_disagrees_with_history_check_drift_or_soil_change")
    result = {"scan": scan.name, "plant_id": int(plant_record["plant_id"]), "timestamp": timestamp,
              "location": location,
              "status": "unmeasurable", "height_mm": None, "height_p99_mm": None, "height_p995_mm": None,
              "soil_source": soil_source, "current_soil": current, "historical_soil": prior,
              "history_key": history_key, "frame_id": "base_link",
              "measurement_basis_columns": basis.tolist(), "settings": asdict(settings),
              "stem_target_measurement_m": center.tolist(), "roi_radius_m": radius,
              "registration": str(registration_path),
              "segmentation": {**asdict(segmentation), "overlap_priority": "soil"},
              "excluded_disconnected_views": registration.get("excluded_disconnected_views", []),
              "views": views, "skipped_views": skipped, "warnings": warnings}
    plant_indices, top_index = np.array([], dtype=int), None
    if soil is not None:
        plant_indices, plant_info = select_plant(points, seed, np.asarray(soil["plane"]), center, settings, soil_color,)
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
    canopy = measure_canopy(points[plant_indices], center, radius, settings)
    result.update(canopy)
    if canopy["canopy_near_crop_boundary"]:
        warnings.append("canopy_near_crop_boundary_enlarge_roi")
    if canopy["canopy_status"] == "estimated" and len(views) == 1:
        warnings.append("canopy_single_view_visible_extent_only")
    observation = soil_observation_from_measurement(result)
    if observation is not None:
        # Reruns replace the same observation rather than increasing its weight.
        history[:] = [o for o in history if o["observation_id"] != observation["observation_id"]]
        history.append(observation)
    write_artifacts(directory, points, soil_candidates, plant_indices, center, soil, top_index, settings)
    write_canopy_preview(directory, points[plant_indices], center, radius, canopy)
    write_json(directory / "measurement.json", result)
    return result


def write_scan_summary(directory, results):
    """Save scan-local measurement metrics."""
    columns = ["scan", "location", "plant_id", "timestamp", "status", "height_mm", "height_p99_mm", "height_p995_mm",
               "canopy_status", "canopy_max_span_mm", "canopy_perpendicular_width_mm",
               "canopy_projected_area_mm2", "canopy_hull_area_mm2", "canopy_grid_mm", "canopy_near_crop_boundary",
               "history_key", "soil_source", "soil_elevation_m", "warnings", "error"]
    with (directory / "measurements.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({**r, "warnings": ";".join(r.get("warnings", []))} for r in results)



def build_parser():
    parser = argparse.ArgumentParser( description="Outputs: each SCAN/phenotyping/<UTC run timestamp>/")
    parser.add_argument("input_dir", type=Path, help="Parent directory containing all timestamped scans")
    parser.add_argument("--experiment-start-date", help="Experiment start YYYY-MM-DD; required to reuse soil history, otherwise use current-scan soil only")
    parser.add_argument("--start-date", help="Inclusive first scan date YYYY-MM-DD; preserves earlier soil history within the experiment")
    parser.add_argument("--end-date", help="Inclusive last scan date YYYY-MM-DD; use with --start-date for a date range")
    parser.add_argument("--locations", nargs="+", help="For example b1_r16 b1_r17")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        settings = Settings()   
        validate_settings(settings)
        experiment_start_value = args.experiment_start_date
        experiment_start_date = datetime.strptime(experiment_start_value, "%Y-%m-%d").date() if experiment_start_value else None
        start_date = datetime.strptime(args.start_date, "%Y-%m-%d").date() if args.start_date else None
        end_date = datetime.strptime(args.end_date, "%Y-%m-%d").date() if args.end_date else None
        if start_date and end_date and start_date > end_date:
            raise ValueError("--start-date must not be later than --end-date")
        if experiment_start_date and end_date and end_date < experiment_start_date:
            raise ValueError("--end-date must not be before the experiment start date")
        if experiment_start_date:
            args.experiment_start_date = experiment_start_date.isoformat()
        
        if not args.input_dir.is_dir():
            raise ValueError(f"input directory does not exist: {args.input_dir}")
        scans = []
        for scan in args.input_dir.iterdir():
            timestamp = scan_timestamp(scan) if scan.is_dir() else None
            if timestamp is None or not (scan / "metadata.yaml").is_file():
                continue
            if experiment_start_date and timestamp.date() < experiment_start_date:
                continue
            if start_date and timestamp.date() < start_date:
                continue
            if end_date and timestamp.date() > end_date:
                continue
            if args.locations and parse_meta_yaml(scan / "metadata.yaml").get("location") not in args.locations:
                continue
            scans.append(scan)
        if not scans:
            raise ValueError("no scans match the requested date/locations")
        
        segmentation = prepare_segmentation()
        run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fUTC")
        output_dirs = {scan: scan / "phenotyping" / run_id for scan in scans}
        for directory in output_dirs.values():
            directory.mkdir(parents=True, exist_ok=False)
    except (ValueError, OSError, TypeError, KeyError) as exc:
        parser.error(str(exc))

    history = []
    results = []
    processed_scans = set()
    for scan in sorted(scans, key=lambda p: (scan_timestamp(p), p.name)):
        meta = parse_meta_yaml(scan / "metadata.yaml")
        saved_history = discover_soil_history(scan, meta, experiment_start_date, processed_scans)
        # Fresh observations from this invocation take priority over saved copies.
        combined_history = {o["observation_id"]: o for o in saved_history}
        combined_history.update({o["observation_id"]: o for o in history})
        history = list(combined_history.values())
        output_dir = output_dirs[scan]
        scan_results = []
        for plant in meta.get("plants", []):
            try:
                result = measure(scan, plant, settings, args, history, output_dir, segmentation)
            except (ValueError, OSError, KeyError) as exc:
                result = {"scan": scan.name, "plant_id": plant["plant_id"], "timestamp": scan_timestamp(scan).isoformat(),
                          "location": meta.get("location"), "status": "failed", "canopy_status": "unmeasurable", "error": str(exc), "height_mm": None}
                print(f"Failed {scan.name}/plant_{plant['plant_id']:02d}: {exc}", file=sys.stderr, flush=True)

            write_json(output_dir / f"plant_{plant['plant_id']:02d}" / "measurement.json", result)
            scan_results.append(result)
            results.append(result)

        write_scan_summary(output_dir, scan_results)
        processed_scans.add(scan)
        print(f"Saved {len(scan_results)} measurements to {output_dir}")

    if not results or any(r["status"] == "failed" for r in results):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
