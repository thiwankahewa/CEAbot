#!/usr/bin/env python3
"""Interactive tuner for plant, pot-rim, and soil segmentation."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


DEFAULTS = {
    # Plant color
    "g_h_lo": (35, 179), "g_s_lo": (27, 255), "g_v_lo": (10, 255),
    "g_h_hi": (95, 179), "g_s_hi": (255, 255), "g_v_hi": (255, 255),
    "y_h_lo": (20, 179), "y_s_lo": (80, 255), "y_v_lo": (100, 255),
    "y_h_hi": (38, 179), "y_s_hi": (255, 255), "y_v_hi": (255, 255),
    # Plant geometry
    "min_area": (3000, 50000), "plant_kernel": (4, 31),
    "yellow_kernel": (2, 15), "dilate": (2, 6),
    "plant_z_min": (150, 2000), "plant_z_max": (900, 2500),
    "pot_count": (4, 12), "roi_x1_px": (176, 4000),
    "roi_y1_px": (379, 4000), "roi_x2_px": (1124, 4000),
    "roi_y2_px": (631, 4000),
    # Rim RANSAC (percentages are relative to one pot-slot width)
    "iterations": (800, 2000), "canny_low": (50, 255),
    "canny_high": (130, 255), "radius_min_pct": (34, 80),
    "radius_max_pct": (50, 90), "radius_expected_pct": (42, 80),
    "inner_radius_pct": (75, 100), "center_penalty_x10": (100, 300),
    "search_radius_pct": (55, 100), "support_dist_x10": (25, 100),
    "min_coverage_pct": (40, 100), "min_support": (70, 1000),
    # Soil
    "s_h_lo": (5, 179), "s_s_lo": (25, 255), "s_v_lo": (20, 255),
    "s_h_hi": (30, 179), "s_s_hi": (255, 255), "s_v_hi": (210, 255),
    "soil_z_min": (150, 2500), "soil_z_max": (1300, 3000),
    "plant_exclusion": (5, 31), "soil_open": (3, 15),
    "soil_min_pixels": (300, 10000), "soil_max_mad": (12, 100),
}

WINDOWS = {
    "Plant color": [key for key in DEFAULTS if key.startswith(("g_", "y_"))],
    "Plant geometry": [
        "min_area", "plant_kernel", "yellow_kernel", "dilate",
        "plant_z_min", "plant_z_max", "pot_count",
        "roi_x1_px", "roi_y1_px", "roi_x2_px", "roi_y2_px",
    ],
    "Pot rim RANSAC": [
        "iterations", "canny_low", "canny_high", "radius_min_pct",
        "radius_max_pct", "radius_expected_pct", "inner_radius_pct",
        "center_penalty_x10", "search_radius_pct", "support_dist_x10",
        "min_coverage_pct", "min_support",
    ],
    "Soil": [key for key in DEFAULTS if key.startswith(("s_", "soil_"))]
    + ["plant_exclusion"],
}


def nothing(_value):
    pass


def create_controls():
    for window, names in WINDOWS.items():
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 430, 520)
        for name in names:
            initial, maximum = DEFAULTS[name]
            cv2.createTrackbar(name, window, initial, maximum, nothing)


def params():
    values = {}
    for window, names in WINDOWS.items():
        for name in names:
            values[name] = cv2.getTrackbarPos(name, window)
    # Values that must never be zero.
    for name in (
        "plant_kernel", "yellow_kernel", "pot_count", "iterations",
        "support_dist_x10", "inner_radius_pct", "plant_exclusion",
        "soil_open", "soil_min_pixels",
    ):
        values[name] = max(1, values[name])
    return values


def load_folder(folder):
    color_path, depth_path = folder / "color.png", folder / "depth.npy"
    if not color_path.exists() or not depth_path.exists():
        raise FileNotFoundError("The folder must contain color.png and depth.npy")
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    if color is None:
        raise RuntimeError(f"Could not read {color_path}")
    depth = np.load(depth_path)
    if not np.issubdtype(depth.dtype, np.integer):
        depth = depth.astype(np.float32) * 1000.0
    else:
        depth = depth.astype(np.float32)
    return color, depth


def circle_from_three(points):
    p1, p2, p3 = points.astype(np.float64)
    matrix = 2.0 * np.array((p2 - p1, p3 - p1))
    vector = np.array((np.dot(p2, p2) - np.dot(p1, p1),
                       np.dot(p3, p3) - np.dot(p1, p1)))
    try:
        center = np.linalg.solve(matrix, vector)
    except np.linalg.LinAlgError:
        return None
    radius = float(np.linalg.norm(p1 - center))
    return None if not np.isfinite(radius) else (center[0], center[1], radius)


def fit_rim(edges, expected, slot_width, plant_id, p):
    height, width = edges.shape
    search = p["search_radius_pct"] / 100.0 * slot_width
    ex, ey = expected
    x1, x2 = max(0, int(ex - search)), min(width, int(ex + search) + 1)
    y1, y2 = max(0, int(ey - search)), min(height, int(ey + search) + 1)
    ys, xs = np.where(edges[y1:y2, x1:x2] > 0)
    points = np.column_stack((xs + x1, ys + y1))
    if len(points) < 3:
        return None

    r_min = p["radius_min_pct"] / 100.0 * slot_width
    r_max = p["radius_max_pct"] / 100.0 * slot_width
    r_expected = p["radius_expected_pct"] / 100.0 * slot_width
    tolerance = p["support_dist_x10"] / 10.0
    penalty = p["center_penalty_x10"] / 10.0
    rng, best = np.random.default_rng(plant_id), None

    for _ in range(p["iterations"]):
        circle = circle_from_three(points[rng.choice(len(points), 3, False)])
        if circle is None:
            continue
        cx, cy, radius = circle
        if not r_min <= radius <= r_max:
            continue
        if abs(cx - ex) > .45 * slot_width or abs(cy - ey) > .45 * slot_width:
            continue
        errors = np.abs(np.hypot(points[:, 0] - cx, points[:, 1] - cy) - radius)
        support = errors <= tolerance
        count = int(support.sum())
        if not count:
            continue
        angles = np.arctan2(points[support, 1] - cy, points[support, 0] - cx)
        bins = np.clip(((angles + np.pi) / (2 * np.pi) * 36).astype(int), 0, 35)
        coverage = len(np.unique(bins)) / 36.0
        score = (count + coverage * 288 - 3 * abs(radius - r_expected)
                 - penalty * np.hypot(cx - ex, cy - ey))
        if best is None or score > best["score"]:
            best = {"x": float(cx), "y": float(cy), "radius": float(radius),
                    "support": count, "coverage": coverage, "score": score}
    if best is None:
        return None
    if best["coverage"] >= .65 and best["support"] >= max(120, p["min_support"]):
        best["confidence"] = "high"
    elif (best["coverage"] >= p["min_coverage_pct"] / 100.0
          and best["support"] >= p["min_support"]):
        best["confidence"] = "medium"
    else:
        best["confidence"] = "low"
    return best


def plant_records(contours, width, count):
    slots = [(count - index - .5) / count for index in range(count)]
    records = {}
    for contour in contours:
        moments = cv2.moments(contour)
        if not moments["m00"]:
            continue
        cx, cy = int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])
        fraction = cx / float(width)
        index = int(np.argmin([abs(fraction - slot) for slot in slots]))
        if abs(fraction - slots[index]) > .45 / count:
            continue
        item = {"id": index + 1, "x": cx, "y": cy,
                "area": cv2.contourArea(contour), "contour": contour}
        old = records.get(item["id"])
        if old is None or item["area"] > old["area"]:
            records[item["id"]] = item
    return [records[key] for key in sorted(records)]


def process(color, depth, p):
    height, width = color.shape[:2]
    x1, y1 = p["roi_x1_px"], p["roi_y1_px"]
    x2, y2 = p["roi_x2_px"], p["roi_y2_px"]
    x1, y1 = np.clip(x1, 0, width - 1), np.clip(y1, 0, height - 1)
    x2, y2 = np.clip(max(x1 + 5, x2), x1 + 1, width), np.clip(max(y1 + 5, y2), y1 + 1, height)
    crop, z = color[y1:y2, x1:x2], depth[y1:y2, x1:x2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    green = cv2.inRange(hsv, (p["g_h_lo"], p["g_s_lo"], p["g_v_lo"]),
                        (p["g_h_hi"], p["g_s_hi"], p["g_v_hi"]))
    yellow = cv2.inRange(hsv, (p["y_h_lo"], p["y_s_lo"], p["y_v_lo"]),
                         (p["y_h_hi"], p["y_s_hi"], p["y_v_hi"]))
    valid_plant_z = np.isfinite(z) & (z >= p["plant_z_min"]) & (z <= p["plant_z_max"])
    green[~valid_plant_z], yellow[~valid_plant_z] = 0, 0
    kernel = np.ones((p["plant_kernel"], p["plant_kernel"]), np.uint8)
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
    green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
    y_kernel = np.ones((p["yellow_kernel"], p["yellow_kernel"]), np.uint8)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, y_kernel)
    measurement = cv2.bitwise_or(green, yellow)
    grouping = cv2.morphologyEx(measurement, cv2.MORPH_CLOSE, kernel)
    if p["dilate"]:
        grouping = cv2.dilate(grouping, kernel, iterations=p["dilate"])
    contours, _ = cv2.findContours(grouping, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= p["min_area"]]
    records = plant_records(contours, crop.shape[1], p["pot_count"])

    canny_high = max(p["canny_low"] + 1, p["canny_high"])
    gray = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (5, 5), 1.2)
    edges = cv2.Canny(gray, p["canny_low"], canny_high)
    overlay, interior, soil = crop.copy(), np.zeros_like(grouping), np.zeros_like(grouping)
    exclusion = cv2.dilate(measurement, np.ones((p["plant_exclusion"],) * 2, np.uint8))
    soil_color = cv2.inRange(hsv, (p["s_h_lo"], p["s_s_lo"], p["s_v_lo"]),
                             (p["s_h_hi"], p["s_s_hi"], p["s_v_hi"]))
    valid_soil_z = np.isfinite(z) & (z >= p["soil_z_min"]) & (z <= p["soil_z_max"])
    slot_width, telemetry = crop.shape[1] / p["pot_count"], []

    for record in records:
        rim = fit_rim(edges, (record["x"], record["y"]), slot_width, record["id"], p)
        if rim is None or rim["confidence"] == "low":
            cx, cy = record["x"], record["y"]
            radius = p["radius_expected_pct"] / 100 * slot_width
            rim_confidence, method = "low", "fallback"
        else:
            cx, cy, radius = rim["x"], rim["y"], rim["radius"]
            rim_confidence, method = rim["confidence"], "ransac"
        pot = np.zeros_like(interior)
        cv2.circle(pot, (round(cx), round(cy)), max(1, round(radius * p["inner_radius_pct"] / 100)), 255, -1)
        interior = cv2.bitwise_or(interior, pot)
        candidate = (pot > 0) & (exclusion == 0) & (soil_color > 0) & valid_soil_z
        raw = z[candidate]
        median = mad = None
        if raw.size:
            median = float(np.median(raw)); mad = float(np.median(np.abs(raw - median)))
            candidate &= np.abs(z - median) <= max(3 * mad, 5)
        accepted = z[candidate]
        soil[candidate] = 255
        soil_confidence = "insufficient"
        if accepted.size >= p["soil_min_pixels"]:
            soil_confidence = "high" if mad <= p["soil_max_mad"] and rim_confidence != "low" else "medium"
        color_code = (0, 255, 0) if rim_confidence == "high" else (0, 200, 255) if rim_confidence == "medium" else (0, 0, 255)
        cv2.circle(overlay, (round(cx), round(cy)), round(radius), color_code, 2)
        cv2.drawContours(overlay, [record["contour"]], -1, (255, 255, 0), 1)
        cv2.putText(overlay, f"P{record['id']} {rim_confidence}", (round(cx) - 35, round(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color_code, 1, cv2.LINE_AA)
        telemetry.append({"plant_id": record["id"], "rim_method": method,
                          "rim_confidence": rim_confidence,
                          "rim_center": [round(cx, 1), round(cy, 1)],
                          "rim_radius_px": round(radius, 1),
                          "soil_pixels": int(accepted.size),
                          "soil_median_depth_mm": round(float(np.median(accepted)), 1) if accepted.size else None,
                          "soil_mad_mm": round(mad, 1) if mad is not None else None,
                          "soil_confidence": soil_confidence})
    soil = cv2.morphologyEx(soil, cv2.MORPH_OPEN, np.ones((p["soil_open"],) * 2, np.uint8))
    segmented = cv2.bitwise_and(crop, crop, mask=measurement)
    return {"crop": crop, "overlay": overlay, "green": green, "yellow": yellow,
            "measurement": measurement, "grouping": grouping, "segmented": segmented,
            "interior": interior, "soil": soil, "telemetry": telemetry,
            "roi": [int(x1), int(y1), int(x2), int(y2)]}


def tile(image, title, size=(360, 210)):
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    output = cv2.resize(image, size)
    cv2.rectangle(output, (0, 0), (size[0], 27), (20, 20, 20), -1)
    cv2.putText(output, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1, cv2.LINE_AA)
    return output


def dashboard(result):
    views = [("overlay", "Plant contours + rim RANSAC"), ("green", "Green mask"),
             ("yellow", "Yellow flower mask"), ("measurement", "Measurement mask"),
             ("grouping", "Dilated grouping mask"), ("segmented", "Plant segmentation"),
             ("overlay", "Dynamic pot rims"), ("interior", "Inner-pot ROI"),
             ("soil", "Accepted soil mask")]
    panels = [tile(result[key], label) for key, label in views]
    return np.vstack((np.hstack(panels[:3]), np.hstack(panels[3:6]), np.hstack(panels[6:])))


def save(folder, result, p):
    for key in ("green", "yellow", "measurement", "grouping", "overlay", "interior", "soil"):
        cv2.imwrite(str(folder / f"tuner_{key}.png"), result[key])
    report = {"roi": result["roi"], "parameters": p, "pots": result["telemetry"]}
    (folder / "tuner_parameters.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    print(f"Saved tuner outputs to {folder}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    folder = args.folder.expanduser()
    color, depth = load_folder(folder)
    create_controls()
    cv2.namedWindow("Dashboard", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dashboard", 1200, 720)
    previous, result, current = None, None, None
    while True:
        current = params()
        snapshot = tuple(current.items())
        if snapshot != previous:
            result = process(color, depth, current)
            previous = snapshot
            print("\nPer-pot result:")
            for item in result["telemetry"]:
                print(item)
        cv2.imshow("Dashboard", dashboard(result))
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            save(folder, result, current)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
