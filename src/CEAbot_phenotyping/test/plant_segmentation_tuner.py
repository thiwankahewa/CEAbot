#!/usr/bin/env python3
"""Interactive plant, pot-rim, and soil tuner with native Tkinter controls."""

import argparse
import sys
from concurrent.futures import ThreadPoolExecutor
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageTk

# Allow running this tuner directly from the source checkout without ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from CEAbot_phenotyping.row_segmentation import draw_row_guides, select_row_contours


DEFAULTS = {
    # Plant color
    "g_h_lo": (35, 179), "g_s_lo": (27, 255), "g_v_lo": (10, 255),
    "g_h_hi": (95, 179), "g_s_hi": (255, 255), "g_v_hi": (255, 255),
    "y_h_lo": (20, 179), "y_s_lo": (80, 255), "y_v_lo": (100, 255),
    "y_h_hi": (38, 179), "y_s_hi": (255, 255), "y_v_hi": (255, 255),
    # Plant geometry
    "min_area": (500, 50000), "row_band_pct": (50, 100), "plant_kernel": (4, 31),
    "yellow_kernel": (2, 15), "dilate": (2, 6),
    "plant_z_min": (150, 2000), "plant_z_max": (900, 2500),
    "pot_count": (4, 12), "roi_x1_px": (93, 298),
    "roi_y1_px": (379, 4000), "roi_x2_px": (1157, 551),
    "roi_y2_px": (631, 4000),
    # Rim RANSAC (percentages are relative to one pot-slot width)
    "iterations": (800, 2000), "canny_low": (50, 255),
    "canny_high": (130, 255), "radius_min_pct": (34, 80),
    "radius_max_pct": (50, 90), "radius_expected_pct": (42, 80),
    "inner_radius_pct": (75, 100), "center_penalty_x10": (100, 300),
    "search_radius_pct": (55, 100), "support_dist_x10": (25, 100),
    "min_coverage_pct": (40, 100), "min_support": (70, 1000),
    # Soil
    "s_h_lo": (0, 179), "s_s_lo": (0, 255), "s_v_lo": (0, 255),
    "s_h_hi": (89, 179), "s_s_hi": (255, 255), "s_v_hi": (255, 255),
    "soil_z_min": (773, 2500), "soil_z_max": (942, 3000),
    "plant_exclusion": (5, 31), "soil_open": (3, 15),
    "soil_min_pixels": (380, 10000), "soil_max_mad": (10, 100),
}

WINDOWS = {
    "Plant color": [key for key in DEFAULTS if key.startswith(("g_", "y_"))],
    "Plant geometry": [
        "min_area", "row_band_pct", "plant_kernel", "yellow_kernel", "dilate",
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


# Keep nonzero constraints visible in the controls and saved parameters.
POSITIVE_PARAMS = {
    "plant_kernel", "yellow_kernel", "pot_count", "iterations",
    "support_dist_x10", "inner_radius_pct", "plant_exclusion",
    "soil_open", "soil_min_pixels",
}


def parameter_limits(color):
    height, width = color.shape[:2]
    limits = {
        key: (1 if key in POSITIVE_PARAMS else 0, maximum)
        for key, (_, maximum) in DEFAULTS.items()
    }
    limits.update({
        "roi_x1_px": (0, width - 1), "roi_x2_px": (1, width),
        "roi_y1_px": (0, height - 1), "roi_y2_px": (1, height),
    })
    return limits


def validate_parameters(values, limits):
    """Reject invalid ranges instead of silently processing different values."""
    for key, (minimum, maximum) in limits.items():
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
        if not minimum <= value <= maximum:
            raise ValueError(f"{key} must be between {minimum} and {maximum}")
    for prefix in ("g", "y", "s"):
        for channel in ("h", "s", "v"):
            if values[f"{prefix}_{channel}_lo"] > values[f"{prefix}_{channel}_hi"]:
                raise ValueError(f"{prefix}_{channel}: lower bound must be ≤ upper bound")
    for low, high in (
        ("plant_z_min", "plant_z_max"), ("soil_z_min", "soil_z_max"),
        ("radius_min_pct", "radius_max_pct"), ("canny_low", "canny_high"),
        ("roi_x1_px", "roi_x2_px"), ("roi_y1_px", "roi_y2_px"),
    ):
        if values[low] >= values[high]:
            raise ValueError(f"{low} must be less than {high}")
    if not values["radius_min_pct"] <= values["radius_expected_pct"] <= values["radius_max_pct"]:
        raise ValueError("Expected rim radius must be within the minimum/maximum range")
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
    if depth.ndim != 2 or depth.shape != color.shape[:2]:
        raise ValueError("color.png and depth.npy must have matching image dimensions")
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


def plant_records(slot_contours):
    records = []
    for plant_id, contour in slot_contours:
        moments = cv2.moments(contour)
        if not moments["m00"]:
            continue
        cx, cy = int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])
        records.append({"id": plant_id, "x": cx, "y": cy,
                        "area": cv2.contourArea(contour), "contour": contour})
    return records


def process(color, depth, p):
    height, width = color.shape[:2]
    x1, y1 = p["roi_x1_px"], p["roi_y1_px"]
    x2, y2 = p["roi_x2_px"], p["roi_y2_px"]
    x1, y1 = np.clip(x1, 0, width - 1), np.clip(y1, 0, height - 1)
    x2, y2 = np.clip(max(x1 + 1, x2), x1 + 1, width), np.clip(max(y1 + 1, y2), y1 + 1, height)
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
    grouping, slot_contours = select_row_contours(
        measurement, p["pot_count"], kernel, p["dilate"],
        p["min_area"], p["row_band_pct"])
    records = plant_records(slot_contours)

    canny_high = max(p["canny_low"] + 1, p["canny_high"])
    gray = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (5, 5), 1.2)
    edges = cv2.Canny(gray, p["canny_low"], canny_high)
    overlay, interior, soil = crop.copy(), np.zeros_like(grouping), np.zeros_like(grouping)
    draw_row_guides(overlay, p["pot_count"], p["row_band_pct"])
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
                          "plant_area_px": float(record["area"]),
                          "plant_center_px": [record["x"], record["y"]],
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
    # Letterbox each view so rims and leaves retain their true proportions.
    output = np.full((size[1], size[0], 3), 24, dtype=np.uint8)
    scale = min(size[0] / image.shape[1], (size[1] - 27) / image.shape[0])
    width, height = max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale))
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    x, y = (size[0] - width) // 2, 27 + (size[1] - 27 - height) // 2
    output[y:y + height, x:x + width] = resized
    cv2.rectangle(output, (0, 0), (size[0], 27), (20, 20, 20), -1)
    cv2.putText(output, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1, cv2.LINE_AA)
    return output


def dashboard(result):
    views = [("overlay", "Plant contours + rim RANSAC"), ("green", "Green mask"),
             ("yellow", "Yellow flower mask"), ("measurement", "Measurement mask"),
             ("grouping", "Dilated grouping mask"), ("segmented", "Plant segmentation"),
             ("crop", "Original crop"), ("interior", "Inner-pot ROI"),
             ("soil", "Accepted soil mask")]
    panels = [tile(result[key], label) for key, label in views]
    return np.vstack((np.hstack(panels[:3]), np.hstack(panels[3:6]), np.hstack(panels[6:])))


def save(folder, result, p):
    for key in ("green", "yellow", "measurement", "grouping", "overlay", "interior", "soil"):
        path = folder / f"tuner_{key}.png"
        if not cv2.imwrite(str(path), result[key]):
            raise OSError(f"Could not save {path}")
    report = {"roi": result["roi"], "parameters": p, "pots": result["telemetry"]}
    (folder / "tuner_parameters.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    print(f"Saved tuner outputs to {folder}")



class TunerApp:
    """Native controls with debounced processing and no OpenCV HighGUI calls."""

    def __init__(self, root, folder, color, depth):
        self.root, self.folder = root, folder
        self.color, self.depth = color, depth
        self.limits = parameter_limits(color)
        self.variables = {}
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.pending = None
        self.result = self.result_params = None
        self.preview_image = None
        self.debounce_id = None
        self.closed = False
        self.updating = False

        root.title(f"Plant segmentation tuner — {folder.name}")
        width = min(1500, root.winfo_screenwidth() - 60)
        height = min(950, root.winfo_screenheight() - 80)
        root.geometry(f"{width}x{height}")
        root.protocol("WM_DELETE_WINDOW", self.close)

        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        self.save_button = ttk.Button(toolbar, text="Save outputs", command=self.save_outputs, state="disabled")
        self.save_button.pack(side="left")
        ttk.Button(toolbar, text="Load parameters…", command=self.load_parameters).pack(side="left", padx=6)
        ttk.Button(toolbar, text="Reset defaults", command=self.reset).pack(side="left")
        ttk.Label(toolbar, text=str(folder)).pack(side="left", padx=12)
        ttk.Button(toolbar, text="Quit", command=self.close).pack(side="right")

        panes = ttk.Panedwindow(root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8)
        controls = ttk.Notebook(panes, width=450)
        panes.add(controls, weight=0)
        display = ttk.Frame(panes)
        panes.add(display, weight=1)

        for title, keys in WINDOWS.items():
            tab = ttk.Frame(controls)
            controls.add(tab, text=title)
            canvas = tk.Canvas(tab, highlightthickness=0, width=440)
            scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scrollbar.set)
            scrollbar.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            content = ttk.Frame(canvas, padding=8)
            window = canvas.create_window((0, 0), window=content, anchor="nw")
            content.bind("<Configure>", lambda event, c=canvas: c.configure(scrollregion=c.bbox("all")))
            canvas.bind("<Configure>", lambda event, c=canvas, w=window: c.itemconfigure(w, width=event.width))
            content.columnconfigure(1, weight=1)
            for row, key in enumerate(keys):
                low, high = self.limits[key]
                value = max(low, min(high, DEFAULTS[key][0]))
                variable = tk.StringVar(value=str(value))
                self.variables[key] = variable
                ttk.Label(content, text=key).grid(row=row, column=0, sticky="w", pady=7)
                scale = ttk.Scale(content, from_=low, to=high, orient="horizontal")
                scale.set(value)
                scale.configure(command=lambda value, k=key: self.set_slider(k, value))
                scale.grid(row=row, column=1, sticky="ew", padx=8)
                entry = ttk.Spinbox(content, from_=low, to=high, textvariable=variable, width=7)
                entry.grid(row=row, column=2)
                variable.trace_add("write", lambda *args, k=key, s=scale: self.parameter_changed(k, s))
            # Scroll controls on Linux/X11 and platforms reporting MouseWheel.
            def wheel(event, c=canvas):
                step = -1 if event.num == 4 or getattr(event, "delta", 0) > 0 else 1
                c.yview_scroll(step * 3, "units")
                return "break"
            for widget in (canvas, content, *content.winfo_children()):
                for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                    widget.bind(event, wheel)

        self.preview = ttk.Label(display, anchor="center")
        self.preview.pack(fill="both", expand=True)
        self.preview.bind("<Configure>", self.draw_preview)
        columns = ("plant_id", "rim_confidence", "soil_pixels", "soil_median_depth_mm", "soil_confidence")
        self.telemetry = ttk.Treeview(display, columns=columns, show="headings", height=5)
        for key, label in zip(columns, ("Pot", "Rim confidence", "Soil pixels", "Depth (mm)", "Soil confidence")):
            self.telemetry.heading(key, text=label)
            self.telemetry.column(key, width=105, anchor="center")
        self.telemetry.pack(fill="x", pady=8)
        self.status = tk.StringVar(value="Preparing preview…")
        ttk.Label(root, textvariable=self.status, padding=8, wraplength=1200).pack(fill="x")
        root.bind("<Control-s>", lambda event: self.save_outputs())
        root.bind("<Escape>", lambda event: self.close())
        self.reset()
        self.poll_id = root.after(50, self.poll)

    def set_slider(self, key, value):
        text = str(round(float(value)))
        if self.variables[key].get() != text:
            self.variables[key].set(text)

    def parameter_changed(self, key, scale):
        if self.updating:
            return
        try:
            value = int(self.variables[key].get())
            low, high = self.limits[key]
            if low <= value <= high and round(scale.get()) != value:
                scale.set(value)
        except ValueError:
            pass  # Allow partially typed numeric entries.
        self.schedule_update()

    def read_parameters(self):
        values = {}
        for key, variable in self.variables.items():
            try:
                values[key] = int(variable.get())
            except ValueError:
                raise ValueError(f"{key} must be an integer") from None
        return validate_parameters(values, self.limits)

    def schedule_update(self):
        self.save_button.configure(state="disabled")
        self.pending = None
        if self.debounce_id is not None:
            self.root.after_cancel(self.debounce_id)
        self.status.set("Parameters changed — waiting to update preview…")
        self.debounce_id = self.root.after(200, self.queue_update)

    def queue_update(self):
        self.debounce_id = None
        try:
            self.pending = self.read_parameters()
            self.status.set("Updating segmentation…")
        except ValueError as exc:
            self.status.set(str(exc))

    def poll(self):
        if self.closed:
            return
        if self.future is not None and self.future.done():
            future, submitted = self.future, self.submitted
            self.future = None
            try:
                result = future.result()
                # A result is only displayed/saved if it matches the controls.
                if submitted == self.read_parameters():
                    self.result, self.result_params = result, submitted
                    self.preview_image = Image.fromarray(cv2.cvtColor(dashboard(result), cv2.COLOR_BGR2RGB))
                    self.draw_preview()
                    self.telemetry.delete(*self.telemetry.get_children())
                    for item in result["telemetry"]:
                        self.telemetry.insert("", "end", values=[
                            item[key] if item[key] is not None else "—"
                            for key in self.telemetry["columns"]
                        ])
                    self.save_button.configure(state="normal")
                    self.status.set(f"{len(result['telemetry'])} pots detected · ROI {result['roi']} · Ctrl+S saves masks and YAML")
            except Exception as exc:
                self.status.set(f"Cannot update preview: {exc}")
        if self.future is None and self.pending is not None:
            self.submitted, self.pending = self.pending, None
            self.future = self.executor.submit(process, self.color, self.depth, self.submitted)
        self.poll_id = self.root.after(50, self.poll)

    def draw_preview(self, event=None):
        if self.preview_image is None:
            return
        image = self.preview_image.copy()
        image.thumbnail((max(1, self.preview.winfo_width()), max(1, self.preview.winfo_height())),
                        Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(image)
        self.preview.configure(image=self.photo)

    def apply_parameters(self, values):
        validate_parameters(values, self.limits)
        self.updating = True
        try:
            for key, value in values.items():
                self.variables[key].set(str(value))
        finally:
            self.updating = False
        # Trigger traces once more to synchronize slider positions.
        for variable in self.variables.values():
            variable.set(variable.get())
        self.schedule_update()

    def reset(self):
        values = {key: max(low, min(high, DEFAULTS[key][0]))
                  for key, (low, high) in self.limits.items()}
        for axis, maximum in (("x", self.color.shape[1]), ("y", self.color.shape[0])):
            if values[f"roi_{axis}1_px"] >= values[f"roi_{axis}2_px"]:
                values[f"roi_{axis}1_px"], values[f"roi_{axis}2_px"] = 0, maximum
        self.apply_parameters(values)

    def load_parameters(self):
        path = filedialog.askopenfilename(parent=self.root, initialdir=self.folder,
                                          filetypes=[("YAML parameters", "*.yaml *.yml")])
        if not path:
            return
        try:
            report = yaml.safe_load(Path(path).read_text())
            if not isinstance(report, dict) or not isinstance(report.get("parameters"), dict):
                raise ValueError("Expected a tuner YAML file containing parameters")
            values = dict(report["parameters"])
            # Saved files from before the row-band control remain loadable.
            values.setdefault("row_band_pct", DEFAULTS["row_band_pct"][0])
            if set(values) != set(DEFAULTS):
                raise ValueError("The file must contain all tuner parameters and no unknown parameters")
            self.apply_parameters(values)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            messagebox.showerror("Could not load parameters", str(exc), parent=self.root)

    def save_outputs(self):
        try:
            current = self.read_parameters()
            if self.result is None or current != self.result_params:
                self.status.set("Wait for the preview to finish updating before saving.")
                return
            save(self.folder, self.result, current)
            self.status.set(f"Saved masks and tuner_parameters.yaml to {self.folder}")
        except (OSError, ValueError, cv2.error) as exc:
            messagebox.showerror("Could not save outputs", str(exc), parent=self.root)

    def close(self):
        self.closed = True
        if self.debounce_id is not None:
            self.root.after_cancel(self.debounce_id)
        self.root.after_cancel(self.poll_id)
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, nargs="?",
                        help="Scan folder containing color.png and depth.npy; opens a picker if omitted")
    args = parser.parse_args()
    root = tk.Tk()
    root.withdraw()
    folder = args.folder
    if folder is None:
        selected = filedialog.askdirectory(parent=root, title="Select scan folder with color.png and depth.npy")
        if not selected:
            root.destroy()
            return
        folder = Path(selected)
    folder = folder.expanduser()
    try:
        color, depth = load_folder(folder)
    except (OSError, ValueError, RuntimeError) as exc:
        messagebox.showerror("Could not open scan", str(exc), parent=root)
        root.destroy()
        return
    TunerApp(root, folder, color, depth)
    root.deiconify()
    root.mainloop()


if __name__ == "__main__":
    main()
