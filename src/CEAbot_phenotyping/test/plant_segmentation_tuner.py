#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_scan_folder(folder: Path):
    color_path = folder / "color.png"
    depth_path = folder / "depth.npy"
    metadata_path = folder / "metadata.yaml"

    if not color_path.exists():
        raise FileNotFoundError(f"Missing color image: {color_path}")
    if not depth_path.exists():
        raise FileNotFoundError(f"Missing depth file: {depth_path}")
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata file: {metadata_path}")

    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    if color is None:
        raise RuntimeError(f"Could not read color image: {color_path}")

    depth = np.load(depth_path)
    try:
        metadata = yaml.safe_load(metadata_path.read_text())
    except Exception as e:
        raise RuntimeError(f"Could not read metadata: {e}")

    return color, depth, metadata


def clamp_odd(value: int) -> int:
    value = max(1, value)
    if value % 2 == 0:
        value += 1
    return value


def draw_contours_preview(display, contours, color=(0, 255, 0), thickness=2):
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
    return display


def build_depth_mask(depth_crop: np.ndarray, min_depth_mm: float, max_depth_mm: float):
    depth_mask = (
        np.isfinite(depth_crop)
        & (depth_crop >= min_depth_mm)
        & (depth_crop <= max_depth_mm)
    )
    return depth_mask.astype(np.uint8) * 255


# Global dictionary holding our custom slider configurations and current values
PARAMS = {
    "Lower H":    {"val": 20,   "min": 0, "max": 179},
    "Lower S":    {"val": 30,   "min": 0, "max": 255},
    "Lower V":    {"val": 0,    "min": 0, "max": 255},
    "Upper H":    {"val": 95,   "min": 0, "max": 179},
    "Upper S":    {"val": 255,  "min": 0, "max": 255},
    "Upper V":    {"val": 255,  "min": 0, "max": 255},
    "Min Area":   {"val": 2000, "min": 1, "max": 50000},
    "Kernel Size":{"val": 7,    "min": 1, "max": 31},
    "Dilate Itr": {"val": 1,    "min": 0, "max": 5},
    "Min Depth":  {"val": 150,  "min": 0, "max": 5000},
    "Max Depth":  {"val": 1500, "min": 0, "max": 5000},
    "ROI X1 %":   {"val": 0,    "min": 0, "max": 100},
    "ROI Y1 %":   {"val": 0,    "min": 0, "max": 100},
    "ROI X2 %":   {"val": 100,  "min": 0, "max": 100},
    "ROI Y2 %":   {"val": 100,  "min": 0, "max": 100},
}

# Keep track of which slider is currently dragged by the mouse
active_slider = None

def mouse_callback(event, x, y, flags, param):
    global active_slider
    
    # Define layout metrics matching our draw loop
    start_y = 60
    row_h = 36
    slider_x_start = 180
    slider_w = 260
    
    if event == cv2.EVENT_LBUTTONDOWN:
        for idx, (name, p) in enumerate(PARAMS.items()):
            s_y = start_y + idx * row_h
            # Check if mouse clicked near the track row
            if s_y - 15 <= y <= s_y + 15 and slider_x_start - 10 <= x <= slider_x_start + slider_w + 10:
                active_slider = name
                # Immediately calculate new value
                pct = np.clip((x - slider_x_start) / slider_w, 0.0, 1.0)
                PARAMS[name]["val"] = int(p["min"] + pct * (p["max"] - p["min"]))
                break
                
    elif event == cv2.EVENT_MOUSEMOVE and active_slider is not None:
        p = PARAMS[active_slider]
        pct = np.clip((x - slider_x_start) / slider_w, 0.0, 1.0)
        PARAMS[active_slider]["val"] = int(p["min"] + pct * (p["max"] - p["min"]))
        
    elif event == cv2.EVENT_LBUTTONUP:
        active_slider = None


def main():
    parser = argparse.ArgumentParser(description="Interactive plant segmentation parameter tuner")
    parser.add_argument("folder", type=Path, help="Saved scan folder containing color.png, depth.npy, and metadata.yaml")
    args = parser.parse_args()

    folder = args.folder.expanduser()
    if not folder.exists():
        raise FileNotFoundError(f"Scan folder does not exist: {folder}")

    color, depth, metadata = load_scan_folder(folder)
    img_h, img_w = color.shape[:2]

    if depth.dtype != np.float32 and depth.dtype != np.float64:
        depth = depth.astype(np.float32)
    depth_mm = depth * 1000.0

    # Initialize two clean Windows
    control_win = "Control Panel (Click & Drag)"
    dashboard_win = "Dashboard (Live View)"

    cv2.namedWindow(control_win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(control_win, mouse_callback)
    cv2.moveWindow(control_win, 10, 10)

    cv2.namedWindow(dashboard_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(dashboard_win, 1100, 750)
    cv2.moveWindow(dashboard_win, 510, 10)

    while True:
        # 1. Dynamic Custom UI Draw Base Canvas (Dark Theme)
        ui_canvas = np.full((600, 480, 3), (30, 28, 28), dtype=np.uint8)
        cv2.putText(ui_canvas, "TUNER CONTROLS", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        
        start_y = 60
        row_h = 36
        slider_x_start = 180
        slider_w = 260
        
        for idx, (name, p) in enumerate(PARAMS.items()):
            s_y = start_y + idx * row_h
            
            # Draw Parameter Text Label (Guaranteed White and visible!)
            cv2.putText(ui_canvas, f"{name}:", (15, s_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
            
            # Draw Tracker Line Backplate
            cv2.line(ui_canvas, (slider_x_start, s_y), (slider_x_start + slider_w, s_y), (70, 70, 70), 4, cv2.LINE_AA)
            
            # Calculate current visual thumb knob placement position
            denom = (p["max"] - p["min"])
            val_pct = (p["val"] - p["min"]) / denom if denom > 0 else 0.0
            knob_x = int(slider_x_start + val_pct * slider_w)
            
            # Draw Fill Fill Bar
            cv2.line(ui_canvas, (slider_x_start, s_y), (knob_x, s_y), (235, 140, 30), 4, cv2.LINE_AA)
            # Draw Thumb Circular Handle Knob
            cv2.circle(ui_canvas, (knob_x, s_y), 7, (255, 255, 255), -1, cv2.LINE_AA)
            
            # Print active slider value at the tail end
            cv2.putText(ui_canvas, str(p["val"]), (slider_x_start + slider_w + 12, s_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 230, 255), 1, cv2.LINE_AA)

        # 2. Extract values from our parameters map
        lower_h = PARAMS["Lower H"]["val"]
        lower_s = PARAMS["Lower S"]["val"]
        lower_v = PARAMS["Lower V"]["val"]
        upper_h = PARAMS["Upper H"]["val"]
        upper_s = PARAMS["Upper S"]["val"]
        upper_v = PARAMS["Upper V"]["val"]

        min_area = PARAMS["Min Area"]["val"]
        kernel_size = clamp_odd(PARAMS["Kernel Size"]["val"])
        dilate_iters = PARAMS["Dilate Itr"]["val"]
        min_depth_mm = PARAMS["Min Depth"]["val"]
        max_depth_mm = PARAMS["Max Depth"]["val"]

        rx1, ry1 = PARAMS["ROI X1 %"]["val"], PARAMS["ROI Y1 %"]["val"]
        rx2, ry2 = PARAMS["ROI X2 %"]["val"], PARAMS["ROI Y2 %"]["val"]

        # Scale ROI coordinates
        x1 = int((rx1 / 100.0) * (img_w - 1))
        y1 = int((ry1 / 100.0) * (img_h - 1))
        x2 = max(x1 + 5, int((rx2 / 100.0) * img_w))
        y2 = max(y1 + 5, int((ry2 / 100.0) * img_h))

        # 3. Process Pipeline Images
        crop = color[y1:y2, x1:x2]
        depth_crop = depth_mm[y1:y2, x1:x2]

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v))

        depth_mask = build_depth_mask(depth_crop, min_depth_mm, max_depth_mm)
        mask = cv2.bitwise_and(mask, depth_mask)

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
        if dilate_iters > 0:
            mask_clean = cv2.dilate(mask_clean, kernel, iterations=dilate_iters)

        segmented = cv2.bitwise_and(crop, crop, mask=mask_clean)

        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= max(1, min_area)]

        detection = crop.copy()
        detection = draw_contours_preview(detection, valid_contours)

        # 4. Assemble Live Grid Presenter Dashboard 
        dash_h, dash_w = 350, 450
        view_det = cv2.resize(detection, (dash_w, dash_h))
        view_mask = cv2.resize(cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR), (dash_w, dash_h))
        view_seg = cv2.resize(segmented, (dash_w, dash_h))
        
        view_info = np.full((dash_h, dash_w, 3), (25, 25, 25), dtype=np.uint8)
        cv2.putText(view_info, f"Detected: {len(valid_contours)} plants", (25, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 255, 120), 1, cv2.LINE_AA)
        cv2.putText(view_info, "Press 'q' or 'ESC' to exit", (25, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1, cv2.LINE_AA)

        top_row = np.hstack((view_det, view_mask))
        bottom_row = np.hstack((view_seg, view_info))
        dashboard_canvas = np.vstack((top_row, bottom_row))

        cv2.putText(dashboard_canvas, "1. Detections & Bounding Boxes", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(dashboard_canvas, "2. Cleaned Binary Mask", (dash_w + 15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(dashboard_canvas, "3. Extracted Plant Segments", (15, dash_h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(dashboard_canvas, "4. Telemetry", (dash_w + 15, dash_h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        # Show frames
        cv2.imshow(control_win, ui_canvas)
        cv2.imshow(dashboard_win, dashboard_canvas)

        key = cv2.waitKey(20) & 0xFF
        if key == 27 or key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()