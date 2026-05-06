import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
import yaml
import csv

# ============================================================
# USER SETTINGS
# ============================================================

BASE_DIR = Path("/home/thiwa/scan_data_zed")

# Select folders by timestamp in folder name:
START_DATETIME = "20260505_110000"
END_DATETIME   = "20260505_180000"

# Crop region from full color.png
x1, y1 = 140, 162
x2, y2 = 1836, 900

# HSV range for green plants
lower_green = np.array([22, 27, 0])
upper_green = np.array([95, 255, 255])

MIN_AREA = 10000        # Contour filtering
KERNEL_SIZE = 5     # Morphology cleaning

DEPTH_SCALE = 1000.0
TOP_PERCENTILE = 5          # use closest 5% depth pixels
CENTER_WINDOW_SIZE = 9      # depth median around centroid

# ============================================================
# FUNCTIONS
# ============================================================

def extract_datetime_from_folder(folder_name):
    parts = folder_name.split("_")
    date_part = parts[-2]
    time_part = parts[-1]

    try:
        return datetime.strptime(f"{date_part}_{time_part}", "%Y%m%d_%H%M%S")
    except ValueError:
        return None
    
def load_camera_k(metadata_path):
    with open(metadata_path, "r") as f:
        metadata = yaml.safe_load(f)

    K = metadata["camera_info"]["rgb"]["K"]
    fx = float(K[0])
    fy = float(K[4])
    cx = float(K[2])
    cy = float(K[5])
    frame_id = metadata["camera_info"]["rgb"].get("frame_id", "camera_frame")
    return fx, fy, cx, cy, frame_id

def pixel_depth_to_3d(u, v, z, fx, fy, cx, cy):  #Convert full-image pixel coordinate + depth to 3D camera-frame coordinate.
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    Z = z
    return round(X, 2), round(Y, 2), round(Z, 2)

def get_depth_median_around_pixel(depth, u, v, window_size=9):
    half = window_size // 2

    h, w = depth.shape[:2]

    u1 = max(0, u - half)
    u2 = min(w, u + half + 1)
    v1 = max(0, v - half)
    v2 = min(h, v + half + 1)

    region = depth[v1:v2, u1:u2]

    valid = region[np.isfinite(region)]
    valid = valid[valid > 0]

    if len(valid) == 0:
        return None

    return float(np.median(valid))

def get_top_point_from_contour(depth_full, contour_crop, crop_x1, crop_y1):
    """
    Finds top point using depth pixels inside plant contour.
    For overhead camera, top point is closest to camera = smaller depth.
    """

    h_crop = y2 - y1
    w_crop = x2 - x1

    contour_mask_crop = np.zeros((h_crop, w_crop), dtype=np.uint8)
    cv2.drawContours(contour_mask_crop, [contour_crop], -1, 255, -1)

    # Full image mask location
    depth_crop = depth_full[crop_y1:y2, crop_x1:x2]

    plant_depth_values = depth_crop[contour_mask_crop > 0]

    valid_depth = plant_depth_values[np.isfinite(plant_depth_values)]
    valid_depth = valid_depth[valid_depth > 0]

    if len(valid_depth) == 0:
        return None

    # Instead of using absolute minimum, use closest percentile for noise robustness
    top_threshold = np.percentile(valid_depth, TOP_PERCENTILE)

    top_mask_crop = np.zeros_like(contour_mask_crop)
    top_mask_crop[
        (contour_mask_crop > 0) &
        (depth_crop > 0) &
        np.isfinite(depth_crop) &
        (depth_crop <= top_threshold)
    ] = 255

    ys, xs = np.where(top_mask_crop > 0)

    if len(xs) == 0:
        return None

    top_u_crop = int(np.mean(xs))
    top_v_crop = int(np.mean(ys))
    top_depth = float(np.median(depth_crop[ys, xs]))

    top_u_full = top_u_crop + crop_x1
    top_v_full = top_v_crop + crop_y1

    return top_u_full, top_v_full, top_depth, top_u_crop, top_v_crop

def process_folder(folder_path):
    color_path = folder_path / "color.png"
    depth_path = folder_path / "depth.npy"
    metadata_path = folder_path / "metadata.yaml"

    fx, fy, cx_intr, cy_intr, frame_id = load_camera_k(metadata_path)

    img = cv2.imread(str(color_path))
    depth = np.load(str(depth_path)).astype(np.float32) * DEPTH_SCALE

    crop = img[y1:y2, x1:x2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)   #creates a binary mask
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    segmented = cv2.bitwise_and(crop, crop, mask=mask_clean)        #applies the mask back onto the original
    contours, _ = cv2.findContours(mask_clean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    detection = crop.copy()
    valid_contours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        valid_contours.append(cnt)

    valid_contours = sorted(valid_contours,key=cv2.contourArea,reverse=True)

    results = []

    for i, cnt in enumerate(valid_contours):
        area = cv2.contourArea(cnt)
        M = cv2.moments(cnt)

        if M["m00"] == 0:
            continue

        center_x_crop = int(M["m10"] / M["m00"])
        center_y_crop = int(M["m01"] / M["m00"])

        center_x_full = center_x_crop + x1
        center_y_full = center_y_crop + y1

        center_depth = get_depth_median_around_pixel(depth,center_x_full,center_y_full,CENTER_WINDOW_SIZE)

        top_result = get_top_point_from_contour(depth, cnt, x1, y1)

        top_3d = None
        top_depth = None

        if top_result is not None:
            top_u_full, top_v_full, top_depth, top_u_crop, top_v_crop = top_result
            top_3d = pixel_depth_to_3d(top_u_full,top_v_full,top_depth,fx,fy,cx_intr,cy_intr)
            cv2.circle(detection, (top_u_crop, top_v_crop), 8, (255, 0, 0), -1)
            cv2.putText(detection,f"TOP",(top_u_crop + 10, top_v_crop),cv2.FONT_HERSHEY_SIMPLEX,0.55,(255, 0, 0),2)

        cv2.circle(detection, (center_x_crop, center_y_crop), 6, (0, 0, 255), -1)
        x, y, bw, bh = cv2.boundingRect(cnt)
        cv2.rectangle(detection, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

        depth_for_center_xy = center_depth if center_depth is not None else top_depth

        center_xy_3d = None

        if depth_for_center_xy is not None:
            center_xy_3d = pixel_depth_to_3d(center_x_full,center_y_full,depth_for_center_xy,fx,fy,cx_intr,cy_intr)

        row = {
            "area_px": area,

            "center_x": center_xy_3d[0] if center_xy_3d else None,
            "center_y": center_xy_3d[1] if center_xy_3d else None,
            "center_z": round(center_depth, 2) if center_depth is not None else None,

            "top_x": top_3d[0] if top_3d else None,
            "top_y": top_3d[1] if top_3d else None,
            "top_z": top_3d[2] if top_3d else None,

            # New combined coordinate:
            "target_x": center_xy_3d[0] if center_xy_3d else None,
            "target_y": center_xy_3d[1] if center_xy_3d else None,
            "target_z": top_3d[2] if top_3d else None,
        }

        results.append(row)
        results = sorted(results,key=lambda r: r["center_x"] if r["center_x"] is not None else -999999,reverse=True)
        for new_id, row in enumerate(results, start=1):
            row["plant_id"] = new_id

    cv2.imwrite(str(folder_path / "green_mask.png"), mask_clean)
    cv2.imwrite(str(folder_path / "segmented_result.png"), segmented)
    cv2.imwrite(str(folder_path / "detection.png"), detection)
    cv2.imwrite(str(folder_path / "cropped_color.png"), crop)

    if results:
        fieldnames = ["plant_id","area_px","center_x","center_y","center_z","top_x","top_y","top_z","target_x","target_y","target_z",]
        with open(folder_path / "plant_coordinates_camera_frame.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    print(f"[OK] {folder_path.name}: detected {len(results)} plants")


# ============================================================
# MAIN
# ============================================================

start_dt = datetime.strptime(START_DATETIME, "%Y%m%d_%H%M%S")
end_dt = datetime.strptime(END_DATETIME, "%Y%m%d_%H%M%S")

folders = [p for p in BASE_DIR.iterdir() if p.is_dir()]

selected_folders = []

for folder in folders:
    folder_dt = extract_datetime_from_folder(folder.name)

    if folder_dt is None:
        continue

    if start_dt <= folder_dt <= end_dt:
        selected_folders.append(folder)

selected_folders = sorted(selected_folders)

for folder in selected_folders:
    process_folder(folder)

print("Done.")