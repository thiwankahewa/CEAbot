"""Slot-wise plant selection shared by the live node and offline tuner."""

from pathlib import Path
import sys

import cv2
import numpy as np


def select_row_contours(measurement, pot_count, kernel, dilation_iterations,
                        min_area=500, row_band_pct=50):
    """Return grouping mask and (right-to-left pot ID, contour) pairs.

    Only centroids within the central ``row_band_pct`` percent of crop height
    qualify. Prefer the centroid nearest the horizontal midline, using area
    only to break ties. Keep complete contours for subsequent measurements;
    the band selects plants without clipping their foliage.
    """
    if pot_count < 1 or not 0 <= row_band_pct <= 100 or min_area < 0:
        raise ValueError("Invalid pot count, row band percentage, or minimum area")
    height, width = measurement.shape
    midline = (height - 1) / 2.0
    half_band = height * row_band_pct / 200.0
    grouping = np.zeros_like(measurement)
    selected = []
    slot_width = width / float(pot_count)
    for slot_index in range(pot_count):
        left = int(round(slot_index * slot_width))
        right = int(round((slot_index + 1) * slot_width))
        if right <= left:
            continue
        slot = cv2.morphologyEx(measurement[:, left:right], cv2.MORPH_CLOSE, kernel)
        if dilation_iterations:
            slot = cv2.dilate(slot, kernel, iterations=dilation_iterations)
        grouping[:, left:right] = slot
        contours, _ = cv2.findContours(slot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            moments = cv2.moments(contour)
            if not moments["m00"]:
                continue
            distance = abs(moments["m01"] / moments["m00"] - midline)
            if distance <= half_band:
                candidates.append((distance, -area, contour))
        if candidates:
            contour = min(candidates, key=lambda item: item[:2])[2].copy()
            contour[:, 0, 0] += left
            selected.append((pot_count - slot_index, contour))

    # Display slot boundaries only after extracting complete per-slot contours.
    for index in range(1, pot_count):
        boundary = int(round(index * slot_width))
        grouping[:, max(0, boundary - 1):boundary + 1] = 0
    return grouping, sorted(selected, key=lambda item: item[0])


def draw_row_guides(image, pot_count, row_band_pct):
    """Draw the slot boundaries, horizontal midline, and acceptance band."""
    height, width = image.shape[:2]
    midline = (height - 1) / 2.0
    half_band = height * row_band_pct / 200.0
    for index in range(1, pot_count):
        x = int(round(index * width / pot_count))
        cv2.line(image, (x, 0), (x, height - 1), (255, 180, 0), 1)
    for y in (midline - half_band, midline + half_band):
        y = max(0, min(height - 1, round(y)))
        cv2.line(image, (0, y), (width - 1, y), (160, 120, 0), 1)
    cv2.line(image, (0, round(midline)), (width - 1, round(midline)), (0, 255, 255), 1)


class YoloPlantSegmenter:
    """Load one local segmentation model and return full-frame plant masks.

    Ultralytics is optional: it is imported only when this backend is selected.
    Input is OpenCV BGR. Native-resolution masks avoid letterbox misalignment.
    """

    def __init__(self, weights, confidence=0.25, imgsz=640, device="", class_name="plant"):
        if not np.isfinite(confidence) or not 0 < confidence <= 1:
            raise ValueError("row_yolo_confidence must be in (0, 1]")
        if isinstance(imgsz, bool) or not isinstance(imgsz, int) or imgsz < 32 or imgsz % 32:
            raise ValueError("row_yolo_imgsz must be a positive multiple of 32")
        path = Path(weights).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f"row_yolo_weights must point to local segmentation weights: {path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ValueError(
                f"YOLO row segmentation could not load in {sys.executable}: {exc}. "
                "Install the YOLO dependencies for this interpreter; see "
                "src/CEAbot_phenotyping/README.md for the Jetson setup."
            ) from exc
        self.model = YOLO(str(path))
        if self.model.task != "segment":
            raise ValueError("row_yolo_weights must be a segmentation model, not a box detector")
        names = self.model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        matches = [int(index) for index, name in items if name == class_name]
        if len(matches) != 1:
            raise ValueError(f"model must contain exactly one class named {class_name!r}; got {names}")
        self.class_id = matches[0]
        self.weights = str(path)
        self.confidence = confidence
        self.imgsz = imgsz
        self.device = device
        self.class_name = class_name

    def predict_mask(self, bgr):
        results = self.model.predict(
            source=bgr, conf=self.confidence, imgsz=self.imgsz,
            device=self.device or None, classes=[self.class_id], retina_masks=True,
            verbose=False, save=False,
        )
        if len(results) != 1:
            raise ValueError("expected one segmentation result for one row image")
        return plant_mask_from_result(results[0], bgr.shape[:2], self.class_id)


def plant_mask_from_result(result, image_shape, class_id):
    """Union only plant instances; preserve full-frame pixel coordinates."""
    output = np.zeros(image_shape, dtype=np.uint8)
    if result.boxes is None or len(result.boxes) == 0:
        return output
    if result.masks is None:
        raise ValueError("segmentation result contains boxes but no masks")
    masks = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    if masks.ndim != 3 or masks.shape[1:] != tuple(image_shape) or len(masks) != len(classes):
        raise ValueError("segmentation masks do not match the original row image; retina_masks is required")
    selected = masks[classes == class_id]
    if len(selected):
        output[np.any(selected > 0.5, axis=0)] = 255
    return output
