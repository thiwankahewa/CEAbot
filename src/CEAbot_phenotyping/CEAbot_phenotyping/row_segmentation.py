"""Slot-wise plant selection shared by the live node and offline tuner."""

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
