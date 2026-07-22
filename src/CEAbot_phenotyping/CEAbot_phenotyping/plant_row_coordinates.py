#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from arm_interfaces.msg import PlantTarget, PlantTargetArray


class PlantCoordinateNode(Node):
    def __init__(self):
        super().__init__("plant_row_coordinate_node")

        self.bridge = CvBridge()

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_camera_info_msg = None
        self.latest_run_dir = None
        self.pending_process = False

        self.x1 = 176
        self.y1 = 379
        self.x2 = 1124
        self.y2 = 631

        self.lower_green = np.array([35, 27, 10])
        self.upper_green = np.array([95, 255, 255])
        self.lower_yellow = np.array([20, 80, 100])
        self.upper_yellow = np.array([38, 255, 255])
        self.min_area = 3000
        self.kernel_size = 4
        self.top_percentile = 5.0
        self.center_window_size = 9
        self.min_depth_mm = 150.0
        self.max_depth_mm = 900.0
        self.dilate_itr = 2

        # convention: pot 1 is the rightmost pot and IDs increase to the left.
        self.declare_parameter("pot_count", 4)
        self.pot_count = int(self.get_parameter("pot_count").value)
        if self.pot_count < 1:
            self.get_logger().warn("pot_count must be at least 1; using 4")
            self.pot_count = 4

        self.pot_slot_x_fractions = tuple((self.pot_count - slot_index - 0.5) / self.pot_count
            for slot_index in range(self.pot_count))

        self.max_pot_slot_error_fraction = 0.45 / self.pot_count

        # Dynamic pot-rim localization and conservative soil segmentation.
        # Fractions below are relative to one expected pot-slot width.
        self.declare_parameter("pot_ransac_iterations", 800)
        self.declare_parameter("pot_radius_min_fraction", 0.34)
        self.declare_parameter("pot_radius_max_fraction", 0.50)
        self.declare_parameter("pot_radius_expected_fraction", 0.42)
        self.declare_parameter("pot_inner_radius_fraction", 0.75)
        self.declare_parameter("pot_center_penalty", 10.0)
        self.declare_parameter("soil_plant_exclusion_px", 5)
        self.declare_parameter("soil_min_pixels", 300)
        self.declare_parameter("soil_max_depth_mad_mm", 12.0)
        self.declare_parameter("soil_min_depth_mm", 150.0)
        self.declare_parameter("soil_max_depth_mm", 1300.0)

        self.pot_ransac_iterations = int(self.get_parameter("pot_ransac_iterations").value)
        self.pot_radius_min_fraction = float(self.get_parameter("pot_radius_min_fraction").value)
        self.pot_radius_max_fraction = float(self.get_parameter("pot_radius_max_fraction").value)
        self.pot_radius_expected_fraction = float(self.get_parameter("pot_radius_expected_fraction").value)
        self.pot_inner_radius_fraction = float(self.get_parameter("pot_inner_radius_fraction").value)
        self.pot_center_penalty = float(self.get_parameter("pot_center_penalty").value)
        self.soil_plant_exclusion_px = int(self.get_parameter("soil_plant_exclusion_px").value)
        self.soil_min_pixels = int(self.get_parameter("soil_min_pixels").value)
        self.soil_max_depth_mad_mm = float(self.get_parameter("soil_max_depth_mad_mm").value)
        self.soil_min_depth_mm = float(self.get_parameter("soil_min_depth_mm").value)
        self.soil_max_depth_mm = float(self.get_parameter("soil_max_depth_mm").value)

        self.lower_soil = np.array([5, 25, 20])
        self.upper_soil = np.array([30, 255, 210])

         # -------- Subscriptions and publishers --------
        self.state_sub = self.create_subscription(String,"/auto_state",self.cb_auto_state,10,)
        self.color_sub = self.create_subscription(Image,"/top_scan/color",self.cb_color,10)
        self.depth_sub = self.create_subscription(Image,"/top_scan/depth",self.cb_depth,10)
        self.camera_info_sub = self.create_subscription(CameraInfo,"/top_scan/camera_info",self.cb_camera_info,10)
        self.run_dir_sub = self.create_subscription(String,"/top_scan/run_dir",self.cb_run_dir,10)

        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.target_pub = self.create_publisher(PlantTargetArray,"/plant_row/targets",10)

    # -------- Callbacks --------
        
    def cb_color(self, msg):
        self.latest_color_msg = msg
        self.try_process_pending_scan()

    def cb_depth(self, msg):
        self.latest_depth_msg = msg
        self.try_process_pending_scan()

    def cb_camera_info(self, msg):
        self.latest_camera_info_msg = msg
        self.try_process_pending_scan()

    def cb_run_dir(self, msg):
        self.latest_run_dir = Path(msg.data)
        self.try_process_pending_scan()

    def try_process_pending_scan(self):
        if not self.pending_process:
            return

        required_messages = (
            self.latest_color_msg,
            self.latest_depth_msg,
            self.latest_camera_info_msg,
        )
        if any(message is None for message in required_messages):
            return

        self.pending_process = False
        self.process_latest_scan()

    def process_latest_scan(self):
        self.get_logger().info("Starting plant row coordinate calculation")

        try:
            color = self.bridge.imgmsg_to_cv2(self.latest_color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="passthrough")

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        camera_info = self.latest_camera_info_msg

        intrinsics = camera_info.k
        fx = float(intrinsics[0])
        fy = float(intrinsics[4])
        cx_intr = float(intrinsics[2])
        cy_intr = float(intrinsics[5])

        output_dir = self.latest_run_dir
        if output_dir is None:
            self.get_logger().warn("No run directory received. Saving outputs disabled.")

        self.process_live_frame(color, depth, fx, fy, cx_intr, cy_intr, output_dir)

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "plant_row_coordinates":
            return

        self.pending_process = True
        self.get_logger().info("Waiting for latest color/depth/camera_info before processing")
        self.try_process_pending_scan()

    @staticmethod
    def pixel_depth_to_3d(u, v, z, fx, fy, cx, cy):
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return round(x, 2), round(y, 2), round(z, 2)

    @staticmethod
    def get_depth_median_around_pixel(depth, u, v, window_size):
        half = window_size // 2
        h, w = depth.shape[:2]

        u1 = max(0, u - half)
        u2 = min(w, u + half + 1)
        v1 = max(0, v - half)
        v2 = min(h, v + half + 1)

        region = depth[v1:v2, u1:u2]

        valid = region[np.isfinite(region) & (region > 0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def get_top_point_from_contour(self,depth_full,contour_crop,measurement_mask_crop,x1,y1,x2,y2,):
        h_crop = y2 - y1
        w_crop = x2 - x1

        contour_mask_crop = np.zeros((h_crop, w_crop), dtype=np.uint8)
        cv2.drawContours(contour_mask_crop, [contour_crop], -1, 255, -1)

        depth_crop = depth_full[y1:y2, x1:x2]

        if (depth_crop.shape[:2] != contour_mask_crop.shape
            or measurement_mask_crop.shape != contour_mask_crop.shape):
            self.get_logger().warn("Depth, measurement-mask, and contour-mask shapes do not match.")
            return None

        plant_pixels = ((contour_mask_crop > 0)& (measurement_mask_crop > 0))
        plant_depth_values = depth_crop[plant_pixels]

        valid_depth = plant_depth_values[np.isfinite(plant_depth_values) & (plant_depth_values > 0)]

        if valid_depth.size == 0:
            return None

        top_threshold = np.percentile(valid_depth, self.top_percentile)

        top_mask_crop = (plant_pixels& (depth_crop > 0)& np.isfinite(depth_crop)& (depth_crop <= top_threshold))

        ys, xs = np.where(top_mask_crop)

        if xs.size == 0:
            return None

        # Keep the robust median depth for the arm target, but choose its image
        # location from an actual top-region plant pixel. The arithmetic mean
        # of separated leaf/flower clusters can fall into an empty gap.
        top_center_x = float(np.mean(xs))
        top_center_y = float(np.mean(ys))
        distances_sq = (
            (xs.astype(np.float64) - top_center_x) ** 2
            + (ys.astype(np.float64) - top_center_y) ** 2
        )
        representative_index = int(np.argmin(distances_sq))
        top_u_crop = int(xs[representative_index])
        top_v_crop = int(ys[representative_index])
        top_depth = float(np.median(depth_crop[ys, xs]))

        top_u_full = top_u_crop + x1
        top_v_full = top_v_crop + y1

        return top_u_full, top_v_full, top_depth, top_u_crop, top_v_crop

    @staticmethod
    def calculate_contour_radius_mm(depth_mm, area, fx, fy):
        if depth_mm is None or depth_mm <= 0:
            return None

        mm_per_px_x = depth_mm / fx
        mm_per_px_y = depth_mm / fy

        area_mm2 = area * mm_per_px_x * mm_per_px_y
        radius_mm = np.sqrt(area_mm2 / np.pi)

        return int(radius_mm)

    @staticmethod
    def depth_to_mm(depth):
        if np.issubdtype(depth.dtype, np.integer):
            return depth.astype(np.float32)
        return depth.astype(np.float32) * 1000.0

    def assign_pot_slot(self, center_x_crop, crop_width):
        """Return the fixed pot ID nearest to a detected plant center."""
        if crop_width <= 0:
            return None, None

        center_fraction = float(center_x_crop) / float(crop_width)
        errors = [
            abs(center_fraction - slot_fraction)
            for slot_fraction in self.pot_slot_x_fractions
        ]
        slot_index = int(np.argmin(errors))
        slot_error = float(errors[slot_index])

        if slot_error > self.max_pot_slot_error_fraction:
            return None, slot_error

        return slot_index + 1, slot_error

    @staticmethod
    def circle_from_three_points(points):
        """Return (center_x, center_y, radius) for three non-collinear points."""
        p1, p2, p3 = points.astype(np.float64)
        matrix = 2.0 * np.array((p2 - p1, p3 - p1))
        vector = np.array(
            (
                np.dot(p2, p2) - np.dot(p1, p1),
                np.dot(p3, p3) - np.dot(p1, p1),
            )
        )
        try:
            center = np.linalg.solve(matrix, vector)
        except np.linalg.LinAlgError:
            return None

        radius = float(np.linalg.norm(p1 - center))
        if not np.isfinite(radius):
            return None
        return float(center[0]), float(center[1]), radius

    def detect_pot_rim_ransac(
        self, edge_mask, expected_x, expected_y, slot_width, seed
    ):
        """Fit a pot rim near an expected location using constrained circle RANSAC."""
        height, width = edge_mask.shape
        search_radius = 0.55 * slot_width
        x1 = max(0, int(expected_x - search_radius))
        x2 = min(width, int(expected_x + search_radius) + 1)
        y1 = max(0, int(expected_y - search_radius))
        y2 = min(height, int(expected_y + search_radius) + 1)

        ys, xs = np.where(edge_mask[y1:y2, x1:x2] > 0)
        edge_points = np.column_stack((xs + x1, ys + y1))
        if edge_points.shape[0] < 3:
            return None

        radius_min = self.pot_radius_min_fraction * slot_width
        radius_max = self.pot_radius_max_fraction * slot_width
        expected_radius = self.pot_radius_expected_fraction * slot_width
        max_center_error = 0.45 * slot_width
        rng = np.random.default_rng(seed)
        best = None

        for _ in range(max(1, self.pot_ransac_iterations)):
            sample_indices = rng.choice(edge_points.shape[0], 3, replace=False)
            circle = self.circle_from_three_points(edge_points[sample_indices])
            if circle is None:
                continue

            center_x, center_y, radius = circle
            if not radius_min <= radius <= radius_max:
                continue
            if (
                abs(center_x - expected_x) > max_center_error
                or abs(center_y - expected_y) > max_center_error
            ):
                continue

            radial_error = np.abs(
                np.hypot(
                    edge_points[:, 0] - center_x,
                    edge_points[:, 1] - center_y,
                )
                - radius
            )
            supporters = radial_error <= 2.5
            support_count = int(np.count_nonzero(supporters))
            if support_count == 0:
                continue

            angles = np.arctan2(
                edge_points[supporters, 1] - center_y,
                edge_points[supporters, 0] - center_x,
            )
            angle_bins = np.clip(
                ((angles + np.pi) / (2.0 * np.pi) * 36).astype(int),
                0,
                35,
            )
            covered_bins = int(np.unique(angle_bins).size)
            coverage = covered_bins / 36.0
            score = (
                support_count
                + covered_bins * 8.0
                - 3.0 * abs(radius - expected_radius)
                - self.pot_center_penalty
                * np.hypot(center_x - expected_x, center_y - expected_y)
            )

            if best is None or score > best["score"]:
                best = {
                    "center_x": center_x,
                    "center_y": center_y,
                    "radius": radius,
                    "support_count": support_count,
                    "coverage": coverage,
                    "score": score,
                }

        if best is None:
            return None

        # A partial rim is acceptable, but random leaf and mesh edges should
        # not be promoted into a high-confidence pot localization.
        best["confidence"] = (
            "high"
            if best["coverage"] >= 0.65 and best["support_count"] >= 120
            else "medium"
            if best["coverage"] >= 0.40 and best["support_count"] >= 70
            else "low"
        )
        return best

    def build_soil_masks(self, crop, depth_crop, measurement_mask, results):
        """Localize pot interiors and produce a conservative soil candidate mask."""
        height, width = crop.shape[:2]
        pot_interior_mask = np.zeros((height, width), dtype=np.uint8)
        soil_mask = np.zeros((height, width), dtype=np.uint8)
        rim_overlay = crop.copy()

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
        edge_mask = cv2.Canny(gray, 50, 130)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        soil_color_mask = cv2.inRange(hsv, self.lower_soil, self.upper_soil)
        valid_depth = (
            np.isfinite(depth_crop)
            & (depth_crop >= self.soil_min_depth_mm)
            & (depth_crop <= self.soil_max_depth_mm)
        )

        exclusion_size = max(1, self.soil_plant_exclusion_px)
        exclusion_kernel = np.ones(
            (exclusion_size, exclusion_size), dtype=np.uint8
        )
        plant_exclusion = cv2.dilate(
            measurement_mask, exclusion_kernel, iterations=1
        )
        slot_width = width / float(self.pot_count)

        for row in results:
            plant_id = int(row["plant_id"])
            expected_x = float(row["center_u_crop"])
            expected_y = float(row["center_v_crop"])
            rim = self.detect_pot_rim_ransac(
                edge_mask,
                expected_x,
                expected_y,
                slot_width,
                seed=plant_id,
            )

            if rim is None or rim["confidence"] == "low":
                center_x = expected_x
                center_y = expected_y
                radius = self.pot_radius_expected_fraction * slot_width
                method = "plant_center_fallback"
                rim_confidence = "low"
            else:
                center_x = rim["center_x"]
                center_y = rim["center_y"]
                radius = rim["radius"]
                method = "circle_ransac"
                rim_confidence = rim["confidence"]

            inner_radius = max(1, int(round(radius * self.pot_inner_radius_fraction)))
            individual_pot_mask = np.zeros_like(pot_interior_mask)
            cv2.circle(
                individual_pot_mask,
                (int(round(center_x)), int(round(center_y))),
                inner_radius,
                255,
                -1,
            )
            pot_interior_mask = cv2.bitwise_or(
                pot_interior_mask, individual_pot_mask
            )

            individual_soil = (
                (individual_pot_mask > 0)
                & (plant_exclusion == 0)
                & (soil_color_mask > 0)
                & valid_depth
            )
            soil_values = depth_crop[individual_soil]
            soil_median_depth = None
            soil_depth_mad = None
            soil_confidence = "insufficient"

            if soil_values.size > 0:
                soil_median_depth = float(np.median(soil_values))
                soil_depth_mad = float(
                    np.median(np.abs(soil_values - soil_median_depth))
                )
                depth_inliers = (
                    np.abs(depth_crop - soil_median_depth)
                    <= max(3.0 * soil_depth_mad, 5.0)
                )
                individual_soil &= depth_inliers
                soil_values = depth_crop[individual_soil]

                if soil_values.size >= self.soil_min_pixels:
                    soil_confidence = (
                        "high"
                        if soil_depth_mad <= self.soil_max_depth_mad_mm
                        and rim_confidence != "low"
                        else "medium"
                    )

            soil_mask[individual_soil] = 255
            row["pot_rim"] = {
                "center_u_crop": round(center_x, 2),
                "center_v_crop": round(center_y, 2),
                "radius_px": round(radius, 2),
                "method": method,
                "confidence": rim_confidence,
                "coverage": (
                    round(float(rim["coverage"]), 3) if rim is not None else None
                ),
            }
            row["soil"] = {
                "visible_pixels": int(soil_values.size),
                "median_camera_depth_mm": (
                    round(float(np.median(soil_values)), 2)
                    if soil_values.size > 0
                    else None
                ),
                "depth_mad_mm": (
                    round(soil_depth_mad, 2)
                    if soil_depth_mad is not None
                    else None
                ),
                "confidence": soil_confidence,
            }

            color = (
                (0, 255, 0)
                if rim_confidence == "high"
                else (0, 200, 255)
                if rim_confidence == "medium"
                else (0, 0, 255)
            )
            cv2.circle(
                rim_overlay,
                (int(round(center_x)), int(round(center_y))),
                int(round(radius)),
                color,
                2,
            )
            cv2.putText(
                rim_overlay,
                f"P{plant_id} {rim_confidence}",
                (int(round(center_x)) - 35, int(round(center_y))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        soil_kernel = np.ones((3, 3), dtype=np.uint8)
        soil_mask = cv2.morphologyEx(soil_mask, cv2.MORPH_OPEN, soil_kernel)
        return pot_interior_mask, soil_mask, rim_overlay

    def save_plant_results_to_metadata(self, output_dir, results):
        if output_dir is None:
            self.get_logger().warn("No run directory received. Plant metadata not saved.")
            return

        metadata_path = output_dir / "metadata.yaml"

        if metadata_path.exists():
            with open(metadata_path, "r", encoding="utf-8") as f:
                metadata = yaml.safe_load(f) or {}
        else:
            metadata = {}

        metadata["plants"] = [
            {
                "plant_id": int(row["plant_id"]),
                "pot_rim": row.get("pot_rim"),
                "soil": row.get("soil"),
                "area_px": (
                    float(row["area_px"])
                    if row["area_px"] is not None
                    else None
                ),
                "center": {
                    "x_mm": row["center_x"],
                    "y_mm": row["center_y"],
                    "z_mm": row["center_z"],
                },
                "top": {
                    "x_mm": row["top_x"],
                    "y_mm": row["top_y"],
                    "z_mm": row["top_z"],
                },
                "target": {
                    "x_mm": row["target_x"],
                    "y_mm": row["target_y"],
                    "z_mm": row["target_z"],
                },
                "radius_mm": row["radius_mm"],
            }
            for row in results
        ]

        with open(metadata_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

    def process_live_frame(self, img, depth, fx, fy, cx_intr, cy_intr, output_dir=None):
        h, w = img.shape[:2]
        x1_clamped = max(0, min(self.x1, w))
        x2_clamped = max(0, min(self.x2, w))
        y1_clamped = max(0, min(self.y1, h))
        y2_clamped = max(0, min(self.y2, h))

        if x1_clamped >= x2_clamped or y1_clamped >= y2_clamped:
            self.get_logger().error("Invalid crop bounds for plant row calculation.")
            return

        crop = img[y1_clamped:y2_clamped, x1_clamped:x2_clamped]
        depth = self.depth_to_mm(depth)
        depth_crop = depth[y1_clamped:y2_clamped, x1_clamped:x2_clamped]

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        depth_mask = (
            np.isfinite(depth_crop)
            & (depth_crop >= self.min_depth_mm)
            & (depth_crop <= self.max_depth_mm)
        )

        green_mask[~depth_mask] = 0
        yellow_mask[~depth_mask] = 0
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        green_clean = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_clean = cv2.morphologyEx(green_clean, cv2.MORPH_CLOSE, kernel)
        yellow_kernel = np.ones((2, 2), np.uint8)
        yellow_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, yellow_kernel)

        # This conservative mask is the only mask allowed to contribute depth
        # measurements. Dilation below is used strictly to group fragmented
        # plant parts into one contour.
        measurement_mask = cv2.bitwise_or(green_clean, yellow_clean)
        grouping_mask = cv2.morphologyEx(measurement_mask, cv2.MORPH_CLOSE, kernel)
        grouping_mask = cv2.dilate(grouping_mask, kernel, iterations=self.dilate_itr)

        segmented = cv2.bitwise_and(crop, crop, mask=measurement_mask)
        contours, _ = cv2.findContours(grouping_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detection = crop.copy()

        valid_contours = sorted(
            (cnt for cnt in contours if cv2.contourArea(cnt) >= self.min_area),
            key=cv2.contourArea,
            reverse=True,
        )

        plant_records = []

        for cnt in valid_contours:
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)

            if M["m00"] == 0:
                continue

            center_x_crop = int(M["m10"] / M["m00"])
            center_y_crop = int(M["m01"] / M["m00"])

            center_x_full = center_x_crop + x1_clamped
            center_y_full = center_y_crop + y1_clamped

            pot_slot_id, pot_slot_error = self.assign_pot_slot(center_x_crop, crop.shape[1])

            if pot_slot_id is None:
                self.get_logger().warn(
                    "Ignoring plant contour because its center does not match "f"error={pot_slot_error:.3f}).")
                continue

            center_depth = self.get_depth_median_around_pixel(depth,center_x_full,center_y_full,self.center_window_size)
            top_result = self.get_top_point_from_contour(
                depth,
                cnt,
                measurement_mask,
                x1_clamped,
                y1_clamped,
                x2_clamped,
                y2_clamped,
            )

            top_3d = None
            top_depth = None

            if top_result is not None:
                top_u_full, top_v_full, top_depth, top_u_crop, top_v_crop = top_result

                top_3d = self.pixel_depth_to_3d(top_u_full,top_v_full,top_depth,fx,fy,cx_intr,cy_intr)

                cv2.circle(detection, (top_u_crop, top_v_crop), 8, (255, 0, 0), -1)
                cv2.putText(detection,"TOP",(top_u_crop + 10, top_v_crop),cv2.FONT_HERSHEY_SIMPLEX,0.55,(255, 0, 0),2)

            cv2.circle(detection, (center_x_crop, center_y_crop), 6, (0, 0, 255), -1)

            x, y, bw, bh = cv2.boundingRect(cnt)
            cv2.rectangle(detection, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

            depth_for_center_xy = center_depth if center_depth is not None else top_depth

            radius_mm = self.calculate_contour_radius_mm(depth_for_center_xy,area,fx,fy)

            center_xy_3d = None

            if depth_for_center_xy is not None:
                center_xy_3d = self.pixel_depth_to_3d(center_x_full,center_y_full,depth_for_center_xy,fx,fy,cx_intr,cy_intr)

            row = {
                "plant_id": pot_slot_id,
                "pot_slot_error": pot_slot_error,
                "center_u_crop": center_x_crop,
                "center_v_crop": center_y_crop,
                "area_px": area,

                "center_x": center_xy_3d[0] if center_xy_3d else None,
                "center_y": center_xy_3d[1] if center_xy_3d else None,
                "center_z": round(center_depth, 2) if center_depth is not None else None,

                "top_x": top_3d[0] if top_3d else None,
                "top_y": top_3d[1] if top_3d else None,
                "top_z": top_3d[2] if top_3d else None,

                "target_x": center_xy_3d[0] if center_xy_3d else None,
                "target_y": center_xy_3d[1] if center_xy_3d else None,
                "target_z": top_3d[2] if top_3d else None,

                "radius_mm": radius_mm,
            }

            plant_records.append(row)

        # A fragmented mask can occasionally produce multiple contours in one
        # slot. Keep the largest contour, but never renumber another pot.
        records_by_slot = {}
        for row in plant_records:
            slot_id = row["plant_id"]
            existing = records_by_slot.get(slot_id)
            if existing is None or row["area_px"] > existing["area_px"]:
                if existing is not None:
                    self.get_logger().warn(f"Multiple plant contours matched pot slot {slot_id}; ""keeping the largest contour.")
                records_by_slot[slot_id] = row

        results = [records_by_slot[slot_id] for slot_id in sorted(records_by_slot)]

        pot_interior_mask, soil_candidate_mask, pot_rim_detection = (
            self.build_soil_masks(crop, depth_crop, measurement_mask, results)
        )

        target_msg = PlantTargetArray()
        target_msg.run_dir = str(output_dir) if output_dir is not None else ""

        for row in results:
            target_values = (row["target_x"], row["target_y"], row["target_z"])
            if any(value is None for value in target_values):
                continue

            t = PlantTarget()
            t.plant_id = int(row["plant_id"])
            t.target_x = float(row["target_x"]) / 1000.0
            t.target_y = float(row["target_y"]) / 1000.0
            t.target_z = float(row["target_z"]) / 1000.0

            t.radius_m = (
                float(row["radius_mm"]) / 1000.0
                if row["radius_mm"] is not None
                else 0.05
            )

            target_msg.targets.append(t)

        self.target_pub.publish(target_msg)
        run_name = (output_dir.name if output_dir is not None else "current frame")
        self.get_logger().info(f"{run_name}: detected {len(results)} plants")

        if output_dir is not None:
            cv2.imwrite(str(output_dir / "crop.png"), crop)
            cv2.imwrite(str(output_dir / "segmented_result.png"), segmented)
            cv2.imwrite(str(output_dir / "measurement_mask.png"), measurement_mask)
            cv2.imwrite(str(output_dir / "grouping_mask.png"), grouping_mask)
            cv2.imwrite(str(output_dir / "pot_interior_mask.png"), pot_interior_mask)
            cv2.imwrite(str(output_dir / "soil_candidate_mask.png"), soil_candidate_mask)
            cv2.imwrite(str(output_dir / "pot_rim_detection.png"), pot_rim_detection)
            cv2.imwrite(str(output_dir / "detection.png"), detection)
            self.save_plant_results_to_metadata(output_dir, results)

        self.pub_auto_state_cmd.publish(String(data="individual_plant_scan"))


def main(args=None):
    rclpy.init(args=args)
    node = PlantCoordinateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
