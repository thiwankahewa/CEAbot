#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime
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

        # -------- States and variables --------
        
        self.bridge = CvBridge()

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_camera_info_msg = None
        self.latest_run_dir = None
        self.pending_process = False

        # tuned using src/CEAbot_phenotyping/test/crop_selector.py
        self.x1 = 176
        self.y1 = 379
        self.x2 = 1124
        self.y2 = 631

        # tuned using src/CEAbot_phenotyping/test/plant_segmentation_tuner.py
        self.lower_green = np.array([35, 27, 10])
        self.upper_green = np.array([95, 255, 255])
        self.lower_yellow = np.array([20, 80, 100])
        self.upper_yellow = np.array([38, 255, 255])
        self.min_area = 2000
        self.kernel_size = 4
        self.top_percentile = 5.0
        self.center_window_size = 9
        self.min_depth_mm = 150.0
        self.max_depth_mm = 900.0
        self.dilate_itr = 2

        # A top scan cannot see foliage hidden by the arm base.  Keep the
        # expected pot lattice occupied in those blind areas instead of
        # interpreting a missing contour as free space.
        self.declare_parameter("infer_occluded_adjacent_plants", True)
        self.declare_parameter("inferred_obstacle_radius_margin_m", 0.08)
        self.declare_parameter("inferred_obstacle_default_radius_m", 0.13)
        self.infer_occluded_adjacent_plants = bool( self.get_parameter("infer_occluded_adjacent_plants").value)
        self.inferred_obstacle_radius_margin_mm = 1000.0 * float( self.get_parameter("inferred_obstacle_radius_margin_m").value)
        self.inferred_obstacle_default_radius_mm = 1000.0 * float( self.get_parameter("inferred_obstacle_default_radius_m").value)

        # convention: pot 1 is the rightmost pot and IDs increase to the left.
        self.declare_parameter("pot_count", 4)
        self.pot_count = int(self.get_parameter("pot_count").value)
        if self.pot_count < 1:
            raise ValueError("pot_count must be at least 1")

        # divide the plant row length to create pot slots 
        self.pot_slot_x_fractions = tuple((self.pot_count - slot_index - 0.5) / self.pot_count for slot_index in range(self.pot_count))

         # -------- Subscriptions --------

        self.state_sub = self.create_subscription(String,"/auto_state",self.cb_auto_state,10,)
        self.color_sub = self.create_subscription(Image,"/top_scan/color",self.cb_color,10)
        self.depth_sub = self.create_subscription(Image,"/top_scan/depth",self.cb_depth,10)
        self.camera_info_sub = self.create_subscription(CameraInfo,"/top_scan/camera_info",self.cb_camera_info,10)
        self.run_dir_sub = self.create_subscription(String,"/top_scan/run_dir",self.cb_run_dir,10)

        # -------- publishers --------

        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.target_pub = self.create_publisher(PlantTargetArray,"/plant_row/targets",10)
        self.obstacle_pub = self.create_publisher(PlantTargetArray,"/plant_row/obstacles",10)

    # -------- Callback functions --------

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "plant_row_coordinates":
            return

        self.pending_process = True
        self.get_logger().info("Waiting for latest color/depth/camera_info before processing")
        self.try_process_pending_scan()
        
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

        required_messages = (self.latest_color_msg,self.latest_depth_msg,self.latest_camera_info_msg,)
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

    # -------- Helper Functions --------

    @staticmethod
    def depth_to_mm(depth):
        if np.issubdtype(depth.dtype, np.integer):
            return depth.astype(np.float32)
        return depth.astype(np.float32) * 1000.0

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

    @staticmethod
    def pixel_depth_to_3d(u, v, z, fx, fy, cx, cy):
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return round(x, 2), round(y, 2), round(z, 2)

    @staticmethod
    def calculate_contour_radius_mm(depth_mm, area, fx, fy):
        if depth_mm is None or depth_mm <= 0:
            return None

        mm_per_px_x = depth_mm / fx
        mm_per_px_y = depth_mm / fy

        area_mm2 = area * mm_per_px_x * mm_per_px_y
        radius_mm = np.sqrt(area_mm2 / np.pi)

        return int(radius_mm)


    def get_top_depth_from_contour(self,depth_full,contour_crop,measurement_mask_crop,x1,y1,x2,y2,):

        contour_mask_crop = np.zeros((y2 - y1, x2 - x1), dtype=np.uint8)
        cv2.drawContours(contour_mask_crop, [contour_crop], -1, 255, -1)

        depth_crop = depth_full[y1:y2, x1:x2]

        if (depth_crop.shape[:2] != contour_mask_crop.shape or measurement_mask_crop.shape != contour_mask_crop.shape):
            self.get_logger().warn("Depth, measurement-mask, and contour-mask shapes do not match.")
            return None

        plant_pixels = ((contour_mask_crop > 0) & (measurement_mask_crop > 0) & np.isfinite(depth_crop) & (depth_crop > 0))
        valid_depth = depth_crop[plant_pixels]

        if valid_depth.size == 0:
            return None

        top_threshold = np.percentile(valid_depth, self.top_percentile)
        top_depth_values = valid_depth[valid_depth <= top_threshold]

        if top_depth_values.size == 0:
            return None

        return float(np.median(top_depth_values))

    def detect_full_frame_obstacles(self, img, depth, fx, fy, cx, cy, selected_records, crop_bounds):
        """Detect plant envelopes in the uncropped image.
        Selected-row records are retained because their slot-wise segmentation
        is more reliable when neighbouring foliage touches.  Extra contours
        whose centres lie outside the crop supply adjacent-row obstacles.
        """

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        valid_depth = ( np.isfinite(depth) & (depth >= self.min_depth_mm) & (depth <= self.max_depth_mm))
        green[~valid_depth] = 0
        yellow[~valid_depth] = 0

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        yellow = cv2.morphologyEx( yellow, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        measurement = cv2.bitwise_or(green, yellow)
        grouping = cv2.morphologyEx(measurement, cv2.MORPH_CLOSE, kernel)
        grouping = cv2.dilate(grouping, kernel, iterations=self.dilate_itr)
        x1, y1, x2, y2 = crop_bounds
        # Use the selected row's known pot spacing to split adjacent-row
        # foliage before contour extraction.  Without these vertical strips,
        # touching plants in an adjacent row become one very large obstacle.
        slot_width = float(x2 - x1) / float(self.pot_count)
        slot_edges = [x1]
        while slot_edges[0] > 0:
            slot_edges.insert(0, max(0, int(round(slot_edges[0] - slot_width))))
        while slot_edges[-1] < img.shape[1]:
            slot_edges.append( min(img.shape[1], int(round(slot_edges[-1] + slot_width))))

        contour_candidates = []
        obstacle_zones = (("above", 0, y1), ("below", y2, img.shape[0]))
        for zone_name, zone_top, zone_bottom in obstacle_zones:
            if zone_bottom - zone_top < 2:
                continue
            for strip_index, (strip_left, strip_right) in enumerate( zip(slot_edges[:-1], slot_edges[1:]) ):
                strip = grouping[zone_top:zone_bottom, strip_left:strip_right]
                strip_contours, _ = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid = [
                    contour for contour in strip_contours
                    if cv2.contourArea(contour) >= self.min_area
                ]
                if not valid:
                    continue
                contour = max(valid, key=cv2.contourArea).copy()
                contour[:, 0, 0] += strip_left
                contour[:, 0, 1] += zone_top
                contour_candidates.append( (zone_name, strip_index, strip_left, strip_right, contour) )

        obstacles = [dict(record) for record in selected_records]
        overlay = img.copy()
        for record in selected_records:
            u = int(record["center_u_crop"] + x1)
            v = int(record["center_v_crop"] + y1)
            cv2.circle(overlay, (u, v), 7, (255, 0, 255), -1)

        next_id = 1001
        detected_slots = set()
        detected_by_zone = {zone_name: [] for zone_name, _, _ in obstacle_zones}
        for zone_name, strip_index, strip_left, strip_right, contour in contour_candidates:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            # The four selected plants already have better, slot-separated
            # records.  Only add full-frame contour centres outside that ROI.
            if x1 <= u < x2 and y1 <= v < y2:
                continue

            contour_mask = np.zeros(depth.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            plant_pixels = ( (contour_mask > 0) & (measurement > 0) & np.isfinite(depth) & (depth > 0))
            values = depth[plant_pixels]
            if values.size == 0:
                continue
            threshold = np.percentile(values, self.top_percentile)
            top_values = values[values <= threshold]
            if top_values.size == 0:
                continue
            top_depth = float(np.median(top_values))
            center_depth = self.get_depth_median_around_pixel( depth, u, v, self.center_window_size)
            xy_depth = center_depth if center_depth is not None else top_depth
            xyz = self.pixel_depth_to_3d(u, v, xy_depth, fx, fy, cx, cy)
            radius_mm = self.calculate_contour_radius_mm(xy_depth, area, fx, fy)
            if radius_mm is None:
                continue

            obstacles.append({
                "plant_id": next_id,
                "target_x": xyz[0],
                "target_y": xyz[1],
                "target_z": top_depth,
                "radius_mm": radius_mm,
                "inferred": False,
            })
            detected_slots.add((zone_name, strip_index))
            detected_by_zone[zone_name].append( {"u": u, "v": v, "xy_depth": xy_depth, "top_depth": top_depth} )
            (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(contour)
            cv2.circle( overlay, (int(circle_x), int(circle_y)), int(circle_radius), (0, 165, 255), 2)
            cv2.putText( overlay, f"O{next_id}", (u, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)
            next_id += 1

        if self.infer_occluded_adjacent_plants:
            selected_v = [
                float(record["center_v_crop"] + y1)
                for record in selected_records
                if record.get("center_v_crop") is not None
            ]
            selected_xy_depth = [
                float(record["target_z"])
                for record in selected_records
                if record.get("target_z") is not None
            ]
            selected_top_depth = selected_xy_depth
            selected_radius = [
                float(record["radius_mm"])
                for record in selected_records
                if record.get("radius_mm") is not None
            ]

            current_row_v = float(np.median(selected_v)) if selected_v else (y1 + y2) / 2.0
            default_depth = (
                float(np.median(selected_xy_depth)) if selected_xy_depth
                else (self.min_depth_mm + self.max_depth_mm) / 2.0
            )
            default_top_depth = (
                float(np.median(selected_top_depth)) if selected_top_depth
                else default_depth
            )
            nominal_radius = (
                float(np.median(selected_radius)) if selected_radius
                else self.inferred_obstacle_default_radius_mm
            )
            inferred_radius = max(
                self.inferred_obstacle_default_radius_mm,
                nominal_radius + self.inferred_obstacle_radius_margin_mm,
            )

            zone_row_v = {}
            for zone_name, _, _ in obstacle_zones:
                observations = detected_by_zone[zone_name]
                if observations:
                    zone_row_v[zone_name] = float( np.median([record["v"] for record in observations]) )

            # If the arm hides an entire adjacent row, mirror the visible
            # opposite row around the selected row.  This uses row spacing in
            # image space and also works when the inferred centre lies just
            # outside the image.
            if "above" not in zone_row_v and "below" in zone_row_v:
                zone_row_v["above"] = 2.0 * current_row_v - zone_row_v["below"]
            if "below" not in zone_row_v and "above" in zone_row_v:
                zone_row_v["below"] = 2.0 * current_row_v - zone_row_v["above"]
            # One clipped plant at an image edge gives a biased row centre.
            # Prefer the mirror of a well-observed opposite row in that case.
            if (len(detected_by_zone["below"]) < 2 and len(detected_by_zone["above"]) >= 2):
                zone_row_v["below"] = ( 2.0 * current_row_v - zone_row_v["above"])
            if (len(detected_by_zone["above"]) < 2 and len(detected_by_zone["below"]) >= 2):
                zone_row_v["above"] = ( 2.0 * current_row_v - zone_row_v["below"] )

            for zone_name, _, _ in obstacle_zones:
                if zone_name not in zone_row_v:
                    continue
                observations = detected_by_zone[zone_name]
                row_depth = (
                    float(np.median([record["xy_depth"] for record in observations]))
                    if observations else default_depth
                )
                row_top_depth = (
                    float(np.median([record["top_depth"] for record in observations]))
                    if observations else default_top_depth
                )
                v = int(round(zone_row_v[zone_name]))

                for strip_index, (strip_left, strip_right) in enumerate( zip(slot_edges[:-1], slot_edges[1:]) ):
                    # Only the four bench pot columns are guaranteed. The
                    # extra edge strips exist solely to catch visible foliage
                    # and must not create imaginary pots beyond the bench.
                    strip_center = (strip_left + strip_right) / 2.0
                    if not (x1 <= strip_center < x2):
                        continue
                    if (zone_name, strip_index) in detected_slots:
                        continue
                    u = int(round(strip_center))
                    xyz = self.pixel_depth_to_3d( u, v, row_depth, fx, fy, cx, cy)
                    obstacles.append({
                        "plant_id": next_id,
                        "target_x": xyz[0],
                        "target_y": xyz[1],
                        "target_z": row_top_depth,
                        "radius_mm": inferred_radius,
                        "inferred": True,
                    })
                    cv2.circle(overlay, (u, max(0, min(img.shape[0] - 1, v))), 12, (0, 0, 255), 3)
                    cv2.putText( overlay, f"I{next_id}", (u, max(15, min(img.shape[0] - 5, v))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA,)
                    next_id += 1

        return obstacles, overlay


    # -------- Main Processing Function --------

    def process_live_frame(self, img, depth, fx, fy, cx_intr, cy_intr, output_dir=None):
            # crop images
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
    
            depth_mask = (np.isfinite(depth_crop) & (depth_crop >= self.min_depth_mm) & (depth_crop <= self.max_depth_mm))
    
            green_mask[~depth_mask] = 0
            yellow_mask[~depth_mask] = 0
            kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
            green_clean = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
            green_clean = cv2.morphologyEx(green_clean, cv2.MORPH_CLOSE, kernel)
            yellow_kernel = np.ones((2, 2), np.uint8)
            yellow_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, yellow_kernel)
    
            measurement_mask = cv2.bitwise_or(green_clean, yellow_clean)
            segmented = cv2.bitwise_and(crop, crop, mask=measurement_mask)
            detection = crop.copy()
            grouping_mask = np.zeros_like(measurement_mask)
            slot_contours = []
            slot_width = crop.shape[1] / float(self.pot_count)

            for slot_index in range(self.pot_count):
                slot_left = int(round(slot_index * slot_width))
                slot_right = int(round((slot_index + 1) * slot_width))
                if slot_index == self.pot_count - 1:
                    slot_right = crop.shape[1]

                slot_measurement = measurement_mask[:, slot_left:slot_right]
                slot_grouping = cv2.morphologyEx(slot_measurement, cv2.MORPH_CLOSE, kernel)
                slot_grouping = cv2.dilate( slot_grouping, kernel, iterations=self.dilate_itr)
                grouping_mask[:, slot_left:slot_right] = slot_grouping

                if slot_right < crop.shape[1]:
                    cv2.line(detection, (slot_right, 0), (slot_right, crop.shape[0] - 1), (255, 180, 0), 1, )

                contours, _ = cv2.findContours( slot_grouping, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
                valid_contours = [
                    contour
                    for contour in contours
                    if cv2.contourArea(contour) >= self.min_area
                ]
                if not valid_contours:
                    continue

                contour = max(valid_contours, key=cv2.contourArea)
                contour = contour.copy()
                contour[:, 0, 0] += slot_left

                # Pot IDs increase from right to left.
                plant_id = self.pot_count - slot_index
                slot_contours.append((plant_id, contour))


            # Keep the saved diagnostic mask visually separated at slot
            # boundaries as well. Contours above were already extracted from
            # their individual slots.
            for boundary_index in range(1, self.pot_count):
                boundary_x = int(round(boundary_index * slot_width))
                grouping_mask[:, max(0, boundary_x - 1):boundary_x + 1] = 0
    
            plant_records = []
    
            for pot_slot_id, cnt in slot_contours:
                area = cv2.contourArea(cnt)
                M = cv2.moments(cnt)
    
                if M["m00"] == 0:
                    continue
    
                center_x_crop = int(M["m10"] / M["m00"])    # center of mass in the cropped image
                center_y_crop = int(M["m01"] / M["m00"])    
    
                center_x_full = center_x_crop + x1_clamped  # center of mass in the full image
                center_y_full = center_y_crop + y1_clamped
    
                expected_center_fraction = self.pot_slot_x_fractions[pot_slot_id - 1]
                center_fraction = float(center_x_crop) / float(crop.shape[1])
                pot_slot_error = abs(center_fraction - expected_center_fraction)
    
                center_depth = self.get_depth_median_around_pixel(depth,center_x_full,center_y_full,self.center_window_size)
                top_depth = self.get_top_depth_from_contour(depth,cnt,measurement_mask,x1_clamped,y1_clamped,x2_clamped,y2_clamped,)
    
                cv2.circle(detection, (center_x_crop, center_y_crop), 6, (0, 0, 255), -1)
    
                x, y, bw, bh = cv2.boundingRect(cnt)
                cv2.rectangle(detection, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(cnt)
                cv2.circle(detection,(int(round(circle_x)), int(round(circle_y))),int(round(circle_radius)),(255, 0, 255),2, )
                cv2.putText(detection,f"P{pot_slot_id}",(x, max(15, y - 5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 0, 255), 1, cv2.LINE_AA,)
    
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
                    "target_x": center_xy_3d[0] if center_xy_3d else None,
                    "target_y": center_xy_3d[1] if center_xy_3d else None,
                    "target_z": top_depth,
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

            obstacle_records, obstacle_detection = self.detect_full_frame_obstacles(img, depth, fx, fy, cx_intr, cy_intr, results, (x1_clamped, y1_clamped, x2_clamped, y2_clamped), )

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
                t.radius_m = (float(row["radius_mm"]) / 1000.0 if row["radius_mm"] is not None else 0.05)
                target_msg.targets.append(t)
    
            self.target_pub.publish(target_msg)

            obstacle_msg = PlantTargetArray()
            obstacle_msg.run_dir = target_msg.run_dir
            for row in obstacle_records:
                values = (row.get("target_x"), row.get("target_y"), row.get("target_z"))
                if any(value is None for value in values):
                    continue
                obstacle = PlantTarget()
                obstacle.plant_id = int(row["plant_id"])
                obstacle.target_x = float(row["target_x"]) / 1000.0
                obstacle.target_y = float(row["target_y"]) / 1000.0
                obstacle.target_z = float(row["target_z"]) / 1000.0
                obstacle.radius_m = (float(row["radius_mm"]) / 1000.0 if row.get("radius_mm") is not None else 0.05)
                obstacle_msg.targets.append(obstacle)
            self.obstacle_pub.publish(obstacle_msg)
            inferred_count = sum( 1 for row in obstacle_records if row.get("inferred", False))
            run_name = (output_dir.name if output_dir is not None else "current frame")
            self.get_logger().info( f"{run_name}: detected {len(results)} scan targets and " f"{len(obstacle_msg.targets)} full-frame plant obstacles " f"({inferred_count} inferred)")
            self.pub_auto_state_cmd.publish(String(data="individual_plant_scan"))
    
            if output_dir is not None:
                self.save_plant_results_to_metadata(output_dir, results)
                cv2.imwrite(str(output_dir / "crop.png"), crop)
                cv2.imwrite(str(output_dir / "segmented_result.png"), segmented)
                cv2.imwrite(str(output_dir / "measurement_mask.png"), measurement_mask)
                cv2.imwrite(str(output_dir / "grouping_mask.png"), grouping_mask)
                cv2.imwrite(str(output_dir / "detection.png"), detection)
                cv2.imwrite(str(output_dir / "full_obstacle_detection.png"),obstacle_detection,)

    def save_plant_results_to_metadata(self, output_dir, results):

        metadata_path = output_dir / "metadata.yaml"

        if metadata_path.exists():
            with open(metadata_path, "r", encoding="utf-8") as f:
                metadata = yaml.safe_load(f) or {}
        else:
            metadata = {}

        metadata["row_segmentation_timestamp"] = datetime.now().strftime("%Y%m%d_%H%M%S")

        metadata["segmentation_parameters"] = {
            "crop": {
                "x_min_px": int(self.x1),
                "y_min_px": int(self.y1),
                "x_max_px": int(self.x2),
                "y_max_px": int(self.y2),
            },
            "shared": {
                "pot_count": int(self.pot_count),
                "minimum_depth_mm": float(self.min_depth_mm),
                "maximum_depth_mm": float(self.max_depth_mm),
                "minimum_contour_area_px": float(self.min_area),
                "grouping_dilation_iterations": int(self.dilate_itr),
            },
            "main_plant": {
                "hsv_lower": [int(value) for value in self.lower_green],
                "hsv_upper": [int(value) for value in self.upper_green],
                "morphology_kernel_size_px": int(self.kernel_size),
            },
            "flower": {
                "hsv_lower": [int(value) for value in self.lower_yellow],
                "hsv_upper": [int(value) for value in self.upper_yellow],
                "morphology_kernel_size_px": 2,
            },
        }

        metadata["plants"] = [
            {
                "plant_id": int(row["plant_id"]),
                "area_px": (
                    float(row["area_px"])
                    if row["area_px"] is not None
                    else None
                ),
                "radius_mm": row["radius_mm"],
                "target": {
                    "x_mm": row["target_x"],
                    "y_mm": row["target_y"],
                    "z_mm": row["target_z"],
                },
            }
            for row in results
        ]

        with open(metadata_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)


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
