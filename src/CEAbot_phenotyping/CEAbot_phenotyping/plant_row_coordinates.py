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
