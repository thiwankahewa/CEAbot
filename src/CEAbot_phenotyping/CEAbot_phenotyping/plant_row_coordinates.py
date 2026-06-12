#!/usr/bin/env python3

import time
import cv2
import numpy as np
from pathlib import Path
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

        self.x1 = 336
        self.y1 = 316
        self.x2 = 1940
        self.y2 = 883

        self.lower_green = np.array([22, 27, 0])
        self.upper_green = np.array([95, 255, 255])
        self.min_area = 15000
        self.kernel_size = 5
        self.depth_scale = 1000
        self.top_percentile = 5.0
        self.center_window_size = 9

         # -------- Subscriptions and publishers --------
        self.state_sub = self.create_subscription(String,"/auto_state",self.cb_auto_state,10,)
        self.color_sub = self.create_subscription(Image,"/zed_top_scan/color",self.cb_color,10)
        self.depth_sub = self.create_subscription(Image,"/zed_top_scan/depth",self.cb_depth,10)
        self.camera_info_sub = self.create_subscription(CameraInfo,"/zed_top_scan/camera_info",self.cb_camera_info,10)
        self.run_dir_sub = self.create_subscription(String,"/zed_top_scan/run_dir",self.cb_run_dir,10)

        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.target_pub = self.create_publisher(PlantTargetArray,"/plant_row/targets",10)

    # -------- Callbacks --------
        
    def cb_color(self, msg):
        self.latest_color_msg = msg

    def cb_depth(self, msg):
        self.latest_depth_msg = msg

    def cb_camera_info(self, msg):
        self.latest_camera_info_msg = msg

    def cb_run_dir(self, msg):
        self.latest_run_dir = Path(msg.data)

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "plant_row_coordinates":
            return
        
        self.get_logger().info("Starting plant row coordinate calculation")

        if self.latest_color_msg is None or self.latest_depth_msg is None or self.latest_camera_info_msg is None:
            self.get_logger().warn("Missing color/depth/camera_info. Cannot calculate plant coordinates.")
            return

        try:
            color = self.bridge.imgmsg_to_cv2(self.latest_color_msg,desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg,desired_encoding="passthrough").astype(np.float32)

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        camera_info = self.latest_camera_info_msg

        K = camera_info.k
        fx = float(K[0])
        fy = float(K[4])
        cx_intr = float(K[2])
        cy_intr = float(K[5])
        frame_id = camera_info.header.frame_id

        if self.latest_run_dir is None:
            self.get_logger().warn("No run directory received. Saving outputs disabled.")
            output_dir = None
        else:
            output_dir = self.latest_run_dir

        self.process_live_frame(color,depth,fx,fy,cx_intr,cy_intr,frame_id,output_dir)

   # -------- Helper functions --------

    def pixel_depth_to_3d(self, u, v, z, fx, fy, cx, cy):
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        return round(X, 2), round(Y, 2), round(Z, 2)

    def get_depth_median_around_pixel(self, depth, u, v, window_size):
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

    def get_top_point_from_contour(self, depth_full, contour_crop):
        h_crop = self.y2 - self.y1
        w_crop = self.x2 - self.x1

        contour_mask_crop = np.zeros((h_crop, w_crop), dtype=np.uint8)
        cv2.drawContours(contour_mask_crop, [contour_crop], -1, 255, -1)

        depth_crop = depth_full[self.y1:self.y2, self.x1:self.x2]

        plant_depth_values = depth_crop[contour_mask_crop > 0]

        valid_depth = plant_depth_values[np.isfinite(plant_depth_values)]
        valid_depth = valid_depth[valid_depth > 0]

        if len(valid_depth) == 0:
            return None

        top_threshold = np.percentile(valid_depth, self.top_percentile)

        top_mask_crop = np.zeros_like(contour_mask_crop)
        top_mask_crop[
            (contour_mask_crop > 0)
            & (depth_crop > 0)
            & np.isfinite(depth_crop)
            & (depth_crop <= top_threshold)
        ] = 255

        ys, xs = np.where(top_mask_crop > 0)

        if len(xs) == 0:
            return None

        top_u_crop = int(np.mean(xs))
        top_v_crop = int(np.mean(ys))
        top_depth = float(np.median(depth_crop[ys, xs]))

        top_u_full = top_u_crop + self.x1
        top_v_full = top_v_crop + self.y1

        return top_u_full, top_v_full, top_depth, top_u_crop, top_v_crop

    def calculate_contour_radius_mm(self, depth_mm, area, fx, fy):
        if depth_mm is None or depth_mm <= 0:
            return None

        mm_per_px_x = depth_mm / fx
        mm_per_px_y = depth_mm / fy

        area_mm2 = area * mm_per_px_x * mm_per_px_y
        radius_mm = np.sqrt(area_mm2 / np.pi)

        return int(radius_mm)

    def save_plant_results_to_metadata(self, output_dir, results):
        if output_dir is None:
            self.get_logger().warn("No run directory received. Plant metadata not saved.")
            return

        metadata_path = output_dir / "metadata.yaml"

        if metadata_path.exists():
            with open(metadata_path, "r") as f:
                metadata = yaml.safe_load(f) or {}
        else:
            metadata = {}

        metadata["plants"] = []

        for row in results:
            metadata["plants"].append({
                "plant_id": int(row["plant_id"]),
                "area_px": float(row["area_px"]) if row["area_px"] is not None else None,
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
            })

        with open(metadata_path, "w") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)
    
    # -------- Main functions --------

    def process_live_frame(self, img, depth, fx, fy, cx_intr, cy_intr, frame_id, output_dir=None):

        crop = img[self.y1:self.y2, self.x1:self.x2]
        depth = depth.astype(np.float32)
        depth = depth * self.depth_scale

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
        segmented = cv2.bitwise_and(crop, crop, mask=mask_clean)
        contours, _ = cv2.findContours(mask_clean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        detection = crop.copy()

        valid_contours = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_area:
                continue

            valid_contours.append(cnt)

        valid_contours = sorted(valid_contours,key=cv2.contourArea,reverse=True)

        plant_records = []

        for i, cnt in enumerate(valid_contours):
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)

            if M["m00"] == 0:
                continue

            center_x_crop = int(M["m10"] / M["m00"])
            center_y_crop = int(M["m01"] / M["m00"])

            center_x_full = center_x_crop + self.x1
            center_y_full = center_y_crop + self.y1

            center_depth = self.get_depth_median_around_pixel(depth,center_x_full,center_y_full,self.center_window_size)
            top_result = self.get_top_point_from_contour(depth, cnt)

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
                "plant_id": i + 1,
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

            plant_records.append({"sort_x": row["center_x"],"csv_row": row})

        plant_records = sorted(plant_records,key=lambda r: r["sort_x"] if r["sort_x"] is not None else -999999,reverse=True)

        results = []

        for plant_id, record in enumerate(plant_records, start=1):
            record["csv_row"]["plant_id"] = plant_id
            results.append(record["csv_row"])

        target_msg = PlantTargetArray()
        target_msg.run_dir = str(output_dir)

        for row in results:
            if row["target_x"] is None or row["target_y"] is None or row["target_z"] is None:
                continue

            t = PlantTarget()
            t.plant_id = int(row["plant_id"])
            t.target_x = float(row["target_x"]) / 1000.0
            t.target_y = float(row["target_y"]) / 1000.0
            t.target_z = float(row["target_z"]) / 1000.0

            if row["radius_mm"] is not None:
                t.radius_m = float(row["radius_mm"]) / 1000.0
            else:
                t.radius_m = 0.05

            target_msg.targets.append(t)

        self.target_pub.publish(target_msg)
        self.get_logger().info(f"{output_dir.name}: detected {len(results)} plants")
        time.sleep(0.5)
        self.pub_auto_state_cmd.publish(String(data="individual_plant_scan"))

        cv2.imwrite(str(output_dir / "segmented_result.png"), segmented)
        cv2.imwrite(str(output_dir / "detection.png"), detection)

        self.save_plant_results_to_metadata(output_dir, results)


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
