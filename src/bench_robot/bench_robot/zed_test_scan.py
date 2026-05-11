#!/usr/bin/env python3

import os
import struct
from datetime import datetime
import cv2
import numpy as np
import yaml
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int16MultiArray, String
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ZedTestScanNode(Node):
    def __init__(self):
        super().__init__("zed_test_scan")

        # -------- States and variables --------
        self.bridge = CvBridge()

        self.current_location = None
        self.save_dir = os.path.expanduser("~/scan_data_zed")
        os.makedirs(self.save_dir, exist_ok=True)

        self.declare_parameter("fresh_wait_time", 2.0)
        self.declare_parameter("capture_timeout", 5.0)
        self.declare_parameter("bench_height", 0.75)
        self.declare_parameter("pot_height", 0.15)

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_cloud_msg = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_rgb_info_msg = None

        self.synced_count = 0
        self.camera_ready = False

        self.scan_active = False
        self.scan_request_time = None
        self.capture_timer = None

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)

        # -------- subs --------
        self.location_sub = self.create_subscription(Int16MultiArray,"/robot_location",self.cb_location,10,)
        self.state_sub = self.create_subscription(String,"/auto_state",self.cb_auto_state,10,)
        self.rgb_info_sub = self.create_subscription(CameraInfo,"/zed/zed_node/rgb/color/rect/camera_info",self.cb_rgb_camera_info,10,)

        self.color_sub = Subscriber(self, Image, "/zed/zed_node/rgb/color/rect/image")
        self.depth_sub = Subscriber(self, Image, "/zed/zed_node/depth/depth_registered")
        self.cloud_sub = Subscriber(self, PointCloud2, "/zed/zed_node/point_cloud/cloud_registered")
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.cloud_sub],queue_size=50,slop=0.5,)
        self.sync.registerCallback(self.synced_callback)

        # -------- pubs --------
        self.scan_done_pub = self.create_publisher(Bool, "/scan_done", 10)

        self.get_logger().info("ZED scan node started.")

    # -------- callback functions --------
    def _load_params(self):
        self.bench_height = self.get_parameter("bench_height").value
        self.pot_height = self.get_parameter("pot_height").value

    def on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)
    
    def cb_location(self, msg: Int16MultiArray):
        if len(msg.data) >= 5:
            self.current_location = msg.data[1], msg.data[2]

    def cb_rgb_camera_info(self, msg: CameraInfo):
        self.latest_rgb_info_msg = msg

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "scan_start":
            return

        if self.scan_active:
            self.get_logger().warn("Scan already active. Ignoring duplicate scan_start.")
            return

        if not self.camera_ready:
            self.get_logger().warn("scan_start received, but ZED is not ready yet. Waiting for frames...")

        self.scan_active = True
        self.scan_request_time = self.get_clock().now()

        self.get_logger().info("scan_start accepted. Waiting for fresh synchronized ZED frame...")

        if self.capture_timer is not None:
            self.capture_timer.cancel()

        self.capture_timer = self.create_timer(0.2, self.try_capture)

    def synced_callback(self, color_msg: Image, depth_msg: Image, cloud_msg: PointCloud2):
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg
        self.latest_cloud_msg = cloud_msg

        self.synced_count += 1

        ready_frames = 3
        if not self.camera_ready and self.synced_count >= ready_frames:
            self.camera_ready = True
            self.get_logger().info("ZED camera is ready. Synchronized frames are available.")

    def try_capture(self):
        now = self.get_clock().now()

        timeout = float(self.get_parameter("capture_timeout").value)
        elapsed = (now - self.scan_request_time).nanoseconds / 1e9

        if elapsed > timeout:
            self.get_logger().error("Capture timeout. No fresh synchronized ZED frame received.")
            self.finish_scan(success=False)
            return

        if self.latest_color_msg is None or self.latest_depth_msg is None or self.latest_cloud_msg is None:
            return

        latest_stamp = Time.from_msg(self.latest_color_msg.header.stamp)
        request_age = (latest_stamp - self.scan_request_time).nanoseconds / 1e9

        fresh_wait_time = float(self.get_parameter("fresh_wait_time").value)

        # Require a frame after scan_start, plus a small stabilization delay
        if request_age < fresh_wait_time:
            return

        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(self.latest_color_msg,desired_encoding="bgr8",)
            self.latest_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg,desired_encoding="passthrough",)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            self.finish_scan(success=False)
            return

        self.get_logger().info("Fresh synchronized ZED frame captured. Saving...")
        self.save_capture()
        self.get_logger().info("ZED capture saved successfully.")

        self.finish_scan(success=True)

    def finish_scan(self, success: bool):
        self.scan_active = False
        self.scan_request_time = None

        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        done_msg = Bool()
        done_msg.data = success
        self.scan_done_pub.publish(done_msg)

    def save_capture(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.current_location is not None:
            bench, row = self.current_location
            location_str = f"b{bench}_r{row}"
        else:
            location_str = "b_unknown_r_unknown"

        run_dir = os.path.join(self.save_dir, f"{location_str}_{stamp}")
        os.makedirs(run_dir, exist_ok=True)

        color_path = os.path.join(run_dir, "color.png")
        color_npy_path = os.path.join(run_dir, "color.npy")
        depth_npy_path = os.path.join(run_dir, "depth.npy")
        cloud_xyzrgb_npy_path = os.path.join(run_dir, "cloud_xyzrgb.npy")
        cloud_ply_path = os.path.join(run_dir, "cloud.ply")
        meta_path = os.path.join(run_dir, "metadata.yaml")

        cv2.imwrite(color_path, self.latest_color)
        np.save(color_npy_path, self.latest_color)
        np.save(depth_npy_path, self.latest_depth)

        xyzrgb_points, has_rgb = self.extract_pointcloud_arrays(self.latest_cloud_msg)

        if has_rgb and xyzrgb_points is not None:
            np.save(cloud_xyzrgb_npy_path, xyzrgb_points)
            self.save_ply_xyzrgb(xyzrgb_points, cloud_ply_path)

        metadata = {
            "camera": "ZED 2i",
            "timestamp": stamp,
            "location": location_str,

            "frames": {
                "color_frame_id": self.latest_color_msg.header.frame_id,
                "depth_frame_id": self.latest_depth_msg.header.frame_id,
                "cloud_frame_id": self.latest_cloud_msg.header.frame_id,
            },

            "point_cloud": {
                "width": self.latest_cloud_msg.width,
                "height": self.latest_cloud_msg.height,
                "point_step": self.latest_cloud_msg.point_step,
                "point_count": int(self.latest_cloud_msg.width * self.latest_cloud_msg.height),
            },

            "camera_info": {
                "rgb": self.build_camera_info_dict(self.latest_rgb_info_msg),
            }
        }

        with open(meta_path, "w") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

    def extract_pointcloud_arrays(self, cloud_msg):
        field_map = {field.name: field for field in cloud_msg.fields}

        for name in ["x", "y", "z"]:
            if name not in field_map:
                self.get_logger().error(f"PointCloud2 missing field: {name}")
                return None, None, False

        has_rgb = "rgb" in field_map or "rgba" in field_map
        rgb_name = "rgb" if "rgb" in field_map else ("rgba" if "rgba" in field_map else None)

        x_off = field_map["x"].offset
        y_off = field_map["y"].offset
        z_off = field_map["z"].offset
        rgb_off = field_map[rgb_name].offset if has_rgb else None

        point_step = cloud_msg.point_step
        data = cloud_msg.data
        n_points = cloud_msg.width * cloud_msg.height
        endian = ">" if cloud_msg.is_bigendian else "<"

        xyzrgb_points = []

        for i in range(n_points):
            base = i * point_step

            x = struct.unpack_from(endian + "f", data, base + x_off)[0]
            y = struct.unpack_from(endian + "f", data, base + y_off)[0]
            z = struct.unpack_from(endian + "f", data, base + z_off)[0]

            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue

            if has_rgb:
                rgb_field = field_map[rgb_name]

                if rgb_field.datatype == 7:
                    rgb_float = struct.unpack_from(endian + "f", data, base + rgb_off)[0]
                    rgb_int = struct.unpack(endian + "I", struct.pack(endian + "f", rgb_float))[0]
                else:
                    rgb_int = struct.unpack_from(endian + "I", data, base + rgb_off)[0]

                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF

                xyzrgb_points.append([x, y, z, r, g, b])

        xyzrgb_np = np.array(xyzrgb_points, dtype=np.float32) if xyzrgb_points else None

        return xyzrgb_np, has_rgb

    def save_ply_xyzrgb(self, xyzrgb_points: np.ndarray, filepath: str):
        with open(filepath, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(xyzrgb_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")

            for pt in xyzrgb_points:
                x, y, z, r, g, b = pt
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")
                
    def build_camera_info_dict(self, msg):
        if msg is None:
            return None

        return {
            "frame_id": str(msg.header.frame_id),
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "height": int(msg.height),
            "width": int(msg.width),
            "distortion_model": str(msg.distortion_model),
            "K": [float(x) for x in msg.k],
            "R": [float(x) for x in msg.r],
            "P": [float(x) for x in msg.p],
            "D": [float(x) for x in msg.d],
            "binning_x": int(msg.binning_x),
            "binning_y": int(msg.binning_y),
            "roi": {
                "x_offset": int(msg.roi.x_offset),
                "y_offset": int(msg.roi.y_offset),
                "height": int(msg.roi.height),
                "width": int(msg.roi.width),
                "do_rectify": bool(msg.roi.do_rectify),
            },
        }


def main(args=None):
    rclpy.init(args=args)
    node = ZedTestScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.capture_timer is not None:
            node.capture_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()