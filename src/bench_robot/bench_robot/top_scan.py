#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool


class TopScanNode(Node):
    def __init__(self):
        super().__init__("top_scan")

        self.bridge = CvBridge()

        self.current_location = None
        self.latest_run_dir = None
        self.capture_timeout = 5.0
        self.fresh_wait_time = 2.0

        self.color_topic = "/gemini335/color/image_raw"
        self.depth_topic = "/gemini335/depth/image_raw"
        self.rgb_camera_info_topic = "/gemini335/color/camera_info"
        self.depth_camera_info_topic = "/gemini335/depth/camera_info"
        self.streams_enable_service = "/gemini335/set_streams_enable"

        self.top_scan_color_topic = "/top_scan/color"
        self.top_scan_depth_topic = "/top_scan/depth"
        self.top_scan_camera_info_topic = "/top_scan/camera_info"
        self.top_scan_run_dir_topic = "/top_scan/run_dir"

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_rgb_info_msg = None
        self.latest_depth_stats = None

        self.synced_count = 0
        self.camera_ready = False
        self.scan_active = False
        self.streams_enabled = False
        self.scan_request_time = None
        self.capture_timer = None
        self.zero_depth_warned = False

        self.declare_parameter("bench_height", 0.75)
        self.declare_parameter("pot_height", 0.15)

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)
        self.save_dir = os.path.expanduser("~/scan_data")
        os.makedirs(self.save_dir, exist_ok=True)

        self.streams_cli = self.create_client(SetBool, self.streams_enable_service)

        self.location_sub = self.create_subscription(Int16MultiArray, "/robot_location", self.cb_location, 10)
        self.state_sub = self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)

        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.rgb_info_sub = Subscriber(self, CameraInfo, self.rgb_camera_info_topic)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.rgb_info_sub], queue_size=20, slop=0.5)
        self.sync.registerCallback(self.synced_callback)

        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_scan_color = self.create_publisher(Image, self.top_scan_color_topic, 10)
        self.pub_scan_depth = self.create_publisher(Image, self.top_scan_depth_topic, 10)
        self.pub_scan_camera_info = self.create_publisher(CameraInfo, self.top_scan_camera_info_topic, 10)
        self.pub_scan_run_dir = self.create_publisher(String, self.top_scan_run_dir_topic, 10)

    def _load_params(self):
        self.bench_height = self.get_parameter("bench_height").value
        self.pot_height = self.get_parameter("pot_height").value

    def on_params(self, params):
        self._load_params()
        os.makedirs(self.save_dir, exist_ok=True)
        return SetParametersResult(successful=True)

    def cb_location(self, msg: Int16MultiArray):
        if len(msg.data) >= 5:
            self.current_location = msg.data[1], msg.data[2]

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "top_view_scan":
            return

        if self.scan_active:
            self.get_logger().warn("Scan already active. Ignoring duplicate scan_start.")
            return

        self.scan_active = True
        self.clear_latest_capture_data()

        self.get_logger().info("top_view_scan accepted. Enabling Gemini 335 streams...")

        if not self.streams_cli.service_is_ready():
            self.get_logger().info(f"Waiting for {self.streams_enable_service} service...")
            if not self.streams_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"{self.streams_enable_service} service is not available.")
                self.finish_scan(success=False)
                return

        req = SetBool.Request()
        req.data = True
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_enabled)

    def on_streams_enabled(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"set_streams_enable on failed: {e}")
            self.finish_scan(success=False)
            return

        if not resp.success:
            self.get_logger().error(f"set_streams_enable on failed: {resp.message}")
            self.finish_scan(success=False)
            return

        self.streams_enabled = True
        self.scan_request_time = self.get_clock().now()
        self.synced_count = 0
        self.camera_ready = False
        self.zero_depth_warned = False

        self.get_logger().info("Gemini 335 streams enabled. Waiting for synchronized color/depth and camera_info...")

        if self.capture_timer is not None:
            self.capture_timer.cancel()

        self.capture_timer = self.create_timer(0.2, self.try_capture)

    def on_streams_disabled(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"set_streams_enable off failed: {e}")
            return

        if not resp.success:
            self.get_logger().error(f"set_streams_enable off failed: {resp.message}")
            return

        self.streams_enabled = False
        self.get_logger().info(f"Gemini 335 streams disabled: {resp.message}")

    def clear_latest_capture_data(self):
        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_rgb_info_msg = None
        self.latest_depth_stats = None
        self.synced_count = 0
        self.camera_ready = False

    def synced_callback(self, color_msg: Image, depth_msg: Image, rgb_info_msg: CameraInfo):
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg
        self.latest_rgb_info_msg = rgb_info_msg

        self.synced_count += 1

        ready_frames = 3
        if not self.camera_ready and self.synced_count >= ready_frames:
            self.camera_ready = True
            self.get_logger().info("Gemini 335 is ready. Synchronized frames and camera_info are available.")

    def try_capture(self):
        now = self.get_clock().now()
        elapsed = (now - self.scan_request_time).nanoseconds / 1e9

        if elapsed > self.capture_timeout:
            self.get_logger().error("Capture timeout. Missing fresh synchronized Gemini 335 images or camera_info.")
            self.finish_scan(success=False)
            return

        if self.latest_color_msg is None or self.latest_depth_msg is None or self.latest_rgb_info_msg is None:
            return

        latest_stamp = Time.from_msg(self.latest_color_msg.header.stamp)
        request_age = (latest_stamp - self.scan_request_time).nanoseconds / 1e9

        if request_age < self.fresh_wait_time:
            return

        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(self.latest_color_msg, desired_encoding="bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            self.finish_scan(success=False)
            return

        self.latest_depth_stats = self.build_depth_stats(self.latest_depth)
        self.get_logger().info(
            "Depth frame stats: "
            f"encoding={self.latest_depth_msg.encoding}, "
            f"dtype={self.latest_depth.dtype}, "
            f"shape={self.latest_depth.shape}, "
            f"nonzero={self.latest_depth_stats['nonzero_pixels']}/{self.latest_depth_stats['total_pixels']}, "
            f"min={self.latest_depth_stats['min']}, "
            f"max={self.latest_depth_stats['max']}"
        )

        if self.latest_depth_stats["positive_pixels"] == 0:
            if not self.zero_depth_warned:
                self.get_logger().warn(
                    "Depth frame has no positive depth values yet. Waiting for a valid depth frame..."
                )
                self.zero_depth_warned = True
            return

        self.pub_scan_color.publish(self.latest_color_msg)
        self.pub_scan_depth.publish(self.latest_depth_msg)
        self.pub_scan_camera_info.publish(self.latest_rgb_info_msg)
        self.get_logger().info("Fresh synchronized Gemini 335 frame and camera_info captured. Saving...")
        self.save_capture()

        run_msg = String()
        run_msg.data = self.latest_run_dir
        self.pub_scan_run_dir.publish(run_msg)
        self.get_logger().info("Gemini 335 capture saved successfully.")
        self.finish_scan(success=True)

    def finish_scan(self, success: bool):
        self.scan_active = False
        self.scan_request_time = None

        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        if self.streams_enabled:
            req = SetBool.Request()
            req.data = False
            future = self.streams_cli.call_async(req)
            future.add_done_callback(self.on_streams_disabled)

        msg = String()
        msg.data = "plant_row_coordinates" if success else "idle"
        self.pub_auto_state_cmd.publish(msg)

    def save_capture(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.current_location is not None:
            bench, row = self.current_location
            location_str = f"b{bench}_r{row}"
        else:
            location_str = "b_unknown_r_unknown"

        run_dir = os.path.join(self.save_dir, f"{location_str}_{stamp}")
        os.makedirs(run_dir, exist_ok=True)
        self.latest_run_dir = run_dir

        color_path = os.path.join(run_dir, "color.png")
        depth_npy_path = os.path.join(run_dir, "depth.npy")
        meta_path = os.path.join(run_dir, "metadata.yaml")
        cv2.imwrite(color_path, self.latest_color)
        np.save(depth_npy_path, self.latest_depth)

        metadata = {
            "camera": "Orbbec Gemini 335",
            "timestamp": stamp,
            "location": location_str,
            "frames": {
                "color_frame_id": self.latest_color_msg.header.frame_id,
                "depth_frame_id": self.latest_depth_msg.header.frame_id,
            },
            "images": {
                "color_encoding": self.latest_color_msg.encoding,
                "depth_encoding": self.latest_depth_msg.encoding,
                "depth_stats": self.latest_depth_stats,
            },
            "camera_info": {"rgb": self.build_camera_info_dict(self.latest_rgb_info_msg),},
        }

        with open(meta_path, "w") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

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

    def build_depth_stats(self, depth):
        finite = depth[np.isfinite(depth)]

        if finite.size == 0:
            return {
                "total_pixels": int(depth.size),
                "finite_pixels": 0,
                "nonzero_pixels": 0,
                "positive_pixels": 0,
                "min": None,
                "max": None,
            }

        return {
            "total_pixels": int(depth.size),
            "finite_pixels": int(finite.size),
            "nonzero_pixels": int(np.count_nonzero(finite)),
            "positive_pixels": int(np.count_nonzero(finite > 0)),
            "min": float(np.min(finite)),
            "max": float(np.max(finite)),
        }


def main(args=None):
    rclpy.init(args=args)
    node = TopScanNode()

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
