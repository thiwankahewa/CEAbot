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


class AstraTopScanNode(Node):
    def __init__(self):
        super().__init__("astra_top_scan")

        self.bridge = CvBridge()

        self.current_location = None
        self.latest_run_dir = None

        self.declare_parameter("camera_model", "Orbbec Astra 2")
        self.declare_parameter("save_dir", os.path.expanduser("~/scan_data_astra2"))
        self.declare_parameter("fresh_wait_time", 2.0)
        self.declare_parameter("capture_timeout", 5.0)
        self.declare_parameter("bench_height", 0.75)
        self.declare_parameter("pot_height", 0.15)

        self.declare_parameter("color_topic", "/astra2/color/image_raw")
        self.declare_parameter("depth_topic", "/astra2/depth/image_raw")
        self.declare_parameter("rgb_camera_info_topic", "/astra2/color/camera_info")
        self.declare_parameter("depth_camera_info_topic", "/astra2/depth/camera_info")

        # Keep the old top-scan contract so plant_row_coordinates does not change.
        self.declare_parameter("scan_color_topic", "/zed_top_scan/color")
        self.declare_parameter("scan_depth_topic", "/zed_top_scan/depth")
        self.declare_parameter("scan_camera_info_topic", "/zed_top_scan/camera_info")
        self.declare_parameter("scan_run_dir_topic", "/zed_top_scan/run_dir")

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_rgb_info_msg = None
        self.latest_depth_info_msg = None

        self.synced_count = 0
        self.camera_ready = False

        self.scan_active = False
        self.scan_request_time = None
        self.capture_timer = None

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)
        os.makedirs(self.save_dir, exist_ok=True)

        self.location_sub = self.create_subscription(
            Int16MultiArray, "/robot_location", self.cb_location, 10
        )
        self.state_sub = self.create_subscription(
            String, "/auto_state", self.cb_auto_state, 10
        )
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, self.rgb_camera_info_topic, self.cb_rgb_camera_info, 10
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo, self.depth_camera_info_topic, self.cb_depth_camera_info, 10
        )

        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=20, slop=0.5
        )
        self.sync.registerCallback(self.synced_callback)

        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_scan_color = self.create_publisher(Image, self.scan_color_topic, 10)
        self.pub_scan_depth = self.create_publisher(Image, self.scan_depth_topic, 10)
        self.pub_scan_camera_info = self.create_publisher(
            CameraInfo, self.scan_camera_info_topic, 10
        )
        self.pub_scan_run_dir = self.create_publisher(String, self.scan_run_dir_topic, 10)

        self.get_logger().info(
            "Astra 2 top scan node started. "
            f"color={self.color_topic}, depth={self.depth_topic}, "
            f"camera_info={self.rgb_camera_info_topic}"
        )

    def _load_params(self):
        self.camera_model = self.get_parameter("camera_model").value
        self.save_dir = os.path.expanduser(self.get_parameter("save_dir").value)
        self.bench_height = self.get_parameter("bench_height").value
        self.pot_height = self.get_parameter("pot_height").value

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.rgb_camera_info_topic = self.get_parameter("rgb_camera_info_topic").value
        self.depth_camera_info_topic = self.get_parameter("depth_camera_info_topic").value

        self.scan_color_topic = self.get_parameter("scan_color_topic").value
        self.scan_depth_topic = self.get_parameter("scan_depth_topic").value
        self.scan_camera_info_topic = self.get_parameter("scan_camera_info_topic").value
        self.scan_run_dir_topic = self.get_parameter("scan_run_dir_topic").value

    def on_params(self, params):
        self._load_params()
        os.makedirs(self.save_dir, exist_ok=True)
        return SetParametersResult(successful=True)

    def cb_location(self, msg: Int16MultiArray):
        if len(msg.data) >= 5:
            self.current_location = msg.data[1], msg.data[2]

    def cb_rgb_camera_info(self, msg: CameraInfo):
        self.latest_rgb_info_msg = msg

    def cb_depth_camera_info(self, msg: CameraInfo):
        self.latest_depth_info_msg = msg

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != "top_view_scan":
            return

        if self.scan_active:
            self.get_logger().warn("Scan already active. Ignoring duplicate scan_start.")
            return

        if not self.camera_ready:
            self.get_logger().warn(
                "top_view_scan received, but Astra 2 is not ready yet. Waiting for frames..."
            )

        self.scan_active = True
        self.scan_request_time = self.get_clock().now()

        self.get_logger().info(
            "top_view_scan accepted. Waiting for fresh synchronized Astra 2 frame..."
        )

        if self.capture_timer is not None:
            self.capture_timer.cancel()

        self.capture_timer = self.create_timer(0.2, self.try_capture)

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg

        self.synced_count += 1

        ready_frames = 3
        if not self.camera_ready and self.synced_count >= ready_frames:
            self.camera_ready = True
            self.get_logger().info(
                "Astra 2 is ready. Synchronized frames are available."
            )

    def try_capture(self):
        now = self.get_clock().now()

        timeout = float(self.get_parameter("capture_timeout").value)
        elapsed = (now - self.scan_request_time).nanoseconds / 1e9

        if elapsed > timeout:
            self.get_logger().error(
                "Capture timeout. No fresh synchronized Astra 2 frame received."
            )
            self.finish_scan(success=False)
            return

        if self.latest_color_msg is None or self.latest_depth_msg is None:
            return

        latest_stamp = Time.from_msg(self.latest_color_msg.header.stamp)
        request_age = (latest_stamp - self.scan_request_time).nanoseconds / 1e9

        fresh_wait_time = float(self.get_parameter("fresh_wait_time").value)

        if request_age < fresh_wait_time:
            return

        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(
                self.latest_color_msg, desired_encoding="bgr8"
            )
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            self.finish_scan(success=False)
            return

        self.pub_scan_color.publish(self.latest_color_msg)
        self.pub_scan_depth.publish(self.latest_depth_msg)

        if self.latest_rgb_info_msg is not None:
            self.pub_scan_camera_info.publish(self.latest_rgb_info_msg)
        else:
            self.get_logger().warn("No RGB camera_info received yet; publishing images only.")

        self.get_logger().info("Fresh synchronized Astra 2 frame captured. Saving...")
        self.save_capture()

        run_msg = String()
        run_msg.data = self.latest_run_dir
        self.pub_scan_run_dir.publish(run_msg)

        self.get_logger().info("Astra 2 capture saved successfully.")
        self.finish_scan(success=True)

    def finish_scan(self, success: bool):
        self.scan_active = False
        self.scan_request_time = None

        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

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
            "camera": self.camera_model,
            "timestamp": stamp,
            "location": location_str,
            "topics": {
                "color": self.color_topic,
                "depth": self.depth_topic,
                "rgb_camera_info": self.rgb_camera_info_topic,
                "depth_camera_info": self.depth_camera_info_topic,
            },
            "frames": {
                "color_frame_id": self.latest_color_msg.header.frame_id,
                "depth_frame_id": self.latest_depth_msg.header.frame_id,
            },
            "images": {
                "color_encoding": self.latest_color_msg.encoding,
                "depth_encoding": self.latest_depth_msg.encoding,
                "depth_dtype": str(self.latest_depth.dtype),
                "depth_shape": [int(v) for v in self.latest_depth.shape],
            },
            "camera_info": {
                "rgb": self.build_camera_info_dict(self.latest_rgb_info_msg),
                "depth": self.build_camera_info_dict(self.latest_depth_info_msg),
            },
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


def main(args=None):
    rclpy.init(args=args)
    node = AstraTopScanNode()

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
