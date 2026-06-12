#!/usr/bin/env python3

import os
from datetime import datetime
import cv2
import yaml
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int16MultiArray, String
from sensor_msgs.msg import Image, CameraInfo
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
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],queue_size=20,slop=0.5,)
        self.sync.registerCallback(self.synced_callback)

        # -------- pubs --------
        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.pub_scan_color = self.create_publisher(Image, "/zed_top_scan/color", 10)
        self.pub_scan_depth = self.create_publisher(Image, "/zed_top_scan/depth", 10)
        self.pub_scan_camera_info = self.create_publisher(CameraInfo, "/zed_top_scan/camera_info", 10)
        self.pub_scan_run_dir = self.create_publisher(String, "/zed_top_scan/run_dir", 10)

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

        if state != "top_view_scan":
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

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg

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

        if self.latest_color_msg is None or self.latest_depth_msg is None:
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

        self.pub_scan_color.publish(self.latest_color_msg)
        self.pub_scan_depth.publish(self.latest_depth_msg)

        if self.latest_rgb_info_msg is not None:
            self.pub_scan_camera_info.publish(self.latest_rgb_info_msg)

        run_msg = String()
        run_msg.data = self.latest_run_dir
        self.pub_scan_run_dir.publish(run_msg)

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
            "camera": "ZED 2i",
            "timestamp": stamp,
            "location": location_str,

            "frames": {
                "color_frame_id": self.latest_color_msg.header.frame_id,
                "depth_frame_id": self.latest_depth_msg.header.frame_id,
            },

            "camera_info": {
                "rgb": self.build_camera_info_dict(self.latest_rgb_info_msg),
            }
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
