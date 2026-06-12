#!/usr/bin/env python3

import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest


class ZedTopScanFolderPublisher(Node):
    def __init__(self, folder: Path, rate_hz: float, duration: float):
        super().__init__("zed_top_scan_folder_publisher")
        self.folder = folder
        self.rate_hz = rate_hz
        self.duration = duration
        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Image, "/zed_top_scan/color", 10)
        self.depth_pub = self.create_publisher(Image, "/zed_top_scan/depth", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/zed_top_scan/camera_info", 10)
        self.run_dir_pub = self.create_publisher(String, "/zed_top_scan/run_dir", 10)
        self.auto_state_pub = self.create_publisher(String, "/auto_state", 10)

        self.color_msg, self.depth_msg, self.camera_info_msg, self.run_dir_msg = self.load_messages()

    def load_messages(self):
        color_path = self.folder / "color.png"
        depth_path = self.folder / "depth.npy"
        metadata_path = self.folder / "metadata.yaml"

        color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
        if color is None:
            raise RuntimeError(f"Could not read {color_path}")

        if not depth_path.exists():
            raise RuntimeError(f"Missing {depth_path}")
        depth = np.load(depth_path)

        if not metadata_path.exists():
            raise RuntimeError(f"Missing {metadata_path}")
        metadata = yaml.safe_load(metadata_path.read_text())
        rgb_info = metadata["camera_info"]["rgb"]

        stamp = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = rgb_info.get("frame_id", "zed_left_camera_frame_optical")

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = rgb_info.get("frame_id", "zed_left_camera_frame_optical")

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = rgb_info.get("frame_id", "zed_left_camera_frame_optical")
        camera_info_msg.height = int(rgb_info["height"])
        camera_info_msg.width = int(rgb_info["width"])
        camera_info_msg.distortion_model = rgb_info.get("distortion_model", "")
        camera_info_msg.d = [float(v) for v in rgb_info.get("D", [])]
        camera_info_msg.k = [float(v) for v in rgb_info["K"]]
        camera_info_msg.r = [float(v) for v in rgb_info["R"]]
        camera_info_msg.p = [float(v) for v in rgb_info["P"]]
        camera_info_msg.binning_x = int(rgb_info.get("binning_x", 0))
        camera_info_msg.binning_y = int(rgb_info.get("binning_y", 0))

        roi_data = rgb_info.get("roi", {})
        camera_info_msg.roi = RegionOfInterest(
            x_offset=int(roi_data.get("x_offset", 0)),
            y_offset=int(roi_data.get("y_offset", 0)),
            height=int(roi_data.get("height", 0)),
            width=int(roi_data.get("width", 0)),
            do_rectify=bool(roi_data.get("do_rectify", False)),
        )

        run_dir_msg = String()
        run_dir_msg.data = str(self.folder)

        return color_msg, depth_msg, camera_info_msg, run_dir_msg

    def publish_once(self):
        stamp = self.get_clock().now().to_msg()
        self.color_msg.header.stamp = stamp
        self.depth_msg.header.stamp = stamp
        self.camera_info_msg.header.stamp = stamp

        self.color_pub.publish(self.color_msg)
        self.depth_pub.publish(self.depth_msg)
        self.camera_info_pub.publish(self.camera_info_msg)
        self.run_dir_pub.publish(self.run_dir_msg)

    def publish_for_duration(self):
        end_time = time.time() + self.duration
        period = 1.0 / self.rate_hz

        while rclpy.ok() and time.time() < end_time:
            self.publish_once()
            time.sleep(period)

    def trigger_plant_row_coordinates(self):
        msg = String()
        msg.data = "plant_row_coordinates"
        self.auto_state_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Replay a saved ZED top-scan folder onto /zed_top_scan topics.")
    parser.add_argument("folder", type=Path, help="Folder containing color.png, depth.npy, and metadata.yaml")
    parser.add_argument("--rate", type=float, default=5.0, help="Publish rate in Hz")
    parser.add_argument("--duration", type=float, default=3.0, help="How long to publish image topics")
    parser.add_argument("--trigger", action="store_true", help="Publish /auto_state=plant_row_coordinates after replay")
    args = parser.parse_args()

    rclpy.init()
    node = ZedTopScanFolderPublisher(args.folder.expanduser(), args.rate, args.duration)

    try:
        node.get_logger().info(f"Publishing saved ZED top scan from {node.folder}")
        node.publish_for_duration()

        if args.trigger:
            node.trigger_plant_row_coordinates()
            node.get_logger().info("Published /auto_state=plant_row_coordinates")
            time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
