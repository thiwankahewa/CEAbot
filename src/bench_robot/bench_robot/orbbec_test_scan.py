#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class OrbbecTestScanNode(Node):
    def __init__(self):
        super().__init__('orbbec_test_scan')

        self.bridge = CvBridge()
        self.capture_requested = False
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_msg = None
        self.latest_depth_msg = None

        self.save_dir = os.path.expanduser('~/scan_data')
        os.makedirs(self.save_dir, exist_ok=True)

        # Listen for scan_start command
        self.cmd_sub = self.create_subscription(String,'/auto_state_cmd',self.auto_state_callback,10)

        # Time-synchronized color + depth subscriptions
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')

        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],queue_size=10,slop=0.2)
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info('Scan capture node started.')
        self.get_logger().info(f'Listening for scan command on: /auto_state_cmd')
        self.get_logger().info(f'Color topic: /camera/color/image_raw')
        self.get_logger().info(f'Depth topic: /camera/depth/image_raw')
        self.get_logger().info(f'Saving data to: {self.save_dir}')

    def auto_state_callback(self, msg: String):
        state = msg.data.strip().lower()

        if state == 'scan_start':
            self.capture_requested = True
            self.get_logger().info('Received scan_start. Waiting for next synchronized color + depth frames...')

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
            return

        try:
            # Keep native depth encoding
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        self.latest_color = color_img
        self.latest_depth = depth_img
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg

        if self.capture_requested:
            self.save_capture()
            self.capture_requested = False

    def save_capture(self):
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn('No synchronized frames available yet.')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(self.save_dir, f'scan_{stamp}')
        os.makedirs(run_dir, exist_ok=True)

        color_path = os.path.join(run_dir, 'color.png')
        depth_npy_path = os.path.join(run_dir, 'depth.npy')
        depth_preview_path = os.path.join(run_dir, 'depth_preview.png')
        meta_path = os.path.join(run_dir, 'meta.txt')

        # Save RGB image
        cv2.imwrite(color_path, self.latest_color)

        # Save raw depth values
        np.save(depth_npy_path, self.latest_depth)

        # Save viewable depth preview
        depth_vis = self.make_depth_preview(self.latest_depth)
        cv2.imwrite(depth_preview_path, depth_vis)

        # Save some metadata
        with open(meta_path, 'w') as f:
            f.write(f'color_shape: {self.latest_color.shape}\n')
            f.write(f'depth_shape: {self.latest_depth.shape}\n')
            f.write(f'depth_dtype: {self.latest_depth.dtype}\n')
            f.write(f'color_frame_id: {self.latest_color_msg.header.frame_id}\n')
            f.write(f'depth_frame_id: {self.latest_depth_msg.header.frame_id}\n')
            f.write(f'color_stamp_sec: {self.latest_color_msg.header.stamp.sec}\n')
            f.write(f'color_stamp_nanosec: {self.latest_color_msg.header.stamp.nanosec}\n')
            f.write(f'depth_stamp_sec: {self.latest_depth_msg.header.stamp.sec}\n')
            f.write(f'depth_stamp_nanosec: {self.latest_depth_msg.header.stamp.nanosec}\n')

        self.get_logger().info(f'Saved scan data to: {run_dir}')

    def make_depth_preview(self, depth_img):
        depth = np.array(depth_img, dtype=np.float32)

        valid = depth > 0
        if not np.any(valid):
            return np.zeros((depth.shape[0], depth.shape[1]), dtype=np.uint8)

        min_val = np.min(depth[valid])
        max_val = np.max(depth[valid])

        if max_val - min_val < 1e-6:
            preview = np.zeros_like(depth, dtype=np.uint8)
            preview[valid] = 255
            return preview

        norm = np.zeros_like(depth, dtype=np.float32)
        norm[valid] = (depth[valid] - min_val) / (max_val - min_val)
        preview = (norm * 255).astype(np.uint8)

        return preview


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecTestScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()