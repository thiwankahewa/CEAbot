#!/usr/bin/env python3

import os
import struct
import math
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs import msg
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer


class OrbbecTestScanNode(Node):
    def __init__(self):
        super().__init__('orbbec_test_scan')

        self.bridge = CvBridge()

        self.capture_requested = False
        self.waiting_for_streams = False
        self.capture_timer = None
        self.scan_done = False

        self.latest_color = None
        self.latest_depth = None
        self.latest_cloud_msg = None
        self.latest_color_msg = None
        self.latest_depth_msg = None

        self.current_location = None

        self.save_dir = os.path.expanduser('~/scan_data')
        os.makedirs(self.save_dir, exist_ok=True)

        # Listen for scan_start command
        self.cmd_sub = self.create_subscription(String,'/auto_state',self.auto_state_callback,10)
        self.location_sub = self.create_subscription(Int16MultiArray, '/robot_location', self.location_callback, 10)
        

        self.scan_done_pub = self.create_publisher(Bool, '/scan_done', 10)

        # Time-synchronized color + depth + point cloud
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.cloud_sub = Subscriber(self, PointCloud2, '/camera/depth_registered/points')

        # Approximate sync across all three streams
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.cloud_sub],
            queue_size=20,
            slop=0.25
        )
        self.sync.registerCallback(self.synced_callback)

        # Service client for runtime stream control
        self.streams_cli = self.create_client(SetBool, '/camera/set_streams_enable')

        self.get_logger().info('Waiting for /camera/set_streams_enable service...')
        self.streams_cli.wait_for_service()
        self.get_logger().info('Service available.')

        self.get_logger().info('Scan capture node started.')
        self.get_logger().info('Listening for scan command on: /auto_state_cmd')
        self.get_logger().info('Color topic: /camera/color/image_raw')
        self.get_logger().info('Depth topic: /camera/depth/image_raw')
        self.get_logger().info('Cloud topic: /camera/depth/points')
        self.get_logger().info(f'Saving data to: {self.save_dir}')

    def location_callback(self, msg: Int16MultiArray):
        if len(msg.data) >= 5:
            self.current_location = msg.data[1], msg.data[2]

    def auto_state_callback(self, msg: String):
        state = msg.data.strip().lower()

        if state != 'scan_start':
            return

        if self.waiting_for_streams or self.capture_requested:
            self.get_logger().warn('Capture already in progress. Ignoring new scan_start.')
            return

        self.get_logger().info('Received scan_start. Enabling all streams...')
        self.waiting_for_streams = True
        self.enable_all_streams()

    def enable_all_streams(self):
        req = SetBool.Request()
        req.data = True
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_enabled)

    def on_streams_enabled(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'set_streams_enable on failed: {e}')
            self.waiting_for_streams = False
            return

        if not resp.success:
            self.get_logger().error(f'set_streams_enable on failed: {resp.message}')
            self.waiting_for_streams = False
            return

        self.get_logger().info(f'All streams enabled: {resp.message}')
        self.get_logger().info('Waiting 1 second before arming capture...')

        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        self.capture_timer = self.create_timer(1.0, self.arm_capture_once)

    def arm_capture_once(self):
        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        self.capture_requested = True
        self.waiting_for_streams = False
        self.get_logger().info('Capture armed. Waiting for next synchronized color + depth + point cloud set...')

    def synced_callback(self, color_msg: Image, depth_msg: Image, cloud_msg: PointCloud2):
        if not self.capture_requested:
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        self.latest_color = color_img
        self.latest_depth = depth_img
        self.latest_cloud_msg = cloud_msg
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg

        self.save_capture()
        self.capture_requested = False

        msg = Bool()
        msg.data = True
        self.scan_done_pub.publish(msg)

        self.get_logger().info('Disabling all streams after capture...')
        self.disable_all_streams()

    def disable_all_streams(self):
        req = SetBool.Request()
        req.data = False
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_disabled)

    def on_streams_disabled(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'set_streams_enable off failed: {e}')
            return

        if not resp.success:
            self.get_logger().error(f'set_streams_enable off failed: {resp.message}')
            return

        self.get_logger().info(f'All streams disabled: {resp.message}')

    def save_capture(self):
        if self.latest_color is None or self.latest_depth is None or self.latest_cloud_msg is None:
            self.get_logger().warn('Missing synchronized data. Capture not saved.')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bench, row = self.current_location
        location_str = f"b{bench}_r{row}"
        run_dir = os.path.join(self.save_dir, f"{location_str}_{stamp}")
        os.makedirs(run_dir, exist_ok=True)

        

        color_path = os.path.join(run_dir, 'color.png')
        depth_npy_path = os.path.join(run_dir, 'depth.npy')
        depth_preview_path = os.path.join(run_dir, 'depth_preview.png')
        cloud_xyz_npy_path = os.path.join(run_dir, 'cloud_xyz.npy')
        cloud_xyzrgb_npy_path = os.path.join(run_dir, 'cloud_xyzrgb.npy')
        cloud_ply_path = os.path.join(run_dir, 'cloud.ply')
        meta_path = os.path.join(run_dir, 'meta.txt')

        # Save RGB
        cv2.imwrite(color_path, self.latest_color)

        # Save depth raw
        np.save(depth_npy_path, self.latest_depth)

        # Save depth preview
        depth_vis = self.make_depth_preview(self.latest_depth)
        cv2.imwrite(depth_preview_path, depth_vis)

        # Save point cloud
        xyz_points, xyzrgb_points, has_rgb = self.extract_pointcloud_arrays(self.latest_cloud_msg)

        if xyz_points is not None and len(xyz_points) > 0:
            np.save(cloud_xyz_npy_path, xyz_points)
            self.get_logger().info(f'Saved XYZ point cloud to: {cloud_xyz_npy_path}')
        else:
            self.get_logger().warn('No XYZ points extracted from point cloud.')

        if has_rgb and xyzrgb_points is not None and len(xyzrgb_points) > 0:
            np.save(cloud_xyzrgb_npy_path, xyzrgb_points)
            self.save_ply_xyzrgb(xyzrgb_points, cloud_ply_path)
            self.get_logger().info(f'Saved XYZRGB point cloud to: {cloud_xyzrgb_npy_path}')
            self.get_logger().info(f'Saved colored PLY point cloud to: {cloud_ply_path}')
        else:
            self.get_logger().warn('Point cloud does not contain RGB fields. Saved XYZ only.')

        print(xyzrgb_points[:10, 3:6])

        # Save metadata
        color_stamp = self.msg_stamp_to_float(self.latest_color_msg.header.stamp)
        depth_stamp = self.msg_stamp_to_float(self.latest_depth_msg.header.stamp)
        cloud_stamp = self.msg_stamp_to_float(self.latest_cloud_msg.header.stamp)

        with open(meta_path, 'w') as f:
            f.write(f'color_shape: {self.latest_color.shape}\n')
            f.write(f'depth_shape: {self.latest_depth.shape}\n')
            f.write(f'depth_dtype: {self.latest_depth.dtype}\n')

            f.write(f'color_frame_id: {self.latest_color_msg.header.frame_id}\n')
            f.write(f'depth_frame_id: {self.latest_depth_msg.header.frame_id}\n')
            f.write(f'cloud_frame_id: {self.latest_cloud_msg.header.frame_id}\n')

            f.write(f'color_stamp_sec: {self.latest_color_msg.header.stamp.sec}\n')
            f.write(f'color_stamp_nanosec: {self.latest_color_msg.header.stamp.nanosec}\n')
            f.write(f'depth_stamp_sec: {self.latest_depth_msg.header.stamp.sec}\n')
            f.write(f'depth_stamp_nanosec: {self.latest_depth_msg.header.stamp.nanosec}\n')
            f.write(f'cloud_stamp_sec: {self.latest_cloud_msg.header.stamp.sec}\n')
            f.write(f'cloud_stamp_nanosec: {self.latest_cloud_msg.header.stamp.nanosec}\n')

            f.write(f'color_stamp_float: {color_stamp:.9f}\n')
            f.write(f'depth_stamp_float: {depth_stamp:.9f}\n')
            f.write(f'cloud_stamp_float: {cloud_stamp:.9f}\n')

            f.write(f'color_depth_dt_sec: {abs(color_stamp - depth_stamp):.9f}\n')
            f.write(f'color_cloud_dt_sec: {abs(color_stamp - cloud_stamp):.9f}\n')
            f.write(f'depth_cloud_dt_sec: {abs(depth_stamp - cloud_stamp):.9f}\n')

            f.write(f'cloud_width: {self.latest_cloud_msg.width}\n')
            f.write(f'cloud_height: {self.latest_cloud_msg.height}\n')
            f.write(f'cloud_is_dense: {self.latest_cloud_msg.is_dense}\n')
            f.write(f'cloud_point_step: {self.latest_cloud_msg.point_step}\n')
            f.write(f'cloud_row_step: {self.latest_cloud_msg.row_step}\n')
            f.write(f'cloud_has_rgb: {has_rgb}\n')
            if xyz_points is not None:
                f.write(f'cloud_xyz_point_count: {len(xyz_points)}\n')
            if xyzrgb_points is not None:
                f.write(f'cloud_xyzrgb_point_count: {len(xyzrgb_points)}\n')

        self.get_logger().info(f'Saved scan data to: {run_dir}')

    def extract_pointcloud_arrays(self, cloud_msg):

        field_map = {f.name: f for f in cloud_msg.fields}

        has_rgb = 'rgb' in field_map or 'rgba' in field_map
        rgb_name = 'rgb' if 'rgb' in field_map else ('rgba' if 'rgba' in field_map else None)

        x_off = field_map['x'].offset
        y_off = field_map['y'].offset
        z_off = field_map['z'].offset
        rgb_off = field_map[rgb_name].offset if has_rgb else None

        point_step = cloud_msg.point_step
        data = cloud_msg.data
        n_points = cloud_msg.width * cloud_msg.height

        xyz_points = []
        xyzrgb_points = []

        for i in range(n_points):
            base = i * point_step

            x = struct.unpack_from('<f', data, base + x_off)[0]
            y = struct.unpack_from('<f', data, base + y_off)[0]
            z = struct.unpack_from('<f', data, base + z_off)[0]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            xyz_points.append([x, y, z])

            if has_rgb:
                rgb_float = struct.unpack_from('<f', data, base + rgb_off)[0]
                rgb_int = struct.unpack('<I', struct.pack('<f', rgb_float))[0]

                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF

                xyzrgb_points.append([x, y, z, r, g, b])

        xyz_np = np.array(xyz_points, dtype=np.float32) if xyz_points else None
        xyzrgb_np = np.array(xyzrgb_points, dtype=np.float32) if xyzrgb_points else None

        return xyz_np, xyzrgb_np, has_rgb

    def unpack_rgb_field(self, rgb_val):
        if isinstance(rgb_val, float):
            if math.isnan(rgb_val):
                return 0, 0, 0
            packed = struct.pack('<f', rgb_val)
            rgb_int = struct.unpack('<I', packed)[0]
        else:
            rgb_int = int(rgb_val)

        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        return r, g, b

    def save_ply_xyzrgb(self, xyzrgb_points: np.ndarray, filepath: str):
        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(xyzrgb_points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
            f.write('end_header\n')

            for pt in xyzrgb_points:
                x, y, z, r, g, b = pt
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n')

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

    def msg_stamp_to_float(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecTestScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.capture_timer is not None:
            node.capture_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()