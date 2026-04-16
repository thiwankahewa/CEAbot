#!/usr/bin/env python3

import os
import struct
import math
from datetime import datetime
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

class OrbbecTestScanNode(Node):
    def __init__(self):
        super().__init__('orbbec_test_scan')

        # ---------------- states and variables ----------------
        self.bridge = CvBridge()

        self.capture_requested = False
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

        # ---------------- services ----------------
        self.streams_cli = self.create_client(SetBool, '/camera/set_streams_enable')
        self.get_logger().info('Waiting for /camera/set_streams_enable service...')
        self.streams_cli.wait_for_service()
        self.get_logger().info('Service available.')

        # ---------------- subs ----------------
        self.location_sub = self.create_subscription(Int16MultiArray, '/robot_location', self.cb_location, 10)
        self.cmd_sub = self.create_subscription(String,'/auto_state',self.cb_auto_state,10)
        
        # Time-synchronized color + depth + point cloud
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.cloud_sub = Subscriber(self, PointCloud2, '/camera/depth_registered/points')
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.cloud_sub],queue_size=20,slop=0.25)
        self.sync.registerCallback(self.synced_callback)
        
        # ---------------- pubs ----------------
        self.scan_done_pub = self.create_publisher(Bool, '/scan_done', 10)

    # ---------------- helper functions ----------------

    def on_streams_enabled(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'set_streams_enable on failed: {e}')
            return

        if not resp.success:
            self.get_logger().error(f'set_streams_enable on failed: {resp.message}')
            return

        self.get_logger().info(f'All streams enabled: {resp.message}')

        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        self.capture_timer = self.create_timer(0.5, self.arm_capture_once)

    def arm_capture_once(self):
        if self.capture_timer is not None:
            self.capture_timer.cancel()
            self.capture_timer = None

        self.capture_requested = True
        self.get_logger().info('Waiting for next synchronized color + depth + point cloud set...')

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

    
    # ---------------- callbacks functions ----------------

    def cb_location(self, msg: Int16MultiArray):
        self.current_location = msg.data[1], msg.data[2]

    def cb_auto_state(self, msg: String):
        state = msg.data.strip().lower()

        if state != 'scan_start':
            return
        
        self.get_logger().info('Received scan_start. Enabling all streams...')  # send command to enable all streams
        req = SetBool.Request()
        req.data = True
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_enabled)

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

        self.get_logger().info('Synchronized color, depth, and point cloud received. Saving capture...')
        self.save_capture()
        self.get_logger().info('Capture saved successfully.')
        self.capture_requested = False

        msg = Bool()
        msg.data = True
        self.scan_done_pub.publish(msg)

        self.get_logger().info('Disabling all streams after capture...')
        req = SetBool.Request()
        req.data = False
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_disabled)

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
        cloud_xyz_npy_path = os.path.join(run_dir, 'cloud_xyz.npy')
        cloud_xyzrgb_npy_path = os.path.join(run_dir, 'cloud_xyzrgb.npy')
        cloud_ply_path = os.path.join(run_dir, 'cloud.ply')
        meta_path = os.path.join(run_dir, 'meta.txt')

        cv2.imwrite(color_path, self.latest_color)      # Save RGB
        np.save(depth_npy_path, self.latest_depth)      # Save depth raw
        xyz_points, xyzrgb_points, has_rgb = self.extract_pointcloud_arrays(self.latest_cloud_msg)      # Save point cloud

        if xyz_points is not None and len(xyz_points) > 0:
            np.save(cloud_xyz_npy_path, xyz_points)
        else:
            self.get_logger().warn('No XYZ points extracted from point cloud.')

        if has_rgb and xyzrgb_points is not None and len(xyzrgb_points) > 0:
            np.save(cloud_xyzrgb_npy_path, xyzrgb_points)
            self.save_ply_xyzrgb(xyzrgb_points, cloud_ply_path)
        else:
            self.get_logger().warn('Point cloud does not contain RGB fields. Saved XYZ only.')

        print(xyzrgb_points[:10, 3:6])

        with open(meta_path, 'w') as f:
            f.write(f'color_shape: {self.latest_color.shape}\n')
            f.write(f'depth_shape: {self.latest_depth.shape}\n')
            f.write(f'depth_dtype: {self.latest_depth.dtype}\n')
            f.write(f'frame_id: {self.latest_color_msg.header.frame_id}\n')
            if xyz_points is not None:
                f.write(f'cloud_xyz_point_count: {len(xyz_points)}\n')
            if xyzrgb_points is not None:
                f.write(f'cloud_xyzrgb_point_count: {len(xyzrgb_points)}\n')

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