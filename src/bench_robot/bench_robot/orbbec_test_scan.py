#!/usr/bin/env python3

import os
from datetime import datetime, timezone

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from arm_interfaces.srv import CaptureView

class OrbbecTestScanNode(Node):
    def __init__(self):
        super().__init__('orbbec_test_scan')

        # ---------------- states and variables ----------------
        self.bridge = CvBridge()

        self.pending_capture = None
        self.capture_requested = False
        self.capture_timer = None
        self.scan_done = False

        self.latest_color = None
        self.latest_depth = None
        self.latest_cloud_msg = None
        self.latest_color_msg = None
        self.latest_depth_msg = None

        # ---------------- services ----------------
        self.streams_cli = self.create_client(SetBool, '/camera/set_streams_enable')
        self.get_logger().info('Waiting for /camera/set_streams_enable service...')
        self.streams_cli.wait_for_service()
        self.get_logger().info('Service available.')

        self.create_service(CaptureView,"/orbbec_test_scan/capture_view",self.cb_capture_view)

        # ---------------- subs ----------------

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

    def cb_capture_view(self, request, response):
        if self.capture_requested or self.pending_capture is not None or self.capture_timer is not None:
            response.success = False
            response.message = "Orbbec capture already running"
            return response

        self.pending_capture = {"run_dir": request.run_dir,"plant_id": request.plant_id,"view_label": request.view_label,}

        self.get_logger().info(f"Capture requested: plant {request.plant_id}, view {request.view_label}")

        req = SetBool.Request()
        req.data = True
        future = self.streams_cli.call_async(req)
        future.add_done_callback(self.on_streams_enabled)

        # This simple service returns after request accepted.
        # If you want true blocking until saved, use an action instead.
        response.success = True
        response.message = "Orbbec capture request accepted"
        return response

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
        self.pending_capture = None
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

        if self.pending_capture is None:
            self.get_logger().warn('No pending capture metadata. Capture not saved.')
            return

        base_run_dir = self.pending_capture["run_dir"]
        plant_id = self.pending_capture["plant_id"]
        view_label = self.pending_capture["view_label"]
        run_dir = os.path.join(base_run_dir,f"plant_{plant_id:02d}",view_label)

        os.makedirs(run_dir, exist_ok=True)

        color_path = os.path.join(run_dir, 'color.png')
        depth_npy_path = os.path.join(run_dir, 'depth.npy')
        cloud_xyzrgb_npy_path = os.path.join(run_dir, 'cloud_xyzrgb.npy')
        cloud_ply_path = os.path.join(run_dir, 'cloud.ply')
        meta_path = os.path.join(run_dir, 'meta.yaml')

        cv2.imwrite(color_path, self.latest_color)      # Save RGB
        np.save(depth_npy_path, self.latest_depth)      # Save depth raw
        xyz_points, xyzrgb_points, has_rgb = self.extract_pointcloud_arrays(self.latest_cloud_msg)      # Save point cloud

        if has_rgb and xyzrgb_points is not None and len(xyzrgb_points) > 0:
            np.save(cloud_xyzrgb_npy_path, xyzrgb_points)
            self.save_ply_xyzrgb(xyzrgb_points, cloud_ply_path)
        else:
            self.get_logger().warn('Point cloud does not contain RGB fields. Saved XYZ only.')

        sensor_timestamp = self.msg_stamp_to_float(
            self.latest_color_msg.header.stamp
        )
        metadata = {
            'plant_id': int(plant_id),
            'view_label': str(view_label),
            'capture_timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'sensor_timestamp_seconds': sensor_timestamp,
            'color_shape': list(self.latest_color.shape),
            'depth_shape': list(self.latest_depth.shape),
            'depth_dtype': str(self.latest_depth.dtype),
            'frame_id': self.latest_color_msg.header.frame_id,
        }
        if xyzrgb_points is not None:
            metadata['cloud_xyzrgb_point_count'] = len(xyzrgb_points)

        temporary_meta_path = meta_path + '.tmp'
        with open(temporary_meta_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(metadata, f, sort_keys=False)
        os.replace(temporary_meta_path, meta_path)

    def extract_pointcloud_arrays(self, cloud_msg):
        field_map = {f.name: f for f in cloud_msg.fields}

        for name in ('x', 'y', 'z'):
            if name not in field_map:
                self.get_logger().error(f'Point cloud missing field: {name}')
                return None, None, False

        has_rgb = 'rgb' in field_map or 'rgba' in field_map
        rgb_name = 'rgb' if 'rgb' in field_map else ('rgba' if 'rgba' in field_map else None)

        endian = '>' if cloud_msg.is_bigendian else '<'
        dtype_fields = {
            'x': (endian + 'f4', field_map['x'].offset),
            'y': (endian + 'f4', field_map['y'].offset),
            'z': (endian + 'f4', field_map['z'].offset),
        }

        if has_rgb:
            dtype_fields[rgb_name] = (endian + 'u4', field_map[rgb_name].offset)

        dtype = np.dtype({
            'names': list(dtype_fields.keys()),
            'formats': [value[0] for value in dtype_fields.values()],
            'offsets': [value[1] for value in dtype_fields.values()],
            'itemsize': cloud_msg.point_step,
        })

        n_points = cloud_msg.width * cloud_msg.height
        points = np.frombuffer(cloud_msg.data, dtype=dtype, count=n_points)

        valid = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
        points = points[valid]

        if len(points) == 0:
            return None, None, has_rgb

        xyz_np = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32, copy=False)

        if not has_rgb:
            return xyz_np, None, False

        rgb = points[rgb_name]
        r = ((rgb >> 16) & 0xFF).astype(np.float32)
        g = ((rgb >> 8) & 0xFF).astype(np.float32)
        b = (rgb & 0xFF).astype(np.float32)

        xyzrgb_np = np.column_stack((xyz_np, r, g, b)).astype(np.float32, copy=False)

        return xyz_np, xyzrgb_np, has_rgb


    def save_ply_xyzrgb(self, xyzrgb_points: np.ndarray, filepath: str):
        ply_points = np.empty(
            len(xyzrgb_points),
            dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')],
        )
        ply_points['x'] = xyzrgb_points[:, 0]
        ply_points['y'] = xyzrgb_points[:, 1]
        ply_points['z'] = xyzrgb_points[:, 2]
        ply_points['red'] = xyzrgb_points[:, 3].astype(np.uint8)
        ply_points['green'] = xyzrgb_points[:, 4].astype(np.uint8)
        ply_points['blue'] = xyzrgb_points[:, 5].astype(np.uint8)

        header = (
            'ply\n'
            'format binary_little_endian 1.0\n'
            f'element vertex {len(ply_points)}\n'
            'property float x\n'
            'property float y\n'
            'property float z\n'
            'property uchar red\n'
            'property uchar green\n'
            'property uchar blue\n'
            'end_header\n'
        )

        with open(filepath, 'wb') as f:
            f.write(header.encode('ascii'))
            ply_points.tofile(f)

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
