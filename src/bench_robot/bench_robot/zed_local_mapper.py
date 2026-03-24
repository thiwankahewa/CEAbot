#!/usr/bin/env python3

import copy
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class ZedLocalMapper(Node):
    def __init__(self):
        super().__init__('zed_local_mapper')

        # ---- parameters ----
        # ROI crop in target frame
        self.declare_parameter('crop_min_x', 0.15)
        self.declare_parameter('crop_max_x', 0.90)
        self.declare_parameter('crop_min_y', -0.40)
        self.declare_parameter('crop_max_y',  0.40)
        self.declare_parameter('crop_min_z', 0.00)
        self.declare_parameter('crop_max_z', 1.20)

        # voxel-like downsample size
        self.declare_parameter('voxel_leaf', 0.1)

        self.crop_min_x = float(self.get_parameter('crop_min_x').value)
        self.crop_max_x = float(self.get_parameter('crop_max_x').value)
        self.crop_min_y = float(self.get_parameter('crop_min_y').value)
        self.crop_max_y = float(self.get_parameter('crop_max_y').value)
        self.crop_min_z = float(self.get_parameter('crop_min_z').value)
        self.crop_max_z = float(self.get_parameter('crop_max_z').value)
        self.voxel_leaf = float(self.get_parameter('voxel_leaf').value)

        self.cloud_topic = '/zed/zed_node/point_cloud/cloud_registered'
        self.target_frame = 'base_link'

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- state ----
        self.latest_cloud = None
        self.captured_clouds: Dict[int, PointCloud2] = {}

        # ---- subs ----
        self.sub_cloud = self.create_subscription(PointCloud2,self.cloud_topic,self.cloud_callback,10)
        self.sub_trigger = self.create_subscription(Int32,'/zed_capture_trigger',self.trigger_callback,10)

        # ---- pubs ----
        self.pub_capture_done = self.create_publisher(Int32, '/zed_capture_done', 10)
        self.pub_map_ready = self.create_publisher(Bool, '/zed_local_map_ready', 10)
        self.pub_local_cloud = self.create_publisher(PointCloud2, '/local_obstacle_cloud', 10)
        self.pub_status = self.create_publisher(String, '/zed_mapper_status', 10)

    # ---------------- basic callbacks ----------------

    def cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def trigger_callback(self, msg: Int32):
        capture_id = int(msg.data)

        if self.latest_cloud is None:
            self.get_logger().warn('No cloud received yet; cannot capture.')
            return

        self.publish_status(f'capturing_{capture_id}')

        cloud_copy = copy.deepcopy(self.latest_cloud)
        transformed = self.transform_cloud_to_target(cloud_copy)
        if transformed is None:
            self.get_logger().error(f'Failed to transform capture {capture_id}.')
            return

        self.captured_clouds[capture_id] = transformed

        done_msg = Int32()
        done_msg.data = capture_id
        self.pub_capture_done.publish(done_msg)

        self.get_logger().info(f'Captured cloud {capture_id} in frame {self.target_frame}.')

        if len(self.captured_clouds) >= 3:
            self.build_and_publish_local_map()

    # ---------------- transforms ----------------

    def transform_cloud_to_target(self, cloud_msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, cloud_msg.header.frame_id, rclpy.time.Time())
            transformed_cloud = do_transform_cloud(cloud_msg, transform)
            transformed_cloud.header.frame_id = self.target_frame
            return transformed_cloud
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None

    # ---------------- point cloud utils ----------------

    def cloud_to_xyzrgb(self, cloud_msg: PointCloud2) -> List[Tuple[float, float, float, int]]:
        points = []
        field_names = [f.name for f in cloud_msg.fields]

        has_rgb = 'rgb' in field_names
        has_rgba = 'rgba' in field_names

        read_fields = ['x', 'y', 'z']
        if has_rgb:
            read_fields.append('rgb')
        elif has_rgba:
            read_fields.append('rgba')

        for p in point_cloud2.read_points(cloud_msg, field_names=read_fields, skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if math.isinf(x) or math.isinf(y) or math.isinf(z):
                continue

            color = 0
            if len(p) > 3:
                # keep packed rgb/rgba value as uint-like payload
                try:
                    color = int(p[3])
                except Exception:
                    color = 0

            points.append((x, y, z, color))
        return points

    def crop_points(self, pts: List[Tuple[float, float, float, int]]) -> List[Tuple[float, float, float, int]]:
        out = []
        for x, y, z, c in pts:
            if not (self.crop_min_x <= x <= self.crop_max_x):
                continue
            if not (self.crop_min_y <= y <= self.crop_max_y):
                continue
            if not (self.crop_min_z <= z <= self.crop_max_z):
                continue
            out.append((x, y, z, c))
        return out

    def voxel_downsample(self, pts: List[Tuple[float, float, float, int]], leaf: float) -> List[Tuple[float, float, float, int]]:
        if leaf <= 0.0:
            return pts

        voxels = {}
        for x, y, z, c in pts:
            key = (
                int(math.floor(x / leaf)),
                int(math.floor(y / leaf)),
                int(math.floor(z / leaf))
            )
            if key not in voxels:
                voxels[key] = (x, y, z, c)
        return list(voxels.values())

    def xyzrgb_to_cloud(self, pts: List[Tuple[float, float, float, int]], frame_id: str, stamp):
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=7, count=1),   # FLOAT32
            point_cloud2.PointField(name='y', offset=4, datatype=7, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=7, count=1),
            point_cloud2.PointField(name='rgb', offset=12, datatype=6, count=1), # UINT32
        ]

        cloud_pts = []
        for x, y, z, rgb in pts:
            cloud_pts.append([x, y, z, rgb])

        msg = point_cloud2.create_cloud(None, fields, cloud_pts)
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        return msg

    # ---------------- map build ----------------

    def build_and_publish_local_map(self):
        self.publish_status('building_local_map')

        all_pts = []
        for capture_id in sorted(self.captured_clouds.keys()):
            pts = self.cloud_to_xyzrgb(self.captured_clouds[capture_id])
            all_pts.extend(pts)

        if len(all_pts) == 0:
            self.get_logger().warn('Merged map is empty.')
            return

        cropped = self.crop_points(all_pts)
        downsampled = self.voxel_downsample(cropped, self.voxel_leaf)

        out_cloud = self.xyzrgb_to_cloud(
            downsampled,
            self.target_frame,
            self.get_clock().now().to_msg()
        )

        self.pub_local_cloud.publish(out_cloud)

        ready = Bool()
        ready.data = True
        self.pub_map_ready.publish(ready)

        self.publish_status('local_map_ready')

        self.get_logger().info(
            f'Published local map with {len(downsampled)} points '
            f'(raw={len(all_pts)}, cropped={len(cropped)}).'
        )

        # Clear for next scan cycle
        self.captured_clouds.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ZedLocalMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()