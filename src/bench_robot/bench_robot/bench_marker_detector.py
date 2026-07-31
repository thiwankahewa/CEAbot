#!/usr/bin/env python3

"""Detect bench-end ArUco markers with the arm-mounted Gemini 336 camera."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int16, Int16MultiArray, String
from std_srvs.srv import SetBool


ROBOT_LOCATION_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class BenchMarkerDetector(Node):
    def __init__(self):
        super().__init__("bench_marker_detector")

        self.auto_state = "idle"
        self.goal_bench = None
        self.requested_bench = None
        self.last_image_time = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.stream_desired = False
        self.stream_enabled = None
        self.stream_request_future = None
        self.stream_request_value = None

        self.declare_parameter("image_topic", "/gemini336/color/image_raw")
        self.declare_parameter("camera_info_topic", "/gemini336/color/camera_info")
        self.declare_parameter("marker_size_m", 0.083)
        self.declare_parameter("bench_marker_id_offset", 0)
        self.declare_parameter("bench_marker_min", 0)
        self.declare_parameter("bench_marker_max", 9)
        self.declare_parameter("image_timeout_s", 0.5)

        self.marker_offset = int(self.get_parameter("bench_marker_id_offset").value)
        self.marker_min = int(self.get_parameter("bench_marker_min").value)
        self.marker_max = int(self.get_parameter("bench_marker_max").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        self.bridge = CvBridge()
        self.stream_client = self.create_client(SetBool, "/gemini336/set_streams_enable")

        self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.create_subscription(
            Int16MultiArray, "/robot_location", self.cb_robot_location,
            ROBOT_LOCATION_QOS)
        self.create_subscription(Int16, "/bench_marker_target", self.cb_marker_target, 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.cb_image,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self.cb_camera_info, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Int16MultiArray, "/bench_side_marker", 10)
        self.watchdog = self.create_timer(0.1, self.watchdog_tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_auto_state(self, msg):
        self.auto_state = (msg.data or "").strip().lower()
        self.stream_desired = self.auto_state == "bench_change_start"
        if self.stream_desired:
            self.last_image_time = None
        else:
            self.publish_marker(False)

    def manage_stream(self):
        if self.stream_request_future is not None:
            if not self.stream_request_future.done():
                return
            requested = self.stream_request_value
            try:
                result = self.stream_request_future.result()
                if result is not None and result.success:
                    self.stream_enabled = requested
                    state = "enabled" if requested else "disabled"
                    self.get_logger().info(f"Gemini 336 streams {state}")
                else:
                    self.get_logger().warn(
                        f"Gemini 336 stream request failed: "
                        f"{getattr(result, 'message', 'no response')}")
            except Exception as exc:
                self.get_logger().warn(f"Gemini 336 stream request failed: {exc}")
            self.stream_request_future = None
            self.stream_request_value = None

        if self.stream_enabled == self.stream_desired:
            return
        if not self.stream_client.service_is_ready():
            return

        self.stream_request_value = self.stream_desired
        self.stream_request_future = self.stream_client.call_async(
            SetBool.Request(data=self.stream_desired))

    def cb_robot_location(self, msg):
        data = list(msg.data)
        if len(data) >= 5:
            self.goal_bench = int(data[3])

    def cb_marker_target(self, msg):
        self.requested_bench = int(msg.data)

    def marker_to_bench(self, marker_id):
        bench = int(marker_id) - self.marker_offset
        return bench if self.marker_min <= bench <= self.marker_max else None

    def cb_camera_info(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def publish_marker(self, visible, bench=0, error_px=0.0, orientation_deg=0.0,
                       distance_m=None):
        distance_mm = -1 if distance_m is None else int(round(distance_m * 1000.0))
        self.publisher.publish(Int16MultiArray(data=[
            1 if visible else 0,
            int(bench),
            int(round(error_px)),
            int(round(orientation_deg)),
            distance_mm,
        ]))

    def marker_distance_m(self, image_points):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None
        half_size = self.marker_size_m / 2.0
        object_points = np.asarray([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float32)
        success, _rvec, tvec = cv2.solvePnP(
            object_points,
            np.asarray(image_points, dtype=np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        return float(np.linalg.norm(tvec.reshape(3)))

    def cb_image(self, msg):
        if self.auto_state != "bench_change_start":
            return

        self.last_image_time = self.now_s()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert Gemini 336 image: {exc}")
            self.publish_marker(False)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            self.publish_marker(False)
            return

        candidates = []
        for index, marker_id in enumerate(ids.flatten()):
            bench = self.marker_to_bench(marker_id)
            if bench is not None:
                candidates.append((bench, corners[index]))

        if not candidates:
            self.publish_marker(False)
            return

        preferred_bench = (self.requested_bench if self.requested_bench is not None
                           else self.goal_bench)
        selected = next(
            ((number, points) for number, points in candidates
             if number == preferred_bench),
            None,
        )
        if selected is None:
            # Do not substitute a different visible marker. bench_changer must
            # know whether its current navigation target was actually seen.
            self.publish_marker(False, bench=preferred_bench or 0)
            return
        bench, marker_corners = selected
        points = np.asarray(marker_corners).reshape(-1, 2)
        # In pose_1 the arm camera is rotated 90 degrees, so the robot's
        # centering axis corresponds to the image's vertical axis.
        marker_center_y = float(np.mean(points[:, 1]))
        error_px = marker_center_y - frame.shape[0] / 2.0
        top_edge = points[1] - points[0]
        orientation_deg = float(np.degrees(np.arctan2(top_edge[1], top_edge[0])))
        distance_m = self.marker_distance_m(points)
        self.publish_marker(True, bench, error_px, orientation_deg, distance_m)

    def watchdog_tick(self):
        self.manage_stream()
        if self.auto_state != "bench_change_start":
            return
        if self.last_image_time is None or self.now_s() - self.last_image_time > self.image_timeout_s:
            self.publish_marker(False)


def main(args=None):
    rclpy.init(args=args)
    node = BenchMarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_marker(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
