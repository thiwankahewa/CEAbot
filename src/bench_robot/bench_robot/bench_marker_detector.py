#!/usr/bin/env python3

"""Detect bench-end ArUco markers with the arm-mounted Gemini 336 camera."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import SetBool


class BenchMarkerDetector(Node):
    def __init__(self):
        super().__init__("bench_marker_detector")

        self.auto_state = "idle"
        self.goal_bench = None
        self.last_image_time = None
        self.camera_matrix = None
        self.dist_coeffs = None

        self.declare_parameter("image_topic", "/gemini336/color/image_raw")
        self.declare_parameter("camera_info_topic", "/gemini336/color/camera_info")
        self.declare_parameter("marker_size_m", 0.10)
        self.declare_parameter("bench_marker_id_offset", 100)
        self.declare_parameter("bench_marker_min", 1)
        self.declare_parameter("bench_marker_max", 99)
        self.declare_parameter("image_timeout_s", 0.5)

        self.marker_offset = int(self.get_parameter("bench_marker_id_offset").value)
        self.marker_min = int(self.get_parameter("bench_marker_min").value)
        self.marker_max = int(self.get_parameter("bench_marker_max").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        self.bridge = CvBridge()
        self.stream_client = self.create_client(SetBool, "/gemini336/set_streams_enable")

        self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.create_subscription(Int16MultiArray, "/robot_location", self.cb_robot_location, 10)
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
        previous = self.auto_state
        self.auto_state = (msg.data or "").strip().lower()
        if self.auto_state == "bench_change_start" and previous != self.auto_state:
            self.last_image_time = None
            if self.stream_client.service_is_ready():
                self.stream_client.call_async(SetBool.Request(data=True))
            else:
                self.get_logger().warn(
                    "Gemini 336 stream service is unavailable; waiting for its color topic")
        if self.auto_state != "bench_change_start":
            self.publish_marker(False)

    def cb_robot_location(self, msg):
        data = list(msg.data)
        if len(data) >= 5:
            self.goal_bench = int(data[3])

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

        # Prefer the requested bench when several end markers are in view.
        bench, marker_corners = next(
            ((number, points) for number, points in candidates if number == self.goal_bench),
            candidates[0],
        )
        points = np.asarray(marker_corners).reshape(-1, 2)
        marker_center_x = float(np.mean(points[:, 0]))
        error_px = marker_center_x - frame.shape[1] / 2.0
        top_edge = points[1] - points[0]
        orientation_deg = float(np.degrees(np.arctan2(top_edge[1], top_edge[0])))
        distance_m = self.marker_distance_m(points)
        self.publish_marker(True, bench, error_px, orientation_deg, distance_m)

    def watchdog_tick(self):
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
        node.publish_marker(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
