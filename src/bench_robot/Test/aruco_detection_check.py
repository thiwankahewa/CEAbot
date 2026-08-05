#!/usr/bin/env python3

"""Quick ArUco check for the arm-mounted Gemini 336 color stream."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ArucoDetectionCheck(Node):
    def __init__(self):
        super().__init__("aruco_detection_check")

        self.declare_parameter("image_topic", "/gemini336/color/image_raw")
        self.declare_parameter("camera_info_topic", "/gemini336/color/camera_info")
        self.declare_parameter("marker_size_m", 0.083)
        self.declare_parameter("show_window", True)

        self.show_window = bool(self.get_parameter("show_window").value)
        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.camera_matrix = None
        self.dist_coeffs = None

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.detector = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())
        self.bridge = CvBridge()
        self.frame_count = 0

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f"OpenCV {cv2.__version__}; waiting for images on {image_topic}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def marker_pose(self, image_points):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None
        half = self.marker_size_m / 2.0
        object_points = np.asarray([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(
            object_points, np.asarray(image_points, dtype=np.float32),
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        normal = rotation[:, 2]
        normal_z = max(abs(float(normal[2])), 1e-6)
        return (
            float(np.linalg.norm(tvec.reshape(3))),
            float(np.degrees(np.arctan2(normal[0], normal_z))),
            float(np.degrees(np.arctan2(normal[1], normal_z))),
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        corners, ids, rejected = self.detector.detectMarkers(frame)
        self.frame_count += 1

        if ids is not None:
            readings = []
            for index, marker_id in enumerate(ids.flatten()):
                pose = self.marker_pose(np.asarray(corners[index]).reshape(-1, 2))
                readings.append((int(marker_id), pose))
            self.get_logger().info(f"Detected ArUco poses (id, (m, x_deg, y_deg)): {readings}")
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        elif self.frame_count % 30 == 0:
            self.get_logger().info(
                f"No marker detected (rejected candidates: {len(rejected)})")

        if self.show_window:
            cv2.imshow("Gemini 336 ArUco check", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
