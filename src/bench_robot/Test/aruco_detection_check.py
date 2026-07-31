#!/usr/bin/env python3

"""Quick ArUco check for the arm-mounted Gemini 336 color stream."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ArucoDetectionCheck(Node):
    def __init__(self):
        super().__init__("aruco_detection_check")

        self.declare_parameter("image_topic", "/gemini336/color/image_raw")
        self.declare_parameter("show_window", True)

        self.show_window = bool(self.get_parameter("show_window").value)
        image_topic = str(self.get_parameter("image_topic").value)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.detector = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())
        self.bridge = CvBridge()
        self.frame_count = 0

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f"OpenCV {cv2.__version__}; waiting for images on {image_topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        corners, ids, rejected = self.detector.detectMarkers(frame)
        self.frame_count += 1

        if ids is not None:
            detected_ids = [int(marker_id) for marker_id in ids.flatten()]
            self.get_logger().info(f"Detected ArUco IDs: {detected_ids}")
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
