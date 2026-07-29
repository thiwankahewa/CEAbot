#!/usr/bin/env python3
"""Interactive eye-in-hand ChArUco observation capture.

Move the arm manually, wait until it is completely still, and press Space to
save a timestamp-matched camera image, robot pose, and board pose.
"""

import argparse
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in (
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_100",
    )
}


def rotation_matrix_to_quaternion(rotation):
    """Return an xyzw quaternion from a proper 3x3 rotation matrix."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    trace = np.trace(rotation)
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        return np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    diagonal = np.diag(matrix[:3, :3])
    index = int(np.argmax(diagonal))
    if index == 0:
        scale = np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        quaternion = [
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        ]
    elif index == 1:
        scale = np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        quaternion = [
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        ]
    else:
        scale = np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
        quaternion = [
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            0.25 * scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ]
    return np.asarray(quaternion)


def transform_to_dict(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        "translation": {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
        },
        "quaternion_xyzw": {
            "x": float(rotation.x),
            "y": float(rotation.y),
            "z": float(rotation.z),
            "w": float(rotation.w),
        },
    }


def extract_xyzrgb(cloud_message):
    field_map = {field.name: field for field in cloud_message.fields}
    for name in ("x", "y", "z"):
        if name not in field_map:
            raise ValueError(f"PointCloud2 is missing {name!r}")
    rgb_name = "rgb" if "rgb" in field_map else "rgba" if "rgba" in field_map else None
    if rgb_name is None:
        raise ValueError("PointCloud2 has no rgb or rgba field")

    endian = ">" if cloud_message.is_bigendian else "<"
    names = ["x", "y", "z", rgb_name]
    formats = [endian + "f4", endian + "f4", endian + "f4", endian + "u4"]
    offsets = [field_map[name].offset for name in names]
    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": cloud_message.point_step,
        }
    )
    count = cloud_message.width * cloud_message.height
    points = np.frombuffer(cloud_message.data, dtype=dtype, count=count)
    valid = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    points = points[valid]
    packed = points[rgb_name]
    return np.column_stack(
        (
            points["x"],
            points["y"],
            points["z"],
            ((packed >> 16) & 0xFF),
            ((packed >> 8) & 0xFF),
            (packed & 0xFF),
        )
    ).astype(np.float32)


def save_xyzrgb_ply(path, points):
    vertices = np.empty(
        len(points),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    vertices["x"], vertices["y"], vertices["z"] = points[:, 0], points[:, 1], points[:, 2]
    vertices["red"] = points[:, 3].astype(np.uint8)
    vertices["green"] = points[:, 4].astype(np.uint8)
    vertices["blue"] = points[:, 5].astype(np.uint8)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        vertices.tofile(stream)


class HandEyeCapture(Node):
    def __init__(self, args):
        super().__init__("handeye_capture")
        self.args = args
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_camera_info = None
        self.latest_cloud_msg = None
        self.latest_detection = None
        self.cached_display = None
        self.last_detection_time = 0.0
        self.observations = []
        self.validation_count = 0
        self.last_validation_capture_time = 0.0

        dictionary = cv2.aruco.getPredefinedDictionary(
            DICTIONARIES[args.dictionary]
        )
        self.board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_length,
            args.marker_length,
            dictionary,
        )
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor-data QoS keeps only fresh images instead of building a stale
        # reliable queue while ChArUco rendering is busy.
        self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, args.camera_info_topic, self.camera_info_callback, 10
        )
        self.create_subscription(
            PointCloud2,
            args.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        session_name = datetime.now().strftime("handeye_%Y%m%d_%H%M%S")
        self.output_dir = args.output_dir.expanduser() / session_name
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.get_logger().info(f"Saving observations to {self.output_dir}")

    def image_callback(self, message):
        self.latest_image_msg = message

    def camera_info_callback(self, message):
        self.latest_camera_info = message

    def cloud_callback(self, message):
        self.latest_cloud_msg = message

    def render(self):
        image_message = self.latest_image_msg
        if image_message is None:
            frame = np.zeros((480, 800, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                f"Waiting for {self.args.image_topic}",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 180, 255),
                2,
            )
            return frame

        now = time.monotonic()
        if (
            self.cached_display is not None
            and now - self.last_detection_time < 1.0 / self.args.display_rate
        ):
            return self.cached_display
        self.last_detection_time = now
        frame = self.bridge.imgmsg_to_cv2(
            image_message, desired_encoding="bgr8"
        )
        display = frame.copy()
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.detector.detectBoard(frame)
        )
        # Keep the detection and exact source image together because image
        # callbacks continue concurrently in the background executor.
        self.latest_detection = (
            charuco_corners,
            charuco_ids,
            image_message,
            frame.copy(),
        )
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
        corner_count = 0 if charuco_ids is None else len(charuco_ids)
        if corner_count:
            cv2.aruco.drawDetectedCornersCharuco(
                display, charuco_corners, charuco_ids
            )

        ready = (
            corner_count >= self.args.minimum_corners
            and self.latest_camera_info is not None
        )
        color = (0, 220, 0) if ready else (0, 180, 255)
        status = (
            f"READY: {corner_count} corners"
            if ready
            else f"NOT READY: {corner_count}/{self.args.minimum_corners} corners"
        )
        cv2.putText(
            display, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
        )
        cv2.putText(
            display,
            f"Calibration: {len(self.observations)}   Validation: {self.validation_count}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            display,
            "SPACE=calibration   C=RGB+cloud validation   Q=quit",
            (20, 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )
        self.cached_display = display
        return display

    def capture(self):
        if self.latest_detection is None or self.latest_camera_info is None:
            self.get_logger().warn("No synchronized image/camera info available")
            return
        corners, ids, image_message, image = self.latest_detection
        if ids is None or len(ids) < self.args.minimum_corners:
            self.get_logger().warn("Not enough ChArUco corners; capture skipped")
            return

        stamp = image_message.header.stamp
        image_time = Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
        used_latest_tf = False
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.end_effector_frame,
                image_time,
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Timestamped TF lookup failed ({exc}); trying latest TF. "
                "Only capture while the arm is completely stationary."
            )
            try:
                robot_transform = self.tf_buffer.lookup_transform(
                    self.args.base_frame,
                    self.args.end_effector_frame,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
                used_latest_tf = True
            except Exception as latest_exc:
                self.get_logger().error(f"Latest TF lookup failed: {latest_exc}")
                return

        tf_stamp = robot_transform.header.stamp
        tf_age_seconds = (
            float(stamp.sec - tf_stamp.sec)
            + float(stamp.nanosec - tf_stamp.nanosec) * 1e-9
        )
        if used_latest_tf and abs(tf_age_seconds) > self.args.max_tf_age:
            self.get_logger().error(
                f"Latest TF is {tf_age_seconds:.3f} s from the image timestamp; "
                f"limit is {self.args.max_tf_age:.3f} s. Capture skipped."
            )
            return

        camera_matrix = np.asarray(self.latest_camera_info.k).reshape(3, 3)
        distortion = np.asarray(self.latest_camera_info.d)
        object_points = self.board.getChessboardCorners()[ids.flatten()]
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points,
            corners.reshape(-1, 2),
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            self.get_logger().warn("Board pose estimation failed")
            return

        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        index = len(self.observations) + 1
        image_name = f"capture_{index:03d}.png"
        cv2.imwrite(str(self.output_dir / image_name), image)

        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        reprojection_rmse = float(
            np.sqrt(
                np.mean(
                    np.square(
                        projected.reshape(-1, 2) - corners.reshape(-1, 2)
                    )
                )
            )
        )
        observation = {
            "index": index,
            "image": image_name,
            "timestamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "robot_tf_timestamp": {
                "sec": int(tf_stamp.sec),
                "nanosec": int(tf_stamp.nanosec),
            },
            "robot_tf_age_seconds": tf_age_seconds,
            "used_latest_tf_fallback": used_latest_tf,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "base_frame": self.args.base_frame,
            "end_effector_frame": self.args.end_effector_frame,
            "camera_frame": image_message.header.frame_id,
            "base_to_end_effector": transform_to_dict(robot_transform),
            "camera_to_board": {
                "translation": {
                    "x": float(translation_vector[0, 0]),
                    "y": float(translation_vector[1, 0]),
                    "z": float(translation_vector[2, 0]),
                },
                "quaternion_xyzw": {
                    "x": float(quaternion[0]),
                    "y": float(quaternion[1]),
                    "z": float(quaternion[2]),
                    "w": float(quaternion[3]),
                },
            },
            "charuco_corner_ids": [int(value) for value in ids.flatten()],
            "reprojection_rmse_px": reprojection_rmse,
        }
        self.observations.append(observation)
        self.write_session()
        self.get_logger().info(
            f"Captured #{index}: {len(ids)} corners, "
            f"reprojection RMSE={reprojection_rmse:.3f} px, "
            f"TF age={tf_age_seconds:.3f} s"
        )

    def capture_validation(self):
        now = time.monotonic()
        if now - self.last_validation_capture_time < 1.0:
            return
        self.last_validation_capture_time = now
        if (
            self.latest_detection is None
            or self.latest_camera_info is None
            or self.latest_cloud_msg is None
        ):
            self.get_logger().warn("Image, camera info, or colored cloud is missing")
            return
        corners, ids, image_message, image = self.latest_detection
        if ids is None or len(ids) < self.args.minimum_corners:
            self.get_logger().warn("Not enough ChArUco corners; validation skipped")
            return

        cloud_message = self.latest_cloud_msg
        image_stamp = image_message.header.stamp
        cloud_stamp = cloud_message.header.stamp
        image_cloud_delta = (
            float(image_stamp.sec - cloud_stamp.sec)
            + float(image_stamp.nanosec - cloud_stamp.nanosec) * 1e-9
        )
        if abs(image_cloud_delta) > self.args.max_image_cloud_delta:
            self.get_logger().warn(
                f"Image/cloud timestamps differ by {image_cloud_delta:.3f} s; "
                "wait for fresh synchronized data"
            )
            return

        cloud_time = Time(seconds=cloud_stamp.sec, nanoseconds=cloud_stamp.nanosec)
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.end_effector_frame,
                cloud_time,
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().error(
                f"Exact cloud-timestamp TF lookup failed; validation skipped: {exc}"
            )
            return

        camera_matrix = np.asarray(self.latest_camera_info.k).reshape(3, 3)
        distortion = np.asarray(self.latest_camera_info.d)
        object_points = self.board.getChessboardCorners()[ids.flatten()]
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points,
            corners.reshape(-1, 2),
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            self.get_logger().warn("Board pose estimation failed")
            return
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion,
        )
        reprojection_rmse = float(
            np.sqrt(
                np.mean(
                    np.square(projected.reshape(-1, 2) - corners.reshape(-1, 2))
                )
            )
        )

        try:
            cloud = extract_xyzrgb(cloud_message)
        except Exception as exc:
            self.get_logger().error(f"Cloud extraction failed: {exc}")
            return

        self.validation_count += 1
        capture_dir = (
            self.output_dir / "validation" / f"capture_{self.validation_count:03d}"
        )
        capture_dir.mkdir(parents=True, exist_ok=False)
        cv2.imwrite(str(capture_dir / "color.png"), image)
        np.save(capture_dir / "cloud_xyzrgb.npy", cloud)
        save_xyzrgb_ply(capture_dir / "cloud.ply", cloud)

        metadata = {
            "index": self.validation_count,
            "capture_type": "hand_eye_point_cloud_validation",
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "image_timestamp": {
                "sec": int(image_stamp.sec),
                "nanosec": int(image_stamp.nanosec),
            },
            "cloud_timestamp": {
                "sec": int(cloud_stamp.sec),
                "nanosec": int(cloud_stamp.nanosec),
            },
            "image_cloud_delta_seconds": image_cloud_delta,
            "base_frame": self.args.base_frame,
            "end_effector_frame": self.args.end_effector_frame,
            "camera_frame": cloud_message.header.frame_id,
            "base_to_end_effector": transform_to_dict(robot_transform),
            "camera_to_board": {
                "translation": {
                    "x": float(translation_vector[0, 0]),
                    "y": float(translation_vector[1, 0]),
                    "z": float(translation_vector[2, 0]),
                },
                "quaternion_xyzw": {
                    "x": float(quaternion[0]),
                    "y": float(quaternion[1]),
                    "z": float(quaternion[2]),
                    "w": float(quaternion[3]),
                },
            },
            "charuco_corner_ids": [int(value) for value in ids.flatten()],
            "reprojection_rmse_px": reprojection_rmse,
            "cloud_point_count": int(len(cloud)),
            "camera_info": {
                "width": int(self.latest_camera_info.width),
                "height": int(self.latest_camera_info.height),
                "distortion_model": self.latest_camera_info.distortion_model,
                "k": [float(value) for value in self.latest_camera_info.k],
                "d": [float(value) for value in self.latest_camera_info.d],
            },
            "board": {
                "squares_x": self.args.squares_x,
                "squares_y": self.args.squares_y,
                "square_length_m": self.args.square_length,
                "marker_length_m": self.args.marker_length,
                "dictionary": self.args.dictionary,
            },
        }
        with (capture_dir / "meta.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)
        self.get_logger().info(
            f"Validation #{self.validation_count}: {len(cloud)} cloud points, "
            f"image/cloud delta={image_cloud_delta * 1000:.1f} ms, "
            f"reprojection RMSE={reprojection_rmse:.3f} px"
        )

    def write_session(self):
        camera_info = self.latest_camera_info
        document = {
            "capture_type": "eye_in_hand_hand_eye_calibration",
            "board": {
                "type": "charuco",
                "squares_x": self.args.squares_x,
                "squares_y": self.args.squares_y,
                "square_length_m": self.args.square_length,
                "marker_length_m": self.args.marker_length,
                "dictionary": self.args.dictionary,
            },
            "camera_info": {
                "width": int(camera_info.width),
                "height": int(camera_info.height),
                "distortion_model": camera_info.distortion_model,
                "k": [float(value) for value in camera_info.k],
                "d": [float(value) for value in camera_info.d],
            },
            "observations": self.observations,
        }
        final_path = self.output_dir / "observations.yaml"
        temporary_path = self.output_dir / "observations.yaml.tmp"
        with temporary_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        os.replace(temporary_path, final_path)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Interactively capture ChArUco eye-in-hand observations."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/thiwa/scan_data/handeye_calibration"),
    )
    parser.add_argument("--image-topic", default="/gemini336/color/image_raw")
    parser.add_argument(
        "--camera-info-topic", default="/gemini336/color/camera_info"
    )
    parser.add_argument(
        "--cloud-topic", default="/gemini336/depth_registered/points"
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--end-effector-frame", default="end_effector_link")
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-length", type=float, default=0.030)
    parser.add_argument("--marker-length", type=float, default=0.022)
    parser.add_argument(
        "--dictionary", choices=sorted(DICTIONARIES), default="DICT_4X4_50"
    )
    parser.add_argument("--minimum-corners", type=int, default=8)
    parser.add_argument(
        "--display-rate",
        type=float,
        default=10.0,
        help="Maximum ChArUco detection/display rate in Hz",
    )
    parser.add_argument(
        "--max-tf-age",
        type=float,
        default=10.0,
        help="Maximum latest-TF fallback age in seconds; arm must be stationary",
    )
    parser.add_argument(
        "--max-image-cloud-delta",
        type=float,
        default=0.10,
        help="Maximum RGB/cloud timestamp separation for validation captures",
    )
    return parser.parse_known_args()


def main():
    args, ros_arguments = parse_arguments()
    rclpy.init(args=ros_arguments)
    node = HandEyeCapture(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    window = "Gemini336 Hand-Eye Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            cv2.imshow(window, node.render())
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                node.capture()
            if key in (ord("c"), ord("C")):
                node.capture_validation()
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
