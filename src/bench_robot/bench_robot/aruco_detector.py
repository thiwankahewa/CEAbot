#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Bool, Int16, Int32MultiArray,Int16MultiArray


class ArucoManager(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---------------- state ----------------
        self.mode = "manual"
        self.auto_state = "idle"

        self.current_bench = 1
        self.current_row = 11
        self.goal_bench = 1
        self.goal_row = 11

        self.last_seen_marker = -1
        self.goal_seen_count = 0
        self.stop_sent = False

        # ---------------- params ----------------
        self.declare_parameter('usb_cam_index', 0)
        self.declare_parameter('usb_cam_width', 640)
        self.declare_parameter('usb_cam_height', 480)
        self.declare_parameter('usb_cam_fps', 30)

        self.declare_parameter('aruco_dictionary', 'DICT_4X4_100')
        self.declare_parameter('detect_period_s', 0.10)
        self.declare_parameter('stable_goal_frames', 3)

        self.declare_parameter('current_bench', 1)
        self.declare_parameter('current_row', 11)
        self.declare_parameter('goal_bench', 1)
        self.declare_parameter('goal_row', 11)

        self.declare_parameter('first_bench_id', 1)
        self.declare_parameter('last_bench_id', 10)
        self.declare_parameter('first_row_id', 11)
        self.declare_parameter('last_row_id', 61)

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)

        # ---------------- aruco setup ----------------
        self.aruco_dict = self._get_aruco_dict(self.aruco_dictionary_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ---------------- camera ----------------
        self.cap = None
        self._open_camera()

        # ---------------- subs ----------------
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_auto_state = self.create_subscription(String, '/auto_state', self.cb_auto_state, 10)

        # optional external goal updates
        self.sub_goal_bench = self.create_subscription(Int16, '/goal_bench', self.cb_goal_bench, 10)
        self.sub_goal_row = self.create_subscription(Int16, '/goal_row', self.cb_goal_row, 10)

        # optional scan done reset
        self.sub_scan_done = self.create_subscription(Bool, '/scan_done', self.cb_scan_done, 10)

        # ---------------- pubs ----------------
        self.pub_stop = self.create_publisher(Bool, '/aruco_stop_request', 10)
        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.pub_location = self.create_publisher(Int32MultiArray, '/robot_location', 10)
        self.pub_seen_id = self.create_publisher(Int16, '/aruco_seen_id', 10)
        self.pub_debug = self.create_publisher(Int16MultiArray, '/aruco_debug', 10)

        # ---------------- timer ----------------
        self.timer = self.create_timer(self.detect_period_s, self.detect_tick)

    def _load_params(self):
        self.usb_cam_index = int(self.get_parameter('usb_cam_index').value)
        self.usb_cam_width = int(self.get_parameter('usb_cam_width').value)
        self.usb_cam_height = int(self.get_parameter('usb_cam_height').value)
        self.usb_cam_fps = int(self.get_parameter('usb_cam_fps').value)

        self.aruco_dictionary_name = str(self.get_parameter('aruco_dictionary').value)
        self.detect_period_s = float(self.get_parameter('detect_period_s').value)
        self.stable_goal_frames = int(self.get_parameter('stable_goal_frames').value)

        self.current_bench = int(self.get_parameter('current_bench').value)
        self.current_row = int(self.get_parameter('current_row').value)
        self.goal_bench = int(self.get_parameter('goal_bench').value)
        self.goal_row = int(self.get_parameter('goal_row').value)

        self.first_bench_id = int(self.get_parameter('first_bench_id').value)
        self.last_bench_id = int(self.get_parameter('last_bench_id').value)
        self.first_row_id = int(self.get_parameter('first_row_id').value)
        self.last_row_id = int(self.get_parameter('last_row_id').value)

    def on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def _get_aruco_dict(self, name: str):
        mapping = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        if name not in mapping:
            self.get_logger().warn(f'Unknown dictionary {name}, using DICT_4X4_100')
            name = 'DICT_4X4_100'
        return cv2.aruco.getPredefinedDictionary(mapping[name])
    
    def marker_to_location(self, marker_id: int):
        # bench start markers
        if self.first_bench_id <= marker_id <= self.last_bench_id:
            return marker_id, self.first_row_id

        # row markers on current bench
        if self.first_row_id <= marker_id <= self.last_row_id:
            return self.current_bench, marker_id

        return self.current_bench, self.current_row

    def publish_stop(self, stop: bool):
        self.pub_stop.publish(Bool(data=bool(stop)))

    def publish_location(self, bench: int, row: int, marker_id: int):
        msg = Int32MultiArray()
        msg.data = [int(bench), int(row), int(marker_id)]
        self.pub_location.publish(msg)

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.usb_cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.usb_cam_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.usb_cam_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.usb_cam_fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open USB camera index {self.usb_cam_index}")
            self.cap = None
        else:
            self.get_logger().info(f"USB camera opened at index {self.usb_cam_index}")

    def request_tracking_direction(self):
        if self.current_bench != self.goal_bench:
            # For now, do nothing here. Later we can add bench-to-bench routing.
            return

        if self.current_row < self.goal_row:
            desired_state = "bench_tracking_f"
        elif self.current_row > self.goal_row:
            desired_state = "bench_tracking_b"
        else:
            return

        if desired_state != self.auto_state:
            self.get_logger().info(
                f"Direction/state change: current=({self.current_bench},{self.current_row}) "
                f"goal=({self.goal_bench},{self.goal_row}) -> {desired_state}"
            )
            self.pub_auto_state_cmd.publish(String(data=desired_state))

    def cb_mode(self, msg: String):
        self.mode = (msg.data or "").strip().lower()
        if self.mode != "auto":
            self.goal_seen_count = 0
            self.stop_sent = False
            self.publish_stop(False)

    def cb_auto_state(self, msg: String):
        self.auto_state = (msg.data or "").strip().lower()
        if self.auto_state not in ("bench_tracking_f", "bench_tracking_b"):
            self.goal_seen_count = 0

    def cb_goal_bench(self, msg: Int16):
        self.goal_bench = int(msg.data)
        self.goal_seen_count = 0
        self.stop_sent = False
        self.publish_stop(False)
        self.get_logger().info(f"goal_bench updated to {self.goal_bench}")

    def cb_goal_row(self, msg: Int16):
        self.goal_row = int(msg.data)
        self.goal_seen_count = 0
        self.stop_sent = False
        self.publish_stop(False)
        self.get_logger().info(f"goal_row updated to {self.goal_row}")

    def cb_scan_done(self, msg: Bool):
        if bool(msg.data):
            self.stop_sent = False
            self.goal_seen_count = 0
            self.publish_stop(False)

    

    def detect_tick(self):
        if self.mode != "auto":
            return

        if self.auto_state not in ("bench_tracking_f", "bench_tracking_b"):
            return

        if self.cap is None:
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.goal_seen_count = 0
            return
        

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self.goal_seen_count = 0
            return

        '''cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)'''

        ids = ids.flatten().tolist()

        selected_id = None

        # Prefer goal marker if visible
        for marker_id in ids:
            bench, row = self.marker_to_location(int(marker_id))
            if bench == self.goal_bench and row == self.goal_row:
                selected_id = int(marker_id)
                break

        if selected_id is None:
            selected_id = int(ids[0])

        self.last_seen_marker = selected_id
        self.pub_seen_id.publish(Int16(data=selected_id))

        new_bench, new_row = self.marker_to_location(selected_id)
        self.current_bench = new_bench
        self.current_row = new_row
        self.publish_location(self.current_bench, self.current_row, selected_id)
        self.request_tracking_direction()

        msg = Int16MultiArray()
        msg.data = [
            int(selected_id),
            int(self.current_bench),
            int(self.current_row),
            int(self.goal_bench),
            int(self.goal_row),
        ]
        self.pub_debug.publish(msg)
        self.get_logger().info(
        f"marker={selected_id}, current=({self.current_bench},{self.current_row}), "
        f"goal=({self.goal_bench},{self.goal_row})"
        )

        if self.current_bench == self.goal_bench and self.current_row == self.goal_row:
            self.goal_seen_count += 1
        else:
            self.goal_seen_count = 0

        if self.goal_seen_count >= self.stable_goal_frames and not self.stop_sent:
            self.stop_sent = True
            self.publish_stop(True)
            self.get_logger().info(
                f"Goal reached: bench={self.current_bench}, row={self.current_row}, marker={selected_id}"
            )

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        node.publish_stop(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()