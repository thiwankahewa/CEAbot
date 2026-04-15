#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Bool, Int16, Int32MultiArray, Int16MultiArray, Float32MultiArray

FIRST_BENCH_ID = 1
LAST_BENCH_ID = 10
FIRST_ROW_ID = 11
LAST_ROW_ID = 61

class ArucoManager(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---------------- state ----------------
        self.mode = "manual"
        self.auto_state = "idle"

        self.current_bench = None
        self.current_row = None
        self.row_known = False

        self.range_from_bench = 1
        self.range_from_row = 11
        self.range_to_bench = 1
        self.range_to_row = 11

        self.scan_start_bench = 1
        self.scan_start_row = 11
        self.scan_end_bench = 1
        self.scan_end_row = 11

        self.goal_bench = 1
        self.goal_row = 11

        self.goal_seen_count = 0
        self.prev_selected_id = None
        self.stop_sent = False
        self.range_active = False

        # ---------------- params ----------------
        self.declare_parameter('usb_cam_index', 0)
        self.declare_parameter('usb_cam_width', 640)
        self.declare_parameter('usb_cam_height', 480)
        self.declare_parameter('usb_cam_fps', 15)

        self.declare_parameter('detect_period_s', 0.10)
        self.declare_parameter('stable_goal_frames', 3)

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)

        # ---------------- camera, aruco setup ----------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.cap = None
        self._open_camera()

        # ---------------- subs ----------------
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_auto_state = self.create_subscription(String, '/auto_state', self.cb_auto_state, 10)
        self.sub_goal_locations = self.create_subscription(Int16MultiArray, '/goal_locations', self.cb_goal_locations, 10)
        self.sub_current_bench = self.create_subscription(Int16, '/current_bench', self.cb_current_bench, 10)
        self.sub_scan_done = self.create_subscription(Bool, '/scan_done', self.cb_scan_done, 10)

        # ---------------- pubs ----------------
        self.pub_stop = self.create_publisher(Bool, '/aruco_stop_request', 10)
        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.pub_location = self.create_publisher(Int16MultiArray, '/robot_location', 10)
        self.pub_align_error = self.create_publisher(Float32MultiArray, '/aruco_target_error', 10)

        # ---------------- timer ----------------
        self.timer = self.create_timer(self.detect_period_s, self.detect_tick)

        # ---------------- helper ----------------

    def _load_params(self):
        self.usb_cam_index = int(self.get_parameter('usb_cam_index').value)
        self.usb_cam_width = int(self.get_parameter('usb_cam_width').value)
        self.usb_cam_height = int(self.get_parameter('usb_cam_height').value)
        self.usb_cam_fps = int(self.get_parameter('usb_cam_fps').value)

        self.detect_period_s = float(self.get_parameter('detect_period_s').value)
        self.stable_goal_frames = int(self.get_parameter('stable_goal_frames').value)

    def on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)
    
    def marker_to_location(self, marker_id: int):
        # bench start markers
        if FIRST_BENCH_ID <= marker_id <= LAST_BENCH_ID:
            return marker_id, FIRST_ROW_ID

        # row markers on current bench
        if FIRST_ROW_ID <= marker_id <= LAST_ROW_ID:
            return self.current_bench, marker_id

        return self.current_bench, self.current_row

    def publish_stop(self, stop: bool):
        self.pub_stop.publish(Bool(data=bool(stop)))

    def publish_location(self, marker_id: int, bench: int, row: int, goal_bench: int, goal_row: int):
        if bench is None or row is None or goal_bench is None or goal_row is None:
            self.get_logger().warn(f"Skipping publish_location: marker={marker_id}, bench={bench}, row={row}, "f"goal_bench={goal_bench}, goal_row={goal_row}")
            return
        msg = Int16MultiArray()
        msg.data = [int(marker_id), int(bench), int(row),int(goal_bench), int(goal_row) ]
        self.pub_location.publish(msg)

    def publish_align_error(self, visible: bool, center_error_px: float = 0.0):
        msg = Float32MultiArray()
        msg.data = [1.0 if visible else 0.0, float(center_error_px)]
        self.pub_align_error.publish(msg)

    @staticmethod
    def marker_center_x(marker_corners) -> float:
        pts = np.asarray(marker_corners).reshape(-1, 2)
        return float(np.mean(pts[:, 1]))

    def _open_camera(self):
        self.cap = cv2.VideoCapture("/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam-B0578-2.3MP-GS_SN001-video-index0",cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.usb_cam_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.usb_cam_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.usb_cam_fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open USB camera index {self.usb_cam_index}")
            self.cap = None
        else:
            self.get_logger().info(f"USB camera opened at index {self.usb_cam_index}")

    def request_tracking_direction(self):
        if self.current_bench is None or self.current_row is None or not self.row_known:
            return
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
            self.get_logger().info(f"Direction/state change: current=({self.current_bench},{self.current_row}) "f"goal=({self.goal_bench},{self.goal_row}) -> {desired_state}")
            self.pub_auto_state_cmd.publish(String(data=desired_state))

    # ---------------- callbacks ----------------

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

    def cb_current_bench(self, msg: Int16):
        self.current_bench = int(msg.data)
        self.current_row = None
        self.row_known = False
        self.goal_seen_count = 0
        self.stop_sent = False
        self.publish_stop(False)

        self.get_logger().info(f"Current bench set to {self.current_bench}, row unknown")
        if self.auto_state != "bench_tracking_f":
            self.pub_auto_state_cmd.publish(String(data="bench_tracking_f"))


    def cb_goal_locations(self, msg: Int16MultiArray):
        data = list(msg.data)

        fb, fr, tb, tr = map(int, data[:4])

        self.range_from_bench = fb
        self.range_from_row = fr
        self.range_to_bench = tb
        self.range_to_row = tr

        self.range_active = True
        self.stop_sent = False
        self.goal_seen_count = 0
        self.publish_stop(False)

         # If row is not known yet, first localize row by moving
        if self.current_bench is not None and not self.row_known:
            self.get_logger().info(f"Range received: ({fb},{fr}) -> ({tb},{tr}), waiting to detect current row on bench {self.current_bench}")
            if self.auto_state != "bench_tracking_f":
                self.pub_auto_state_cmd.publish(String(data="bench_tracking_f"))
            return

        self.choose_range_start_end()

        self.goal_bench = self.scan_start_bench
        self.goal_row = self.scan_start_row

        self.get_logger().info(f"New range: ({fb},{fr}) -> ({tb},{tr}), "f"start=({self.goal_bench},{self.goal_row}), "f"end=({self.scan_end_bench},{self.scan_end_row})")

        self.request_tracking_direction()

    def choose_range_start_end(self):
        fb, fr = self.range_from_bench, self.range_from_row
        tb, tr = self.range_to_bench, self.range_to_row

        if self.current_bench is None or self.current_row is None:
            self.get_logger().warn("Cannot choose range start/end because current location is incomplete")
            return

        # same exact point
        if fb == tb and fr == tr:
            self.scan_start_bench = fb
            self.scan_start_row = fr
            self.scan_end_bench = tb
            self.scan_end_row = tr
            return

        # same bench range
        if fb == tb == self.current_bench:
            d_from = abs(self.current_row - fr)
            d_to = abs(self.current_row - tr)

            if d_from <= d_to:
                self.scan_start_bench = fb
                self.scan_start_row = fr
                self.scan_end_bench = tb
                self.scan_end_row = tr
            else:
                self.scan_start_bench = tb
                self.scan_start_row = tr
                self.scan_end_bench = fb
                self.scan_end_row = fr
            return

        # fallback
        self.scan_start_bench = fb
        self.scan_start_row = fr
        self.scan_end_bench = tb
        self.scan_end_row = tr

    def cb_scan_done(self, msg: Bool):
        if bool(msg.data):
            self.stop_sent = False
            self.goal_seen_count = 0
            self.publish_stop(False)
            if self.range_active:
                has_next = self.advance_to_next_goal()
                if not has_next:
                    self.pub_auto_state_cmd.publish(String(data="idle"))
            else:
                self.pub_auto_state_cmd.publish(String(data="idle"))

    def advance_to_next_goal(self):
        # only same-bench range for now
        if self.goal_bench != self.scan_end_bench:
            self.get_logger().warn("Multi-bench range advance not implemented yet")
            self.range_active = False
            return False

        if self.goal_row == self.scan_end_row:
            self.get_logger().info("Range scan complete")
            self.range_active = False
            return False

        step = 1 if self.scan_end_row > self.goal_row else -1
        self.goal_row += step
        self.goal_bench = self.scan_end_bench

        self.get_logger().info(
            f"Next goal in range: ({self.goal_bench},{self.goal_row})"
        )

        self.request_tracking_direction()
        return True

    def is_in_selected_range(self, bench: int, row: int) -> bool:
        if not self.range_active:
            return bench == self.goal_bench and row == self.goal_row

        # only same-bench range for now
        if self.range_from_bench != self.range_to_bench:
            return False

        if bench != self.range_from_bench:
            return False

        low = min(self.range_from_row, self.range_to_row)
        high = max(self.range_from_row, self.range_to_row)

        return low <= row <= high
    
    def is_active_stop_target(self, bench: int, row: int) -> bool:
        if bench is None or row is None:
            return False

        if not self.is_in_selected_range(bench, row):
            return False

        return bench == self.goal_bench and row == self.goal_row

    

    def detect_tick(self):
        if self.mode != "auto":
            self.publish_align_error(False)
            return

        if self.auto_state not in ("bench_tracking_f", "bench_tracking_b","align_center", "aruco_centering"):
            self.publish_align_error(False)
            return

        if self.cap is None:
            self.publish_align_error(False)
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.goal_seen_count = 0
            self.publish_align_error(False)
            return
        

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self.goal_seen_count = 0
            self.publish_align_error(False)
            return

        ids = ids.flatten().tolist()

        selected_id = None
        selected_corners = None

        # Prefer goal marker if visible
        for idx, marker_id in enumerate(ids):
            bench, row = self.marker_to_location(int(marker_id))
            if bench == self.goal_bench and row == self.goal_row:
                selected_id = int(marker_id)
                selected_corners = corners[idx]
                break

        if selected_id is None:
            selected_id = int(ids[0])
            selected_corners = corners[0]

        # If bench is known but row is not, use the first row marker to initialize row
        if self.current_bench is not None and not self.row_known:
            if FIRST_ROW_ID <= selected_id <= LAST_ROW_ID:
                self.current_row = selected_id
                self.row_known = True

                self.get_logger().info(f"Initial row fixed: current=({self.current_bench},{self.current_row})")

                # If a range is already active, now we can decide nearest endpoint
                if self.range_active:
                    self.choose_range_start_end()
                    self.goal_bench = self.scan_start_bench
                    self.goal_row = self.scan_start_row

                    self.get_logger().info(f"Range start selected after row detection: "f"start=({self.goal_bench},{self.goal_row}), "f"end=({self.scan_end_bench},{self.scan_end_row})")

                self.request_tracking_direction()
            return

        new_bench, new_row = self.marker_to_location(selected_id)
        self.current_bench = new_bench
        self.current_row = new_row
        
        self.request_tracking_direction()

        if selected_id != self.prev_selected_id:
            self.publish_location(selected_id, self.current_bench, self.current_row,  self.goal_bench, self.goal_row)
            self.get_logger().info(f"marker={selected_id}, current=({self.current_bench},{self.current_row}), "f"goal=({self.goal_bench},{self.goal_row})")
            self.prev_selected_id = selected_id

        center_error_px = None
        active_goal_visible = False
        if selected_corners is not None:
            frame_center_x = frame.shape[0] / 2.0
            marker_center_x = self.marker_center_x(selected_corners)
            center_error_px = marker_center_x - frame_center_x
            active_goal_visible = self.is_active_stop_target(self.current_bench, self.current_row)

        if active_goal_visible and center_error_px is not None:
            self.publish_align_error(True, center_error_px)
            self.get_logger().debug(f"Goal visible: center_error_px={center_error_px:.1f}")
        else:
            self.publish_align_error(False, 0.0)

        if active_goal_visible:
            self.goal_seen_count += 1
        else:
            self.goal_seen_count = 0

        if self.goal_seen_count >= self.stable_goal_frames and not self.stop_sent:
            self.stop_sent = True
            self.publish_stop(True)
            self.prev_selected_id = None
            logged_center_error_px = center_error_px if center_error_px is not None else 0.0

            self.get_logger().info(f"Goal centered and reached: bench={self.current_bench}, row={self.current_row}, "f"marker={selected_id}, center_error_px={logged_center_error_px:.1f}")

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
        node.publish_stop(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
