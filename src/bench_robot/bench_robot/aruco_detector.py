#!/usr/bin/env python3

import cv2
import numpy as np
from std_srvs.srv import Trigger
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16, Int16MultiArray, Float32MultiArray

FIRST_BENCH_ID = 1
LAST_BENCH_ID = 10
FIRST_ROW_ID = 11
LAST_ROW_ID = 61
END_ROWS = [FIRST_ROW_ID, LAST_ROW_ID]

class ArucoManager(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---------------- states and variables ----------------
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

        self.scan_plan = []
        self.scan_plan_index = 0
        self.pending_next_plan_index = None
        self.routing_to_next_bench = False

        self.goal_seen_count = 0
        self.prev_selected_id = None
        self.stop_sent = False

        self.stable_goal_frames = 2
        self.usb_cam_index = 0
        self.usb_cam_width = 640
        self.usb_cam_height = 480
        self.usb_cam_fps = 15

                # ---------------- services ----------------
        self.srv_reconnect = self.create_service(Trigger,"/aruco_detector/aruco_camera_reconnect",self.on_camera_reconnect)

        # ---------------- camera, aruco setup ----------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.cap = None
        self._open_camera()


        # ---------------- subs ----------------
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
        self.timer = self.create_timer(0.1, self.detect_tick)
    
        # ---------------- helper functions ----------------
    
    def marker_to_location(self, marker_id: int):
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
        self.cap = cv2.VideoCapture("/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0",cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.usb_cam_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.usb_cam_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.usb_cam_fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open USB camera index {self.usb_cam_index}")
            self.cap.release()
            self.cap = None
            return False
        
        self.get_logger().info(f"Aruco camera opened at index {self.usb_cam_index}")
        return True
    
    def on_camera_reconnect(self, request, response):
        try:
            self.get_logger().warn("Aruco camera reconnect requested...")

            try:
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
                    time.sleep(0.5)
            except Exception as e:
                self.get_logger().warning(f"Aruco camera release failed: {e}")

            self.goal_seen_count = 0
            self.prev_selected_id = None
            self.stop_sent = False
            self.publish_stop(False)
            self.publish_align_error(False)

            ok = self._open_camera()
            if not ok:
                raise RuntimeError("Aruco camera reopen failed")

            time.sleep(0.2)
            frame_ok, frame = self.cap.read()

            if not frame_ok or frame is None:
                raise RuntimeError("Aruco camera opened, but frame read failed")

            response.success = True
            response.message = "Aruco camera reconnected successfully."
            self.get_logger().info("Aruco camera reconnected successfully.")

        except Exception as e:
            response.success = False
            response.message = f"Aruco camera reconnect failed: {e}"
            self.get_logger().error(response.message)

        return response

    def request_tracking_direction(self):
        if self.current_bench != self.goal_bench:
            if self.current_row in END_ROWS:
                desired_state = "bench_change_start"
            else:
                # Move toward nearest bench end first
                dist_to_first = abs(self.current_row - FIRST_ROW_ID)
                dist_to_last = abs(self.current_row - LAST_ROW_ID)

                if dist_to_last <= dist_to_first:
                    desired_state = "bench_tracking_f"
                else:
                    desired_state = "bench_tracking_b"

            if desired_state != self.auto_state:
                self.get_logger().info(
                    f"Multi-bench routing: current=({self.current_bench},{self.current_row}) "
                    f"goal=({self.goal_bench},{self.goal_row}) -> {desired_state}"
                )
                self.pub_auto_state_cmd.publish(String(data=desired_state))

            return

        if self.current_row < self.goal_row:
            desired_state = "bench_tracking_f"
        elif self.current_row > self.goal_row:
            desired_state = "bench_tracking_b"
        else:
            return

        if desired_state != self.auto_state:
            self.pub_auto_state_cmd.publish(String(data=desired_state))

    def advance_to_next_goal(self):
        # Normal scanning inside current scan range
        if self.goal_row != self.scan_end_row:
            self.goal_row += 1
            self.get_logger().info(f"Next goal of current scan range: ({self.goal_bench},{self.goal_row})")
            self.request_tracking_direction()
            return 

        # Current scan range finished
        next_index = self.scan_plan_index + 1

        if next_index >= len(self.scan_plan):
            self.pub_auto_state_cmd.publish(String(data="idle"))
            self.get_logger().info("Full scan range plan complete")
            return 

        next_bench, next_from_row, next_to_row = self.scan_plan[next_index]

        # If next scan range is on same bench, directly start next scan row
        if next_bench == self.current_bench:
            self.scan_plan_index = next_index
            self.goal_row = next_from_row
            self.scan_start_bench = next_bench
            self.scan_start_row = next_from_row
            self.scan_end_bench = next_bench
            self.scan_end_row = next_to_row
            self.get_logger().info(f"Next scan range on same bench")
            self.request_tracking_direction()
            return 

        # Next scan range is on another bench: route to nearest exit first, then enter next bench, then move to next_from_row.
        self.routing_to_next_bench = True
        self.pending_next_plan_index = next_index
        dist_to_first = abs(self.current_row - FIRST_ROW_ID)
        dist_to_last = abs(self.current_row - LAST_ROW_ID)
        exit_row = FIRST_ROW_ID if dist_to_first <= dist_to_last else LAST_ROW_ID
        self.goal_row = exit_row
        self.get_logger().info(f"Starting route to current bench end: "f"current=({self.current_bench},{self.current_row}), "f"exit=({self.current_bench},{self.goal_row}), "f"next=({next_bench},{next_from_row})")
        self.request_tracking_direction()
        return 

    def is_in_selected_range(self, bench: int, row: int) -> bool:
        if not self.scan_plan:
            return bench == self.goal_bench and row == self.goal_row
        
        current_bench, from_row, to_row = self.scan_plan[self.scan_plan_index]
        if bench != current_bench:
            return False

        low_row = min(from_row, to_row)
        high_row = max(from_row, to_row)

        return low_row <= row <= high_row
    
    def is_active_stop_target(self, bench: int, row: int) -> bool:
        if not self.is_in_selected_range(bench, row):
            return False

        return bench == self.goal_bench and row == self.goal_row

    # ---------------- callbacks ----------------
    def cb_auto_state(self, msg: String):
        self.auto_state = (msg.data or "").strip().lower()
        if self.auto_state not in ("bench_tracking_f", "bench_tracking_b"):
            self.goal_seen_count = 0

    def cb_current_bench(self, msg: Int16):
        self.current_bench = int(msg.data)

    def cb_goal_locations(self, msg: Int16MultiArray):
        data = list(msg.data)
        if len(data) < 3 or len(data) % 3 != 0:
            self.get_logger().warn("Goal locations must be [bench, from_row, to_row, bench, from_row, to_row, ...]")
            return
        
        self.scan_plan = []
        for i in range(0, len(data), 3):
            bench = int(data[i])
            from_row = int(data[i + 1])
            to_row = int(data[i + 2])
            self.scan_plan.append((bench, from_row, to_row))

        self.scan_plan_index = 0
        self.current_row = None
        self.row_known = False
        self.stop_sent = False
        self.goal_seen_count = 0
        self.publish_stop(False)

        first_bench, first_from_row, first_to_row = self.scan_plan[0]
        self.scan_start_bench = first_bench
        self.scan_start_row = first_from_row
        self.scan_end_bench = first_bench
        self.scan_end_row = first_to_row
        self.get_logger().info(f"New scan plan received. "f"Starting with ({self.goal_bench},{self.goal_row})")
        self.get_logger().info(f"Waiting to detect current row")

        if self.auto_state != "bench_tracking_f":
            self.pub_auto_state_cmd.publish(String(data="bench_tracking_f"))

    def cb_scan_done(self, msg: Bool):
        if bool(msg.data):
            self.stop_sent = False
            self.goal_seen_count = 0
            self.publish_stop(False)
            self.advance_to_next_goal()

    # ---------------- timer functions ----------------

    def detect_tick(self):
        if self.auto_state not in ("bench_tracking_f", "bench_tracking_b", "aruco_centering"):
            self.publish_align_error(False)
            return

        if self.cap is None:
            self.publish_align_error(False)
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().error("Failed to read frame from aruco camera")
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

        # this block run in first cycle only
        if self.current_bench is not None and not self.row_known:
            if FIRST_ROW_ID <= selected_id <= LAST_ROW_ID:
                self.current_row = selected_id
                self.row_known = True
                self.get_logger().info(f"Initial row fixed: current=({self.current_bench},{self.current_row})")
                self.goal_bench = self.scan_start_bench
                self.goal_row = self.scan_start_row
                self.get_logger().info(f"Range start selected : "f"start=({self.goal_bench},{self.goal_row}), "f"end=({self.scan_end_bench},{self.scan_end_row})")

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

        # ---------------- routing mode ----------------
        if self.routing_to_next_bench:
            routing_goal_visible = active_goal_visible and center_error_px is not None

            if routing_goal_visible:
                self.goal_seen_count += 1
            else:
                self.goal_seen_count = 0

            if self.goal_seen_count >= self.stable_goal_frames:
                self.goal_seen_count = 0
                self.prev_selected_id = None

                next_bench, next_from_row, next_to_row = self.scan_plan[self.pending_next_plan_index]

                # Reached current bench exit row. Now start bench_changer.
                if self.current_row in END_ROWS:
                    self.get_logger().info(f"Reached bench exit: current=({self.current_bench},{self.current_row}), "f"next_bench={next_bench}. Starting bench change.")
                    self.pub_auto_state_cmd.publish(String(data="bench_change_start"))
                    return
            return

        if active_goal_visible and center_error_px is not None:
            self.goal_seen_count += 1
            self.publish_align_error(True, center_error_px)
            self.get_logger().debug(f"Goal visible: center_error_px={center_error_px:.1f}")
        else:
            self.goal_seen_count = 0
            self.publish_align_error(False, 0.0)
            
        if self.goal_seen_count >= self.stable_goal_frames and not self.stop_sent:
            self.stop_sent = True
            self.publish_stop(True)
            self.prev_selected_id = None
            logged_center_error_px = center_error_px if center_error_px is not None else 0.0
            self.get_logger().info(f"Goal centered and reached: bench={self.current_bench}, row={self.current_row}, center_error_px={logged_center_error_px:.1f}")

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
