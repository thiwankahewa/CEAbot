#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs import msg
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Float32, String, Bool


class BenchChanger(Node):
    def __init__(self):
        super().__init__("bench_changer")

        # ---------------- state ----------------
        self.mode = "manual"
        self.state = "idle"

        self.marker_id = None
        self.current_bench = None
        self.current_row = None
        self.goal_bench = None
        self.goal_row = None

        self.range_from_bench = None
        self.range_from_row = None
        self.range_to_bench = None
        self.range_to_row = None

        self.tof = None
        self.center_marker = None
        self.left_bench = None
        self.right_bench = None

        # ---------------- parameters ----------------
        self.declare_parameter("min_tof", 25)
        self.declare_parameter("max_tof", 500)
        self.declare_parameter("exit_rpm", 5.0)
        self.declare_parameter("enter_rpm", 5.0)
        self.declare_parameter("turn_rpm", 4.0)
        self.declare_parameter("steer_straight_deg", 0.0)
        self.declare_parameter("tof_timeout_s", 0.5)

        self.min_tof = int(self.get_parameter("min_tof").value)
        self.max_tof = int(self.get_parameter("max_tof").value)
        self.exit_rpm = float(self.get_parameter("exit_rpm").value)
        self.enter_rpm = float(self.get_parameter("enter_rpm").value)
        self.turn_rpm = float(self.get_parameter("turn_rpm").value)
        self.steer_straight_deg = float(self.get_parameter("steer_straight_deg").value)

        # ---------------- subscribers ----------------
        self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.sub_auto_state = self.create_subscription(String,"/auto_state",self.cb_auto_state,10)
        self.create_subscription(Int16MultiArray, "/robot_location", self.cb_location, 10)
        self.create_subscription(Int16MultiArray, "/goal_locations", self.cb_goal_locations, 10)
        self.create_subscription(Int16MultiArray, "/bench_robot/tof_raw", self.cb_tof, 10)

        # Expected message:
        # [center_marker_visible, left_bench_number, right_bench_number]
        self.create_subscription(Int16MultiArray, "/side_bench_markers", self.cb_side_markers, 10)

        # Optional trigger from row scanner / aruco node when end of bench is reached
        self.create_subscription(Bool, "/bench_change_request", self.cb_change_request, 10)

        # ---------------- publishers ----------------
        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_rpm_cmd = self.create_publisher(Float32MultiArray, "/wheel_rpm_cmd", 10)
        self.pub_steer = self.create_publisher(Float32, "/steer_angle_deg", 10)

        self.timer = self.create_timer(0.05, self.control_tick)

    # ---------------- callbacks ----------------

    def cb_mode(self, msg: String):
        self.mode = (msg.data or "").strip().lower()

        if self.mode != "auto":
            self.stop()
            self.state = "idle"

    def cb_auto_state(self, msg: String):
        self.auto_state = (msg.data or "").strip().lower()

    def cb_location(self, msg: Int16MultiArray):
        data = list(msg.data)

        if len(data) < 5:
            return

        self.marker_id = int(data[0])
        self.current_bench = int(data[1])
        self.current_row = int(data[2])
        self.goal_bench = int(data[3])
        self.goal_row = int(data[4])

    def cb_goal_locations(self, msg: Int16MultiArray):
        data = list(msg.data)

        if len(data) < 4:
            self.get_logger().warn("Goal location message should be [from_bench, from_row, to_bench, to_row]")
            return

        self.range_from_bench = int(data[0])
        self.range_from_row = int(data[1])
        self.range_to_bench = int(data[2])
        self.range_to_row = int(data[3])

    def cb_tof(self, msg: Int16MultiArray):
        data = list(msg.data)

        if len(data) < 4:
            self.tof = None
            return

        # Existing order from your bench tracker:
        # rl, fl, rr, fr
        self.tof = {
            "rl": int(data[0]),
            "fl": int(data[1]),
            "rr": int(data[2]),
            "fr": int(data[3]),
        }

    def cb_side_markers(self, msg: Int16MultiArray):
        data = list(msg.data)

        if len(data) < 3:
            return

        self.center_marker = int(data[0])
        self.left_bench = int(data[1])
        self.right_bench = int(data[2])

    def cb_change_request(self, msg: Bool):
        if not msg.data:
            return

        if self.should_change_bench():
            self.get_logger().info("Bench change requested. Starting bench change.")
            self.state = "exit_current_bench"
            self.set_auto_state("bench_changing")
        else:
            self.get_logger().info("Bench change requested, but no next bench is needed.")

    # ---------------- helper functions ----------------

    def valid_tof(self, x):
        return self.min_tof <= x <= self.max_tof

    def all_side_tof_out_of_range(self):
        if self.tof is None:
            return False

        return not any(
            self.valid_tof(v)
            for v in [self.tof["rl"], self.tof["fl"], self.tof["rr"], self.tof["fr"]]
        )

    def middle_tof_valid(self):
        """
        For entering the next bench, use the front/middle pair first.
        You can change this depending on which sensors see the bench first.
        """
        if self.tof is None:
            return False

        return self.valid_tof(self.tof["fl"]) and self.valid_tof(self.tof["fr"])

    def all_tof_valid(self):
        if self.tof is None:
            return False

        return all(
            self.valid_tof(v)
            for v in [self.tof["rl"], self.tof["fl"], self.tof["rr"], self.tof["fr"]]
        )

    def should_change_bench(self):
        if self.current_bench is None or self.range_to_bench is None:
            return False

        return self.current_bench != self.range_to_bench

    def target_bench_is_left(self):
        return self.left_bench == self.range_to_bench

    def target_bench_is_right(self):
        return self.right_bench == self.range_to_bench

    def publish_rpm(self, left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.pub_rpm_cmd.publish(msg)

    def publish_steer(self, deg):
        self.pub_steer.publish(Float32(data=float(deg)))

    def stop(self):
        self.publish_rpm(0.0, 0.0)

    def set_auto_state(self, state_name):
        self.pub_auto_state_cmd.publish(String(data=state_name))

    # ---------------- main state machine ----------------

    def control_tick(self):
        if self.mode != "auto":
            return

        if self.auto_state != "bench_change_start":
            return

        self.publish_steer(self.steer_straight_deg)

        if self.state == "exit_current_bench":
            self.run_exit_current_bench()
            return

        if self.state == "find_target_bench":
            self.run_find_target_bench()
            return

        if self.state == "enter_target_bench":
            self.run_enter_target_bench()
            return

        if self.state == "finish":
            self.finish_bench_change()
            return

        self.stop()

    def run_exit_current_bench(self):
        """
        Move forward out of the current bench until all side ToF sensors lose the bench.
        This bypasses the normal bench tracker safety stop intentionally.
        """
        self.publish_rpm(self.exit_rpm, self.exit_rpm)

        if self.all_side_tof_out_of_range():
            self.stop()
            self.get_logger().info("Robot is out of current bench. Looking for target bench.")
            self.state = "find_target_bench"

    def run_find_target_bench(self):
        """
        Later you can replace this with ArUco centering logic.
        For now:
        - if target bench is left, rotate/move left
        - if target bench is right, rotate/move right
        - when center marker is visible, enter target bench
        """

        if self.center_marker == 1:
            self.stop()
            self.get_logger().info("Centered with target bench. Entering bench.")
            self.state = "enter_target_bench"
            return

        if self.target_bench_is_left():
            self.get_logger().info("Target bench is on left side.")
            self.publish_rpm(-self.turn_rpm, self.turn_rpm)
            return

        if self.target_bench_is_right():
            self.get_logger().info("Target bench is on right side.")
            self.publish_rpm(self.turn_rpm, -self.turn_rpm)
            return

        self.stop()
        self.get_logger().warn(
            f"Target bench {self.range_to_bench} not found. "
            f"Left={self.left_bench}, Right={self.right_bench}"
        )

    def run_enter_target_bench(self):
        """
        Move into the next bench until the middle/front ToF sensors become valid.
        Then hand control back to normal bench tracking.
        """
        self.publish_rpm(self.enter_rpm, self.enter_rpm)

        if self.middle_tof_valid():
            self.stop()
            self.get_logger().info("Entered target bench. Returning to bench tracking.")
            self.state = "finish"

    def finish_bench_change(self):
        self.stop()

        # Choose direction depending on your scan direction.
        # For now, continue forward tracking.
        self.set_auto_state("bench_tracking_f")

        self.state = "idle"


def main(args=None):
    rclpy.init(args=args)
    node = BenchChanger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()