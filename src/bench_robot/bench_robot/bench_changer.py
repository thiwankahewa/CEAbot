#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray, Float32


class BenchChanger(Node):
    def __init__(self):
        super().__init__("bench_changer")

        self.auto_state = "idle"
        self.phase = "idle"

        self.current_bench = None
        self.goal_bench = None
        self.goal_row = None

        self.side_marker_visible = False
        self.side_marker_bench = None
        self.side_marker_err_px = 0.0
        self.side_marker_orientation = 0.0

        self.tof = None
        self.phase_start_time = None

        self.declare_parameter("exit_time_s", 2.0)
        self.declare_parameter("exit_rpm", 6.0)
        self.declare_parameter("side_rpm", 5.0)
        self.declare_parameter("enter_rpm", 5.0)
        self.declare_parameter("center_err_px", 8.0)
        self.declare_parameter("steer_straight_deg", 0.0)
        self.declare_parameter("steer_side_deg", 90.0)
        self.declare_parameter("min_tof", 25)
        self.declare_parameter("max_tof", 500)

        self.exit_time_s = float(self.get_parameter("exit_time_s").value)
        self.exit_rpm = float(self.get_parameter("exit_rpm").value)
        self.side_rpm = float(self.get_parameter("side_rpm").value)
        self.enter_rpm = float(self.get_parameter("enter_rpm").value)
        self.center_err_px = float(self.get_parameter("center_err_px").value)
        self.steer_straight_deg = float(self.get_parameter("steer_straight_deg").value)
        self.steer_side_deg = float(self.get_parameter("steer_side_deg").value)
        self.min_tof = int(self.get_parameter("min_tof").value)
        self.max_tof = int(self.get_parameter("max_tof").value)

        self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.create_subscription(Int16MultiArray, "/robot_location", self.cb_robot_location, 10)
        self.create_subscription(Int16MultiArray, "/bench_side_marker", self.cb_side_marker, 10)
        self.create_subscription(Int16MultiArray, "/bench_robot/tof_raw", self.cb_tof, 10)

        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_rpm_cmd = self.create_publisher(Float32MultiArray, "/wheel_rpm_cmd", 10)
        self.pub_steer = self.create_publisher(Float32, "/steer_angle_deg", 10)

        self.timer = self.create_timer(0.05, self.control_tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_rpm(self, left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.pub_rpm_cmd.publish(msg)

    def publish_steer(self, deg):
        self.pub_steer.publish(Float32(data=float(deg)))

    def set_auto_state(self, state):
        self.pub_auto_state_cmd.publish(String(data=state))

    def stop(self):
        self.publish_rpm(0.0, 0.0)

    def cb_auto_state(self, msg):
        self.auto_state = (msg.data or "").strip().lower()
        if self.auto_state == "bench_change_start" and self.phase == "idle":
            self.get_logger().info("Bench change started")
            self.phase = "exit_current_bench"
            self.phase_start_time = self.now_s()

    def cb_robot_location(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            return

        self.current_bench = int(data[1])
        self.goal_bench = int(data[3])
        self.goal_row = int(data[4])

    def cb_side_marker(self, msg):
        data = list(msg.data)
        if len(data) < 4:
            self.side_marker_visible = False
            return

        self.side_marker_visible = bool(data[0])
        self.side_marker_bench = int(data[1])
        self.side_marker_err_px = float(data[2])
        self.side_marker_orientation = float(data[3])

    def cb_tof(self, msg):
        data = list(msg.data)
        if len(data) < 4:
            self.tof = None
            return

        self.tof = {"rl": int(data[0]),"fl": int(data[1]),"rr": int(data[2]),"fr": int(data[3]),}

    def valid_tof(self, value):
        return self.min_tof <= value <= self.max_tof

    def front_side_tof_valid(self):
        if self.tof is None:
            return False
        return self.valid_tof(self.tof["fl"]) and self.valid_tof(self.tof["fr"])

    def control_tick(self):
        if self.auto_state != "bench_change_start":
            return

        if self.phase == "exit_current_bench":
            self.run_exit_current_bench()
        elif self.phase == "find_target_direction":
            self.run_find_target_direction()
        elif self.phase == "move_sideways_to_target_bench":
            self.run_move_sideways_to_target_bench()
        elif self.phase == "center_with_target_bench":
            self.run_center_with_target_bench()
        elif self.phase == "enter_target_bench":
            self.run_enter_target_bench()
        elif self.phase == "finish":
            self.finish()
        else:
            self.stop()

    def run_exit_current_bench(self):
        self.publish_steer(self.steer_straight_deg)
        self.publish_rpm(self.exit_rpm, self.exit_rpm)

        if self.now_s() - self.phase_start_time >= self.exit_time_s:
            self.stop()
            self.phase = "find_target_direction"
            self.get_logger().info("Cleared current bench. Finding target direction.")

    def run_find_target_direction(self):
        if self.current_bench is None or self.goal_bench is None:
            self.stop()
            return

        if self.goal_bench == self.current_bench:
            self.phase = "enter_target_bench"
            return

        # Your rule:
        # left side bench numbers are increasing
        # right side bench numbers are decreasing
        if self.goal_bench > self.current_bench:
            self.side_direction = "left"
            self.side_rpm_cmd = self.side_rpm
        else:
            self.side_direction = "right"
            self.side_rpm_cmd = -self.side_rpm

        self.get_logger().info(f"Target bench {self.goal_bench}; moving {self.side_direction}")

        self.publish_steer(self.steer_side_deg)
        self.phase_start_time = self.now_s()
        self.phase = "move_sideways_to_target_bench"

    def run_move_sideways_to_target_bench(self):
        self.publish_steer(self.steer_side_deg)
        self.publish_rpm(self.side_rpm_cmd, self.side_rpm_cmd)

        if self.side_marker_visible and self.side_marker_bench == self.goal_bench:
            self.stop()
            self.get_logger().info(f"Detected target bench marker: {self.goal_bench}")
            self.phase = "center_with_target_bench"

    def run_center_with_target_bench(self):
        if not self.side_marker_visible:
            self.stop()
            return

        err = self.side_marker_err_px

        if abs(err) <= self.center_err_px:
            self.stop()
            self.get_logger().info("Centered with target bench")
            self.phase = "enter_target_bench"
            return

        rpm = self.side_rpm * 0.5

        if err > 0:
            self.publish_rpm(rpm, rpm)
        else:
            self.publish_rpm(-rpm, -rpm)

    def run_enter_target_bench(self):
        self.publish_steer(self.steer_straight_deg)
        self.publish_rpm(self.enter_rpm, self.enter_rpm)

        # Stop entering once row-side ToF becomes valid again.
        # Later you can replace this with first row-marker detection.
        if self.front_side_tof_valid():
            self.stop()
            self.phase = "finish"

    def finish(self):
        self.stop()
        self.publish_steer(self.steer_straight_deg)

        self.get_logger().info("Bench change finished. Returning to tracking.")
        self.phase = "idle"

        # Let aruco_detector decide correct direction toward goal row.
        self.set_auto_state("bench_tracking_f")


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