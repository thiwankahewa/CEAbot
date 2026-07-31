#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray, String
from std_srvs.srv import Trigger


FIRST_ROW_ID = 11
LAST_ROW_ID = 61


class BenchChanger(Node):
    """Move between benches using the arm camera, end markers, and ToF sensors."""

    def __init__(self):
        super().__init__("bench_changer")

        self.auto_state = "idle"
        self.phase = "idle"
        self.current_bench = None
        self.goal_bench = None
        self.goal_row = None
        self.exit_row = None

        self.marker_visible = False
        self.marker_bench = None
        self.marker_err_px = 0.0
        self.marker_orientation_deg = 0.0
        self.marker_distance_m = None
        self.last_marker_time = None
        self.last_valid_marker_time = None
        self.tof = None
        self.last_tof_time = None
        self.phase_start_time = None
        self.side_rpm_cmd = 0.0
        self.center_ok_cycles = 0
        self.arm_future = None

        self.declare_parameter("arm_center_service", "/arm/pose_1")
        self.declare_parameter("arm_move_timeout_s", 60.0)
        self.declare_parameter("exit_clearance_m", 1.25)
        self.declare_parameter("exit_timeout_s", 30.0)
        self.declare_parameter("exit_rpm", 4.0)
        self.declare_parameter("side_rpm", 4.0)
        self.declare_parameter("enter_rpm", 5.0)
        self.declare_parameter("center_rpm", 2.0)
        self.declare_parameter("center_err_px", 8.0)
        self.declare_parameter("center_stable_cycles", 3)
        self.declare_parameter("steer_straight_deg", 0.0)
        self.declare_parameter("steer_side_deg", 90.0)
        self.declare_parameter("steer_settle_s", 2.0)
        self.declare_parameter("marker_timeout_s", 0.6)
        self.declare_parameter("marker_loss_stop_s", 1.0)
        self.declare_parameter("side_search_timeout_s", 60.0)
        self.declare_parameter("enter_timeout_s", 20.0)
        self.declare_parameter("tof_timeout_s", 0.5)
        self.declare_parameter("use_marker_orientation_for_side_polarity", True)
        self.declare_parameter("min_tof", 25)
        self.declare_parameter("max_tof", 500)

        for name in ("arm_move_timeout_s", "exit_clearance_m", "exit_timeout_s", "exit_rpm", "side_rpm", "enter_rpm", "center_rpm",
                     "center_err_px", "steer_straight_deg", "steer_side_deg", "steer_settle_s", "marker_timeout_s",
                     "marker_loss_stop_s", "side_search_timeout_s", "enter_timeout_s", "tof_timeout_s"):
            setattr(self, name, float(self.get_parameter(name).value))
        self.center_stable_cycles = int(self.get_parameter("center_stable_cycles").value)
        self.min_tof = int(self.get_parameter("min_tof").value)
        self.max_tof = int(self.get_parameter("max_tof").value)
        self.use_marker_orientation_for_side_polarity = bool(
            self.get_parameter("use_marker_orientation_for_side_polarity").value)

        self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.create_subscription(Int16MultiArray, "/robot_location", self.cb_robot_location, 10)
        self.create_subscription(Int16MultiArray, "/bench_side_marker", self.cb_marker, 10)
        self.create_subscription(Int16MultiArray, "/bench_robot/tof_raw", self.cb_tof, 10)

        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_rpm_cmd = self.create_publisher(Float32MultiArray, "/wheel_rpm_cmd", 10)
        self.pub_steer = self.create_publisher(Float32, "/steer_angle_deg", 10)
        self.arm_client = self.create_client(Trigger, str(self.get_parameter("arm_center_service").value))
        self.timer = self.create_timer(0.05, self.control_tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_phase(self, phase):
        self.phase = phase
        self.phase_start_time = self.now_s()
        self.center_ok_cycles = 0

    def publish_rpm(self, left, right):
        self.pub_rpm_cmd.publish(Float32MultiArray(data=[float(left), float(right)]))

    def publish_steer(self, deg):
        self.pub_steer.publish(Float32(data=float(deg)))

    def stop(self):
        self.publish_rpm(0.0, 0.0)

    def fail_safe(self, reason):
        self.stop()
        self.get_logger().error(f"Bench change stopped: {reason}")
        self.set_phase("fault")

    def cb_auto_state(self, msg):
        previous = self.auto_state
        self.auto_state = (msg.data or "").strip().lower()
        if self.auto_state == "bench_change_start" and previous != "bench_change_start":
            if self.current_bench is None or self.goal_bench is None or self.exit_row not in (FIRST_ROW_ID, LAST_ROW_ID):
                self.fail_safe("bench/goal/exit row is not known")
                return
            self.marker_visible = False
            self.marker_distance_m = None
            self.last_valid_marker_time = None
            self.arm_future = None
            self.stop()
            self.set_phase("position_arm")
            self.get_logger().info("Positioning the arm camera over the chassis")
        elif self.auto_state != "bench_change_start" and self.phase != "idle":
            self.stop()
            self.phase = "idle"

    def cb_robot_location(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            return
        self.current_bench = int(data[1])
        self.exit_row = int(data[2])
        self.goal_bench = int(data[3])
        self.goal_row = int(data[4])

    def cb_marker(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            self.marker_visible = False
            self.marker_distance_m = None
            return
        self.marker_visible = bool(data[0])
        self.marker_bench = int(data[1])
        self.marker_err_px = float(data[2])
        self.marker_orientation_deg = float(data[3])
        distance_mm = int(data[4])
        self.marker_distance_m = distance_mm / 1000.0 if distance_mm >= 0 else None
        self.last_marker_time = self.now_s()
        if self.marker_visible:
            self.last_valid_marker_time = self.last_marker_time

    def cb_tof(self, msg):
        data = list(msg.data)
        if len(data) < 4:
            self.tof = None
            return
        self.tof = {"rl": int(data[0]), "fl": int(data[1]), "rr": int(data[2]), "fr": int(data[3])}
        self.last_tof_time = self.now_s()

    def marker_fresh(self, bench=None):
        return (self.marker_visible and self.last_marker_time is not None
                and self.now_s() - self.last_marker_time <= self.marker_timeout_s
                and (bench is None or self.marker_bench == bench))

    def all_tof_valid(self):
        if self.tof is None or self.last_tof_time is None or self.now_s() - self.last_tof_time > self.tof_timeout_s:
            return False
        return all(self.min_tof <= value <= self.max_tof for value in self.tof.values())

    def exit_sign(self):
        # Existing FIRST_ROW motion is negative; the opposite end must reverse it.
        return -1.0 if self.exit_row == FIRST_ROW_ID else 1.0

    def side_end_polarity(self):
        """Convert physical left/right into wheel polarity at this bench end.

        An end marker mounted 180 degrees from the marker at the other end gives
        the opposite cosine sign. If it was not seen, the known exit row is the
        deterministic fallback.
        """
        if (self.use_marker_orientation_for_side_polarity
                and self.marker_bench == self.current_bench
                and self.last_marker_time is not None):
            cosine = math.cos(math.radians(self.marker_orientation_deg))
            if abs(cosine) > 0.25:
                return 1.0 if cosine >= 0.0 else -1.0
        return 1.0 if self.exit_row == FIRST_ROW_ID else -1.0

    def control_tick(self):
        if self.auto_state != "bench_change_start":
            return
        handlers = {
            "position_arm": self.run_position_arm,
            "exit_current_bench": self.run_exit_current_bench,
            "settle_side_steering": self.run_settle_side_steering,
            "move_sideways_to_target_bench": self.run_move_sideways,
            "center_with_target_bench": self.run_center_target,
            "settle_entry_steering": self.run_settle_entry_steering,
            "enter_target_bench": self.run_enter_target,
            "finish": self.finish,
        }
        handler = handlers.get(self.phase)
        if handler:
            handler()
        else:
            self.stop()

    def run_position_arm(self):
        self.stop()
        if self.arm_future is None:
            if not self.arm_client.service_is_ready():
                if self.now_s() - self.phase_start_time > self.arm_move_timeout_s:
                    self.fail_safe("arm center-pose service unavailable")
                return
            self.arm_future = self.arm_client.call_async(Trigger.Request())
            return
        if not self.arm_future.done():
            if self.now_s() - self.phase_start_time > self.arm_move_timeout_s:
                self.fail_safe("arm center-pose request timed out")
            return
        try:
            result = self.arm_future.result()
            if result is None or not result.success:
                self.fail_safe(f"arm center pose failed: {getattr(result, 'message', 'no response')}")
                return
        except Exception as exc:
            self.fail_safe(f"arm center pose failed: {exc}")
            return
        self.get_logger().info("Arm camera centered; exiting current bench")
        self.set_phase("exit_current_bench")

    def run_exit_current_bench(self):
        self.publish_steer(self.steer_straight_deg)
        rpm = self.exit_sign() * self.exit_rpm
        # Distance, not elapsed travel time, decides when the chassis is clear.
        if not self.marker_fresh(self.current_bench) or self.marker_distance_m is None:
            self.stop()
            last_valid = self.last_valid_marker_time or self.phase_start_time
            if self.now_s() - last_valid > self.marker_loss_stop_s:
                self.fail_safe("current bench marker/distance unavailable during exit")
            return

        if self.marker_distance_m >= self.exit_clearance_m:
            self.stop()
            self.get_logger().info(f"Current bench cleared at marker distance {self.marker_distance_m:.3f} m")
            if self.goal_bench == self.current_bench:
                self.set_phase("settle_entry_steering")
                return
            # Bench numbers increase to the physical left. At opposite bench ends,
            # wheel polarity for that same physical direction is reversed.
            toward_larger = 1.0 if self.goal_bench > self.current_bench else -1.0
            end_polarity = self.side_end_polarity()
            self.side_rpm_cmd = toward_larger * end_polarity * self.side_rpm
            self.publish_steer(self.steer_side_deg)
            self.set_phase("settle_side_steering")
            return

        correction = max(-0.5, min(0.5, self.marker_err_px / max(self.center_err_px * 8.0, 1.0)))
        self.publish_rpm(rpm * (1.0 - correction), rpm * (1.0 + correction))
        if self.now_s() - self.phase_start_time > self.exit_timeout_s:
            self.fail_safe("exit clearance distance was not reached before safety timeout")

    def run_settle_side_steering(self):
        self.stop()
        self.publish_steer(self.steer_side_deg)
        if self.now_s() - self.phase_start_time >= self.steer_settle_s:
            self.set_phase("move_sideways_to_target_bench")

    def run_move_sideways(self):
        self.publish_steer(self.steer_side_deg)
        self.publish_rpm(self.side_rpm_cmd, self.side_rpm_cmd)
        if self.marker_fresh(self.goal_bench):
            self.stop()
            self.get_logger().info(f"Detected target bench marker {self.goal_bench}")
            self.set_phase("center_with_target_bench")
        elif self.now_s() - self.phase_start_time > self.side_search_timeout_s:
            self.fail_safe("target bench marker search timed out")

    def run_center_target(self):
        self.publish_steer(self.steer_side_deg)
        if not self.marker_fresh(self.goal_bench):
            self.stop()
            last_valid = self.last_valid_marker_time or self.phase_start_time
            if self.now_s() - last_valid > self.marker_loss_stop_s:
                self.fail_safe("lost target marker while centering")
            return
        if abs(self.marker_err_px) <= self.center_err_px:
            self.stop()
            self.center_ok_cycles += 1
            if self.center_ok_cycles >= self.center_stable_cycles:
                self.publish_steer(self.steer_straight_deg)
                self.set_phase("settle_entry_steering")
            return
        self.center_ok_cycles = 0
        direction = 1.0 if self.marker_err_px > 0.0 else -1.0
        self.publish_rpm(direction * self.center_rpm, direction * self.center_rpm)

    def run_settle_entry_steering(self):
        self.stop()
        self.publish_steer(self.steer_straight_deg)
        if self.now_s() - self.phase_start_time >= self.steer_settle_s:
            self.set_phase("enter_target_bench")

    def run_enter_target(self):
        self.publish_steer(self.steer_straight_deg)
        # Entry is the exact opposite of exiting at the selected end.
        rpm = -self.exit_sign() * self.enter_rpm
        if self.marker_fresh(self.goal_bench):
            correction = max(-0.5, min(0.5, self.marker_err_px / max(self.center_err_px * 8.0, 1.0)))
            self.publish_rpm(rpm * (1.0 - correction), rpm * (1.0 + correction))
        elif not self.all_tof_valid():
            self.stop()
            last_valid = self.last_valid_marker_time or self.phase_start_time
            if self.now_s() - last_valid > self.marker_loss_stop_s:
                self.fail_safe("target marker lost before all four ToF sensors became valid")
            return
        else:
            self.publish_rpm(rpm, rpm)
        if self.all_tof_valid():
            self.stop()
            self.set_phase("finish")
        elif self.now_s() - self.phase_start_time > self.enter_timeout_s:
            self.fail_safe("bench entry timed out")

    def finish(self):
        self.stop()
        self.publish_steer(self.steer_straight_deg)
        self.current_bench = self.goal_bench
        self.get_logger().info("Bench change finished; returning to row tracking")
        self.phase = "idle"
        direction = "bench_tracking_f" if self.goal_row >= self.exit_row else "bench_tracking_b"
        self.pub_auto_state_cmd.publish(String(data=direction))


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
