#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray, String, Bool, Float32
from rcl_interfaces.msg import SetParametersResult

STOP_THRESHOLD_MM = 25
TOF_MAX_MM = 500
WHEEL_DIAMETER_M = 0.2032


class BenchTracker(Node):
    def __init__(self):
        super().__init__('bench_tracker_v3')

        # -------- state --------
        self.eStop = False
        self.mode = "manual"
        self.auto_state = "idle"
        self.bench_track_dir = "bench_tracking_f"

        self.last_tof_stamp = None
        self.invalid_data_warned = False

        self.offset_err_m = 0.0
        self.yaw_err_m = 0.0

        # align_center phases (non-blocking)
        # 0=inactive, 1=steer->90 settle, 2=correcting, 3=steer->0 settle
        self.align_phase = 0
        self.align_phase_start = None

        # debounce counters
        self._yaw_ok = 0
        self._off_ok = 0

        # -------- params --------
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('tof_timeout_s', 0.3)

        self.declare_parameter('yaw_threshold_m', 0.02)
        self.declare_parameter('offset_threshold_m', 0.01)

        self.declare_parameter('offset_enter_m', 0.10)
        self.declare_parameter('yaw_enter_m', 0.05)
        self.declare_parameter('offset_exit_m', 0.01)
        self.declare_parameter('yaw_exit_m', 0.02)

        self.declare_parameter('Kp_offset', 0.2)
        self.declare_parameter('Kp_yaw_track', 0.005)

        # yaw correction (slow)
        self.declare_parameter('yaw_corr_rpm', 2.0)
        self.declare_parameter('Kp_yaw_corr', 30.0)   # maps meters -> rpm-ish after clamp

        # align center (crab) slow
        self.declare_parameter('align_rpm', 2.0)
        self.declare_parameter('Kp_align', 30.0)      # maps meters -> rpm-ish after clamp

        self.declare_parameter('base_rpm', 12.0)
        self.declare_parameter('max_rpm', 25.0)
        self.track_width_m = 1.515

        # steering angles / timing
        self.declare_parameter('steer_track_deg', 0.0)
        self.declare_parameter('steer_align_deg', 90.0)
        self.declare_parameter('steer_settle_s', 2)  # wait after steering change

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)

        # -------- subs --------
        self.sub_tof = self.create_subscription(Int16MultiArray, '/bench_robot/tof_raw', self.dist_cb, 10)
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_auto_state = self.create_subscription(String, '/auto_state', self.cb_auto_state, 10)
        self.sub_estop = self.create_subscription(Bool, '/e_stop', self.cb_estop, 10)

        # -------- pubs --------
        self.pub_auto_state_cmd = self.create_publisher(String, '/auto_state_cmd', 10)
        self.pub_errors = self.create_publisher(Float32MultiArray, '/bench_errors', 10)
        self.pub_rpm_cmd = self.create_publisher(Float32MultiArray, '/wheel_rpm_cmd', 10)
        self.pub_steer = self.create_publisher(Float32, '/steer_angle_deg', 10)

        # -------- timer --------
        period = 1.0 / max(1.0, self.cmd_rate_hz)
        self.timer = self.create_timer(period, self.control_tick)

    def _load_params(self):
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.tof_timeout_s = float(self.get_parameter('tof_timeout_s').value)

        self.yaw_threshold_m = float(self.get_parameter('yaw_threshold_m').value)
        self.offset_threshold_m = float(self.get_parameter('offset_threshold_m').value)

        self.offset_enter_m = float(self.get_parameter('offset_enter_m').value)
        self.offset_exit_m  = float(self.get_parameter('offset_exit_m').value)
        self.yaw_enter_m    = float(self.get_parameter('yaw_enter_m').value)
        self.yaw_exit_m     = float(self.get_parameter('yaw_exit_m').value)

        self.Kp_offset_track = float(self.get_parameter('Kp_offset').value)
        self.Kp_yaw_track = float(self.get_parameter('Kp_yaw_track').value)

        self.yaw_corr_rpm = float(self.get_parameter('yaw_corr_rpm').value)
        self.Kp_yaw_corr = float(self.get_parameter('Kp_yaw_corr').value)

        self.align_rpm = float(self.get_parameter('align_rpm').value)
        self.Kp_align = float(self.get_parameter('Kp_align').value)

        self.base_rpm = float(self.get_parameter('base_rpm').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)

        self.steer_track_deg = float(self.get_parameter('steer_track_deg').value)
        self.steer_align_deg = float(self.get_parameter('steer_align_deg').value)
        self.steer_settle_s = float(self.get_parameter('steer_settle_s').value)

    def on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    # -------- helpers --------
    @property
    def wheel_circumference(self) -> float:
        return math.pi * WHEEL_DIAMETER_M

    def mps_to_rpm(self, v_mps: float) -> float:
        return (60.0 * v_mps) / self.wheel_circumference

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def publish_rpm(self, left_rpm: float, right_rpm: float):
        m = Float32MultiArray()
        m.data = [float(left_rpm), float(right_rpm)]
        self.pub_rpm_cmd.publish(m)

    def publish_steer(self, deg: float):
        self.pub_steer.publish(Float32(data=float(deg)))

    def set_state(self, st: str):
        st = (st or "").strip().lower()
        if st == self.auto_state:
            return
        self.auto_state = st
        self.pub_auto_state_cmd.publish(String(data=st))

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------- callbacks --------
    def cb_estop(self, msg: Bool):
        self.eStop = bool(msg.data)

    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m != self.mode:
            self.mode = m
            if self.mode != "auto":
                self.set_state("idle")
                self.align_phase = 0
                self.publish_steer(self.steer_track_deg)

    def cb_auto_state(self, msg: String):
        st = (msg.data or "").strip().lower()
        if st in ("bench_tracking_f", "bench_tracking_b"):
            self.bench_track_dir = st
        self.auto_state = st

    def dist_cb(self, msg: Int16MultiArray):
        data = msg.data
        if data is None or len(data) < 4:
            self.invalid_data_warned = True
            return

        rl, fl, rr, fr = data

        def valid(x):
            return (x is not None) and (STOP_THRESHOLD_MM <= x <= TOF_MAX_MM)

        if not all(valid(x) for x in (fl, fr, rl, rr)):
            self.invalid_data_warned = True
            self.last_tof_stamp = self.get_clock().now()
            return

        self.invalid_data_warned = False
        self.last_tof_stamp = self.get_clock().now()

        left_avg = (fl + rl) / 2.0
        right_avg = (fr + rr) / 2.0
        self.offset_err_m = (right_avg - left_avg) / 1000.0

        yaw_left = fl - rl
        yaw_right = rr - fr
        self.yaw_err_m = ((yaw_left + yaw_right) / 2.0) / 1000.0

        be = Float32MultiArray()
        be.data = [float(self.offset_err_m), float(self.yaw_err_m)]
        self.pub_errors.publish(be)

    # -------- main control loop --------
    def control_tick(self):
        if self.mode != "auto":
            return
        # basic stop logic
        if self.eStop:
            self.publish_rpm(0.0, 0.0)
            return

        if self.last_tof_stamp is None:
            self.publish_rpm(0.0, 0.0)
            return

        age_s = (self.get_clock().now() - self.last_tof_stamp).nanoseconds * 1e-9
        if age_s > self.tof_timeout_s or self.invalid_data_warned:
            self.publish_rpm(0.0, 0.0)
            return

        off = float(self.offset_err_m)
        yaw = float(self.yaw_err_m)

        

        # If we are currently running align_center phases, handle it FIRST
        if self.auto_state == "align_center":
            self._run_align_center(off)
            return

        if self.auto_state in ("bench_tracking_f", "bench_tracking_b"):
        # transitions to correction states
            if abs(yaw) > self.yaw_enter_m:
                # ensure steer is in tracking (0 deg) for yaw correction
                self.publish_steer(self.steer_track_deg)
                self.set_state("yaw_correction")
                self._yaw_ok = 0
                self.publish_rpm(0.0, 0.0)
                return

            if abs(off) > self.offset_enter_m:
                # start align_center with phases
                self.set_state("align_center")
                self.align_phase = 1
                self.align_phase_start = self.now_s()
                # command steer to 90 and stop while settling
                self.publish_steer(self.steer_align_deg)
                self.publish_rpm(0.0, 0.0)
                return

        # normal tracking
        if self.auto_state in ("bench_tracking_f", "bench_tracking_b"):
            self.publish_steer(self.steer_track_deg)
            direction = 1.0 if self.auto_state == "bench_tracking_f" else -1.0

            base_v_mps = direction * ((self.base_rpm * self.wheel_circumference) / 60.0)

            # combined correction
            w = direction * (-(self.Kp_offset_track * off))

            v_left = base_v_mps - w * (self.track_width_m / 2.0)
            v_right = base_v_mps + w * (self.track_width_m / 2.0)

            left_rpm = self.clamp(self.mps_to_rpm(v_left), -self.max_rpm, self.max_rpm)
            right_rpm = self.clamp(self.mps_to_rpm(v_right), -self.max_rpm, self.max_rpm)
            self.get_logger().info(f"TRACKING: off={off:.3f} yaw={yaw:.3f} rpm_left={left_rpm:.1f} rpm_right={right_rpm:.1f}")

            self.publish_rpm(left_rpm, right_rpm)
            return

        # yaw correction state
        if self.auto_state == "yaw_correction":
            self._run_yaw_correction(yaw, off)
            return

        self.publish_rpm(0.0, 0.0)

    def _run_yaw_correction(self, yaw: float, off: float):
        # steer should be 0 for yaw correction
        self.publish_steer(self.steer_track_deg)

        if abs(yaw) <= self.yaw_exit_m:
            self._yaw_ok += 1
        else:
            self._yaw_ok = 0

        # differential spin in place
        if yaw >= 0:
            left, right = -self.yaw_corr_rpm, +self.yaw_corr_rpm
        else:
            left, right = +self.yaw_corr_rpm, -self.yaw_corr_rpm
        self.get_logger().info(f"YAW CORR: off={off:.3f} yaw={yaw:.3f} rpm_left={left:.1f} rpm_right={right:.1f}")
        self.publish_rpm(left, right)

        # exit after stable
        if self._yaw_ok >= 10:
            self._yaw_ok = 0
            # if offset still large -> go to align_center, else back to track
            if abs(off) > self.offset_threshold_m:
                self.set_state("align_center")
                self.align_phase = 1
                self.align_phase_start = self.now_s()
                self.publish_steer(self.steer_align_deg)
                self.publish_rpm(0.0, 0.0)
            else:
                self.set_state(self.bench_track_dir)

    def _run_align_center(self, off: float):
        """
        Phases:
        1) steer->90 settle (stop)
        2) correction with slow RPM (steer 90)
        3) steer->0 settle (stop)
        then return to bench_tracking dir
        """
        t = self.now_s()

        # Phase 1: steer to 90 and wait settle
        if self.align_phase == 1:
            self.publish_steer(self.steer_align_deg)
            self.publish_rpm(0.0, 0.0)
            if (t - (self.align_phase_start or t)) >= self.steer_settle_s:
                self.align_phase = 2
                self.align_phase_start = t
            return

        # Phase 2: apply correction at steer=90
        if self.align_phase == 2:
            self.publish_steer(self.steer_align_deg)

            # if centered for N cycles, go to phase 3
            if abs(off) <= self.offset_exit_m:
                self._off_ok += 1
            else:
                self._off_ok = 0


            if off >= 0:
                left, right = -self.align_rpm, -self.align_rpm
            else:
                left, right = +self.align_rpm, +self.align_rpm
            self.get_logger().info(f"ALIGN CENTER: off={off:.3f} yaw={self.yaw_err_m:.3f} rpm_left={left:.1f} rpm_right={right:.1f}")
            self.publish_rpm(left, right)

            if self._off_ok >= 10:
                self._off_ok = 0
                self.align_phase = 3
                self.align_phase_start = t
                # start steering back to 0
                self.publish_rpm(0.0, 0.0)
                self.publish_steer(self.steer_track_deg)
            return

        # Phase 3: steer back to 0 settle
        if self.align_phase == 3:
            self.publish_steer(self.steer_track_deg)
            self.publish_rpm(0.0, 0.0)
            if (t - (self.align_phase_start or t)) >= self.steer_settle_s:
                self.align_phase = 0
                self.align_phase_start = None
                self.set_state(self.bench_track_dir)
            return

        # fallback
        self.align_phase = 1
        self.align_phase_start = t


def main(args=None):
    rclpy.init(args=args)
    node = BenchTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_rpm(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()