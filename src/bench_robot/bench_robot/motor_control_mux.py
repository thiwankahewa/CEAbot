#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, Float32, Bool, Float32MultiArray
from rcl_interfaces.msg import SetParametersResult


class MotorControlMux(Node):
    def __init__(self):
        super().__init__('motor_control_mux')

        self.auto_state = "idle"

        self.mode = "manual" 
        self.eStop = False

        self.last_auto_RPM = Float32MultiArray()
        self.last_auto_RPM.data = [0.0, 0.0]
        self.last_auto_RPM_time = None
        self.auto_timeout_s = 0.30

        self.declare_parameter('manual_rpm', 10.0)
        self.MANUAL_RPM = self.get_parameter('manual_rpm').value 
        self._rebuild_cmd_map()
        self.cmd_to_rpm = {
            "forward":  ( -self.MANUAL_RPM,  -self.MANUAL_RPM),
            "backward": (self.MANUAL_RPM, self.MANUAL_RPM),
            "left":     (self.MANUAL_RPM,  -self.MANUAL_RPM),
            "right":    ( -self.MANUAL_RPM, self.MANUAL_RPM),
            "stop":     (0, 0),
        }

        self.last_auto_steer = Float32()
        self.last_auto_steer.data = 0.0

        # Timers
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz output

        # Subscribers
        self.sub_auto_state = self.create_subscription(String, '/auto_state', self.cb_auto_state, 10)
        self.sub_auto_rpm = self.create_subscription(Float32MultiArray, '/wheel_rpm_auto', self.cb_auto_RPM, 10)
        self.sub_man_rpm = self.create_subscription(String, '/wheel_rpm_manual', self.cb_manual_RPM, 10)
        self.sub_auto_steer = self.create_subscription(Float32, '/steer_auto', self.cb_auto_steer, 10)
        self.sub_man_steer = self.create_subscription(Float32, '/steer_manual', self.cb_manual_steer, 10)
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.sub_estop = self.create_subscription(Bool, '/e_stop', self.cb_estop, 10)

        # Publishers
        self.pub_RPM = self.create_publisher(Float32MultiArray, '/wheel_rpm_cmd', 10)
        self.pub_steer = self.create_publisher(Float32, '/steer_angle_deg', 10)

        self.add_on_set_parameters_callback(self.on_params)

    def on_params(self, params):
        for p in params:
            if p.name == 'manual_rpm':
                self.MANUAL_RPM = p.value
                self._rebuild_cmd_map()
                self.get_logger().info(f"[Settings] manual_rpm={self.MANUAL_RPM}")
        return SetParametersResult(successful=True)
    
    #--- Helpers ---

    def _rebuild_cmd_map(self):
        r = float(self.MANUAL_RPM)
        self.cmd_to_rpm = {
            "forward":  ( -r,  -r),
            "backward": (r, r),
            "left":     (r,  -r),
            "right":    ( -r, r),
            "stop":     (0, 0),
        }

    @staticmethod
    def _is_zero(msg: Float32MultiArray) -> bool:
        return (msg.data[0] == 0.0) and (msg.data[1] == 0.0)
    
    def _auto_is_fresh(self, now) -> bool:
        if self.auto_timeout_s is None:
            return True
        if self.last_auto_RPM_time is None:
            return False
        dt = (now - self.last_auto_RPM_time).nanoseconds * 1e-9
        return dt < self.auto_timeout_s
     
    #---Main functions---
    def cb_auto_state(self, msg: String):
        new_state = (msg.data or "").strip().lower()
        if new_state == self.auto_state:
            return
        self.auto_state = new_state

    def cb_estop(self, msg: Bool):
        self.eStop = msg.data
        self.get_logger().info(f"E-Stop set to: {self.eStop}")

    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m != self.mode:
            self.mode = m
            
    def cb_auto_RPM(self, msg: Float32MultiArray):
        if self.eStop:
            return
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_auto (need [L,R])")
            return
        self.last_auto_RPM = msg
        self.last_auto_RPM_time = self.get_clock().now()

    def cb_manual_RPM(self, msg: String):
        cmd = msg.data.strip().lower()
        rpm_l, rpm_r = self.cmd_to_rpm[cmd]
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [float(rpm_l), float(rpm_r)]
        self.pub_RPM.publish(rpm_msg)

    def cb_auto_steer(self, msg: Float32):
        self.last_auto_steer = msg

    def cb_manual_steer(self, msg: Float32):
        self.pub_steer.publish(msg)

    def tick(self):
        if self.eStop:
            return
        if self.mode != "auto" :
            return
        if self.auto_state in ("yaw_correction", "align_center"):
            return
        now = self.get_clock().now()

        if not self._auto_is_fresh(now):               # Don't publish anything; watchdog should stop wheels
            return
        if self._is_zero(self.last_auto_RPM):               # Don't publish zeros; watchdog should stop wheels
            return
        self.pub_RPM.publish(self.last_auto_RPM)
        self.pub_steer.publish(self.last_auto_steer)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
