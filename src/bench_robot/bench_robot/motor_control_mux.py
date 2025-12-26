#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, Float32, Bool


class MotorControlMux(Node):
    def __init__(self):
        super().__init__('motor_control_mux')

        # ---- Topics ----
        self.pub_RPM = self.create_publisher(Int16MultiArray, '/wheel_rpm_cmd', 10)
        self.pub_steer = self.create_publisher(Float32, '/steer_angle_deg', 10)

        self.sub_auto_rpm = self.create_subscription(
            Int16MultiArray, '/wheel_rpm_auto', self.cb_auto_RPM, 10
        )
        self.sub_man_rpm = self.create_subscription(
            Int16MultiArray, '/wheel_rpm_manual', self.cb_manual_RPM, 10
        )
        self.sub_auto_steer = self.create_subscription(
            Float32, '/steer_auto', self.cb_auto_steer, 10
        )
        self.sub_man_steer = self.create_subscription(
            Float32, '/steer_manual', self.cb_manual_steer, 10
        )
        self.sub_mode = self.create_subscription(
            String, '/mode', self.cb_mode, 10
        )
        self.sub_estop = self.create_subscription(
            Bool, '/e_stop', self.cb_estop, 10
        )

        # ---- State ----
        self.mode = "manual" 
        self.eStop = False

        self.last_auto_RPM = Int16MultiArray()
        self.last_auto_RPM.data = [0, 0]
        self.last_auto_RPM_time = None
        self.auto_timeout_s = 0.30

        self.last_manual_RPM = Int16MultiArray()
        self.last_manual_RPM.data = [0, 0]

        self.last_auto_steer = Float32()
        self.last_auto_steer.data = 0.0

        self.last_manual_steer = Float32()
        self.last_manual_steer.data = 0.0
        
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz output

    
    @staticmethod
    def _is_zero(msg: Int16MultiArray) -> bool:
        return (int(msg.data[0]) == 0) and (int(msg.data[1]) == 0)
    
    def _auto_is_fresh(self, now) -> bool:
        if self.auto_timeout_s is None:
            return True
        if self.last_auto_RPM_time is None:
            return False
        dt = (now - self.last_auto_RPM_time).nanoseconds * 1e-9
        return dt < self.auto_timeout_s
    
    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m not in ("auto", "manual"):
            self.get_logger().warning(f"Invalid /mode '{msg.data}'. Use 'auto' or 'manual'.")
            return
        if m != self.mode:
            self.mode = m
            self.get_logger().info(f"Mode set to: {self.mode}")

    def cb_estop(self, msg: Bool):
        self.eStop = msg.data
        self.get_logger().info(f"E-Stop set to: {self.eStop}")
            
    
    def cb_auto_RPM(self, msg: Int16MultiArray):
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_auto (need [L,R])")
            return
        self.last_auto_RPM = msg
        self.last_auto_RPM_time = self.get_clock().now()

    def cb_manual_RPM(self, msg: Int16MultiArray):
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_manual (need [L,R])")
            return
        self.last_manual_RPM = msg
        if self.mode == "manual" and not self._is_zero(msg):
            self.pub_RPM.publish(msg)

    def cb_auto_steer(self, msg: Float32):
        if msg.data is None :
            self.get_logger().warning("Invalid /steer_auto")
            return
        self.last_auto_steer = msg

    def cb_manual_steer(self, msg: Float32):
        if msg.data is None :
            self.get_logger().warning("Invalid /steer_manual")
        self.last_manual_steer = msg
        if self.mode == "manual":
            self.pub_steer.publish(msg)

    def tick(self):
        if self.eStop:

            return
        if self.mode != "auto" :
            return
        now = self.get_clock().now()

        if not self._auto_is_fresh(now):
            # Don't publish anything; watchdog should stop wheels
            return
        
        if self._is_zero(self.last_auto_RPM):
            # Don't publish zeros; watchdog should stop wheels
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
