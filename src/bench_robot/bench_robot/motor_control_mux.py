#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from rcl_interfaces.msg import SetParametersResult

class MotorControlMux(Node):
    def __init__(self):
        super().__init__('motor_control_mux')

        # -------- params --------
        self.declare_parameter('manual_rpm', 10.0)
        self.MANUAL_RPM = self.get_parameter('manual_rpm').value

        # -------- states and variables --------
        self.cmd_to_rpm = {
            "forward":  ( -self.MANUAL_RPM,  -self.MANUAL_RPM),
            "backward": (self.MANUAL_RPM, self.MANUAL_RPM),
            "left":     (self.MANUAL_RPM,  -self.MANUAL_RPM),
            "right":    ( -self.MANUAL_RPM, self.MANUAL_RPM),
            "stop":     (0, 0),
        }

        # -------- subs --------
        self.sub_man_rpm = self.create_subscription(String, '/wheel_rpm_manual', self.cb_manual_RPM, 10)
        self.sub_man_steer = self.create_subscription(Float32, '/steer_manual', self.cb_manual_steer, 10)

        # -------- pubs --------
        self.pub_RPM = self.create_publisher(Float32MultiArray, '/wheel_rpm_cmd', 10)
        self.pub_steer = self.create_publisher(Float32, '/steer_angle_deg', 10)

        self._rebuild_cmd_map()
        self.add_on_set_parameters_callback(self.on_params)

    def on_params(self, params):
        for p in params:
            if p.name == 'manual_rpm':
                self.MANUAL_RPM = p.value
                self._rebuild_cmd_map()
                self.get_logger().info(f"[Settings] manual_rpm={self.MANUAL_RPM}")
        return SetParametersResult(successful=True)
    
    # -------- Helpers --------``

    def _rebuild_cmd_map(self):
        r = float(self.MANUAL_RPM)
        self.cmd_to_rpm = {
            "forward":  ( -r,  -r),
            "backward": (r, r),
            "left":     (r,  -r),
            "right":    ( -r, r),
            "stop":     (0, 0),
        }
     
    # -------- callbacks --------

    def cb_manual_RPM(self, msg: String):
        cmd = msg.data.strip().lower()
        rpm_l, rpm_r = self.cmd_to_rpm[cmd]
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [float(rpm_l), float(rpm_r)]
        self.pub_RPM.publish(rpm_msg)

    def cb_manual_steer(self, msg: Float32):
        self.pub_steer.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
