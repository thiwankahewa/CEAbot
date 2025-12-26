#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        self.declare_parameter("pid.kp")
        self.declare_parameter("pid.ki")
        self.declare_parameter("pid.kd")
        self.declare_parameter("limits.max_rpm")
        self.declare_parameter("drive.mode")

        self.kp = float(self.get_parameter("pid.kp").value)
        self.ki = float(self.get_parameter("pid.ki").value)
        self.kd = float(self.get_parameter("pid.kd").value)
        self.max_rpm = int(self.get_parameter("limits.max_rpm").value)
        self.mode = str(self.get_parameter("drive.mode").value)

        self.add_on_set_parameters_callback(self.on_params)

        self.get_logger().info(
            f"MotorController start: kp={self.kp}, ki={self.ki}, kd={self.kd}, max_rpm={self.max_rpm}"
        )

    def on_params(self, params):
        # validate
        for p in params:
            if p.name in ("pid.kp", "pid.ki", "pid.kd"):
                if float(p.value) < 0.0 or float(p.value) > 10.0:
                    return SetParametersResult(False, "PID out of range [0..10]")
            if p.name == "limits.max_rpm":
                v = int(p.value)
                if v < 0 or v > 60:
                    return SetParametersResult(False, "max_rpm out of range [0..60]")

        # apply
        for p in params:
            if p.name == "pid.kp":
                self.kp = float(p.value)
                self.get_logger().info(f"[LIVE] pid.kp={self.kp}")
            elif p.name == "pid.ki":
                self.ki = float(p.value)
                self.get_logger().info(f"[LIVE] pid.ki={self.ki}")
            elif p.name == "pid.kd":
                self.kd = float(p.value)
                self.get_logger().info(f"[LIVE] pid.kd={self.kd}")
            elif p.name == "limits.max_rpm":
                self.max_rpm = int(p.value)
                self.get_logger().info(f"[LIVE] limits.max_rpm={self.max_rpm}")
            elif p.name == "drive.mode":
                self.mode = str(p.value)
                self.get_logger().info(f"[LIVE] drive.mode={self.mode}")

        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    rclpy.spin(MotorController())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
