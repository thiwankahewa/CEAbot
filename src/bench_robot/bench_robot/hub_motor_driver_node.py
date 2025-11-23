# bench_robot/motor_driver_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymodbus.client import ModbusSerialClient

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # RS485 client (adjust device, baudrate, etc.)
        self.client = ModbusSerialClient(
            method='rtu',
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=0.05
        )
        ok = self.client.connect()
        if not ok:
            self.get_logger().error("Could not connect to ZLAC8015D over RS485")

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # msg.linear.x -> forward/back speed
        # msg.angular.z -> rotate
        # Here convert to left/right wheel speed, then write to driver registers
        wheel_base = 0.6  # m, set yours
        v = msg.linear.x
        w = msg.angular.z

        left = v - (w * wheel_base / 2.0)
        right = v + (w * wheel_base / 2.0)

        # convert m/s to driver’s unit (e.g., rpm)
        left_cmd = int(left * 1000)
        right_cmd = int(right * 1000)

        # WRITE to driver holding registers
        # NOTE: you must check your ZLAC8015D modbus map!
        # For example:
        # self.client.write_register(0x2000, left_cmd, unit=1)
        # self.client.write_register(0x2001, right_cmd, unit=1)

        self.get_logger().info(f"cmd_vel -> L:{left_cmd} R:{right_cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
