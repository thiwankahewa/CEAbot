#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            #self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Opened serial {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        self.pub_tof = self.create_publisher(
            Int16MultiArray,
            '/bench_robot/tof_raw',
            10
        )

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    return

                # Example: VL53,v1,4,mm1,mm2,mm3,mm4
                if line.startswith('VL53'):
                    parts = line.split(',')
                    if len(parts) < 3:
                        return  

                    pkt_type = parts[0]
                    count_str = parts[1]

                    try:
                        count = int(count_str)
                    except ValueError:
                        return

                    if len(parts) < 2 + count:
                        return  # not enough values

                    vals = []
                    for i in range(count):
                        try:
                            vals.append(int(parts[2 + i]))
                        except ValueError:
                            vals.append(-1)

                    msg = Int16MultiArray()
                    msg.data = vals
                    self.pub_tof.publish(msg)
                    self.get_logger().info(f"VL53 distances (mm): {vals}")
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
