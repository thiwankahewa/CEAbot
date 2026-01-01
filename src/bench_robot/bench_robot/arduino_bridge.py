#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32
import serial
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        port = '/dev/controllino'
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Opened serial {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        self.timer = self.create_timer(0.01, self.read_serial)

        self.pub_tof = self.create_publisher(Int16MultiArray,'/bench_robot/tof_raw',10)

        self.sub_steer = self.create_subscription( Float32,'/steer_angle_deg',self.steer_cb,10)
        self.send_period = 20.0  # Hz
        self.send_delta = 0.1
        self.last_send_time = 0.0
        self.last_sent_angle = None

    def send_line(self, line: str):
        try:
            self.ser.write((line + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")

    def read_serial(self):
        try:
            if self.ser.in_waiting:
                raw = self.ser.readline()
                if not raw:
                    return
                line = raw.decode('utf-8', errors='replace').strip()
                self.get_logger().info(f"[RAW SERIAL] '{line}'")

                # Example: VL53,v1,4,mm1,mm2,mm3,mm4
                if line.startswith('VL53'):
                    parts = line.split(',')
                    vals = []
                    for i in parts[1:5]:
                        vals.append(int(i))

                    msg = Int16MultiArray()
                    msg.data = vals
                    self.pub_tof.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def steer_cb(self, msg: Float32):
        angle = float(msg.data)

        now = time.time()
        if (now - self.last_send_time) < (1/self.send_period):
            return
        
        # only send on change
        if self.last_sent_angle is not None and abs(angle - self.last_sent_angle) < self.send_delta:
            return

        cmd = f"CMD A={angle},{angle}"
        self.send_line(cmd)

        self.last_send_time = now
        self.last_sent_angle = angle

        self.get_logger().info(f"Sent -> {cmd}")
        

    

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
