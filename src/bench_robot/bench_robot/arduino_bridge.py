#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32
from std_srvs.srv import Trigger
import serial
from serial.serialutil import SerialException
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.port = '/dev/controllino'
        self.baud = 115200
        self.ser = None
        self.connected = False

        # Services
        self.srv_tof_restart = self.create_service(Trigger, "/arduino_bridge/tof_restart", self.on_tof_restart)
        self.srv_arduino_reconnect = self.create_service(Trigger, "/arduino_bridge/arduino_reconnect", self.on_arduino_reconnect)

        # Timer to read serial data
        self.timer = self.create_timer(0.01, self.read_serial)

        # Subscribers
        self.sub_steer = self.create_subscription( Float32,'/steer_angle_deg',self.steer_cb,10)
        self.send_period = 100.0  # Hz
        self.send_delta = 1
        self.last_send_time = 0.0
        self.last_sent_angle = None

        # Publishers
        self.pub_tof = self.create_publisher(Int16MultiArray,'/bench_robot/tof_raw',10)

        self.open_serial(initial=True)

    #---Helper functions---#
    def open_serial(self, initial=False) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.connected = True
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
            return True
        except Exception as e:
            self.ser = None
            self.connected = False
            if initial:
                self.get_logger().warn(f"Arduino not connected ({self.port}): {e} (node will stay alive)")
            return False
        
    def ensure_connected(self) -> bool:
        if self.connected and self.ser and self.ser.is_open:
            return True
        return self.open_serial(initial=False)

    def send_line(self, line: str):
        if not self.connected or self.ser is None:
            return
        try:
            self.ser.write((line + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")

    #---Main functions---#
    def read_serial(self):
        try:
            if self.ser.in_waiting:
                raw = self.ser.readline()
                self.get_logger().debug(f"Serial read: {raw}")
                if not raw:
                    return
                line = raw.decode('utf-8', errors='replace').strip()
                # Example: VL53,4,mm1,mm2,mm3,mm4
                if line.startswith('VL53'):    #ToF sensor data
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
        if (now - self.last_send_time) < (1/self.send_period):   # frequency limit
            return
        
        if self.last_sent_angle is not None and abs(angle - self.last_sent_angle) < self.send_delta:        # only send on change
            return

        cmd = f"CMD A={angle - 15},{angle - 18}"   #right, left steer values 
        self.send_line(cmd)
        self.last_send_time = now
        self.last_sent_angle = angle

    def on_tof_restart(self, request, response):
        if not self.ensure_connected():
            response.success = False
            response.message = "Arduino not connected."
            self.get_logger().warn("Cannot restart ToF: Arduino not connected.")
            return response
        
        ok = self.send_line("TOF_RESET")
        response.success = bool(ok)
        response.message = "ToF restart command sent." if ok else "Failed to send TOF_RESET."
        self.get_logger().info(response.message)
        return response
    
    def on_arduino_reconnect(self, request, response):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.connected = False

        ok = self.open_serial(initial=False)
        response.success = bool(ok)
        response.message = "Arduino reconnected." if ok else f"Reconnect failed on {self.port}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
