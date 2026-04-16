#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32
from std_srvs.srv import Trigger
import serial
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # -------- States and variables --------
        self.port = '/dev/controllino'
        self.baud = 115200
        self.ser = None
        self.connected = False

        self.send_period = 10.0  # Hz
        self.send_delta = 1
        self.last_send_time = 0.0
        self.last_sent_angle = None

        # -------- services --------
        self.srv_arduino_reconnect = self.create_service(Trigger, "/arduino_bridge/arduino_reconnect", self.on_arduino_reconnect)

        # -------- subs --------
        self.sub_steer = self.create_subscription( Float32,'/steer_angle_deg',self.steer_cb,10)

        # -------- pubs --------
        self.pub_tof = self.create_publisher(Int16MultiArray,'/bench_robot/tof_raw',10)

        # -------- timers --------
        self.timer = self.create_timer(0.01, self.read_serial)

        self.open_serial(initial=True)

    # -------- helper functions --------

    def perform_startup_wiggle(self):
        try:
            # 1. Move to 45 degrees
            angle_target = 45.0
            cmd_45 = f"CMD A={angle_target - 15},{angle_target - 18}\n"
            self.ser.write(cmd_45.encode('utf-8'))
            
            # Wait for hardware to physically move (adjust time as needed)
            time.sleep(1.5) 
            
            # 2. Move back to 0 degrees
            angle_zero = 0.0
            cmd_0 = f"CMD A={angle_zero - 15},{angle_zero - 18}\n"
            self.ser.write(cmd_0.encode('utf-8'))
            
            # Update trackers so the next callback doesn't think it's already at 0
            self.last_sent_angle = angle_zero
            self.last_send_time = time.time()
            self.get_logger().info("Performed servo calibration.")
            
        except Exception as e:
            self.get_logger().error(f"Failed startup wiggle: {e}")

    def send_line(self, line: str):
        if not self.connected or self.ser is None:
            return
        try:
            self.ser.write((line + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")

    # -------- callbacks --------

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

    # -------- main functions --------

    def open_serial(self, initial=False) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.connected = True
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
            if initial:
                self.perform_startup_wiggle()
            return True
        except Exception as e:
            self.ser = None
            self.connected = False
            if initial:
                self.get_logger().warn(f"Arduino not connected ({self.port}): {e} (node will stay alive)")
            return False
        
    def read_serial(self):
        try:
            if self.ser.in_waiting:
                raw = self.ser.readline()
                if not raw:
                    return
                line = raw.decode('utf-8', errors='replace').strip()
                if line.startswith('VL53'):    #ToF sensor data: VL53,4,mm1,mm2,mm3,mm4
                    parts = line.split(',')
                    vals = []
                    for i in parts[1:5]:
                        vals.append(int(i))

                    msg = Int16MultiArray()
                    msg.data = vals
                    self.pub_tof.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return
    
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
