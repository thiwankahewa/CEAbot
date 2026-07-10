#!/usr/bin/env python3

import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray, String
from std_srvs.srv import Trigger

ACTIVE_TOF_STATES = ("bench_tracking_f", "bench_tracking_b", "yaw_correction", "align_center")


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        self.auto_state = "manual"
        self.port = "/dev/controllino"
        self.baud = 115200
        self.ser = None
        self.connected = False
        self.serial_error_reported = False

        self.send_period = 10.0
        self.send_delta = 1
        self.last_send_time = 0.0
        self.last_sent_angle = None

        self.srv_arduino_reconnect = self.create_service(Trigger, "/arduino_bridge/arduino_reconnect", self.on_arduino_reconnect)
        self.sub_steer = self.create_subscription(Float32, "/steer_angle_deg", self.steer_cb, 10)
        self.sub_auto_state = self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.pub_tof = self.create_publisher(Int16MultiArray, "/bench_robot/tof_raw", 10)
        self.timer = self.create_timer(0.01, self.read_serial)

        self.open_serial(initial=True)

    def perform_startup_wiggle(self):
        try:
            self.send_line("CMD A=30.0,27.0")
            time.sleep(1.5)
            self.send_line("CMD A=-15.0,-18.0")
            self.last_sent_angle = 0.0
            self.last_send_time = time.monotonic()
            self.get_logger().info("Performed servo calibration.")
        except Exception as e:
            self.get_logger().error(f"Failed startup wiggle: {e}")

    def send_line(self, line: str):
        if not self.connected or self.ser is None:
            return
        try:
            self.ser.write(f"{line}\n".encode("utf-8"))
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")

    def cb_auto_state(self, msg: String):
        self.auto_state = (msg.data or "").strip().lower()

    def steer_cb(self, msg: Float32):
        angle = float(msg.data)
        now = time.monotonic()
        if now - self.last_send_time < 1 / self.send_period:
            return
        if self.last_sent_angle is not None and abs(angle - self.last_sent_angle) < self.send_delta:
            return

        self.send_line(f"CMD A={angle - 15},{angle - 18}")
        self.last_send_time = now
        self.last_sent_angle = angle

    def open_serial(self, initial=False) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.connected = True
            self.serial_error_reported = False
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
            if initial:
                self.perform_startup_wiggle()
            return True
        except Exception as e:
            self.ser = None
            self.connected = False
            if not self.serial_error_reported:
                self.get_logger().warn(f"Arduino not connected ({self.port}): {e} (node will stay alive)")
                self.serial_error_reported = True
            return False

    def read_serial(self):
        if not self.connected or self.ser is None:
            return
        try:
            if not self.ser.in_waiting:
                return
            raw = self.ser.readline()
            if not raw:
                return
            line = raw.decode("utf-8", errors="replace").strip()
            if not line.startswith("VL53") or self.auto_state not in ACTIVE_TOF_STATES:
                return

            values = [int(value) for value in line.split(",")[1:5]]
            if len(values) != 4:
                return
            self.pub_tof.publish(Int16MultiArray(data=values))
        except Exception as e:
            if not self.serial_error_reported:
                self.get_logger().warn(f"Serial read error: {e}")
                self.serial_error_reported = True

    def on_arduino_reconnect(self, _request, response):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.connected = False

        ok = self.open_serial()
        response.success = bool(ok)
        response.message = "Arduino reconnected." if ok else f"Reconnect failed on {self.port}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
