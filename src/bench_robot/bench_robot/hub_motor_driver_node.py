#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from std_msgs.msg import Int16MultiArray


def to_u16_signed(val: int) -> int:
    return val & 0xFFFF


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.port = '/dev/ttyUSB0'
        self.device_id = 1

        self.client = ModbusSerialClient(port=self.port,baudrate=115200,)

        if self.client.connect():
            self.get_logger().info("Connected to ZLAC8015D")
            self.client.write_register(0x200D, 0x0003, device_id=1)  # velocity mode
            time.sleep(0.05)
            self.client.write_register(0x200E, 0x0008, device_id=1)  # enable
        else:
            self.get_logger().error("ZLAC8015D connection failed")

        self.cmd_timeout_s = 0.5     # watchdog timeout
        self.last_cmd_time = None

        self.sub = self.create_subscription(
            Int16MultiArray,
            '/wheel_rpm_cmd',
            self.rpm_cb,
            10
        )

        self.timer = self.create_timer(0.1, self.watchdog_tick)  # 10 Hz watchdo

    # ---------------- Modbus helpers ----------------
    def _write_single(self, addr: int, value: int):
        res = self.client.write_register(addr, value, device_id=self.device_id)  # FC 0x06
        if res.isError():
            self.get_logger().error(f"write_register failed @0x{addr:04X}: {res}")

    def _write_rpms(self, left_rpm: int, right_rpm: int):
        vals = [to_u16_signed(left_rpm), to_u16_signed(right_rpm)]
        res = self.client.write_registers(0x2088, vals, device_id=self.device_id)  # FC 0x10
        if res.isError():
            self.get_logger().error(f"write_registers failed @0x2088: {res}")

    def stop(self):
        self._write_rpms(0, 0)

    # ---------------- Callbacks ----------------
    def rpm_cb(self, msg: Int16MultiArray):
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_cmd (need [left_rpm, right_rpm])")
            return

        left_rpm = int(msg.data[0])
        right_rpm = int(msg.data[1])

        self._write_rpms(left_rpm, -right_rpm)

        # update watchdog time
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info(f"wheel_rpm_cmd -> L={left_rpm:+d} rpm R={right_rpm:+d} rpm")

    def watchdog_tick(self):
        if self.last_cmd_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout_s:
            self.get_logger().warning("RPM command timeout -> STOP")
            self.stop()
            self.last_cmd_time = None 



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Ctrl+C received -> STOP motors")
    finally:
        node.stop()
        time.sleep(0.05) 
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
