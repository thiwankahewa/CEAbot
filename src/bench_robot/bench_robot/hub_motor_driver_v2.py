#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from std_msgs.msg import Float32MultiArray, Bool, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


def to_u16_signed(val: int) -> int:
    return val & 0xFFFF

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('hub_motor_driver_v2')

        # -------- States and variables --------
        self.eStop = False
        self.auto_state = "idle"

        self.connected = False
        self.port = '/dev/ttyUSB0'
        self.device_id = 1
        self.parked = False
        self._profile_is_corr = False

        self.client = ModbusSerialClient(port=self.port, baudrate=115200)

        self.cmd_left_rpm = 0.0
        self.cmd_right_rpm = 0.0
        self.last_cmd_time = None

        # -------- params --------
        self.declare_parameter('acel_ms', 1200)
        self.declare_parameter('decel_ms', 1200)
        self.declare_parameter('acel_ms_corr', 1200)
        self.declare_parameter('decel_ms_corr', 1200)

        # -------- services --------
        self.srv_reconnect = self.create_service(Trigger, "/arduino_bridge/hub_servo_reconnect", self.on_reconnect)

        # -------- subs --------
        self.sub_auto_state = self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.sub_rpm = self.create_subscription(Float32MultiArray, '/wheel_rpm_cmd', self.cb_rpm_cmd, 10)
        self.sub_estop = self.create_subscription(Bool, '/e_stop', self.cb_estop, 10)

        # -------- timers --------
        self.write_timer = self.create_timer(0.05, self.write_tick)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_tick)

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)
        self.try_connect(init=True)
        self._apply_profile(force=True)
        
    def _load_params(self):
        self.acel_ms = int(self.get_parameter('acel_ms').value)
        self.decel_ms = int(self.get_parameter('decel_ms').value)
        self.acel_ms_corr = int(self.get_parameter('acel_ms_corr').value)
        self.decel_ms_corr = int(self.get_parameter('decel_ms_corr').value)

    def on_params(self, params):
        self._load_params()
        self._apply_profile(force=True)
        return SetParametersResult(successful=True)

    # ---- helper functions ----

    def ensure_connected(self) -> bool:
        if self.connected:
            return True
        return self.try_connect(init=False)

    def _write_single(self, addr: int, value: int):
        res = self.client.write_register(addr, value, device_id=self.device_id)
        if res is None or res.isError():
            self.get_logger().error(f"write_register failed @0x{addr:04X}: {res}")
            self.connected = False

    def _write_rpms(self, left_rpm: float, right_rpm: float):
        l_cmd = int(round(left_rpm * 10))
        r_cmd = int(round(right_rpm * 10))
        vals = [to_u16_signed(l_cmd), to_u16_signed(r_cmd)]
        res = self.client.write_registers(0x2088, vals, device_id=self.device_id)
        if res is None or res.isError():
            self.get_logger().error(f"write_registers failed @0x2088: {res}")
            self.connected = False

    def set_parking(self, enable: bool):
        if not self.ensure_connected():
            return
        self._write_single(0x200C, 1 if enable else 0)
        self.parked = enable

    def stop(self):
        if not self.ensure_connected():
            return
        self._write_rpms(0.0, 0.0)
        if not self.parked:
            self.set_parking(True)

    def _apply_profile(self, force: bool = False):
        if not self.ensure_connected():
            return
        
        corr = self.auto_state in ("yaw_correction", "align_center", "aruco_centering")

        if (not force) and (corr == self._profile_is_corr):
            return

        if corr:
            acel = self.acel_ms_corr
            decel = self.decel_ms_corr
        else:
            acel = self.acel_ms
            decel = self.decel_ms

        self._write_single(0x2080, int(acel))
        self._write_single(0x2081, int(acel))
        self._write_single(0x2082, int(decel))
        self._write_single(0x2083, int(decel))

        self._profile_is_corr = corr

    # ---- callbacks ----

    def cb_estop(self, msg: Bool):
        self.eStop = bool(msg.data)

    def cb_auto_state(self, msg: String):
        st = (msg.data or "").strip().lower()
        if st != self.auto_state:
            self.auto_state = st
            self._apply_profile(force=False)

    def cb_rpm_cmd(self, msg: Float32MultiArray):
        if msg.data is None or len(msg.data) < 2:
            return
        self.cmd_left_rpm = float(msg.data[0])
        self.cmd_right_rpm = float(msg.data[1])
        self.last_cmd_time = self.get_clock().now()

    # ---- timer functions ----
    
    def watchdog_tick(self):
        if self.last_cmd_time is None:
            return
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > 0.2:
            #self.get_logger().warning("cmd timeout -> STOP")
            self.stop()
            self.last_cmd_time = None

    def write_tick(self):
        if self.eStop:
            self.stop()
            return
        if not self.ensure_connected():
            return

        left = float(self.cmd_left_rpm)
        right = float(self.cmd_right_rpm)

        if self.parked and (abs(left) > 0.01 or abs(right) > 0.01):
            self.set_parking(False)

        # Keep your mapping (right inverted)
        self._write_rpms(left, -right)

    # ---- connect functions ----

    def try_connect(self, init=False) -> bool:
        try:
            if self.client.connect():
                self.connected = True
                self.get_logger().info("Connected to ZLAC8015D" if init else "Reconnected to ZLAC8015D")
                self._write_single(0x2022, 10)
                time.sleep(0.02)
                self._write_single(0x200D, 0x0003)  # velocity mode
                time.sleep(0.02)
                self._write_single(0x200E, 0x0008)
                self.set_parking(False)
                return True
            self.connected = False
            self.get_logger().error("ZLAC8015D connection failed")
            return False
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"connect exception: {e}")
            return False

    def on_reconnect(self, request, response):
        try:
            try:
                if self.connected:
                    self.stop()
                    self._write_single(0x200E, 0)
                    time.sleep(0.05)
                    self.client.close()
                    time.sleep(0.2)
            except Exception as e:
                self.get_logger().warning(f"disable failed: {e}")

            self.connected = False
            if not self.client.connect():
                raise RuntimeError("reconnect failed")

            if not self.try_connect(init=False):
                raise RuntimeError("try_connect failed")

            self.last_cmd_time = None
            response.success = True
            response.message = "Reconnected."
            self.get_logger().info("Reconnected to ZLAC8015D")
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.client.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()