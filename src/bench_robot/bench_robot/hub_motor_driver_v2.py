#!/usr/bin/env python3


import csv
import os
from datetime import datetime
import time
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from std_msgs.msg import Float32MultiArray, Bool, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


def to_u16_signed(val: int) -> int:
    return val & 0xFFFF

def to_i16(val: int) -> int:
    return val - 65536 if val & 0x8000 else val

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('hub_motor_driver_v2')

        # -------- States and variables --------
        self.eStop = False
        self.auto_state = "manual"

        self.connected = False
        self.port = '/dev/ttyUSB0'
        self.device_id = 1
        self.parked = False
        self.motor_enabled = True
        self._profile_is_corr = False

        self.client = ModbusSerialClient(port=self.port, baudrate=115200)

        self.cmd_left_rpm = 0.0
        self.cmd_right_rpm = 0.0
        self.last_cmd_time = None

        self.power_log_path = '/home/thiwa/CEAbot/power_logs/drive_power_log.csv'
        self.power_samples = []
        self.state_start_time = self.get_clock().now()

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

        # -------- pubs --------
        self.power_pub = self.create_publisher(Float32MultiArray, '/drive_power', 10)

        # -------- timers --------
        self.write_timer = self.create_timer(0.05, self.write_tick)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_tick)
        self.power_timer = self.create_timer(1.0, self.power_tick)

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)
        self.try_connect(init=True)
        self._apply_profile(force=True)
        self._init_power_log()
        
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

    def _read_registers(self, addr: int, count: int):
        try:
            res = self.client.read_holding_registers(addr,count=count,device_id=self.device_id)
            if res is None or res.isError():
                self.get_logger().warning(f"read_registers failed @0x{addr:04X}: {res}")
                self.connected = False
                return None
            return res.registers
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f"read exception @0x{addr:04X}: {e}")
            return None

    def set_parking(self, enable: bool):
        if not self.ensure_connected():
            return
        self._write_single(0x200C, 1 if enable else 0)
        self.parked = enable

    def idle_motor_off(self):
        if not self.ensure_connected():
            return
        self._write_rpms(0.0, 0.0)
        time.sleep(0.02)
        self._write_single(0x200E, 0x0007)  # Shutdown command
        self.motor_enabled = False
        self.parked = False

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
        new_state = bool(msg.data)
        self.eStop = new_state

        if self.eStop:
            self.idle_motor_off()

        else:
            self._write_single(0x200D, 0x0003)  # velocity mode
            time.sleep(0.02)
            self._write_single(0x200E, 0x0008)  # Enable driver again
            time.sleep(0.05)
            self.motor_enabled = True

    def cb_auto_state(self, msg: String):
        st = (msg.data or "").strip().lower()
        if st != self.auto_state:
            old_state = self.auto_state
            # Save average power for the previous state before switching.
            self.save_state_power_average(old_state)
            self.auto_state = st
            self._apply_profile(force=False)

        if self.auto_state == "idle" or self.auto_state == "scan_start":
            self.idle_motor_off()
        else:
            if not self.motor_enabled:
                self._write_single(0x200D, 0x0003)  # velocity mode
                time.sleep(0.02)
                self._write_single(0x200E, 0x0008)  # enable
                time.sleep(0.05)
                self.motor_enabled = True

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
        if self.auto_state == "idle":
            return
        if self.eStop:
            return
        if not self.ensure_connected():
            return

        left = float(self.cmd_left_rpm)
        right = float(self.cmd_right_rpm)

        if not self.motor_enabled:
            self._write_single(0x200E, 0x0008)
            time.sleep(0.05)
            self.motor_enabled = True

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
    
    # ---- power logging ----

    def _init_power_log(self):
        folder = os.path.dirname(self.power_log_path)
        if folder:
            os.makedirs(folder, exist_ok=True)

        if not os.path.exists(self.power_log_path):
            with open(self.power_log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp',
                    'auto_state',
                    'duration_s',
                    'sample_count',
                    'avg_bus_voltage_v',
                    'avg_left_current_a',
                    'avg_right_current_a',
                    'avg_total_current_a',
                    'avg_left_power_w',
                    'avg_right_power_w',
                    'avg_total_power_w',
                    'energy_wh'
                ])

    def read_drive_power(self):
        if not self.ensure_connected():
            return None

        voltage_regs = self._read_registers(0x20A1, 1)
        current_regs = self._read_registers(0x20AD, 2)

        if voltage_regs is None or current_regs is None:
            return None

        bus_v = voltage_regs[0] * 0.01

        left_a = to_i16(current_regs[0]) * 0.1
        right_a = to_i16(current_regs[1]) * 0.1

        # Use absolute current because one motor current can become negative based on direction.
        left_power_w = bus_v * abs(left_a)
        right_power_w = bus_v * abs(right_a)
        total_power_w = left_power_w + right_power_w
        total_current_a = abs(left_a) + abs(right_a)

        return {
            'bus_v': bus_v,
            'left_a': left_a,
            'right_a': right_a,
            'total_current_a': total_current_a,
            'left_power_w': left_power_w,
            'right_power_w': right_power_w,
            'total_power_w': total_power_w
        }

    def power_tick(self):
        data = self.read_drive_power()
        if data is None:
            return

        now = self.get_clock().now()
        self.power_samples.append((now, data))

        msg = Float32MultiArray()
        msg.data = [
            float(data['total_power_w']),
            float(data['left_power_w']),
            float(data['right_power_w']),
            float(data['bus_v']),
            float(data['left_a']),
            float(data['right_a'])
        ]
        self.power_pub.publish(msg)

    def save_state_power_average(self, state_name: str):
        if not self.power_samples:
            return

        now = self.get_clock().now()
        duration_s = (now - self.state_start_time).nanoseconds * 1e-9
        n = len(self.power_samples)

        avg_bus_v = sum(s[1]['bus_v'] for s in self.power_samples) / n
        avg_left_a = sum(s[1]['left_a'] for s in self.power_samples) / n
        avg_right_a = sum(s[1]['right_a'] for s in self.power_samples) / n
        avg_total_current_a = sum(s[1]['total_current_a'] for s in self.power_samples) / n
        avg_left_power_w = sum(s[1]['left_power_w'] for s in self.power_samples) / n
        avg_right_power_w = sum(s[1]['right_power_w'] for s in self.power_samples) / n
        avg_total_power_w = sum(s[1]['total_power_w'] for s in self.power_samples) / n

        energy_wh = avg_total_power_w * duration_s / 3600.0

        with open(self.power_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(timespec='seconds'),
                state_name,
                round(duration_s, 3),
                n,
                round(avg_bus_v, 3),
                round(avg_left_a, 3),
                round(avg_right_a, 3),
                round(avg_total_current_a, 3),
                round(avg_left_power_w, 3),
                round(avg_right_power_w, 3),
                round(avg_total_power_w, 3),
                round(energy_wh, 6)
            ])

        self.get_logger().info(
            f"Saved power average for state '{state_name}': "
            f"{avg_total_power_w:.2f} W, energy={energy_wh:.4f} Wh"
        )

        self.power_samples = []
        self.state_start_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_state_power_average(node.auto_state)
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