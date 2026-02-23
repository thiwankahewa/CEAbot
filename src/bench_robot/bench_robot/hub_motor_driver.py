#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from std_msgs import msg
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Bool, Float32, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


def to_u16_signed(val: int) -> int:
    return val & 0xFFFF


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('hub_motor_driver')

        self.eStop = False
        self.mode = "manual"
        self.auto_state = None
        self.bench_track_dir = None

        self.connected = False
        self.port = '/dev/ttyUSB0'
        self.device_id = 1
        self.driver_mode = "vel"  # "vel" or "pos_abs"
        self.motion_active = False
        self.cmd_timeout_s = 0.1     # watchdog timeout
        self.last_cmd_time = None

        self.target_l = 0
        self.target_r = 0

        self.parked = False
        self.client = ModbusSerialClient(port=self.port,baudrate=115200,)
        self.declare_parameter('decel_ms', 500)
        self.decel_ms = self.get_parameter('decel_ms').value
        self.declare_parameter('acel_ms', 500)
        self.acel_ms = self.get_parameter('acel_ms').value
        self.try_connect(init=True)

        # Service
        self.srv_hub_servo_restart = self.create_service(Trigger, "/arduino_bridge/hub_servo_reconnect", self.on_hub_servo_reconnect)

        # Timers
        self.timer = self.create_timer(0.1, self.watchdog_tick)  
        self.pos_read_timer = self.create_timer(0.05, self.pos_read_tick)  # 20 Hz
        self.power_timer = self.create_timer(1, self.publish_motor_power_tick)

        # Subscribers
        self.sub_auto_state = self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)
        self.sub_rpm = self.create_subscription(Float32MultiArray,'/wheel_rpm_cmd',self.cb_rpm,10)
        self.sub_abs_pos = self.create_subscription(Int16MultiArray,'/wheel_abs_pos',self.cb_abs_pos,10)
        self.sub_estop = self.create_subscription(Bool, '/e_stop', self.cb_estop, 10)
        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)

        # Publishers
        self.pub_auto_state = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_motor_power = self.create_publisher(Float32MultiArray, "/motor_power", 10)
        self.pub_correction_done = self.create_publisher(Bool, '/yaw_correction_done', 10)
        self.pub_steer = self.create_publisher(Float32, '/steer_angle_deg', 10)

        self.add_on_set_parameters_callback(self.on_params)

    def on_params(self, params):
        for p in params:
            if p.name == 'decel_ms':
                self.decel_ms = p.value
                self.client.write_register(0x2082, int(self.decel_ms), device_id=self.device_id)  
                self.client.write_register(0x2083, int(self.decel_ms), device_id=self.device_id)
                self.get_logger().info(f"[Settings] decel_ms={self.decel_ms}")
            elif p.name == 'acel_ms':
                self.acel_ms = p.value
                self.client.write_register(0x2080, int(self.acel_ms), device_id=self.device_id)
                self.client.write_register(0x2081, int(self.acel_ms), device_id=self.device_id)
                self.get_logger().info(f"[Settings] acel_ms={self.acel_ms}")
        return SetParametersResult(successful=True)


    # ---------------- Modbus helpers ----------------

    def ensure_connected(self) -> bool:
        if self.connected:
            return True
        return self.try_connect(init=False)
    
    def to_u32(self, val: int) -> int:
        return val & 0xFFFFFFFF
    
    def split_i32_to_u16s(self, val: int):
        u = self.to_u32(val)
        hi = (u >> 16) & 0xFFFF
        lo = u & 0xFFFF
        return hi, lo
    
    def _read_i16(self, addr: int) -> int:
        rr = self.client.read_holding_registers(address=addr, count=1, device_id=self.device_id)
        if rr is None or rr.isError():
            raise RuntimeError(f"Modbus read failed at 0x{addr:04X}")
        v = rr.registers[0] & 0xFFFF
        return v - 0x10000 if v & 0x8000 else v

    def _read_u16(self, addr: int) -> int:
        rr = self.client.read_holding_registers(address=addr, count=1, device_id=self.device_id)
        if rr is None or rr.isError():
            raise RuntimeError(f"Modbus read failed at 0x{addr:04X}")
        return rr.registers[0] & 0xFFFF
    
    def _read_i32(self, addr_hi: int, addr_lo: int) -> int:
        hi = self._read_u16(addr_hi)
        lo = self._read_u16(addr_lo)
        u32 = (hi << 16) | lo
        return u32 - 0x100000000 if (u32 & 0x80000000) else u32
    
    def set_velocity_mode(self):
        self.client.write_register(0x200D, 0x0003, device_id=self.device_id)  # velocity
        time.sleep(0.02)
        self.client.write_register(0x200E, 0x0008, device_id=self.device_id)  # enable
        self.driver_mode = "vel"
        self.get_logger().info("Switched to VELOCITY mode")
        return True
    
    def set_abs_position_mode(self):
        self.client.write_register(0x200D, 0x0002, device_id=self.device_id)  # absolute position
        time.sleep(0.02)
        self.client.write_register(0x2082, 2000, device_id=self.device_id)  
        self.client.write_register(0x2083, 2000, device_id=self.device_id)
        self.client.write_register(0x2080, 2000, device_id=self.device_id)
        self.client.write_register(0x2081, 2000, device_id=self.device_id)
        self.client.write_register(0x208E, 1, device_id=self.device_id)  # left max speed
        self.client.write_register(0x208F, 1, device_id=self.device_id)  # right max speed
        self.client.write_register(0x200E, 0x0008, device_id=self.device_id)  # enable
        self.driver_mode = "pos_abs"
        self.get_logger().info("Switched to ABS POSITION mode")
        return True

    def _write_single(self, addr: int, value: int):
        res = self.client.write_register(addr, value, device_id=self.device_id)  # FC 0x06
        if res.isError():
            self.get_logger().error(f"write_register failed @0x{addr:04X}: {res}")
            self.connected = False

    def _write_rpms(self, left_rpm: float, right_rpm: float):
        l_cmd = int(round(left_rpm ))
        r_cmd = int(round(right_rpm))
        vals = [to_u16_signed(l_cmd), to_u16_signed(r_cmd)]
        res = self.client.write_registers(0x2088, vals, device_id=self.device_id)  # FC 0x10
        if res.isError():
            self.get_logger().error(f"write_registers failed @0x2088: {res}")
            self.connected = False

    def _write_abs_pos(self, left_move: int, right_move: int):
        l_hi, l_lo = self.split_i32_to_u16s(left_move)
        r_hi, r_lo = self.split_i32_to_u16s(right_move)
        vals = [l_hi, l_lo, r_hi, r_lo]
        res = self.client.write_registers(0x208A, vals, device_id=self.device_id)
        if res.isError():
            self.connected = False
            raise RuntimeError(f"write_registers(0x208A) failed: {res}")
        
    def read_actual_pos(self):
        l = self._read_i32(0x20A7, 0x20A8)
        r = self._read_i32(0x20A9, 0x20AA)
        return l, r
    
    def is_abs_move_done(self, target_l: int, target_r: int, tol: int = 50) -> bool:
        lpos, rpos = self.read_actual_pos()
        self.get_logger().info(f"Checking move done: target=({target_l}, {target_r}), actual=({lpos}, {rpos})")
        if target_l >= 0:left_done = (lpos >= target_l - tol)
        else:left_done = (lpos <= target_l + tol)
        if target_r >= 0:right_done = (rpos >= target_r - tol)
        else:right_done = (rpos <= target_r + tol)
        return left_done and right_done
    
    def stop(self):
        self._write_rpms(0, 0)
        if not self.parked:
            self.set_parking(True)

    def set_parking(self, enable: bool):
        if not self.ensure_connected():
            return
        val = 1 if enable else 0
        try:
            self._write_single(0x200C, val)
            self.parked = enable
        except Exception as e:
            self.get_logger().warning(f"Failed to set parking={enable}: {e}")

    def maybe_unpark_for_motion(self, left_rpm: int, right_rpm: int):
        if self.parked and (left_rpm != 0 or right_rpm != 0):
            self.set_parking(False)

    # ---------------- Callbacks ----------------
    def cb_estop(self, msg: Bool):
        self.eStop = msg.data

    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m != self.mode:
            self.mode = m
    
    def cb_auto_state(self, msg: String):
        new_state = (msg.data or "").strip().lower()
        if new_state in ("bench_tracking_f", "bench_tracking_b"):
            self.bench_track_dir = new_state
        self.auto_state = new_state

    def cb_rpm(self, msg: Float32MultiArray):
        if not self.ensure_connected():
            self.get_logger().warning("Motor driver not connected -> ignoring rpm command")
            return

        left_rpm = msg.data[0]
        right_rpm = msg.data[1]

        if self.driver_mode != "vel":
            ok = self.set_velocity_mode()
            if not ok:
                return
            
        self.maybe_unpark_for_motion(left_rpm, right_rpm)
        self._write_rpms(left_rpm, -right_rpm)
        self.last_cmd_time = self.get_clock().now()

    def watchdog_tick(self):
        if self.last_cmd_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout_s:
            self.stop()
            self.last_cmd_time = None 

    def cb_abs_pos(self, msg: Int16MultiArray):
        if not self.ensure_connected():
            self.get_logger().warning("Motor driver not connected -> ignoring move command")
            return
        self._write_single(0x2006, 3)    #reset current absolute position
        self._write_single(0x2005, 3)    #reset feedback position
        time.sleep(1)
        self.left_abs_pos = int(msg.data[0])
        self.right_abs_pos = int(msg.data[1])

        cur_l, cur_r = self.read_actual_pos()
        self.target_l = cur_l + self.left_abs_pos
        self.target_r = cur_r + self.right_abs_pos

        if self.driver_mode != "pos_abs":
            ok = self.set_abs_position_mode()
            if not ok:
                return
            
        try:
            if self.left_abs_pos == 0 and self.right_abs_pos == 0:
                self.pub_steer.publish(Float32(data=90.0)) 
                self.get_logger().info("steer 90 triggered with zero positions -> waiting for 4s")
                time.sleep(2)
                self.pub_auto_state.publish(String(data="align_center"))
                return
            self._write_abs_pos(self.target_l, self.target_r)
            self._write_single(0x200E, 0x0010)
            self.motion_active = True
            self.get_logger().info(f"ABS move started: L={self.left_abs_pos} pulses, R={self.right_abs_pos} pulses")
        except Exception as e:
            self.get_logger().error(f"ABS move failed: {e}")
            self.motion_active = False

    def pos_read_tick(self):
        if not self.motion_active or self.mode != "auto":
            return
        try:
            if self.is_abs_move_done(self.target_l, self.target_r, tol=100):
                self.motion_active = False
                if self.auto_state == "yaw_correction":
                    self.pub_steer.publish(Float32(data=90.0)) 
                    self.get_logger().info("steer 90 triggered -> waiting for 4s")
                    time.sleep(2)
                    self.pub_auto_state.publish(String(data="align_center"))
                elif self.auto_state == "align_center":
                    self.pub_steer.publish(Float32(data=0.0)) 
                    self.get_logger().info("steer 0 triggered -> waiting for 4s")
                    time.sleep(2)
                    self.pub_auto_state.publish(String(data=self.bench_track_dir))
                    self.pub_correction_done.publish(Bool(data=True))
        except Exception as e:
            self.get_logger().warning(f"pos_read_tick status read failed: {e}")

    def publish_motor_power_tick(self):
        if not self.connected:
            self.try_connect(init=False)
            return
        try:
            v_raw  = self._read_u16(0x20A1)  #bus voltage (0.01 V)
            il_raw = self._read_i16(0x20AD)  #left current (0.1 A, signed)
            ir_raw = self._read_i16(0x20AE)  #right current (0.1 A, signed)

            voltage_v = v_raw * 0.01
            i_left_a  = il_raw * 0.1
            i_right_a = ir_raw * 0.1

            current_total_a = abs(i_left_a) + abs(i_right_a)
            power_w = voltage_v * current_total_a
            msg = Float32MultiArray()
            msg.data = [float(power_w), float(current_total_a), float(voltage_v)]
            self.pub_motor_power.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"/motor_power read/publish failed: {e}")
            
    def try_connect(self, init=False) -> bool:
        try:
            if self.client.connect():
                self.connected = True
                if init:
                    self.get_logger().info("Connected to ZLAC8015D")
                else:
                    self.get_logger().info("Reconnected to ZLAC8015D")
                self._write_single(0x2022, 10)    #RPM resolution 10
                time.sleep(0.02)
                self.set_velocity_mode()
                self._write_single(0x2082, int(self.decel_ms))  
                self._write_single(0x2083, int(self.decel_ms))
                self._write_single(0x2080, int(self.acel_ms))
                self._write_single(0x2081, int(self.acel_ms))
                self._write_single(0x200C, 0)
                self.parked = False
                return True
            else:
                self.connected = False
                self.get_logger().error("ZLAC8015D connection failed")
                return False
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"ZLAC8015D connect exception: {e} (node will stay alive)")
            return False
        
    
    def on_hub_servo_reconnect(self, request, response):
        try:
            self.get_logger().info("Reconnecting hub servo driver...")

            try:
                self.stop()
                self._write_single(0x200E, 0)
                time.sleep(0.05)
                self.client.close()
                time.sleep(0.2)
            except Exception as e:
                self.get_logger().warning(f"Disable failed: {e}")

            self.connected = False

            if not self.client.connect():
                raise RuntimeError("Failed to reconnect to ZLAC8015D")

            if not self.try_connect(init=False):
                raise RuntimeError("Failed to reconnect to ZLAC8015D")

            self.last_cmd_time = None
            response.success = True
            response.message = "Hub servo driver reconnected."
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
        node.get_logger().warning("Ctrl+C received -> STOP motors")
    finally:
        node.stop()
        time.sleep(0.05) 
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
