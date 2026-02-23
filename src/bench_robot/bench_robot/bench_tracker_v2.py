#!/usr/bin/env python3
import math

from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray, String, Bool
from rcl_interfaces.msg import SetParametersResult

STOP_THRESHOLD = 25      #if any sensor reads below this,  stop immediately
YAW_THRESHOLD = 0.05       #if the difference between front and rear on either side exceeds this, trigger yaw correction
YAW_CORRECTION_FACTOR = 8
CENTER_CORRECTION_FACTOR = 20
WHEEL_DIAMETER_M = 0.2032

class BenchTracker(Node):
    def __init__(self):
        super().__init__('bench_tracker_v2')

        self.eStop = False
        self.mode = "manual" 
        self.auto_state = None
        self.yaw_correction_trigered = False
        self.align_center_triggered = False

        # Subscribes
        self.sub_auto_state = self.create_subscription(String, "/auto_state", self.cb_auto_state, 10)

        self.sub = self.create_subscription(Int16MultiArray,'/bench_robot/tof_raw',self.dist_cb,10)
        self.last = None
        self.invalid_data_warned = False
        self.declare_parameter('Kp_offset', 0.005)
        self.declare_parameter('Kp_yaw', 0.005)
        self.Kp_offset = self.get_parameter('Kp_offset').value
        self.Kp_yaw = self.get_parameter('Kp_yaw').value

        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)

        self.sub_auto_state = self.create_subscription(String, '/auto_state', self.cb_auto_state, 10)

        self.sub_correction_done = self.create_subscription(Bool, '/yaw_correction_done', self.cb_correction_done, 10)

        self.sub_estop = self.create_subscription(Bool, '/e_stop', self.cb_estop, 10)

        # Publishes
        self.pub_auto_state = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_rpm = self.create_publisher(Float32MultiArray, '/wheel_rpm_auto', 10)
        self.pub_abs_pos = self.create_publisher(Int16MultiArray, '/wheel_abs_pos', 10)
        #self.steer_pub = self.create_publisher(Float32, '/steer_auto', 10)

        self.declare_parameter('base_rpm', 12)
        self.declare_parameter('max_rpm', 25)
        self.base_rpm = self.get_parameter('base_rpm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.track_width_m = 1.515    

        self.declare_parameter('w_small',0.01)
        self.w_small = self.get_parameter('w_small').value
        self.declare_parameter('k_steer', 100.0)
        self.k_steer = self.get_parameter('k_steer').value
        self.declare_parameter('max_steer_deg', 25.0)
        self.max_steer_deg = self.get_parameter('max_steer_deg').value

        
        self.add_on_set_parameters_callback(self.on_params)
    
    def on_params(self, params):
        for p in params:
            if p.name == 'Kp_offset':
                self.Kp_offset = p.value
                self.get_logger().info(f"[Settings] Kp_offset={self.Kp_offset}")
            elif p.name == 'Kp_yaw':
                self.Kp_yaw = p.value
                self.get_logger().info(f"[Settings] Kp_yaw={self.Kp_yaw}")
            elif p.name == 'base_rpm':
                self.base_rpm = p.value
                self.get_logger().info(f"[Settings] base_rpm={self.base_rpm}")
            elif p.name == 'max_rpm':
                self.max_rpm = p.value
                self.get_logger().info(f"[Settings] max_rpm={self.max_rpm}")
            elif p.name == 'k_steer':
                self.k_steer = p.value
                self.get_logger().info(f"[Settings] k_steer={self.k_steer}")
            elif p.name == 'w_small':
                self.w_small = p.value
                self.get_logger().info(f"[Settings] w_small={self.w_small}")
            elif p.name == 'max_steer_deg':
                self.max_steer_deg = p.value
                self.get_logger().info(f"[Settings] max_steer_deg={self.max_steer_deg}")
        return SetParametersResult(successful=True)
    
    
            
    # ---------- Helpers ----------
    @property
    def wheel_circumference(self) -> float:
        return math.pi * WHEEL_DIAMETER_M
    
    def mps_to_rpm(self, v_mps: float) -> float:
        return (60.0 * v_mps) / self.wheel_circumference

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))
    
    def publish_rpm(self, left_rpm: float, right_rpm: float):
        msg = Float32MultiArray()
        msg.data = [float(left_rpm), float(right_rpm)]
        self.pub_rpm.publish(msg)

    def publish_abs_pos(self, left_abs_pos: int, right_abs_pos: int):
        msg = Int16MultiArray()
        msg.data = [int(left_abs_pos), int(right_abs_pos)]
        self.pub_abs_pos.publish(msg)

    # --------- Main functions ----------
    def cb_estop(self, msg: Bool):
        self.eStop = msg.data

    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m != self.mode:
            self.mode = m
            if self.mode != "auto":
                self.yaw_correction_trigered = False
                self.align_center_triggered = False

    def cb_auto_state(self, msg: String):
        new_state = (msg.data or "").strip().lower()
        self.auto_state = new_state
        
    def dist_cb(self, msg):
        self.last = msg.data  
        if self.last is None or len(self.last) < 4:
            if not self.invalid_data_warned:
                self.get_logger().warning("Received invalid /bench_robot/tof_raw message")
                self.invalid_data_warned = True
            return
        
        if self.mode == "auto":
            rl , fl, rr , fr = self.last

            def valid(d_mm: float) -> bool:
                return (d_mm is not None) and (STOP_THRESHOLD <= d_mm <= 500)
            
            if not all(valid(x) for x in (fl, fr, rl, rr)):
                self.invalid_data_warned = True
                self.get_logger().warn(f"Invalid ToF data: FL={fl} FR={fr} RL={rl} RR={rr} -> holding/stop")
                self.publish_rpm(0, 0)
                return
            else:
                self.invalid_data_warned = False

            left_avg  = (fl + rl) / 2.0
            right_avg = (fr + rr) / 2.0
            offset_err = (right_avg - left_avg) / 1000.0  # Lateral offset

            yaw_left  = fl - rl
            yaw_right = rr - fr
            yaw_ave = ((yaw_left + yaw_right) / 2.0) / 1000
            yaw_distance = int(yaw_ave / self.wheel_circumference * 1024) * YAW_CORRECTION_FACTOR

            if (abs(yaw_ave) > YAW_THRESHOLD):
                if self.yaw_correction_trigered:
                    return
                self.yaw_correction_trigered = True
                self.publish_rpm(0.0, 0.0)
                sleep(1)
                self.pub_auto_state.publish(String(data="yaw_correction"))
                
                if abs(yaw_ave) > 0.3:
                    self.get_logger().info(f"Large yaw detected (ave={yaw_ave:.4f} m)")
                    return
                
                self.get_logger().info(f"Aligning... yaw_ave={yaw_ave:.4f} m -> move {yaw_distance} ticks")
                self.publish_abs_pos(-yaw_distance, -yaw_distance)
                return
            
            if self.auto_state == "align_center" :
                if self.align_center_triggered:
                    return
                self.align_center_triggered = True
                align_distance = int((offset_err/2) / self.wheel_circumference * 1024) * CENTER_CORRECTION_FACTOR
                self.get_logger().info(f"Aligning... align_distance={align_distance:.4f} m")
                self.publish_abs_pos(-align_distance, -align_distance)
                return
            
            if ((self.auto_state == "bench_tracking_f") or (self.auto_state == "bench_tracking_b")):
                direction = 1
                if self.auto_state == "bench_tracking_b":
                    direction = -1

                w = -(self.Kp_offset * offset_err  ) * direction

                base_v_mps = direction *((self.base_rpm * self.wheel_circumference) / 60.0)
                v_left  = base_v_mps + w * (self.track_width_m / 2.0) 
                v_right = base_v_mps - w * (self.track_width_m / 2.0)

                left_rpm  = self.clamp(self.mps_to_rpm(v_left), -self.max_rpm, self.max_rpm)
                right_rpm = self.clamp(self.mps_to_rpm(v_right), -self.max_rpm, self.max_rpm)

                self.get_logger().info(
                    f"Dist: FL={fl} FR={fr} RL={rl} RR={rr} "
                    f"off={w:.4f} yaw={yaw_ave:.4f} | "
                    f"L={left_rpm:.1f} R={right_rpm:+.1f} | "
                )

                self.publish_rpm(right_rpm, left_rpm)

    def cb_correction_done(self, msg: Bool):
        if msg.data:
            self.yaw_correction_trigered = False
            self.align_center_triggered = False
            self.get_logger().info("Yaw correction completed, resuming normal tracking.")

def main(args=None):
    rclpy.init()
    node = BenchTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
