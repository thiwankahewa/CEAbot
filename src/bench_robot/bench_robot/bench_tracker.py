#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32, String
from rcl_interfaces.msg import SetParametersResult

class BenchTracker(Node):
    def __init__(self):
        super().__init__('bench_tracker')

        # Subscribes
        self.sub = self.create_subscription(Int16MultiArray,'/bench_robot/tof_raw',self.dist_cb,10)
        self.last = None
        self.invalid_data_warned = False
        self.declare_parameter('Kp_offset', 0.005)
        self.declare_parameter('Kp_yaw', 0.005)
        self.declare_parameter('Kp_error', 0.01)
        self.Kp_offset = self.get_parameter('Kp_offset').value
        self.Kp_yaw = self.get_parameter('Kp_yaw').value
        self.Kp_error = self.get_parameter('Kp_error').value


        self.sub_mode = self.create_subscription(String, '/mode', self.cb_mode, 10)
        self.mode = "manual" 
        self.sub_auto_mode = self.create_subscription(String, '/auto_mode', self.cb_auto_mode, 10)
        self.auto_mode = "mode1"

        # Publishes
        self.rpm_pub = self.create_publisher(Int16MultiArray, '/wheel_rpm_auto', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer_auto', 10)

        self.declare_parameter('base_rpm', 12)
        self.declare_parameter('max_rpm', 25)
        self.base_rpm = self.get_parameter('base_rpm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.track_width_m = 1.515    
        self.wheel_diameter_m = 0.2032  

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
            elif p.name == 'Kp_error':
                self.Kp_error = p.value
                self.get_logger().info(f"[Settings] Kp_error={self.Kp_error}")
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
        return math.pi * self.wheel_diameter_m
    
    def mps_to_rpm(self, v_mps: float) -> float:
        return (60.0 * v_mps) / self.wheel_circumference

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))
    
    def publish_rpm(self, left_rpm: int, right_rpm: int):
        msg = Int16MultiArray()
        msg.data = [int(left_rpm), int(right_rpm)]
        self.rpm_pub.publish(msg)

    # --------- Main functions ----------
    def dist_cb(self, msg):
        self.last = msg.data  # [fl, fr, rl, rr]
        if self.last is None or len(self.last) < 4:
            if not self.invalid_data_warned:
                self.get_logger().warning("Received invalid /bench_robot/tof_raw message")
                self.invalid_data_warned = True
            return
        
        if self.mode == "auto":

            GOOD_MIN = 120
            GOOD_MAX = 160
            CENTER_TOL_M = 0.02
            STOP_THRESHOLD = 25.0

            self.invalid_data_warned = False
            rl , fl, rr , fr = self.last

            def valid(d_mm: float) -> bool:
                return (d_mm is not None) and (STOP_THRESHOLD <= d_mm <= 2000)
            
            if not all(valid(x) for x in (fl, fr, rl, rr)):
                if not self.invalid_data_warned:
                    self.get_logger().warn(f"Invalid ToF data: FL={fl} FR={fr} RL={rl} RR={rr} -> holding/stop")
                    self.invalid_data_warned = True
                # safest: stop; or hold last cmd
                self.publish_rpm(0, 0)
                return
            else:
                self.invalid_data_warned = False

            

            left_avg  = (fl + rl) / 2.0
            right_avg = (fr + rr) / 2.0
            offset_err = (right_avg - left_avg) / 1000.0  # Lateral offset

            L_m = 0.53
            yaw_err_rad = (((fr - rr) - (fl - rl)) / 1000.0) / L_m   # yaw error

            direction = 1
            if self.auto_mode == "mode2":
                direction = -1
                yaw_err_rad = -yaw_err_rad

            in_good_band = all(GOOD_MIN <= x <= GOOD_MAX for x in (fl, fr, rl, rr))
            #near_center  = abs(offset_err) <= CENTER_TOL_M
            #use_yaw = in_good_band and near_center

            if in_good_band:
                yaw_err_rad = 0

            #w = -(self.Kp_offset * offset_err + self.Kp_yaw * yaw_err_rad )
            w = -(self.Kp_offset * offset_err  ) * direction

            
            base_v_mps = direction *((self.base_rpm * self.wheel_circumference) / 60.0)
            v_left  = base_v_mps + w * (self.track_width_m / 2.0) 
            v_right = base_v_mps - w * (self.track_width_m / 2.0)

            left_rpm  = int(self.clamp(int(round(self.mps_to_rpm(v_left))), -self.max_rpm, self.max_rpm))
            right_rpm = int(self.clamp(int(round(self.mps_to_rpm(v_right))), -self.max_rpm, self.max_rpm))

            # --- Steering assist (only for non-small errors) ---
            steer_deg = 0.0
            mode = "DIFF_ONLY"
            '''if abs(w) >= self.w_small:
                mode = "DIFF+STEER"
                steer_deg = self.k_steer * w
                steer_deg = self.clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg)'''

            self.publish_rpm( right_rpm,left_rpm)
            self.steer_pub.publish(Float32(data=float(steer_deg)))

            
            self.get_logger().info(
                f"Dist: FL={fl} FR={fr} RL={rl} RR={rr} "
                f"off={offset_err:.4f} yaw={yaw_err_rad:.4f} w={w:.4f} | "
                f"L={left_rpm:+d} R={right_rpm:+d} | "
                
            )


    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m != self.mode:
            self.mode = m
            self.get_logger().info(f"Mode changed to: {self.mode}")

    def cb_auto_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if not m:
            return
        if m != self.auto_mode:
            self.auto_mode = m
            self.get_logger().info(f"Auto mode changed to: {self.auto_mode}")

def main(args=None):
    rclpy.init()
    node = BenchTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
