#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32

class BenchTracker(Node):
    def __init__(self):
        super().__init__('bench_tracker')
        
        self.sub = self.create_subscription(
            Int16MultiArray,
            '/bench_robot/tof_raw',
            self.dist_cb,
            10
        )
        self.last = None
        self.Kp_offset = 0.005
        self.Kp_yaw = 0.005

        self.rpm_pub = self.create_publisher(Int16MultiArray, '/wheel_rpm_auto', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer_auto', 10)

        self.base_rpm = 12 
        self.max_rpm = 25
        self.track_width_m = 1.37       
        self.wheel_diameter_m = 0.2032  

        

        # steering assist thresholds
        self.w_small = 0.05            #below -> diff only, steer=0
        self.w_medium = 0.30      #above -> steer saturates 
        self.k_steer = 100.0          
        self.max_steer_deg = 25.0

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

    def dist_cb(self, msg):

        self.last = msg.data  # [fl, fr, rl, rr]
        if self.last is None or len(self.last) < 4:
            self.get_logger().warning("Received invalid /bench_robot/tof_raw message")
            return

        fr, rr, fl, rl = self.last

        #self._logger.info(f"Distances (mm): FL={fl} FR={fr} RL={rl} RR={rr}")

        left_avg  = (fl + rl) / 2.0
        right_avg = (fr + rr) / 2.0
        offset_err = right_avg - left_avg  # Lateral offset

        left_delta  = fl - rl
        right_delta = fr - rr
        yaw_err = (left_delta - right_delta) / 2.0   # yaw error
        w = -(self.Kp_offset * offset_err + self.Kp_yaw * yaw_err)

        base_v_mps = (self.base_rpm * self.wheel_circumference) / 60.0
        v_left  = base_v_mps - (w * self.track_width_m / 2.0)
        v_right = base_v_mps + (w * self.track_width_m / 2.0)

        left_rpm  = int(self.clamp(int(round(self.mps_to_rpm(v_left))), -self.max_rpm, self.max_rpm))
        right_rpm = int(self.clamp(int(round(self.mps_to_rpm(v_right))), -self.max_rpm, self.max_rpm))

        # --- Steering assist (only for non-small errors) ---
        steer_deg = 0.0
        mode = "DIFF_ONLY"
        if abs(w) >= self.w_small:
            mode = "DIFF+STEER"
            steer_deg = self.k_steer * w
            steer_deg = self.clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg)

        # publish outputs
        self.publish_rpm(left_rpm, right_rpm)
        self.steer_pub.publish(Float32(data=float(steer_deg)))

        self.get_logger().info(
            f"[{mode}] off={offset_err:.1f} yaw={yaw_err:.1f} w={w:.4f} | "
            f"rpm L={left_rpm:+d} R={right_rpm:+d} | steer={steer_deg:+.1f}°"
        )

def main(args=None):
    rclpy.init()
    node = BenchTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
