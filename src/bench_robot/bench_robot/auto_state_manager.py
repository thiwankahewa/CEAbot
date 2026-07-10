#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class AutoStateManager(Node):
    def __init__(self):
        super().__init__("auto_state_manager")

        # -------- States and variables --------
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        # Allowed transitions
        self.allowed = {
            "manual": {"idle", "bench_tracking_f", "bench_tracking_b", "top_view_scan"},
            "idle": {"manual", "align_center", "bench_tracking_f", "bench_tracking_b", "yaw_correction"},
            "bench_tracking_f": {"manual", "idle", "yaw_correction", "bench_tracking_b", "align_center", "bench_change_start"},
            "bench_tracking_b": {"manual", "yaw_correction", "bench_tracking_f", "align_center", "bench_change_start"},
            "yaw_correction": {"manual", "idle", "align_center"},
            "align_center": {"manual", "idle", "bench_tracking_f", "bench_tracking_b", "aruco_centering"},
            "aruco_centering": {"manual", "idle", "top_view_scan"},
            "top_view_scan": {"manual", "idle", "plant_row_coordinates", "bench_tracking_f", "bench_tracking_b"},
            "plant_row_coordinates": {"manual", "idle", "individual_plant_scan"},
            "individual_plant_scan": {"manual", "idle"},
            "bench_change_start": {"manual", "idle"},
        }

        self.mode = "manual"
        self.auto_state = "idle"

        # -------- subs --------
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.sub_cmd = self.create_subscription(String, "/auto_state_cmd", self.cb_state_cmd, 10)

        # -------- pubs --------
        self.pub_state = self.create_publisher(String, "/auto_state", qos_latched)

        # Publish initial state once
        self.publish_state()

    # -------- helper functions --------

    def publish_state(self):
        self.pub_state.publish(String(data=self.auto_state))

    # ---------- Callbacks ----------

    def cb_mode(self, msg: String):
        mode = (msg.data or "").strip().lower()
        if mode != self.mode:
            self.mode = mode
            self.get_logger().info(f"Robot Mode changed -> {self.mode}")

        # If leaving AUTO, force state to manual (global safety)
        if self.mode != "auto":
            self.set_state("manual", reason="mode!=auto (forced)")

    def cb_state_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if not cmd:
            return

        if self.mode != "auto":
            self.get_logger().warning(f"Ignoring auto_state_cmd='{cmd}' because mode='{self.mode}'")
            return

        self.set_state(cmd, reason="cmd")

    # -------- main function --------

    def set_state(self, new_state: str, reason: str = "") -> bool:
        new_state = (new_state or "").strip().lower()

        if new_state not in self.allowed:
            self.get_logger().warning(f"Reject state '{new_state}' (invalid). Valid: {sorted(self.allowed)}")
            return False

        if new_state == self.auto_state:
            return True

        if new_state not in self.allowed[self.auto_state]:
            self.get_logger().warning(f"Reject transition {self.auto_state} -> {new_state} (not allowed)")
            return False

        self.auto_state = new_state
        self.publish_state()
        self.get_logger().info(f"auto_state -> {self.auto_state}" + (f" ({reason})" if reason else ""))
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AutoStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
