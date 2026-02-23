#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class AutoStateManager(Node):

    def __init__(self):
        super().__init__("auto_state_manager")

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # --- Allowed states ---
        self.valid_states = {
            "idle",
            "bench_tracking_f",
            "bench_tracking_b",
            "yaw_correction",
            "align_center",
        }

        self.allowed = {
            "idle": {"align_center", "bench_tracking_f", "bench_tracking_b", "yaw_correction","steer_0", "steer_90"},
            "align_center": {"steer_0", "idle"},
            "bench_tracking_f": {"yaw_correction", "idle",  "bench_tracking_b", "align_center"},
            "bench_tracking_b": {"yaw_correction", "idle", "bench_tracking_f", "align_center"},
            "yaw_correction": {"align_center", "idle"},
        }

        self.mode = "manual"
        self.auto_state = "idle"

        # Subscribers
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.sub_cmd = self.create_subscription(String, "/auto_state_cmd", self.cb_state_cmd, 10)

        # Publishers
        self.pub_state = self.create_publisher(String, "/auto_state", qos_latched)

        # Publish initial state once
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.auto_state
        self.pub_state.publish(msg)

    def set_state(self, new_state: str, reason: str = "") -> bool:
        new_state = (new_state or "").strip().lower()

        if new_state not in self.valid_states:
            self.get_logger().warning(
                f"Reject state '{new_state}' (invalid). Valid: {sorted(self.valid_states)}"
            )
            return False

        if self.allowed is not None:
            allowed_next = self.allowed.get(self.auto_state, set())
            if new_state != self.auto_state and new_state not in allowed_next:
                self.get_logger().warning(
                    f"Reject transition {self.auto_state} -> {new_state} (not allowed)"
                )
                return False

        if new_state == self.auto_state:
            return True

        self.auto_state = new_state
        self.publish_state()
        self.get_logger().info(f"auto_state -> {self.auto_state}" + (f" ({reason})" if reason else ""))
        return True

    # ---------- Callbacks ----------
    def cb_mode(self, msg: String):
        m = (msg.data or "").strip().lower()
        if m == self.mode:
            return

        self.mode = m
        self.get_logger().info(f"Robot Mode changed -> {self.mode}")

        # If leaving AUTO, force state to idle (global safety)
        if self.mode != "auto":
            self.set_state("idle", reason="mode!=auto (forced)")

    def cb_state_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if not cmd:
            return

        if self.mode != "auto":
            self.get_logger().warning(f"Ignoring auto_state_cmd='{cmd}' because mode='{self.mode}'")
            return

        self.set_state(cmd, reason="cmd")


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
