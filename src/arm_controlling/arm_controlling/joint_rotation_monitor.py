#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class JointRotationMonitor(Node):
    """Track reported and accumulated joint-angle extrema."""

    def __init__(self):
        super().__init__("joint_rotation_monitor")

        self.declare_parameter(
            "joint_names", [f"joint_{index}" for index in range(1, 8)]
        )
        self.declare_parameter("report_interval_sec", 2.0)

        self.joint_names = set(self.get_parameter("joint_names").value)
        report_interval = float(self.get_parameter("report_interval_sec").value)

        self.samples = {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.create_timer(max(report_interval, 0.1), self.report)

        self.get_logger().info(
            "Monitoring /joint_states. Reset extrema with "
            "'ros2 service call /joint_rotation_monitor/reset std_srvs/srv/Trigger {}'."
        )

    @staticmethod
    def shortest_angle_delta(current, previous):
        return math.atan2(math.sin(current - previous), math.cos(current - previous))

    def on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name not in self.joint_names:
                continue

            position = float(position)
            sample = self.samples.get(name)
            if sample is None:
                self.samples[name] = {
                    "raw": position,
                    "raw_min": position,
                    "raw_max": position,
                    "unwrapped": position,
                    "unwrapped_min": position,
                    "unwrapped_max": position,
                }
                continue

            sample["unwrapped"] += self.shortest_angle_delta(position, sample["raw"])
            sample["raw"] = position
            sample["raw_min"] = min(sample["raw_min"], position)
            sample["raw_max"] = max(sample["raw_max"], position)
            sample["unwrapped_min"] = min(
                sample["unwrapped_min"], sample["unwrapped"]
            )
            sample["unwrapped_max"] = max(
                sample["unwrapped_max"], sample["unwrapped"]
            )

    def on_reset(self, _request, response):
        self.samples.clear()
        response.success = True
        response.message = "Joint rotation extrema reset"
        return response

    def report(self):
        if not self.samples:
            self.get_logger().info("Waiting for selected joints on /joint_states")
            return

        lines = [
            "Joint rotation extrema (degrees):",
            "joint       current    raw_min    raw_max   unwrap_min  unwrap_max",
        ]
        for name in sorted(self.samples):
            sample = self.samples[name]
            values = [
                sample["raw"],
                sample["raw_min"],
                sample["raw_max"],
                sample["unwrapped_min"],
                sample["unwrapped_max"],
            ]
            degrees = [math.degrees(value) for value in values]
            lines.append(
                f"{name:<10} {degrees[0]:>8.1f}  {degrees[1]:>8.1f}  "
                f"{degrees[2]:>8.1f}  {degrees[3]:>10.1f}  {degrees[4]:>10.1f}"
            )

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = JointRotationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
