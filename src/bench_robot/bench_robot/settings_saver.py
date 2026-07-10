#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

OUTPUT_DIR = "/home/thiwa/CEAbot/src/bench_robot/config"
TARGET_NODES = (
    "/bench_tracker_v3",
    "/motor_control_mux",
    "/hub_motor_driver_v2",
    "/plant_view_scanner",
)


class SettingsSaver(Node):
    def __init__(self):
        super().__init__("settings_saver")

        # -------- services --------
        self.srv = self.create_service(Trigger, "/settings/save_all", self.on_save)

    # -------- main functions --------

    def on_save(self, _request, response):
        failed = []
        for node_name in TARGET_NODES:
            cmd = ["ros2", "param", "dump", node_name, "--output-dir", OUTPUT_DIR]
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
                if result.returncode != 0:
                    failed.append((node_name, result.stderr.strip() or "dump failed"))
            except (OSError, subprocess.TimeoutExpired) as error:
                failed.append((node_name, str(error)))

        if failed:
            response.success = False
            response.message = "Failed: " + "; ".join(f"{node}: {error}" for node, error in failed)
            return response

        response.success = True
        response.message = "All settings saved successfully."
        self.get_logger().info("All settings saved successfully.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SettingsSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
