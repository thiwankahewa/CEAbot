#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

OUTPUT_DIR = "/home/thwia/CEArobot/src/bench_robot/Config"
TARGET_NODES = [
    "/motor_controller",
]

class SettingsSaver(Node):
    def __init__(self):
        super().__init__("settings_saver")
        self.srv = self.create_service(Trigger, "/settings/save_all", self.on_save)
        self.get_logger().info("Ready: /settings/save_all (Trigger)")

    def on_save(self, request, response):
        failed = []
        for node_name in TARGET_NODES:
            cmd = ["ros2", "param", "dump", node_name, "--output-dir", OUTPUT_DIR]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                failed.append((node_name, r.stderr.strip() or "dump failed"))

        if failed:
            response.success = False
            response.message = "Failed: " + "; ".join([f"{n}: {e}" for n, e in failed])
            return response

        response.success = True
        return response

def main():
    rclpy.init()
    rclpy.spin(SettingsSaver())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
