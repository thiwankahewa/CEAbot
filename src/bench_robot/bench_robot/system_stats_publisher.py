#!/usr/bin/env python3

import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemStatsPublisher(Node):
    def __init__(self):
        super().__init__("system_stats_publisher")

        self.declare_parameter("topic", "/system_stats")
        self.declare_parameter("interval_ms", 1000)
        self.declare_parameter("command", "tegrastats")

        self.topic = str(self.get_parameter("topic").value)
        self.interval_ms = int(self.get_parameter("interval_ms").value)
        self.command = str(self.get_parameter("command").value)

        self.pub_stats = self.create_publisher(String, self.topic, 10)
        self.proc = None
        self.reader_thread = None
        self.stop_event = threading.Event()

        if shutil.which(self.command) is None:
            self.get_logger().warning(f"'{self.command}' not found. {self.topic} will not publish system stats.")
            return

        self.start_tegrastats()

    def start_tegrastats(self):
        cmd = [self.command, "--interval", str(self.interval_ms)]
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start {' '.join(cmd)}: {e}")
            return

        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()
        self.get_logger().info(f"Publishing {' '.join(cmd)} output to {self.topic}")

    def read_loop(self):
        if self.proc is None or self.proc.stdout is None:
            return

        for line in self.proc.stdout:
            if self.stop_event.is_set():
                break

            line = line.strip()
            if not line:
                continue

            self.pub_stats.publish(String(data=line))

        if not self.stop_event.is_set():
            code = self.proc.poll()
            self.get_logger().warning(f"{self.command} stopped unexpectedly with code {code}")

    def destroy_node(self):
        self.stop_event.set()

        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2.0)

        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
