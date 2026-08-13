#!/usr/bin/env python3

import csv
import json
import re
import shutil
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemStatsPublisher(Node):
    POWER_RE = re.compile(r"([A-Za-z0-9_]+)\s+(\d+)mW/(\d+)mW")
    TOTAL_POWER_RAILS = ("VDD_IN", "POM_5V_IN", "VIN_SYS_5V0")

    def __init__(self):
        super().__init__("system_stats_publisher")

        # -------- States and variables --------

        self.topic = "/system_stats"
        self.interval_ms = 1000
        self.command = "tegrastats"

        self.power_log_path = Path("/home/thiwa/CEAbot/power_logs/jetson_power_log.csv").expanduser()

        self.proc = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.power_log_file = None
        self.power_log_writer = None

        # -------- publishers --------
        self.pub_stats = self.create_publisher(String, self.topic, 10)

        # -------- initialization --------
        self.init_power_log()

        if shutil.which(self.command) is None:
            self.get_logger().warning(f"'{self.command}' not found. {self.topic} will not publish system stats.")
            return

        self.start_tegrastats()

    def init_power_log(self):
        """Open the Jetson power CSV and add a header when it is new."""
        try:
            self.power_log_path.parent.mkdir(parents=True, exist_ok=True)
            is_empty = not self.power_log_path.exists() or self.power_log_path.stat().st_size == 0
            self.power_log_file = self.power_log_path.open("a", newline="", encoding="utf-8")
            self.power_log_writer = csv.writer(self.power_log_file)
            if is_empty:
                self.power_log_writer.writerow(["timestamp_utc", "total_power_rail", "total_power_mw", "total_power_avg_mw", "power_rails_json", "tegrastats_raw",])
                self.power_log_file.flush()
            self.get_logger().info(f"Saving Jetson power data to {self.power_log_path}")
        except OSError as e:
            self.get_logger().error(f"Could not open Jetson power log {self.power_log_path}: {e}")
            self.power_log_file = None
            self.power_log_writer = None

    @classmethod
    def parse_power_rails(cls, line):
        """Return all tegrastats power rails as {name: {current_mw, average_mw}}."""
        return {
            name: {"current_mw": int(current), "average_mw": int(average)}
            for name, current, average in cls.POWER_RE.findall(line)
        }

    def save_power_sample(self, line):
        if self.power_log_writer is None or self.power_log_file is None:
            return

        rails = self.parse_power_rails(line)
        if not rails:
            return

        total_rail = next((name for name in self.TOTAL_POWER_RAILS if name in rails), "")
        total = rails.get(total_rail, {})
        try:
            self.power_log_writer.writerow(
                [
                    datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    total_rail,
                    total.get("current_mw", ""),
                    total.get("average_mw", ""),
                    json.dumps(rails, separators=(",", ":")),
                    line,
                ]
            )
            self.power_log_file.flush()
        except OSError as e:
            self.get_logger().error(f"Failed writing Jetson power log: {e}")
            self.power_log_file.close()
            self.power_log_file = None
            self.power_log_writer = None

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
            self.save_power_sample(line)

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

        if self.power_log_file is not None:
            self.power_log_file.close()
            self.power_log_file = None

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
