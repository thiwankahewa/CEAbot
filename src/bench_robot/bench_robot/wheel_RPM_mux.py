#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class WheelRpmMux(Node):
    def __init__(self):
        super().__init__('wheel_rpm_mux')

        # ---- Topics ----
        self.pub = self.create_publisher(Int16MultiArray, '/wheel_rpm_cmd', 10)

        self.sub_auto = self.create_subscription(
            Int16MultiArray, '/wheel_rpm_auto', self.cb_auto, 10
        )
        self.sub_man = self.create_subscription(
            Int16MultiArray, '/wheel_rpm_manual', self.cb_manual, 10
        )

        # ---- State ----
        self.last_auto = Int16MultiArray()
        self.last_auto.data = [0, 0]

        self.last_manual = Int16MultiArray()
        self.last_manual.data = [0, 0]

        self.last_manual_time = None
        self.manual_timeout_s = 0.5   # manual override active if updated within this time

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz output

    def cb_auto(self, msg: Int16MultiArray):
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_auto (need [L,R])")
            return
        self.last_auto = msg

    def cb_manual(self, msg: Int16MultiArray):
        if msg.data is None or len(msg.data) < 2:
            self.get_logger().warning("Invalid /wheel_rpm_manual (need [L,R])")
            return
        self.last_manual = msg
        self.last_manual_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()

        use_manual = False
        if self.last_manual_time is not None:
            dt = (now - self.last_manual_time).nanoseconds * 1e-9
            use_manual = dt < self.manual_timeout_s

        out = self.last_manual if use_manual else self.last_auto
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelRpmMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
