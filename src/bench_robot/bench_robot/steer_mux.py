#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteerMux(Node):
    def __init__(self):
        super().__init__('steer_mux')

        # ---- Topics ----
        self.pub = self.create_publisher(Float32, '/steer_angle_deg', 10)

        self.sub_auto = self.create_subscription(
            Float32, '/steer_auto', self.cb_auto, 10
        )
        self.sub_man = self.create_subscription(
            Float32, '/steer_manual', self.cb_manual, 10
        )

        # ---- State ----
        self.last_auto = Float32()
        self.last_auto.data = 0.0

        self.last_manual = Float32()
        self.last_manual.data = 0.0

        self.last_manual_time = None
        self.manual_timeout_s = 0.5

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz output

    def cb_auto(self, msg: Float32):
        self.last_auto = msg

    def cb_manual(self, msg: Float32):
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
    node = SteerMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
