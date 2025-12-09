#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class BenchTracker(Node):
    def __init__(self):
        super().__init__('bench_tracker')
        
        self.sub = self.create_subscription(
            Int16MultiArray,
            '/bench_robot/tof_raw',
            self.dist_cb,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last = None

    def dist_cb(self, msg):

        self.last = msg.data  # [fl, fr, rl, rr]
        self._logger.info(f"Received distances: {self.last}")

        fl, fr, rl, rr = self.last

        left_avg  = (rl + rl) / 2.0
        right_avg = (fr + fl) / 2.0

        error = (right_avg - left_avg)

        Kp = 0.0005  # tune: depends on ADC scale
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = -Kp * error
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"err={error:.1f} w={twist.angular.z:.3f}")

def main(args=None):
    rclpy.init()
    node = BenchTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
