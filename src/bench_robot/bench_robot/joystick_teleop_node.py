#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__('joystick_teleop')

        self.declare_parameter('max_linear_speed', 0.5)      # m/s
        self.declare_parameter('max_angular_speed', 1.5)     # rad/s
        self.declare_parameter('max_steering_angle_deg', 45.0)

        self.declare_parameter('throttle_axis', 1)           # left stick vertical
        self.declare_parameter('steering_axis', 3)           # right stick horizontal
        self.declare_parameter('dpad_x_axis', 6)             # D-pad left/right
        self.declare_parameter('dpad_y_axis', 7)             # D-pad up/down

        self.declare_parameter('btn_x_index', 2)             # X
        self.declare_parameter('btn_y_index', 3)             # Y
        self.declare_parameter('btn_b_index', 1)             # B

        self.declare_parameter('enable_button', 5)

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        max_steer_deg = float(
            self.get_parameter('max_steering_angle_deg').value
        )
        self.max_steering_angle_rad = math.radians(max_steer_deg)

        self.throttle_axis = int(self.get_parameter('throttle_axis').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.dpad_x_axis = int(self.get_parameter('dpad_x_axis').value)
        self.dpad_y_axis = int(self.get_parameter('dpad_y_axis').value)

        self.btn_x_index = int(self.get_parameter('btn_x_index').value)
        self.btn_y_index = int(self.get_parameter('btn_y_index').value)
        self.btn_b_index = int(self.get_parameter('btn_b_index').value)

        self.enable_button = int(self.get_parameter('enable_button').value)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steering_pub = self.create_publisher(Float32, '/steering_angle_cmd', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # State
        self.prev_buttons = []
        self.last_steering_angle = 0.0  # rad

        self.get_logger().info("JoystickTeleop node started")

    # --- Utility -----------------------------------------------------

    @staticmethod
    def _get(lst, idx, default=0.0):
        if idx < 0 or idx >= len(lst):
            return default
        return lst[idx]

    @staticmethod
    def _deadzone(value, dz=0.1):
        return 0.0 if abs(value) < dz else value

    # --- Main callback -----------------------------------------------

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        # If enable_button is not pressed, force everything to zero
        enable_val = 0
        if 0 <= self.enable_button < len(buttons):
            enable_val = buttons[self.enable_button]

        enable_pressed = (enable_val == 1)

        if not enable_pressed:
            # No motion allowed: zero twist and steering
            twist = Twist()
            steer_msg = Float32()
            steer_msg.data = 0.0

            self.cmd_vel_pub.publish(twist)
            self.steering_pub.publish(steer_msg)

            # Update prev_buttons so we don't get false "rising edges"
            self.prev_buttons = list(buttons)
            self.last_steering_angle = 0.0
            return

        # --- read axes ---
        throttle_axis_val = self._deadzone(
            self._get(axes, self.throttle_axis, 0.0)
        )
        steering_axis_val = self._deadzone(
            self._get(axes, self.steering_axis, 0.0)
        )
        dpad_x = self._get(axes, self.dpad_x_axis, 0.0)
        dpad_y = self._get(axes, self.dpad_y_axis, 0.0)

        twist = Twist()

        # --- Ackermann base (normal joystick) ---
        twist.linear.x = -throttle_axis_val * self.max_linear_speed

        steering_norm = steering_axis_val  # -1..1
        steering_angle = steering_norm * self.max_steering_angle_rad
        twist.angular.z = steering_norm * self.max_angular_speed

        # --- Differential override via D-pad ---
        diff_mode_active = False
        if abs(dpad_x) > 0.5 or abs(dpad_y) > 0.5:
            diff_mode_active = True

            # Always command servo to center in diff mode
            steering_angle = 0.0

            # You choose diff behavior:
            if abs(dpad_x) > 0.5:
                # spin in place
                twist.linear.x = 0.0
                twist.angular.z = dpad_x * self.max_angular_speed
            elif abs(dpad_y) > 0.5:
                # straight forward/back
                twist.angular.z = 0.0
                twist.linear.x = dpad_y * self.max_linear_speed

        # --- Discrete steering via X / Y / B buttons ---
        # Only allow these if NOT in diff mode
        if not diff_mode_active:
            if not self.prev_buttons:
                self.prev_buttons = list(buttons)

            def rising(idx: int) -> bool:
                if idx < 0 or idx >= len(buttons):
                    return False
                prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
                return (prev == 0 and buttons[idx] == 1)

            steer_angle_override = None
            if rising(self.btn_x_index):
                steer_angle_override = -math.pi / 2.0
            elif rising(self.btn_y_index):
                steer_angle_override = 0.0
            elif rising(self.btn_b_index):
                steer_angle_override = math.pi / 2.0

            if steer_angle_override is not None:
                steering_angle = steer_angle_override
                self.get_logger().info(
                    f"Discrete steering: {math.degrees(steer_angle_override):.1f} deg"
                )

        self.last_steering_angle = steering_angle

        # --- Publish ---
        self.cmd_vel_pub.publish(twist)

        steer_msg = Float32()
        steer_msg.data = float(steering_angle)
        self.steering_pub.publish(steer_msg)

        self.prev_buttons = list(buttons)



def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
