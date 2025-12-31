from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bench_robot",
            executable="motor_controller",
            parameters=["/home/thwia/CEArobot/src/bench_robot/Config/motor_controller.yaml"],
            output="screen",
        ),
         Node(
            package="bench_robot",
            executable="settings_saver",
            output="screen",
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
        ),
    ])
