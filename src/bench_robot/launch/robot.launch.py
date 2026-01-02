from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        Node(
            package='bench_robot',
            executable='arduino_bridge',
            parameters=["/home/thwia/CEArobot/src/bench_robot/Config/motor_controller.yaml"],
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='motor_control_mux',
            parameters=["/home/thwia/CEArobot/src/bench_robot/Config/motor_control_mux.yaml"],
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='hub_motor_driver',
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='bench_tracker',
            output='screen',
        ),
        Node(
            package="bench_robot",
            executable="settings_saver",
            output="screen",
        ),
        
    ])
