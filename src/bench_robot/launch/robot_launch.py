from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        Node(
            package="bench_robot",
            executable="rosout_streamer",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='bench_robot',
            executable='arduino_bridge',
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='motor_control_mux',
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
