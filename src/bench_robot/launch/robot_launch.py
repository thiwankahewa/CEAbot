from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bench_robot',
            executable='arduino_bridge_node',
            name='arduino_bridge',
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='bench_tracker_node',
            name='bench_tracker',
            output='screen',
        ),
         Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
        ),
    ])
