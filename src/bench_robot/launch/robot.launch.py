from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bench_robot')

    motor_params = os.path.join(pkg_share,'config','motor_controller.yaml')
    motor_mux_params = os.path.join(pkg_share,'config','motor_control_mux.yaml')


    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        Node(
            package='bench_robot',
            executable='arduino_bridge',
            parameters=[motor_params],
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='motor_control_mux',
            parameters=[motor_mux_params],
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
