from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bench_robot')

    motor_mux_params = os.path.join(pkg_share,'config','motor_control_mux.yaml')
    hub_motor_driver_params = os.path.join(pkg_share,'config','hub_motor_driver_v2.yaml')
    bench_tracker_params = os.path.join(pkg_share,'config','bench_tracker_v3.yaml')


    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        Node(
            package='bench_robot',
            executable='arduino_bridge',
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='motor_control_mux',
            name='motor_control_mux',
            parameters=[motor_mux_params],
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='hub_motor_driver_v2',
            name='hub_motor_driver_v2',
            parameters=[hub_motor_driver_params],
            output='screen',
        ),
        Node(
            package='bench_robot',
            executable='bench_tracker_v3',
            name='bench_tracker_v3',
            parameters=[bench_tracker_params],
            output='screen',
        ),
        Node(
            package="bench_robot",
            executable="settings_saver",
            output="screen",
        ),
        Node(
            package="bench_robot",
            executable="auto_state_manager",
            output="screen",)
        
    ])
