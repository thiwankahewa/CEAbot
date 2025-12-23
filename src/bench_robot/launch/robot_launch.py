from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bench_robot',
            executable='arduino_bridge_node',
            name='arduino_bridge',
            output='screen',
        ),
        TimerAction(
            period=0.1,
            actions=[
                Node(
                    package='bench_robot',
                    executable='bench_tracker_node',
                    name='bench_tracker',
                    output='screen',
                ),
            ],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
        )
    ])
