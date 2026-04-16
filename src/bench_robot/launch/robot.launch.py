from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bench_robot')

    motor_mux_params = os.path.join(pkg_share,'config','motor_control_mux.yaml')
    hub_motor_driver_params = os.path.join(pkg_share,'config','hub_motor_driver_v2.yaml')
    bench_tracker_params = os.path.join(pkg_share,'config','bench_tracker_v3.yaml')
    zed_params = os.path.join(pkg_share, 'config', 'zed_mini.yaml')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("zed_wrapper"),"launch","zed_camera.launch.py"])),
        launch_arguments={
            "camera_model": "zedm",
            "ros_params_override_path": zed_params,
            "serial_number": "0",
            "publish_urdf": "false",
            "publish_tf": "false",
            "enable_ipc": "false",
        }.items()
    )

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini_330_series.launch.py"
            ])
        ),
        launch_arguments={
        'enable_colored_point_cloud': 'true',
        'depth_registration': 'true'
    }.items()
    )


    return LaunchDescription([
        orbbec_launch,

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            parameters=[{
            'default_call_service_timeout': 5.0, # Sets a 5s timeout instead of blocking forever
            'call_services_in_new_thread': True, # Prevents service calls from freezing the main thread
            'send_action_goals_in_new_thread': True, # Prevents action goals from freezing the main thread
            }]
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
            output="screen",),
        Node(
            package="bench_robot",
            executable="aruco_detector",
            output="screen",),
        Node(
            package="bench_robot",
            executable="orbbec_test_scan",
            output="screen",)
    ])
