from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gemini335_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini_330_series.launch.py",
            ])
        ),
        launch_arguments={
            "camera_name": "gemini335",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "depth_registration": "true",
            "enable_ir": "false",
            "publish_tf": "false",
        }.items(),
    )

    top_scan_node = Node(
        package="bench_robot",
        executable="top_scan",
        output="screen",
    )

    return LaunchDescription([
        gemini335_launch,
        top_scan_node,
    ])
