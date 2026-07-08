from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    astra2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "astra2.launch.py",
            ])
        ),
        launch_arguments={
            "camera_name": "astra2",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "depth_registration": "true",
            "enable_ir": "false",
            "publish_tf": "false",
        }.items(),
    )

    astra_top_scan_node = Node(
        package="bench_robot",
        executable="astra_top_scan",
        output="screen",
    )

    return LaunchDescription([
        astra2_launch,
        astra_top_scan_node,
    ])
