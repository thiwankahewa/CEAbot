from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package that contains zed_camera.launch.py
    zed_wrapper_dir = get_package_share_directory("zed_wrapper")
    zed_launch = os.path.join(zed_wrapper_dir, "launch", "zed_camera.launch.py")

    # Replace this with your actual package name that contains the yaml file
    my_pkg_dir = get_package_share_directory("bench_robot")
    zed_params = os.path.join(my_pkg_dir, "config", "zed_mini.yaml")

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": "zed2i",
            "ros_params_override_path": zed_params,
            "serial_number": "0",
            "publish_urdf": "false",
            "publish_tf": "false",
            "enable_ipc": "false",
        }.items()
    )

    return LaunchDescription([
        zed_camera
    ])