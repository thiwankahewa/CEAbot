from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    orbbec_launch = os.path.join(
        FindPackageShare('orbbec_camera').find('orbbec_camera'),
        'launch',
        'gemini_330_series.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch),
            launch_arguments={
                'enable_point_cloud': 'false',
                'depth_registration': 'true',
                'enable_color': 'false',
                'enable_depth': 'false',
                'enable_colored_point_cloud': 'false',
                'enable_color_auto_white_balance': 'true',
                'enable_color_auto_exposure': 'true',
                #'color_exposure': '1000',
            }.items()
        )
    ])