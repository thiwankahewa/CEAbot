from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_arm_controller = LaunchConfiguration("launch_arm_controller")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_arm = LaunchConfiguration("launch_arm")
    launch_base = LaunchConfiguration("launch_base")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bench_robot"),
                "launch",
                "robot.launch.py"
            ])
        ),
        condition=IfCondition(launch_base),
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bench_robot"),
                "launch",
                "robot_sim_gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_arm_controller": launch_arm_controller,
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(launch_arm),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("launch_arm_controller", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        DeclareLaunchArgument("launch_base", default_value="true"),
        DeclareLaunchArgument("launch_arm", default_value="true"),

        base_launch,
        arm_launch,
    ])