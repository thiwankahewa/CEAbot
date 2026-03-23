import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bench_robot_description_dir = FindPackageShare("bench_robot_description")
    default_model = PathJoinSubstitution([bench_robot_description_dir, "urdf", "fixed_structure.urdf.xacro"])

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model,
        description="Absolute path to robot urdf xacro file",
    )

    robot_description = ParameterValue(
    Command([
        FindExecutable(name="xacro"),
        " ",
        LaunchConfiguration("model"),
    ]),
    value_type=str,
)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [bench_robot_description_dir, "rviz", "display.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])