import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression, NotSubstitution

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # --- Launch Configurations ---
    dof = LaunchConfiguration("dof")
    robot_ip = LaunchConfiguration("robot_ip")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")
    port = LaunchConfiguration("port")
    use_fake_hardware_arg = LaunchConfiguration("use_fake_hardware")
    launch_arm_controller = LaunchConfiguration("launch_arm_controller")

    # --- Internal Logic for Hardware Selection ---
    # Detect if we are connecting to a real robot (IP is not empty/default)
    ip_val = context.perform_substitution(robot_ip)
    is_real_robot = ip_val != "xxx.xxx.xxx.xxx" and ip_val != ""
    
    is_fake_hw_str = context.perform_substitution(use_fake_hardware_arg).lower()
    is_fake_hw = is_fake_hw_str == 'true' and not is_real_robot

    # Determine simulation status
    # If it's a real robot OR fake hardware, sim_ignition MUST be false
    is_sim = not is_real_robot and not is_fake_hw
    sim_ignition_val = "true" if is_sim else "false"
    
    # Final check for the URDF xacro arguments 
    use_fake_hw_val = "true" if is_fake_hw else "false"
    
    # Real robots and fake hardware both use system clock (sim_time = false)
    actual_sim_time = False if (is_real_robot or is_fake_hw) else True

    # --- Robot Description (URDF/Xacro) ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bench_robot_description"), "urdf", "fixed_structure.urdf.xacro"]
            ),
            " ",
            "dof:=", dof,
            " ",
            "robot_ip:=", robot_ip,
            " ",
            "username:=", username,
            " ",
            "password:=", password,
            " ",
            "port:=", port,
            " ",
            "sim_ignition:=", sim_ignition_val,
            " ",
            "use_fake_hardware:=", use_fake_hw_val,
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    dof_raw = context.perform_substitution(LaunchConfiguration("dof"))
    moveit_package_name = PythonExpression(["'kinova_' + '", dof, "' + 'dof_moveit_config'"])
    moveit_package_str = f"kinova_{dof_raw}dof_moveit_config"

    # --- Controller Manager Path ---
    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_package_str), "config", "ros2_controllers.yaml"
    )

    # --- Nodes Definition ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": actual_sim_time}],
    )

    # Standalone Controller Manager (Required for Real Robot and Fake Hardware)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path, {"use_sim_time": actual_sim_time}],
        output="both",
        condition=UnlessCondition(sim_ignition_val), # Run if NOT in simulation
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # --- MoveIt 2 Configuration ---
    moveit_config = (
        MoveItConfigsBuilder("CEAbot", package_name=moveit_package_str)
        .robot_description(mappings={
            "use_fake_hardware": use_fake_hw_val, 
            "sim_ignition": sim_ignition_val,
            "robot_ip": ip_val
        })
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": actual_sim_time}],
        
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory(moveit_package_str), "config", "moveit.rviz")],
        parameters=[moveit_config.to_dict(), {"use_sim_time": actual_sim_time}],
        condition=UnlessCondition(launch_arm_controller),
    )

    marker_422_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.792', '--y', '0', '--z', '-0.452', # 422mm down from beam
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'height_marker_422_left'
        ]
    )

    marker_722_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.792', '--y', '0', '--z', '-0.752', 
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'height_marker_722_left'
        ]
    )

    marker_bench_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.792', '--y', '0', '--z', '-1.202', 
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'height_marker_bench_left'
        ]
    )

    moveit_py_param_file = os.path.join(get_package_share_directory("bench_robot"),"config","moveit_py.yaml",)

    arm_controller_node = Node(
        package="bench_robot_cpp",
        executable="arm_controller",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": actual_sim_time},
        ],
        condition=IfCondition(launch_arm_controller),
    )

    # --- Execution Logic ---
    nodes_to_start = [robot_state_publisher_node, ros2_control_node, marker_422_left, marker_722_left,marker_bench_left]

    gz_bridge_params = os.path.join(get_package_share_directory("bench_robot_moveit"), "config", "gazebo_default.config")

    if is_sim:
        # GAZEBO MODE
        ignition_spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string", robot_description_content, "-name", "gen3", "-z", "0.3"],
        )
        nodes_to_start += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
                launch_arguments={"ign_args": "-r -v 3 empty.sdf",}.items(),
            ),
            ignition_spawn_entity,
            RegisterEventHandler(
                OnProcessExit(target_action=ignition_spawn_entity, on_exit=[joint_state_broadcaster_spawner])
            ),
        ]
    else:
        # REAL ROBOT or FAKE MODE
        nodes_to_start += [joint_state_broadcaster_spawner]

    # Shared Controller -> MoveIt chain
    nodes_to_start += [
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_traj_controller_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=robot_traj_controller_spawner, on_exit=[move_group_node, rviz_node,arm_controller_node])
        ),
        
    ]

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        # Robot Connection Arguments
        DeclareLaunchArgument("dof", default_value="7"),
        DeclareLaunchArgument("robot_ip", default_value="xxx.xxx.xxx.xxx"),
        DeclareLaunchArgument("username", default_value="admin"),
        DeclareLaunchArgument("password", default_value="admin"),
        DeclareLaunchArgument("port", default_value="10000"),
        
        # Mode Arguments
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("launch_arm_controller", default_value="false"),
        
        OpaqueFunction(function=launch_setup)
    ])