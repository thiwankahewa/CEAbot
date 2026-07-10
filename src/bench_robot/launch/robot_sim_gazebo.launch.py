import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,RegisterEventHandler,SetEnvironmentVariable,TimerAction,)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command,FindExecutable,LaunchConfiguration,PathJoinSubstitution,)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression, NotSubstitution
from launch.actions import SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # --- Launch Configurations ---
    robot_ip = LaunchConfiguration("robot_ip")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")
    port = LaunchConfiguration("port")
    use_fake_hardware_arg = LaunchConfiguration("use_fake_hardware")
    launch_arm_controller = LaunchConfiguration("launch_arm_controller")
    use_rviz = LaunchConfiguration("use_rviz")

    # --- Internal Logic for Hardware Selection ---
    # Detect if we are connecting to a real robot (IP is not empty/default)
    ip_val = context.perform_substitution(robot_ip)
    is_real_robot =  ip_val != ""
    
    is_fake_hw_str = context.perform_substitution(use_fake_hardware_arg).lower()
    is_fake_hw = is_fake_hw_str == 'true' and not is_real_robot

    # If it's a real robot OR fake hardware, sim_ignition MUST be false
    is_sim = not is_fake_hw and not is_real_robot
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
            PathJoinSubstitution([FindPackageShare("bench_robot_description"), "urdf", "fixed_structure.urdf.xacro"]),
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

    moveit_package_str = "kinova_7dof_moveit_config"

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[robot_description, {"use_sim_time": actual_sim_time}],

    )

    

    # Standalone Controller Manager (Required for Real Robot and Fake Hardware)
    ros2_controllers_path = os.path.join(get_package_share_directory(moveit_package_str), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ robot_description,ros2_controllers_path, {"use_sim_time": actual_sim_time}],
        #arguments=["--ros-args", "--log-level", "warn",],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="robot_traj_controller_spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager","--ros-args", "--log-level", "warn", "--disable-stdout-logs"],
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
        arguments=["--ros-args", "--log-level", "warn","--disable-stdout-logs",],
        parameters=[moveit_config.to_dict(), {"use_sim_time": actual_sim_time}],
        
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory(moveit_package_str), "config", "moveit.rviz"), 
                   "--ros-args", "--log-level", "warn"],
        parameters=[moveit_config.to_dict(), {"use_sim_time": actual_sim_time}],
        condition=IfCondition(use_rviz),
    )

    '''marker_422_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.792', '--y', '0', '--z', '-0.452', # 422mm down from beam
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'marker_422_left',
            "--ros-args", "--log-level", "warn"
        ],
    )

    marker_722_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.792', '--y', '0', '--z', '-0.752', 
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'marker_722_left',
            "--ros-args", "--log-level", "warn"
        ],
    )

    marker_bench_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '-1.202', 
            '--frame-id', 'top_beam_link', 
            '--child-frame-id', 'marker_bench_center',
            "--ros-args", "--log-level", "warn"
        ],
    )'''

    arm_manager_node = Node(
        package="arm_controlling",
        executable="arm_manager",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": actual_sim_time},
        ],
        condition=IfCondition(launch_arm_controller),
    )

    plant_view_scanner_node = Node(
        package="arm_controlling",
        executable="plant_view_scanner",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": actual_sim_time},
        ],
        condition=IfCondition(launch_arm_controller),
    )

    # --- Execution Logic ---
    nodes_to_start = [robot_state_publisher_node]

    if is_sim:

        pkg_desc = get_package_share_directory("bench_robot_description")
        world_path = os.path.join(pkg_desc, "worlds", "greenhouse.world.sdf")
        models_path = os.path.join(pkg_desc, "models")

        gz_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH",value=models_path + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
        
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": f"-r -v 3 {world_path}"}.items(),
            #launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
        )
            
        # GAZEBO MODE
        ignition_spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string",  robot_description["robot_description"], "-name", "gen3", "-z", "0.3"],
        )

        delayed_spawn = TimerAction(period=5.0,actions=[ignition_spawn_entity])

        nodes_to_start += [
            gz_resource_path,
            gz_sim,
            delayed_spawn,
            RegisterEventHandler(OnProcessExit(target_action=ignition_spawn_entity, on_exit=[joint_state_broadcaster_spawner])),
        ]
    else:               
        nodes_to_start += [ros2_control_node, joint_state_broadcaster_spawner]     # REAL ROBOT or FAKE MODE

    nodes_to_start += [
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_traj_controller_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=robot_traj_controller_spawner, on_exit=[move_group_node, rviz_node, arm_manager_node, plant_view_scanner_node])), 
    ]

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        # Robot Connection Arguments
        DeclareLaunchArgument("robot_ip", default_value=""),
        DeclareLaunchArgument("username", default_value="admin"),
        DeclareLaunchArgument("password", default_value="admin"),
        DeclareLaunchArgument("port", default_value="10000"),
        
        # Mode Arguments
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("launch_arm_controller", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        OpaqueFunction(function=launch_setup)
    ])