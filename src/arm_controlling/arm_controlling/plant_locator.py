#!/usr/bin/env python3

import csv
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import (MotionPlanRequest,Constraints,PositionConstraint,OrientationConstraint,BoundingVolume,PlanningOptions,MoveItErrorCodes,)


class PlantLocatorNode(Node):
    def __init__(self):
        super().__init__("plant_locator")

        #--------- States and variables ---------#
        self.csv_path = "/home/thiwa/scan_data_zed/b1_r12_20260504_170451/plant_coordinates_camera_frame.csv"
        self.planning_group = "arm"
        self.base_frame = "zed2i_left_camera_frame_optical"
        self.ee_link = "end_effector_link"

        self.declare_parameter("bench_height", 0.75)
        self.declare_parameter("pot_height", 0.15)
        self.declare_parameter("position_tolerance", 0.02)   # meters
        self.declare_parameter("orientation_tolerance", 0.20) # radians
        self.declare_parameter("z_offset", 0.2)              # meters above target
        self.declare_parameter("circle_radius_offset", 0.20)   # distance from top view to side-view circle
        self.declare_parameter("circle_height_offset", 0.1)
        self.declare_parameter("look_at_angle_offset", 0.2)
        self.declare_parameter("view_count", 3)              

        self.declare_parameter("velocity_scaling", 0.3)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("planning_time", 5.0)

        # Fixed tool orientation.
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.current_joint_state = None
        self.max_plant_z = None

        self._load_params()
        self.add_on_set_parameters_callback(self.on_params)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joint_state_sub = self.create_subscription(JointState,"/joint_states",self.joint_state_callback,10)

        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    #-------- Callbacks functions ---------#
    def _load_params(self):
        self.bench_height = self.get_parameter("bench_height").value
        self.pot_height = self.get_parameter("pot_height").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.orientation_tolerance = self.get_parameter("orientation_tolerance").value
        self.z_offset = self.get_parameter("z_offset").value
        self.circle_radius_offset = self.get_parameter("circle_radius_offset").value
        self.circle_height_offset = self.get_parameter("circle_height_offset").value
        self.look_at_angle_offset = self.get_parameter("look_at_angle_offset").value
        self.view_count = self.get_parameter("view_count").value
        self.velocity_scaling = self.get_parameter("velocity_scaling").value
        self.acceleration_scaling = self.get_parameter("acceleration_scaling").value
        self.planning_time = self.get_parameter("planning_time").value

    def on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)
    
    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    #-------- csv reading functions ---------#
    def read_targets_from_csv(self):
        targets = []

        with open(self.csv_path, "r") as f:
            reader = csv.DictReader(f)

            for i, row in enumerate(reader):
                try:
                    x = float(row["target_x"]) / 1000.0
                    y = float(row["target_y"]) / 1000.0
                    z = float(row["target_z"]) / 1000.0
                    r = float(row["radius_mm"]) / 1000.0
                except Exception as e:
                    self.get_logger().warn(f"Skipping row {i}: {e}")
                    continue

                if not all(math.isfinite(v) for v in [x, y, z]):
                    self.get_logger().warn(f"Skipping row {i}: non-finite value")
                    continue

                targets.append((x, y, z, r))

        targets.sort(key=lambda p: p[0], reverse=True)      # sort by X, largest first
        return targets
    
    #------- helper math functions ---------#
    
    def normalize_vector(self, v):
        norm = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if norm < 1e-9:
            return [0.0, 0.0, 1.0]
        return [v[0] / norm, v[1] / norm, v[2] / norm]

    def cross(self, a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    def dot(self, a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    def rotation_matrix_to_quaternion(self, m):
        # m is 3x3 rotation matrix
        trace = m[0][0] + m[1][1] + m[2][2]

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m[2][1] - m[1][2]) / s
            qy = (m[0][2] - m[2][0]) / s
            qz = (m[1][0] - m[0][1]) / s
        elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            qw = (m[2][1] - m[1][2]) / s
            qx = 0.25 * s
            qy = (m[0][1] + m[1][0]) / s
            qz = (m[0][2] + m[2][0]) / s
        elif m[1][1] > m[2][2]:
            s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            qw = (m[0][2] - m[2][0]) / s
            qx = (m[0][1] + m[1][0]) / s
            qy = 0.25 * s
            qz = (m[1][2] + m[2][1]) / s
        else:
            s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
            qw = (m[1][0] - m[0][1]) / s
            qx = (m[0][2] + m[2][0]) / s
            qy = (m[1][2] + m[2][1]) / s
            qz = 0.25 * s

        return qx, qy, qz, qw
    
    def look_at_quaternion(self, camera_pos, target_pos):
        z_axis = [target_pos[0] - camera_pos[0],target_pos[1] - camera_pos[1],target_pos[2] - camera_pos[2],]
        z_axis = self.normalize_vector(z_axis)

        # Reference up/down direction in your base frame.
        # Try this first. If camera rolls strangely, change this.
        y_ref = [0.0, 1.0, 0.0]

        if abs(self.dot(z_axis, y_ref)) > 0.95:
            y_ref = [1.0, 0.0, 0.0]

        # Optical frame:
        # Z = viewing direction
        # X = right
        # Y = down
        x_axis = self.cross(y_ref, z_axis)
        x_axis = self.normalize_vector(x_axis)
        y_axis = self.cross(z_axis, x_axis)
        y_axis = self.normalize_vector(y_axis)

        # Rotation matrix columns are local X, Y, Z axes
        m = [
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ]

        return self.rotation_matrix_to_quaternion(m)
    
    #-------- helper functions -------------#

    def make_joint_goal(self, joint_targets):
        constraints = Constraints()

        for joint_name, target in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        return constraints
    

    #------- main motion functions ---------#
    def generate_view_poses(self, plant_x, plant_y, plant_z, radius):
        poses = []

        # ---------- Top view ----------
        top_x = plant_x
        top_y = plant_y
        top_z = plant_z - self.z_offset

        poses.append({"label": "top","x": top_x,"y": top_y,"z": top_z,"qx": self.qx,"qy": self.qy,"qz": self.qz,"qw": self.qw,})

        # ---------- Views around top-view sphere/circle ----------
        if self.view_count <= 0:
            return poses

        angle_step = 360.0 / self.view_count

        for k in range(self.view_count):
            angle_deg = k * angle_step
            theta = math.radians(angle_deg)

            # Circle around top view in camera optical X-Y plane with Z offset. 
            view_x = top_x + (self.circle_radius_offset + radius) * math.cos(theta)
            view_y = top_y + (self.circle_radius_offset + radius) * math.sin(theta)
            view_z = top_z + self.circle_height_offset

            qx, qy, qz, qw = self.look_at_quaternion(camera_pos=(view_x, view_y, view_z - self.look_at_angle_offset),target_pos=(plant_x, plant_y, plant_z))
            poses.append({"label": f"view_{k+1}_{angle_deg:.0f}deg","x": view_x,"y": view_y,"z": view_z,"qx": qx,"qy": qy,"qz": qz,"qw": qw,})

        return poses
    
    def plan_to_target(self, x, y, z, qx, qy, qz, qw):
        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True      #Start from current robot joint positions
        request.goal_constraints.append(self.make_pose_goal(x, y, z, qx, qy, qz, qw))

        #safe_constraint = self.make_safe_z_path_constraint()
        #if safe_constraint is not None:
            #request.path_constraints = safe_constraint

        options = PlanningOptions()
        options.plan_only = True
        options.look_around = False
        options.replan = True
        options.replan_attempts = 3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        self.move_group_client.wait_for_server()

        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt planning goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Planning failed. MoveIt error code: {result.error_code.val}")
            return None

        return result.planned_trajectory
    
    def make_pose_goal(self, x, y, z, qx, qy, qz, qw):
        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.ee_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.ee_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def plan_to_joint_positions(self, joint_targets):

        request = MotionPlanRequest()

        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.05

        request.start_state.is_diff = True

        request.goal_constraints.append(self.make_joint_goal(joint_targets))

        options = PlanningOptions()
        options.plan_only = True
        options.replan = True
        options.replan_attempts = 3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        self.move_group_client.wait_for_server()

        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Joint planning goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Joint planning failed: {result.error_code.val}"
            )
            return None

        return result.planned_trajectory

    def get_current_joint_map(self):
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        current = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        self.get_logger().info(f"Current joints: {current}")
        return current

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.execute_client.wait_for_server()

        send_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Execution failed. MoveIt error code: {result.error_code.val}")
            return False

        return True
    
    def wait_until_trajectory_finished(self, trajectory, tolerance=0.01, timeout=10.0):
        if not trajectory.joint_trajectory.points:
            return False

        joint_names = trajectory.joint_trajectory.joint_names
        final_positions = trajectory.joint_trajectory.points[-1].positions

        target = dict(zip(joint_names, final_positions))

        start_time = time.time()

        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout waiting for robot to reach final trajectory point")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.current_joint_state is None:
                continue

            current = dict(zip(
                self.current_joint_state.name,
                self.current_joint_state.position
            ))

            max_error = 0.0
            all_close = True

            for joint_name, target_pos in target.items():
                if joint_name not in current:
                    all_close = False
                    break

                error = abs(current[joint_name] - target_pos)
                max_error = max(max_error, error)

                if error > tolerance:
                    all_close = False
                    break

            if all_close:
                return True
            
    def wait_until_robot_stops(self, velocity_tolerance=0.005, timeout=10.0):
        start_time = time.time()

        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout waiting for robot to stop")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.current_joint_state is None or len(self.current_joint_state.velocity) == 0:
                continue

            max_vel = max(abs(v) for v in self.current_joint_state.velocity)

            if max_vel < velocity_tolerance:
                return True

    #------- main execution loop ---------#
    def run(self):
        targets = self.read_targets_from_csv()
        self.max_plant_z = max(p[2] for p in targets)
        self.get_logger().info(f"Max plant Z: {self.max_plant_z:.4f} m")

        if len(targets) == 0:
            self.get_logger().error("No valid targets found in CSV")
            return

        for i, (x, y, z, r) in enumerate(targets, start=1):
            view_poses = self.generate_view_poses(x, y, z, r)

            for j, pose in enumerate(view_poses, start=1):
                self.get_logger().info(
                    f"Plant {i} - planned pose {j}/{len(view_poses)} [{pose['label']}]: "
                    f"x={pose['x']:.4f}, y={pose['y']:.4f}, z={pose['z']:.4f}, "
                    f"q=({pose['qx']:.3f}, {pose['qy']:.3f}, {pose['qz']:.3f}, {pose['qw']:.3f})"
                )
                self.wait_until_robot_stops(timeout=10.0)

                trajectory = self.plan_to_target(pose["x"], pose["y"], pose["z"],pose["qx"], pose["qy"], pose["qz"], pose["qw"])

                if trajectory is None:
                    self.get_logger().warn(f"Skipping plant {i}, pose {pose['label']}")
                    continue

                success = self.execute_trajectory(trajectory)

                if not success:
                    self.get_logger().warn(f"Execution failed for plant {i}, pose {pose['label']}")
                    self.get_logger().info("Waiting for controller to settle after failed execution...")
                    self.wait_until_robot_stops(timeout=10.0)
                    continue

                finished = self.wait_until_trajectory_finished(trajectory,tolerance=0.008,timeout=15.0)
                stopped = self.wait_until_robot_stops(timeout=10.0)

                if finished and stopped:
                    try:
                        transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link,Time())
                        t = transform.transform.translation
                        r = transform.transform.rotation
                        self.get_logger().info(f"Plant {i} - actual pose {j}/{len(view_poses)} [{pose['label']}]: "f"x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}, "f"q=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})")
                    except Exception as e:
                        self.get_logger().warn(f"TF lookup failed: {e}")
                else:
                    self.get_logger().warn("Robot may not be fully settled")

    def move_arm_to_safe_rest(self):
        self.get_logger().info("Moving arm to REST pose...")

        rest_joints = {
            "joint_1": 2.5514,
            "joint_2": -2.04,
            "joint_3": 0.0521,
            "joint_4": 1.6613,
            "joint_5": 3.1415,
            "joint_6": -2.09,
            "joint_7": -0.0868,
        }

        traj = self.plan_to_joint_positions(rest_joints)

        if traj is None:
            self.get_logger().error("Failed to plan REST pose")
            return False

        if not self.execute_trajectory(traj):
            self.get_logger().error("Failed to execute REST pose")
            return False

        self.wait_until_trajectory_finished(traj)
        self.wait_until_robot_stops()

        self.get_logger().info("Aligning arm with holder...")

        current_joints = self.get_current_joint_map()

        holder_angle_rad = math.radians(180.0)

        joint_targets = {
            "joint_1": holder_angle_rad,
            "joint_2": current_joints["joint_2"],
            "joint_3": current_joints["joint_3"],
            "joint_4": current_joints["joint_4"],
            "joint_5": current_joints["joint_5"],
            "joint_6": current_joints["joint_6"],
            "joint_7": current_joints["joint_7"],
        }

        traj = self.plan_to_joint_positions(joint_targets)

        if traj is None:
            self.get_logger().error("Failed holder alignment plan")
            return False

        success = self.execute_trajectory(traj)

        if not success:
            self.get_logger().error("Failed holder alignment execute")
            return False

        self.wait_until_trajectory_finished(traj)
        self.wait_until_robot_stops()

        self.get_logger().info("Arm parked safely")

        return True

def main(args=None):
    rclpy.init(args=args)
    node = PlantLocatorNode()
    node.move_arm_to_safe_rest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
