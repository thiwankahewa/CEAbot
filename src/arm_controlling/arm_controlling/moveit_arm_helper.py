import rclpy
from rclpy.node import Node
import time
import math
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    MoveItErrorCodes,
    BoundingVolume,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
)
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.msg import SetParametersResult


class MoveItArmHelper(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.moveit_cb_group = ReentrantCallbackGroup()

        self.planning_group = "arm"
        self.base_frame = "gemini335_color_optical_frame"
        self.ee_link = "end_effector_link"

        self.current_joint_state = None
        self.active_execute_goal = None
        self.one_turn_limited_joints = ["joint_1", "joint_3", "joint_5", "joint_7"]
        self.max_joint_turn = 2.0 * math.pi

        self.declare_parameter("bench_height", 0.75)
        self.declare_parameter("pot_height", 0.15)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance", 0.01)
        self.declare_parameter("velocity_scaling", 0.3)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("planning_time", 5.0)

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.moveit_cb_group,
        )

        self._load_arm_params()
        self.add_on_set_parameters_callback(self.on_params)

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
            callback_group=self.moveit_cb_group,
        )
        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=self.moveit_cb_group,
        )

    # -------------------------
    # Parameter callbacks
    # -------------------------
    def _load_arm_params(self):
        self.bench_height = self.get_parameter("bench_height").value
        self.pot_height = self.get_parameter("pot_height").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.orientation_tolerance = self.get_parameter("orientation_tolerance").value
        self.velocity_scaling = self.get_parameter("velocity_scaling").value
        self.acceleration_scaling = self.get_parameter("acceleration_scaling").value
        self.planning_time = self.get_parameter("planning_time").value

    def on_params(self, params):
        self._load_arm_params()
        return SetParametersResult(successful=True)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    # -------------------------
    # Safe future wait helper
    # IMPORTANT: do not spin this node from inside helper methods
    # when the node is already running in an executor.
    # -------------------------
    def wait_future(self, future, timeout=30.0, check_stop=False):
        start = time.time()

        while rclpy.ok():
            if future.done():
                return future.result()

            if check_stop and self.check_stop_requested():
                self.get_logger().warn("Future wait interrupted by stop request")
                return None

            if time.time() - start > timeout:
                self.get_logger().error("Future timeout")
                return None

            time.sleep(0.02)

    # -------------------------
    # MoveIt helper functions
    # -------------------------
    def get_current_joint_map(self, timeout=5.0):
        start = time.time()

        while rclpy.ok() and self.current_joint_state is None:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for /joint_states")
                return None
            time.sleep(0.05)

        return dict(zip(self.current_joint_state.name, self.current_joint_state.position))

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

    def make_min_z_path_constraints(self):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [5.0, 5.0, 5.0]

        # With z increasing downward in this setup, place the box so its
        # allowed range ends at the bench height rather than extending below it.
        box_pose = Pose()
        box_pose.position = Point(x=0.0, y=0.0, z=self.bench_height - 2.5)
        box_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        region = BoundingVolume()
        region.primitives.append(box)
        region.primitive_poses.append(box_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.ee_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        return constraints

    def plan_to_joint_positions(self, joint_targets):
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action server not ready")
            return None

        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints.append(self.make_joint_goal(joint_targets))
        #request.path_constraints = self.make_min_z_path_constraints()

        options = PlanningOptions()
        options.plan_only = True
        options.replan = True
        options.replan_attempts = 3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        send_future = self.move_group_client.send_goal_async(goal_msg)
        goal_handle = self.wait_future(send_future, timeout=10.0)

        if goal_handle is None:
            return None

        if not goal_handle.accepted:
            self.get_logger().error("Joint planning goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        result_wrap = self.wait_future(result_future, timeout=self.planning_time + 20.0)

        if result_wrap is None:
            return None

        result = result_wrap.result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Joint planning failed: {result.error_code.val}")
            return None

        return result.planned_trajectory

    def plan_to_target(self, x, y, z, qx, qy, qz, qw):
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action server not ready")
            return None

        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints.append(self.make_pose_goal(x, y, z, qx, qy, qz, qw))
        #request.path_constraints = self.make_min_z_path_constraints()

        options = PlanningOptions()
        options.plan_only = True
        options.look_around = False
        options.replan = True
        options.replan_attempts = 3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        send_future = self.move_group_client.send_goal_async(goal_msg)
        goal_handle = self.wait_future(send_future, timeout=10.0)

        if goal_handle is None:
            return None

        if not goal_handle.accepted:
            self.get_logger().error("MoveIt planning goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        result_wrap = self.wait_future(result_future, timeout=self.planning_time + 20.0)

        if result_wrap is None:
            return None

        result = result_wrap.result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Planning failed. MoveIt error code: {result.error_code.val}")
            return None

        return result.planned_trajectory

    def trajectory_respects_one_turn_limits(self, trajectory):
        points = trajectory.joint_trajectory.points
        joint_names = trajectory.joint_trajectory.joint_names

        if not points:
            return True, ""

        for joint_name in self.one_turn_limited_joints:
            if joint_name not in joint_names:
                continue

            joint_index = joint_names.index(joint_name)
            positions = [
                point.positions[joint_index]
                for point in points
                if len(point.positions) > joint_index
            ]

            if len(positions) < 2:
                continue

            unwrapped = [positions[0]]
            for position in positions[1:]:
                previous = unwrapped[-1]
                delta = math.atan2(math.sin(position - previous), math.cos(position - previous))
                unwrapped.append(previous + delta)

            travel = max(unwrapped) - min(unwrapped)
            if travel > self.max_joint_turn:
                return (
                    False,
                    f"{joint_name} trajectory rotates {math.degrees(travel):.1f} deg "
                    f"(limit {math.degrees(self.max_joint_turn):.1f} deg)",
                )

        return True, ""

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

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/execute_trajectory server not ready")
            return False

        send_future = self.execute_client.send_goal_async(goal_msg)
        goal_handle = self.wait_future(send_future, timeout=10.0)

        if goal_handle is None:
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution rejected")
            return False

        self.active_execute_goal = goal_handle

        try:
            result_future = goal_handle.get_result_async()
            result_wrap = self.wait_future(result_future, timeout=60.0, check_stop=True)

            if result_wrap is None:
                return False

            result = result_wrap.result

            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Execution failed. MoveIt error code: {result.error_code.val}"
                )
                return False

            return True

        finally:
            self.active_execute_goal = None
    
    def cancel_active_execution(self, timeout=0.5):
        if self.active_execute_goal is None:
            return False, "No active trajectory goal"

        cancel_future = self.active_execute_goal.cancel_goal_async()
        cancel_response = self.wait_future(cancel_future, timeout=timeout)

        self.active_execute_goal = None

        if cancel_response is None:
            return False, "Cancel request timeout"

        return True, "Active trajectory cancel requested"

    def wait_until_robot_stops(self, velocity_tolerance=0.005, timeout=10.0):
        start_time = time.time()

        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout waiting for robot to stop")
                return False

            if self.current_joint_state is None:
                time.sleep(0.05)
                continue

            if len(self.current_joint_state.velocity) == 0:
                self.get_logger().warn("No velocity in JointState; assuming stopped")
                return True

            max_vel = max(abs(v) for v in self.current_joint_state.velocity)

            if max_vel < velocity_tolerance:
                self.get_logger().info("Robot arm stopped")
                return True

            time.sleep(0.05)

    def angle_error(self, target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def wait_until_trajectory_finished(self, trajectory, tolerance=0.03, timeout=10.0):
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

            if self.current_joint_state is None:
                time.sleep(0.05)
                continue

            current = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

            all_close = True
            max_error = 0.0
            for joint_name, target_pos in target.items():
                if joint_name not in current:
                    all_close = False
                    break

                error = abs(self.angle_error(target_pos, current[joint_name]))
                max_error = max(max_error, error)
                if error > tolerance:
                    all_close = False

            if all_close:
                self.get_logger().info("Reached final trajectory point")
                return True

            if len(self.current_joint_state.velocity) > 0:
                max_vel = max(abs(v) for v in self.current_joint_state.velocity)
                if max_vel < 0.005 and max_error < max(tolerance * 3.0, 0.05):
                    self.get_logger().info("Robot stopped near final trajectory point")
                    return True

            time.sleep(0.05)
