#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    MoveItErrorCodes,
)


class PlantLocatorNode(Node):
    def __init__(self):
        super().__init__("plant_locator")

        self.declare_parameter("csv_path","/home/thiwa/scan_data_zed/b1_r12_20260505_115253/plant_coordinates_camera_frame.csv",)
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("base_frame", "arm_mount_link")
        self.declare_parameter("ee_link", "end_effector_link")

        self.declare_parameter("position_tolerance", 0.02)   # meters
        self.declare_parameter("orientation_tolerance", 0.20) # radians

        self.declare_parameter("velocity_scaling", 0.5)
        self.declare_parameter("acceleration_scaling", 0.05)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("sleep_between_targets", 1.0)

        # Fixed tool orientation.
        # Change this after checking your end-effector frame in RViz.
        self.declare_parameter("qx", 0.0)
        self.declare_parameter("qy", 0.0)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 1.0)

        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def read_targets_from_csv(self):
        csv_path = self.get_parameter("csv_path").value
        targets = []

        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)

            for i, row in enumerate(reader):
                try:
                    x = float(row["target_x"]) / 1000.0
                    y = float(row["target_y"]) / 1000.0
                    z = float(row["target_z"]) / 1000.0
                except Exception as e:
                    self.get_logger().warn(f"Skipping row {i}: {e}")
                    continue

                if not all(math.isfinite(v) for v in [x, y, z]):
                    self.get_logger().warn(f"Skipping row {i}: non-finite value")
                    continue

                targets.append((x, y, z))

        # Optional: sort by X, largest first
        targets.sort(key=lambda p: p[0], reverse=True)

        return targets

    def make_pose_goal(self, x, y, z):
        base_frame = self.get_parameter("base_frame").value
        ee_link = self.get_parameter("ee_link").value

        qx = self.get_parameter("qx").value
        qy = self.get_parameter("qy").value
        qz = self.get_parameter("qz").value
        qw = self.get_parameter("qw").value

        pos_tol = self.get_parameter("position_tolerance").value
        ori_tol = self.get_parameter("orientation_tolerance").value

        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = base_frame
        position_constraint.link_name = ee_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = base_frame
        orientation_constraint.link_name = ee_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ori_tol
        orientation_constraint.absolute_y_axis_tolerance = ori_tol
        orientation_constraint.absolute_z_axis_tolerance = ori_tol
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def plan_to_target(self, x, y, z):
        planning_group = self.get_parameter("planning_group").value

        request = MotionPlanRequest()
        request.group_name = planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.get_parameter("planning_time").value
        request.max_velocity_scaling_factor = self.get_parameter("velocity_scaling").value
        request.max_acceleration_scaling_factor = self.get_parameter("acceleration_scaling").value

        request.start_state.is_diff = True
        request.goal_constraints.append(self.make_pose_goal(x, y, z))

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

    def run(self):
        targets = self.read_targets_from_csv()

        if len(targets) == 0:
            self.get_logger().error("No valid targets found in CSV")
            return

        self.get_logger().info(f"Loaded {len(targets)} targets")

        for i, (x, y, z) in enumerate(targets, start=1):
            self.get_logger().info(f"Target {i}/{len(targets)}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

            trajectory = self.plan_to_target(x, y, z)

            if trajectory is None:
                self.get_logger().warn(f"Skipping target {i}")
                continue

            success = self.execute_trajectory(trajectory)

            if not success:
                self.get_logger().warn(f"Execution failed for target {i}")
                continue

            time.sleep(self.get_parameter("sleep_between_targets").value)


def main(args=None):
    rclpy.init(args=args)

    node = PlantLocatorNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()