#!/usr/bin/env python3
"""MoveIt-driven ChArUco eye-in-hand calibration from one visible seed pose."""

import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from arm_controlling.moveit_arm_helper import MoveItArmHelper
from bench_robot.handeye_capture import HandEyeCapture, argument_parser, transform_to_dict
from bench_robot.handeye_geometry import (
    board_in_image, board_outline, generate_camera_views, rotation_matrix_to_quaternion,
)
from bench_robot.handeye_solve import pose_dict_to_matrix, solve_session, transform_result


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class CalibrationArm(MoveItArmHelper):
    """Reuse planning, with strict execution and fresh measured settling checks."""

    def __init__(self, args):
        self.joint_record = None
        super().__init__("handeye_auto_arm")
        self.base_frame = args.base_frame
        self.ee_link = args.end_effector_frame
        self.planning_group = args.planning_group
        self.velocity_scaling = args.velocity_scaling
        self.acceleration_scaling = args.acceleration_scaling
        self.planning_time = args.planning_time
        self.position_tolerance = 0.003
        self.orientation_tolerance = 0.01
        self.args = args

    def joint_state_callback(self, msg):
        super().joint_state_callback(msg)
        self.joint_record = (msg, time.monotonic())

    def fresh_joints(self):
        record = self.joint_record
        if record is None:
            return None
        msg, received = record
        age = (self.get_clock().now().nanoseconds - stamp_ns(msg.header.stamp)) / 1e9
        if (time.monotonic() - received > self.args.max_sensor_age
                or not 0 <= age <= self.args.max_sensor_age):
            return None
        if (not msg.name or len(msg.position) != len(msg.name)
                or not np.isfinite(msg.position).all()):
            return None
        if msg.velocity and (not np.isfinite(msg.velocity).all()
                             or max(abs(v) for v in msg.velocity) > 0.005):
            return None
        return dict(zip(msg.name, msg.position))

    @staticmethod
    def same_joints(reference, current, tolerance=0.002):
        return current is not None and all(
            name in current and abs(np.arctan2(np.sin(current[name] - position),
                                               np.cos(current[name] - position))) < tolerance
            for name, position in reference.items()
        )

    def wait_still(self, endpoint=None):
        deadline = time.monotonic() + self.args.motion_timeout
        baseline, stable_since = None, None
        while rclpy.ok() and time.monotonic() < deadline:
            current = self.fresh_joints()
            if current is None or (endpoint and not self.same_joints(endpoint, current, 0.01)):
                baseline, stable_since = None, None
            elif baseline is None or not self.same_joints(baseline, current):
                baseline, stable_since = current, time.monotonic()
            elif time.monotonic() - stable_since >= self.args.settle_time:
                return current
            time.sleep(0.05)
        raise RuntimeError("Arm did not settle at its endpoint with fresh joint feedback")

    def execute_checked(self, trajectory):
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty trajectory")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory is unavailable")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        pending = self.execute_client.send_goal_async(goal)
        handle = None
        completed = False
        try:
            handle = self.wait_future(pending, timeout=10.0)
            if handle is None or not handle.accepted:
                raise RuntimeError("Trajectory goal was rejected or acceptance timed out")
            self.active_execute_goal = handle
            result = self.wait_future(handle.get_result_async(), timeout=self.args.motion_timeout)
            if (result is None or result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code.val != MoveItErrorCodes.SUCCESS):
                raise RuntimeError("Trajectory execution failed or timed out; stopping calibration")
            completed = True
        finally:
            if not completed:
                if handle is not None and handle.accepted:
                    self.wait_future(handle.cancel_goal_async(), timeout=2.0)
                elif handle is None:
                    # An acceptance may arrive after timeout/Ctrl-C. Cancel that
                    # goal as well while the executor is still alive.
                    def cancel_late(future):
                        late = future.result()
                        if late is not None and late.accepted:
                            late.cancel_goal_async()
                    pending.add_done_callback(cancel_late)
                    self.wait_future(pending, timeout=2.0)
            self.active_execute_goal = None
        return self.wait_still(self.trajectory_final_joint_map(trajectory))


class AutoCalibration:
    def __init__(self, capture, arm, args):
        self.capture, self.arm, self.args = capture, arm, args
        self.outline = board_outline(args.squares_x, args.squares_y, args.square_length)
        self.camera_frame = None
        self.camera_model = None
        self.report = {"execute": args.execute, "views": []}

    def save_report(self):
        path = self.capture.output_dir / "auto_run.yaml"
        temporary = path.with_suffix(".yaml.tmp")
        temporary.write_text(yaml.safe_dump(self.report, sort_keys=False), encoding="utf-8")
        temporary.replace(path)

    def visible_detection(self, after_ns):
        """Only accept fresh, correctly modelled, fully framed board detections."""
        node = self.capture
        node.render()
        if node.latest_detection is None or node.latest_camera_info is None:
            return None
        corners, ids, message, frame = node.latest_detection
        timestamp = stamp_ns(message.header.stamp)
        age = (node.get_clock().now().nanoseconds - timestamp) / 1e9
        if timestamp <= after_ns or not 0 <= age <= self.args.max_sensor_age:
            return None
        info = node.latest_camera_info
        if (info.header.frame_id != message.header.frame_id
                or info.width != frame.shape[1] or info.height != frame.shape[0]):
            raise RuntimeError("Image and CameraInfo frames/resolutions do not match")
        if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
            raise RuntimeError(f"Unsupported camera distortion model: {info.distortion_model}")
        model = (info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model)
        if self.camera_model is not None and model != self.camera_model:
            raise RuntimeError("Camera intrinsics changed during calibration")
        self.camera_model = model
        if not message.header.frame_id:
            raise RuntimeError("Image has no optical frame ID")
        if self.camera_frame is not None and message.header.frame_id != self.camera_frame:
            raise RuntimeError("Image optical frame changed during calibration")
        self.camera_frame = message.header.frame_id
        if ids is None or len(ids) < self.args.minimum_corners:
            return None
        objects = node.board.getChessboardCorners()[ids.flatten()]
        if np.linalg.matrix_rank(objects[:, :2] - objects[:, :2].mean(axis=0)) < 2:
            return None
        matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=float)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise RuntimeError("CameraInfo has invalid intrinsics")
        try:
            success, rvec, tvec = cv2.solvePnP(objects, corners.reshape(-1, 2),
                                             matrix, distortion)
            if not success:
                return None
            board_pose = np.eye(4)
            board_pose[:3, :3] = cv2.Rodrigues(rvec)[0]
            board_pose[:3, 3] = tvec.ravel()
            if not board_in_image(board_pose, self.outline, matrix, distortion,
                                  info.width, info.height, self.args.image_margin):
                return None
            projected = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)[0]
            rmse = np.sqrt(np.mean((projected.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2))
            if not np.isfinite(rmse) or rmse > self.args.max_reprojection_rmse:
                return None
        except cv2.error:
            return None
        return timestamp

    def capture_at_rest(self, joints):
        # Use camera timestamps newer than settling, never a cached moving frame.
        barrier = self.capture.get_clock().now().nanoseconds
        deadline = time.monotonic() + self.args.capture_timeout
        last_stamp, count = barrier, 0
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.arm.same_joints(joints, self.arm.fresh_joints()):
                raise RuntimeError("Arm moved or joint feedback became stale during capture")
            timestamp = self.visible_detection(barrier)
            if timestamp is None:
                count = 0
            elif timestamp > last_stamp:
                count += 1
                last_stamp = timestamp
                if count >= self.args.stable_frames:
                    result = self.capture.capture(
                        require_timestamped_tf=True,
                        max_reprojection_rmse=self.args.max_reprojection_rmse,
                    )
                    if result is not None:
                        return result
            time.sleep(1.0 / self.args.display_rate)
        return None

    def run(self):
        node, args = self.capture, self.args
        node.get_logger().info("Waiting for a stationary arm and a fully visible ChArUco board")
        seed = self.capture_at_rest(self.arm.wait_still())
        if seed is None:
            raise RuntimeError("No valid seed view. Place the fixed board fully in view and retry")
        mount_frame = args.camera_mount_frame or self.camera_frame
        try:
            mount_tf = node.tf_buffer.lookup_transform(
                args.end_effector_frame, mount_frame, Time(), timeout=Duration(seconds=3.0))
        except Exception as exc:
            raise RuntimeError(f"No approximate optical mount TF for {mount_frame}. "
                               "Set --camera-mount-frame to the matching URDF optical frame") from exc
        mount = pose_dict_to_matrix(transform_to_dict(mount_tf))
        seed_ee = pose_dict_to_matrix(seed["base_to_end_effector"])
        seed_camera = seed_ee @ mount
        base_board = seed_camera @ pose_dict_to_matrix(seed["camera_to_board"])
        center = base_board @ np.array([args.squares_x * args.square_length / 2,
                                       args.squares_y * args.square_length / 2, 0., 1.])
        info = node.latest_camera_info
        matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=float)
        self.report.update({"camera_frame": self.camera_frame, "camera_mount_frame": mount_frame,
                            "base_frame": args.base_frame,
                            "end_effector_frame": args.end_effector_frame,
                            "approximate_end_effector_from_camera": transform_result(mount),
                            "seed_end_effector_pose": transform_result(seed_ee)})
        for index, camera in enumerate(generate_camera_views(
                seed_camera, center[:3], args.candidates, args.tilt_deg,
                args.roll_deg, args.distance_variation), start=1):
            target = camera @ np.linalg.inv(mount)
            record = {"candidate": index, "target": transform_result(target)}
            self.report["views"].append(record)
            if np.linalg.norm(target[:3, 3] - seed_ee[:3, 3]) > args.max_displacement:
                record["status"] = "outside_seed_displacement_limit"
            elif not board_in_image(np.linalg.inv(camera) @ base_board, self.outline,
                                    matrix, distortion, info.width, info.height, args.image_margin):
                record["status"] = "predicted_board_out_of_view"
            elif not args.execute:
                record["status"] = "preview_only_not_motion_planned"
            else:
                self.arm.wait_still()
                node.get_logger().info(f"Planning candidate {index}/{args.candidates}")
                quaternion = rotation_matrix_to_quaternion(target[:3, :3])
                trajectory = self.arm.plan_to_target(
                    *[float(v) for v in target[:3, 3]], *[float(v) for v in quaternion])
                if trajectory is None:
                    record["status"] = "planning_failed"
                else:
                    record["status"] = "executing"
                    self.save_report()
                    joints = self.arm.execute_checked(trajectory)
                    observation = self.capture_at_rest(joints)
                    record["status"] = "captured" if observation else "visibility_or_capture_failed"
                    if observation:
                        record["observation_index"] = observation["index"]
            node.get_logger().info(f"Candidate {index}: {record['status']}")
            self.save_report()
            if args.execute and len(node.observations) >= args.samples:
                break
        if not args.execute:
            node.get_logger().info(f"Preview saved to {node.output_dir / 'auto_run.yaml'}. "
                                   "Use --execute to move, capture, and solve.")
            return 0
        if len(node.observations) < args.samples:
            node.get_logger().warn(f"Captured {len(node.observations)}/{args.samples} requested views")
        result = solve_session(node.output_dir, args.max_translation_rms_mm,
                               args.max_rotation_rms_deg, args.max_tf_age)
        self.report["solution_accepted"] = result["accepted"]
        self.save_report()
        return 0 if result["accepted"] else 2


def parse_arguments(argv=None):
    parser = argument_parser()
    parser.description = __doc__
    parser.set_defaults(max_tf_age=0.1)
    for action in parser._actions:
        if action.dest == "max_tf_age":
            action.help = "Maximum saved observation TF age accepted by the solver (seconds)"
    parser.add_argument("--execute", action="store_true", help="Enable real arm motion and solving")
    parser.add_argument("--camera-mount-frame", help="Approximate URDF optical frame; defaults to image frame")
    parser.add_argument("--planning-group", default="arm")
    parser.add_argument("--samples", type=int, default=20, help="Requested total observations including seed")
    parser.add_argument("--candidates", type=int, default=32)
    parser.add_argument("--tilt-deg", type=float, default=15.)
    parser.add_argument("--roll-deg", type=float, default=12.)
    parser.add_argument("--distance-variation", type=float, default=0.08)
    parser.add_argument("--max-displacement", type=float, default=0.20,
                        help="Maximum end-effector target distance from seed, metres")
    parser.add_argument("--image-margin", type=float, default=30., help="Board perimeter margin, pixels")
    parser.add_argument("--velocity-scaling", type=float, default=0.10)
    parser.add_argument("--acceleration-scaling", type=float, default=0.10)
    parser.add_argument("--planning-time", type=float, default=5.)
    parser.add_argument("--settle-time", type=float, default=1.)
    parser.add_argument("--capture-timeout", type=float, default=10.)
    parser.add_argument("--motion-timeout", type=float, default=90.)
    parser.add_argument("--max-sensor-age", type=float, default=0.5)
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--max-reprojection-rmse", type=float, default=1.)
    parser.add_argument("--max-translation-rms-mm", type=float, default=5.)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=1.)
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"Unknown arguments: {' '.join(ros_args)}")
    for key in ("max_displacement", "image_margin", "planning_time", "settle_time",
                "capture_timeout", "motion_timeout", "max_sensor_age", "display_rate",
                "max_reprojection_rmse", "max_translation_rms_mm", "max_rotation_rms_deg",
                "max_tf_age", "square_length", "marker_length"):
        value = getattr(args, key)
        if not np.isfinite(value) or value <= 0:
            parser.error(f"--{key.replace('_', '-')} must be finite and positive")
    for key, low, high in (("velocity_scaling", 0., 1.), ("acceleration_scaling", 0., 1.),
                           ("tilt_deg", 0., 35.), ("roll_deg", 0., 45.)):
        if not low < getattr(args, key) <= high:
            parser.error(f"--{key.replace('_', '-')} must be in ({low}, {high}]")
    if not 0 <= args.distance_variation <= 0.25:
        parser.error("--distance-variation must be in [0, 0.25]")
    if args.samples < 10 or args.candidates < args.samples - 1 or args.stable_frames < 2:
        parser.error("Need >=10 samples, candidates >= samples - 1, and >=2 stable frames")
    if (args.squares_x < 3 or args.squares_y < 3 or args.marker_length >= args.square_length
            or not 8 <= args.minimum_corners <= (args.squares_x - 1) * (args.squares_y - 1)):
        parser.error("Invalid board dimensions, marker size, or minimum corners (>=8 required)")
    return args, ros_args


def main():
    args, ros_args = parse_arguments()
    # Keep the executor alive during Ctrl-C cleanup so cancellation can be sent.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    capture = arm = executor = thread = routine = None
    status = 1
    try:
        capture = HandEyeCapture(args)
        arm = CalibrationArm(args)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(capture)
        executor.add_node(arm)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        routine = AutoCalibration(capture, arm, args)
        status = routine.run()
    except (KeyboardInterrupt, RuntimeError, ValueError, OSError, cv2.error) as exc:
        message = "Interrupted by operator" if isinstance(exc, KeyboardInterrupt) else str(exc)
        if capture is not None:
            capture.get_logger().error(message)
        if routine is not None:
            routine.report["error"] = message
            routine.save_report()
        status = 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if arm is not None and arm.active_execute_goal is not None:
            arm.cancel_active_execution(timeout=2.0)
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=2.0)
        if arm is not None:
            arm.destroy_node()
        if capture is not None:
            capture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
