#!/usr/bin/env python3

import threading
import math
import os
import time
from datetime import datetime, timezone

import yaml
import rclpy
import tf2_ros
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from arm_controlling.moveit_arm_helper import MoveItArmHelper
from std_msgs.msg import Bool, String
from arm_interfaces.srv import ExecutePlannedTrajectory, MoveToPose, PlanToPose
from std_srvs.srv import Trigger
from arm_interfaces.msg import PlantTargetArray
from arm_interfaces.srv import CaptureView


REST_SETTLED_JOINTS = {"joint_1": -0.628,"joint_2": -2.23,"joint_3": 0.0521,"joint_4": 1.6613,"joint_5": 3.1415,"joint_6": -2.09,"joint_7": -0.0868,}

class PlantViewScanner(MoveItArmHelper):
    def __init__(self):
        super().__init__("plant_view_scanner")

        #--------- States and variables ---------#
        self.latest_targets = []
        self.latest_run_dir = None
        self.base_frame = "gemini335_color_optical_frame"
        self.ee_link = "camera_color_optical_frame"

        self.declare_parameter("z_offset", 0.2)              # meters above target
        self.declare_parameter("circle_radius_offset", 0.05)   # distance from top view to side-view circle
        self.declare_parameter("circle_height_offset", 0.1)
        self.declare_parameter("look_at_angle_offset", 0.2)
        self.declare_parameter("view_count", 5)
        self.declare_parameter("optimize_view_order", True)
        # Limit greedy optimization to adjacent groups of plants.  A value of
        # 2 optimizes plants 1-2, then plants 3-4, while carrying the predicted
        # final joint state from one group into the next.  Use 0 to optimize
        # every plant in one global batch (the previous behavior).
        self.declare_parameter("optimizer_plant_batch_size", 4)

        # Fixed tool orientation.
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.scan_busy = False
        self.stop_scan = False
        self.scan_paused = False
        self.scan_stopped = False
        self.scan_lock = threading.Lock()

        self._load_scanner_params()
        self.add_on_set_parameters_callback(self.on_params_1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #-------- Services and clients ---------#

        self.move_pose_client = self.create_client(MoveToPose, "/arm/move_to_pose")
        self.plan_pose_client = self.create_client(PlanToPose, "/arm/plan_to_pose")
        self.execute_planned_client = self.create_client(ExecutePlannedTrajectory,"/arm/execute_planned_trajectory")
        self.go_rest_client = self.create_client(Trigger, "/arm/go_rest")
        self.orbbec_capture_client = self.create_client(CaptureView,"/orbbec_test_scan/capture_view")

        self.create_service(Trigger, "/plant_view_scanner/pause", self.cb_pause_scan)
        self.create_service(Trigger, "/plant_view_scanner/resume", self.cb_resume_scan)
        self.create_service(Trigger, "/plant_view_scanner/stop", self.cb_stop_scan)

        #-------- Subscriptions and publishers ---------#
        self.state_sub = self.create_subscription(String,"/auto_state",self.cb_auto_state,10)
        self.target_sub = self.create_subscription(PlantTargetArray,"/plant_row/targets",self.cb_targets,10)
        self.pub_auto_state_cmd = self.create_publisher(String, "/auto_state_cmd", 10)
        self.pub_top_scan_done = self.create_publisher(Bool, "/top_scan/scan_done", 10)

    #-------- Callbacks functions ---------#
    def _load_scanner_params(self):
        self.z_offset = self.get_parameter("z_offset").value
        self.circle_radius_offset = self.get_parameter("circle_radius_offset").value
        self.circle_height_offset = self.get_parameter("circle_height_offset").value
        self.look_at_angle_offset = self.get_parameter("look_at_angle_offset").value
        self.view_count = self.get_parameter("view_count").value
        self.optimize_view_order = self.get_parameter("optimize_view_order").value
        self.optimizer_plant_batch_size = self.get_parameter("optimizer_plant_batch_size").value

    def on_params_1(self, params):
        scanner_params = {
            "z_offset",
            "circle_radius_offset",
            "circle_height_offset",
            "look_at_angle_offset",
            "view_count",
            "optimize_view_order",
            "optimizer_plant_batch_size",
        }
        for param in params:
            if param.name in scanner_params:
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)
    
    def cb_targets(self, msg):
        self.latest_run_dir = msg.run_dir
        self.latest_targets = []

        for t in msg.targets:
            self.latest_targets.append((int(t.plant_id),float(t.target_x),float(t.target_y),float(t.target_z),float(t.radius_m)))

        self.latest_targets.sort(key=lambda p: p[1], reverse=True)
        self.get_logger().info(f"Received {len(self.latest_targets)} plant targets ")

    def cb_auto_state(self, msg):
        state = msg.data.strip().lower()

        if state != "individual_plant_scan":
            return

        if self.scan_busy:
            self.get_logger().warn("Scanner already running")
            return
        
        if len(self.latest_targets) == 0:
            self.get_logger().warn("No targets received yet")
            return

        self.get_logger().info(f"Starting plant scan with {len(self.latest_targets)} targets")

        self.scan_busy = True
        self.stop_scan = False
        threading.Thread(target=self.run_scan_thread, daemon=True).start()

    def run_scan_thread(self):
        self.scan_results = {}
        self.scan_processing_complete = False
        try:
            self.run()
        except Exception as exc:
            self.get_logger().error(f"Plant scan crashed: {exc}")
        finally:
            self.save_scan_summary()
            self.scan_busy = False

    def cb_pause_scan(self, request, response):
        with self.scan_lock:
            self.scan_paused = True

        response.success = True
        response.message = "Plant scanner paused"
        return response


    def cb_resume_scan(self, request, response):
        with self.scan_lock:
            self.scan_paused = False
            self.scan_stopped = False

        response.success = True
        response.message = "Plant scanner resumed"
        return response


    def cb_stop_scan(self, request, response):
        with self.scan_lock:
            self.scan_stopped = True
            self.scan_paused = False

        response.success = True
        response.message = "Plant scanner stopped"
        return response
    
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
    
    #------- main motion functions ---------#
    def wait_if_paused_or_stopped(self):
        while rclpy.ok():
            with self.scan_lock:
                if self.scan_stopped:
                    return False
                paused = self.scan_paused

            if not paused:
                return True

            self.get_logger().info("Scanner paused. Waiting to resume...")
            time.sleep(0.5)

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
    
    def call_arm_move_to_pose(self, pose, timeout=60.0):
        if not self.move_pose_client.wait_for_service(timeout_sec=5.0):
            return False, "/arm/move_to_pose service not available"

        req = MoveToPose.Request()
        req.x = pose["x"]
        req.y = pose["y"]
        req.z = pose["z"]
        req.qx = pose["qx"]
        req.qy = pose["qy"]
        req.qz = pose["qz"]
        req.qw = pose["qw"]
        req.label = pose["label"]

        future = self.move_pose_client.call_async(req)

        start = time.time()
        while rclpy.ok():
            if future.done():
                res = future.result()
                return res.success, res.message

            if time.time() - start > timeout:
                return False, "Timeout waiting for arm_manager"

            time.sleep(0.05)

    def call_arm_plan_to_pose(self, pose, start_joint_map, timeout=60.0):
        if not self.plan_pose_client.wait_for_service(timeout_sec=5.0):
            return False, "/arm/plan_to_pose service not available", None, None, None

        req = PlanToPose.Request()
        req.x = pose["x"]
        req.y = pose["y"]
        req.z = pose["z"]
        req.qx = pose["qx"]
        req.qy = pose["qy"]
        req.qz = pose["qz"]
        req.qw = pose["qw"]
        req.label = pose["label"]
        req.start_joint_names = list(start_joint_map.keys())
        req.start_joint_positions = [float(start_joint_map[name]) for name in req.start_joint_names]

        future = self.plan_pose_client.call_async(req)

        start = time.time()
        while rclpy.ok():
            if future.done():
                res = future.result()
                if res is None:
                    return False, "No response from arm_manager", None, None, None

                final_joint_map = dict(zip(res.final_joint_names, res.final_joint_positions))
                return res.success, res.message, res.cost, final_joint_map, res.planned_trajectory

            if time.time() - start > timeout:
                return False, "Timeout waiting for arm plan", None, None, None

            time.sleep(0.05)

    def call_arm_execute_planned(self, trajectory, label, timeout=90.0):
        if not self.execute_planned_client.wait_for_service(timeout_sec=5.0):
            return False, "/arm/execute_planned_trajectory service not available"

        req = ExecutePlannedTrajectory.Request()
        req.trajectory = trajectory
        req.label = label
        future = self.execute_planned_client.call_async(req)

        start = time.time()
        while rclpy.ok():
            if future.done():
                res = future.result()
                if res is None:
                    return False, "No response from arm_manager"
                return res.success, res.message

            if time.time() - start > timeout:
                return False, "Timeout waiting for cached trajectory execution"

            time.sleep(0.05)

    
    #------- Plant capture function ---------#

    def call_arm_go_rest(self, timeout=30.0):
        if not self.go_rest_client.wait_for_service(timeout_sec=5.0):
            return False, "/arm/go_rest service not available"

        req = Trigger.Request()
        future = self.go_rest_client.call_async(req)

        start = time.time()
        while rclpy.ok():
            if future.done():
                res = future.result()
                if res is None:
                    return False, "No response from arm_manager"
                return res.success, res.message

            if time.time() - start > timeout:
                return False, "Timeout waiting for arm to go rest"

            time.sleep(0.05)

    def wait_until_arm_rest(self, timeout=90.0, joint_tolerance=0.08):
        start = time.time()

        while rclpy.ok():
            current = self.get_current_joint_map(timeout=1.0)

            if current is not None:
                joint_errors = [
                    abs(self.angle_error(target, current[name]))
                    for name, target in REST_SETTLED_JOINTS.items()
                    if name in current
                ]

                if len(joint_errors) == len(REST_SETTLED_JOINTS) and max(joint_errors) <= joint_tolerance:
                    self.get_logger().info("Arm reached rest joint target. Waiting for it to settle.")
                    return self.wait_until_robot_stops(timeout=10.0)

            if time.time() - start > timeout:
                self.get_logger().warn("Timeout waiting for arm to reach rest before finishing scan")
                return False

            time.sleep(0.2)

    def call_orbbec_capture(self, run_dir, plant_id, view_label, timeout=20.0):
        if not self.orbbec_capture_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/orbbec_test_scan/capture_view service not available")
            return False

        req = CaptureView.Request()
        req.run_dir = run_dir
        req.plant_id = int(plant_id)
        req.view_label = str(view_label)

        future = self.orbbec_capture_client.call_async(req)

        start = time.time()
        while rclpy.ok():
            if future.done():
                res = future.result()
                if res is None:
                    return False

                self.get_logger().info(res.message)
                return res.success

            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for Orbbec capture")
                return False

            time.sleep(0.05)

    def wait_for_capture_meta(self, run_dir, plant_id, view_label, started_at, timeout=30.0):
        meta_path = os.path.join(run_dir, f"plant_{int(plant_id):02d}", str(view_label), "meta.txt")
        start = time.time()

        while rclpy.ok():
            if os.path.exists(meta_path) and os.path.getmtime(meta_path) >= started_at:
                return meta_path

            if time.time() - start > timeout:
                self.get_logger().warn(f"Timed out waiting for Orbbec metadata: {meta_path}")
                return None

            time.sleep(0.1)

    def append_actual_pose_to_meta(self, meta_path, plant_id, view_label, transform):
        t = transform.transform.translation
        r = transform.transform.rotation

        with open(meta_path, "a") as f:
            f.write(f"actual_pose_frame: {self.base_frame}\n")
            f.write(f"actual_pose_child_frame: {self.ee_link}\n")
            f.write(f"plant_id: {int(plant_id)}\n")
            f.write(f"view_label: {view_label}\n")
            f.write(f"actual_x: {t.x:.6f}\n")
            f.write(f"actual_y: {t.y:.6f}\n")
            f.write(f"actual_z: {t.z:.6f}\n")
            f.write(f"actual_qx: {r.x:.6f}\n")
            f.write(f"actual_qy: {r.y:.6f}\n")
            f.write(f"actual_qz: {r.z:.6f}\n")
            f.write(f"actual_qw: {r.w:.6f}\n")

    def build_scan_items(self, targets):
        items = []

        for plant_id, x, y, z, r in targets:
            view_poses = self.generate_view_poses(x, y, z, r)

            for j, pose in enumerate(view_poses, start=1):
                items.append(
                    {
                        "plant_id": plant_id,
                        "pose": pose,
                        "pose_index": j,
                        "pose_count": len(view_poses),
                    }
                )

        return items

    def initialize_scan_summary(self, items):
        self.scan_processing_complete = False
        self.scan_results = {
            (item["plant_id"], item["pose"]["label"]): {
                "plant_id": int(item["plant_id"]),
                "view": item["pose"]["label"],
                "motion_status": "pending",
                "capture_status": "not_attempted",
            }
            for item in items
        }

    def set_scan_result(self, item, motion_status=None, capture_status=None):
        key = (item["plant_id"], item["pose"]["label"])
        result = self.scan_results.get(key)
        if result is None:
            return
        if motion_status is not None:
            result["motion_status"] = motion_status
        if capture_status is not None:
            result["capture_status"] = capture_status

    def save_scan_summary(self):
        if not getattr(self, "scan_results", None) or not self.latest_run_dir:
            return

        metadata_path = os.path.join(self.latest_run_dir, "metadata.yaml")
        try:
            if os.path.exists(metadata_path):
                with open(metadata_path, "r", encoding="utf-8") as metadata_file:
                    metadata = yaml.safe_load(metadata_file) or {}
            else:
                metadata = {}

            results = list(self.scan_results.values())
            reached = sum(r["motion_status"] == "reached" for r in results)
            planning_failed = sum(
                r["motion_status"] == "planning_failed" for r in results
            )
            execution_failed = sum(
                r["motion_status"] == "execution_failed" for r in results
            )
            capture_succeeded = sum(
                r["capture_status"] == "succeeded" for r in results
            )
            capture_failed = sum(
                r["capture_status"] == "failed" for r in results
            )

            per_plant = []
            for plant_id in sorted({r["plant_id"] for r in results}):
                plant_results = [r for r in results if r["plant_id"] == plant_id]
                per_plant.append(
                    {
                        "plant_id": plant_id,
                        "requested_poses": len(plant_results),
                        "reached_poses": sum(
                            r["motion_status"] == "reached" for r in plant_results
                        ),
                        "planning_failed": sum(
                            r["motion_status"] == "planning_failed"
                            for r in plant_results
                        ),
                        "execution_failed": sum(
                            r["motion_status"] == "execution_failed"
                            for r in plant_results
                        ),
                        "capture_succeeded": sum(
                            r["capture_status"] == "succeeded"
                            for r in plant_results
                        ),
                        "failed_views": [
                            {
                                "label": r["view"],
                                "motion_status": r["motion_status"],
                            }
                            for r in plant_results
                            if r["motion_status"] != "reached"
                        ],
                    }
                )

            requested = len(results)
            metadata["plant_scan"] = {
                "processing_complete": self.scan_processing_complete,
                "requested_poses": requested,
                "reached_poses": reached,
                "success_rate_percent": (
                    round(100.0 * reached / requested, 2) if requested else 0.0
                ),
                "planning_failed": planning_failed,
                "execution_failed": execution_failed,
                "capture_succeeded": capture_succeeded,
                "capture_failed": capture_failed,
                "pending_poses": sum(
                    r["motion_status"] == "pending" for r in results
                ),
                "plants": per_plant,
            }

            temporary_path = metadata_path + ".plant_scan.tmp"
            with open(temporary_path, "w", encoding="utf-8") as metadata_file:
                yaml.safe_dump(metadata, metadata_file, sort_keys=False)
            os.replace(temporary_path, metadata_path)
            self.get_logger().info(f"Saved plant scan summary to {metadata_path}")
        except Exception as exc:
            self.get_logger().error(f"Could not save plant scan metadata: {exc}")

    def select_next_scan_item(self, remaining, step, batch_index, batch_count):
        start_joint_map = self.get_current_joint_map(timeout=5.0)
        if start_joint_map is None:
            self.get_logger().warn(
                "No current joint state. Using the next pose in original order."
            )
            return remaining.pop(0), None, step + 1

        candidate_count = len(remaining)
        best_index = None
        best_duration = None
        best_distance = None
        best_trajectory = None
        best_message = ""
        failed_items = []

        self.get_logger().info(
            f"Online optimizer step {step} (batch {batch_index}/{batch_count}): "
            f"evaluating {candidate_count} candidates from actual joint state"
        )

        for i, item in enumerate(remaining):
            pose = item["pose"]
            plan_label = f"plant_{item['plant_id']}_{pose['label']}"
            plan_pose = dict(pose)
            plan_pose["label"] = plan_label

            success, message, cost, _, trajectory = self.call_arm_plan_to_pose(
                plan_pose, start_joint_map
            )

            if not success:
                self.get_logger().warn(
                    f"Online step {step} candidate {i + 1}/{candidate_count}: "
                    f"{plan_label} -> FAILED ({message})"
                )
                failed_items.append(item)
                continue

            self.get_logger().info(
                f"Online step {step} candidate {i + 1}/{candidate_count}: "
                f"{plan_label} -> duration={self.trajectory_duration(trajectory):.3f}s, "
                f"joint_distance={cost:.4f}rad"
            )

            duration = self.trajectory_duration(trajectory)
            is_better = (
                best_duration is None
                or duration < best_duration - 0.05
                or (
                    abs(duration - best_duration) <= 0.05
                    and cost < best_distance
                )
            )
            if is_better:
                best_index = i
                best_duration = duration
                best_distance = cost
                best_trajectory = trajectory
                best_message = message

        # Do not retry poses that failed during the next optimizer step. Keep
        # them for one final pass after all normal batches have completed.
        if best_index is None:
            if failed_items:
                remaining[:] = [item for item in remaining if item not in failed_items]
                self.deferred_scan_items.extend(failed_items)
            self.get_logger().warn(
                f"Could not plan to any remaining view in optimizer batch "
                f"{batch_index}/{batch_count}. Deferred failed candidates."
            )
            return None, None, step + 1

        chosen = remaining[best_index]
        remaining[:] = [
            item for item in remaining
            if item is not chosen and item not in failed_items
        ]
        self.deferred_scan_items.extend(failed_items)
        pose = chosen["pose"]
        self.get_logger().info(
            f"Online optimized step {step} (batch {batch_index}/{batch_count}): "
            f"plant {chosen['plant_id']} [{pose['label']}], "
            f"duration={best_duration:.3f}s, joint_distance={best_distance:.4f}rad "
            f"({best_message}); executing this plan now"
        )
        return chosen, best_trajectory, step + 1

    def retry_deferred_scan_items(self, order_index, total_items):
        """Retry each previously unreachable pose once from the actual state."""
        if not self.deferred_scan_items:
            return order_index

        deferred = self.deferred_scan_items
        self.deferred_scan_items = []
        self.get_logger().info(
            f"Retrying {len(deferred)} deferred unreachable view(s) once"
        )

        for item in deferred:
            start_joint_map = self.get_current_joint_map(timeout=5.0)
            if start_joint_map is None:
                self.get_logger().warn(
                    f"Deferred retry skipped for plant {item['plant_id']} "
                    f"[{item['pose']['label']}]: no joint state"
                )
                self.set_scan_result(item, motion_status="planning_failed")
                continue

            pose = item["pose"]
            plan_pose = dict(pose)
            plan_pose["label"] = f"plant_{item['plant_id']}_{pose['label']}"
            success, message, cost, _, trajectory = self.call_arm_plan_to_pose(
                plan_pose, start_joint_map
            )
            if not success:
                self.get_logger().warn(
                    f"Deferred retry failed for {plan_pose['label']}: {message}"
                )
                self.set_scan_result(item, motion_status="planning_failed")
                continue

            self.get_logger().info(
                f"Deferred retry succeeded for {plan_pose['label']}, "
                f"duration={self.trajectory_duration(trajectory):.3f}s, "
                f"joint_distance={cost:.4f}rad"
            )
            if not self.execute_scan_item(item, order_index, total_items, trajectory):
                return order_index
            order_index += 1

        return order_index

    def split_scan_batches(self, items):
        if not self.optimize_view_order:
            return [items]

        plant_ids = list(dict.fromkeys(item["plant_id"] for item in items))
        batch_size = int(self.optimizer_plant_batch_size)
        if batch_size <= 0:
            batch_size = len(plant_ids)

        plant_batches = [
            plant_ids[i:i + batch_size]
            for i in range(0, len(plant_ids), batch_size)
        ]
        return [
            [item for item in items if item["plant_id"] in batch_plant_ids]
            for batch_plant_ids in plant_batches
        ]

    #------- main execution loop ---------#
    def run(self):
        targets = self.latest_targets
        if self.latest_run_dir is None:
            self.get_logger().error("No directory received")
            return

        if len(targets) == 0:
            self.get_logger().error("No plant targets received")
            return

        all_scan_items = self.build_scan_items(targets)
        self.initialize_scan_summary(all_scan_items)
        scan_batches = self.split_scan_batches(all_scan_items)
        self.deferred_scan_items = []
        optimizer_step = 1
        order_index = 1

        for batch_index, batch_items in enumerate(scan_batches, start=1):
            remaining = list(batch_items)
            plant_ids = list(dict.fromkeys(item["plant_id"] for item in remaining))
            self.get_logger().info(
                f"Online optimizer batch {batch_index}/{len(scan_batches)}: "
                f"plants {plant_ids}, {len(remaining)} view poses"
            )

            while remaining:
                if self.optimize_view_order:
                    item, trajectory, optimizer_step = self.select_next_scan_item(
                        remaining,
                        optimizer_step,
                        batch_index,
                        len(scan_batches),
                    )
                else:
                    item = remaining.pop(0)
                    trajectory = None

                if item is None:
                    break

                if not self.execute_scan_item(
                    item, order_index, len(all_scan_items), trajectory
                ):
                    return
                order_index += 1

        order_index = self.retry_deferred_scan_items(
            order_index, len(all_scan_items)
        )
        self.scan_processing_complete = True
        self.finish_scan()

    def execute_scan_item(self, item, order_index, total_items, trajectory=None):
            plant_id = item["plant_id"]
            pose = item["pose"]
            j = item["pose_index"]
            pose_count = item["pose_count"]

            self.get_logger().info(
                f"Scan order {order_index}/{total_items} - plant {plant_id}, "
                f"pose {j}/{pose_count} [{pose['label']}]: "
                f"x={pose['x']:.4f}, y={pose['y']:.4f}, z={pose['z']:.4f}, "
                f"q=({pose['qx']:.3f}, {pose['qy']:.3f}, {pose['qz']:.3f}, {pose['qw']:.3f})"
            )
            if not self.wait_if_paused_or_stopped():
                self.get_logger().warn("Scanner stopped")
                return False

            move_pose = dict(pose)
            move_pose["label"] = f"plant_{plant_id}_{pose['label']}"
            if trajectory is not None:
                success, message = self.call_arm_execute_planned(
                    trajectory, move_pose["label"]
                )
                if not success and "Stopped by user" not in message:
                    self.get_logger().warn(
                        f"Optimizer trajectory failed for {move_pose['label']}: "
                        f"{message}. Falling back to replanning from actual state."
                    )
                    success, message = self.call_arm_move_to_pose(move_pose)
            else:
                success, message = self.call_arm_move_to_pose(move_pose)

            if not success:
                self.set_scan_result(
                    item,
                    motion_status="execution_failed",
                    capture_status="not_attempted",
                )
                self.get_logger().warn(f"Arm move failed for plant {plant_id}, pose {pose['label']}: {message}")
                if ("Stopped by user" in message or "STOP" in message or "stop" in message or "Controller deactivated" in message):
                    with self.scan_lock:
                        self.scan_paused = True

                    self.get_logger().warn("Scanner paused because arm was stopped")

                    if not self.wait_if_paused_or_stopped():
                        self.get_logger().warn("Scanner stopped")
                        return False

                return True

            self.get_logger().info(message)

            capture_started_at = time.time()
            capture_success = self.call_orbbec_capture(run_dir=self.latest_run_dir,plant_id=plant_id,view_label=pose["label"])
            self.set_scan_result(
                item,
                motion_status="reached",
                capture_status="succeeded" if capture_success else "failed",
            )
            if not capture_success:
                self.get_logger().warn(f"Orbbec capture failed for plant {plant_id}, view {pose['label']}")

            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame,self.ee_link,Time())
                t = transform.transform.translation
                r = transform.transform.rotation

                self.get_logger().info(
                    f"Plant {plant_id} - actual pose {j}/{pose_count} [{pose['label']}]: "
                    f"x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}, "
                    f"q=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})"
                )

                if capture_success:
                    meta_path = self.wait_for_capture_meta(self.latest_run_dir, plant_id, pose["label"], capture_started_at)
                    if meta_path is not None:
                        self.append_actual_pose_to_meta(meta_path, plant_id, pose["label"], transform)

            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

            return True

    def finish_scan(self):
        self.get_logger().info("All targets processed. Sending arm to rest.")
        success, message = self.call_arm_go_rest()
        if success:
            self.get_logger().info(message)
            if not self.wait_until_arm_rest():
                self.get_logger().warn("Arm did not confirm rest. Not publishing /top_scan/scan_done.")
                return

            self.pub_auto_state_cmd.publish(String(data="idle"))
            self.pub_top_scan_done.publish(Bool(data=True))
            self.get_logger().info("Plant row scan complete.")
        else:
            self.get_logger().warn(message)

def main(args=None):
    rclpy.init(args=args)

    node = PlantViewScanner()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
