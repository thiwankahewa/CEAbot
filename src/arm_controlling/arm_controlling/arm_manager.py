#!/usr/bin/env python3

import math
import time
import threading
import traceback
import rclpy

from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_controlling.moveit_arm_helper import MoveItArmHelper

from arm_interfaces.srv import ExecutePlannedTrajectory, MoveToPose, PlanToPose


REST_APPROACH = {"joint_1": -0.628,"joint_2": -2.23,"joint_3": 0.0521,"joint_4": 1.6613,"joint_5": 3.1415,"joint_6": -2.09,"joint_7": -0.0868,}
POSE_1 = {"joint_1": -0.764,"joint_2": -2.04,"joint_3": 0.0521,"joint_4": 1.6613,"joint_5": 3.1415,"joint_6": -2.09,"joint_7": -0.0868,}


class ArmManager(MoveItArmHelper):
    def __init__(self):
        super().__init__("arm_manager")

        #--------- States and variables ---------#

        self.command_busy = False
        self.stop_requested = False
        self.command_lock = threading.Lock()

        self.cb_group = ReentrantCallbackGroup()

        #--------- Services ---------#

        self.switch_controller_client = self.create_client(SwitchController,"/controller_manager/switch_controller",callback_group=self.cb_group,)
        self.srv_move_to_pose = self.create_service(MoveToPose,"/arm/move_to_pose",self.cb_move_to_pose,callback_group=self.cb_group,)
        self.srv_plan_to_pose = self.create_service(PlanToPose,"/arm/plan_to_pose",self.cb_plan_to_pose,callback_group=self.cb_group,)
        self.srv_execute_planned = self.create_service(ExecutePlannedTrajectory,"/arm/execute_planned_trajectory",self.cb_execute_planned_trajectory,callback_group=self.cb_group,)
        self.srv_go_rest = self.create_service(Trigger, "/arm/go_rest", self.cb_go_rest,callback_group=self.cb_group,)
        self.srv_pose_1 = self.create_service(Trigger, "/arm/pose_1", self.cb_pose_1,callback_group=self.cb_group,)
        self.srv_reset = self.create_service(Trigger, "/arm/reset_moveit_control", self.cb_reset_moveit_control,callback_group=self.cb_group,)
        self.srv_stop = self.create_service(Trigger, "/arm/stop", self.cb_stop,callback_group=self.cb_group,)

    #--------- Callbacks ---------#

    def cb_go_rest(self, request, response):
        response.success, response.message = self.start_command("go_rest",self.move_to_rest,)
        return response

    def cb_pose_1(self, request, response):
        response.success, response.message = self.start_command("pose_1",lambda: self.move_to_joint_pose("pose_1", POSE_1),)
        return response

    def cb_reset_moveit_control(self, request, response):
        response.success, response.message = self.start_command("reset",self.run_reset,)
        return response

    def cb_stop(self, request, response):
        with self.command_lock:
            self.stop_requested = True

        self.cancel_active_execution()
        ok = self.deactivate_trajectory_controller()
        response.success = ok
        response.message = "STOP sent. Controller deactivated."
        return response

    def cb_move_to_pose(self, request, response):
        with self.command_lock:
            if self.command_busy:
                response.success = False
                response.message = "Arm is busy. Wait until current command finishes."
                return response

            self.command_busy = True
            self.stop_requested = False

        target = {
            "x": request.x,
            "y": request.y,
            "z": request.z,
            "qx": request.qx,
            "qy": request.qy,
            "qz": request.qz,
            "qw": request.qw,
            "label": request.label,
        }

        try:
            self.get_logger().info(f"Starting blocking move_to_pose: {request.label}")

            ok, msg = self.move_to_cartesian_pose(target)

            response.success = ok
            response.message = msg
            return response

        except Exception as e:
            self.get_logger().error(f"move_to_pose crashed: {e}")
            response.success = False
            response.message = str(e)
            return response

        finally:
            with self.command_lock:
                self.command_busy = False

            self.get_logger().info(f"Finished blocking move_to_pose: {request.label}")

    def cb_plan_to_pose(self, request, response):
        with self.command_lock:
            if self.command_busy:
                response.success = False
                response.message = "Arm is busy. Cannot plan while current command is running."
                return response

        start_joint_map = None
        if len(request.start_joint_names) > 0 or len(request.start_joint_positions) > 0:
            if len(request.start_joint_names) != len(request.start_joint_positions):
                response.success = False
                response.message = "start_joint_names and start_joint_positions length mismatch"
                return response

            start_joint_map = dict(zip(request.start_joint_names, request.start_joint_positions))

        try:
            traj = self.plan_to_target(request.x,request.y,request.z,request.qx,request.qy,request.qz,request.qw,start_joint_map=start_joint_map,)

            if traj is None:
                response.success = False
                response.message = f"Planning failed for {request.label}"
                return response

            final_joint_map = self.trajectory_final_joint_map(traj)
            response.success = True
            response.message = f"Planned {request.label}"
            response.cost = self.trajectory_joint_cost(traj)
            response.final_joint_names = list(final_joint_map.keys())
            response.final_joint_positions = [final_joint_map[name] for name in response.final_joint_names]
            response.planned_trajectory = traj
            return response

        except Exception as e:
            self.get_logger().error(f"plan_to_pose crashed: {e}")
            response.success = False
            response.message = str(e)
            return response

    def cb_execute_planned_trajectory(self, request, response):
        with self.command_lock:
            if self.command_busy:
                response.success = False
                response.message = "Arm is busy. Wait until current command finishes."
                return response

            self.command_busy = True
            self.stop_requested = False

        try:
            self.get_logger().info(f"Executing optimizer trajectory: {request.label}")

            if not self.execute_trajectory(request.trajectory):
                response.success = False
                response.message = f"Cached trajectory execution failed for {request.label}"
                return response

            # execute_trajectory() already waits for the ExecuteTrajectory
            # action result. Keep one joint-position confirmation, but avoid
            # an additional zero-velocity wait after every view.
            finished = self.wait_until_trajectory_finished(
                request.trajectory, tolerance=0.008, timeout=25.0
            )

            if self.check_stop_requested():
                response.success = False
                response.message = "Stopped by user"
            elif finished:
                response.success = True
                response.message = f"Executed optimizer trajectory for {request.label}"
            else:
                response.success = False
                response.message = f"Robot may not be fully settled at {request.label}"

            return response

        except Exception as e:
            self.get_logger().error(f"execute_planned_trajectory crashed: {e}")
            response.success = False
            response.message = str(e)
            return response

        finally:
            with self.command_lock:
                self.command_busy = False
    
    #--------- Helper Methods ---------#

    def start_command(self, name, target_func):
        with self.command_lock:
            if self.command_busy:
                return False, "Arm is busy. Wait until current command finishes."

            self.command_busy = True
            self.stop_requested = False

        threading.Thread(target=self.run_command,args=(name, target_func),daemon=True,).start()
        return True, f"{name} started"

    def run_command(self, name, target_func):
        try:
            self.get_logger().info(f"Starting command: {name}")
            ok, msg = target_func()

            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().error(msg)

        except Exception:
            self.get_logger().error(f"Command {name} crashed:\n{traceback.format_exc()}")

        finally:
            with self.command_lock:
                self.command_busy = False

    def check_stop_requested(self):
        with self.command_lock:
            return self.stop_requested

    def wait_for_arm_ready(self, timeout=10.0):
        start = time.time()

        while rclpy.ok():
            if self.move_group_client.wait_for_server(timeout_sec=0.2):
                if self.current_joint_state is not None and len(self.current_joint_state.name) > 0:
                    return True

            if time.time() - start > timeout:
                return False

            time.sleep(0.2)

        return False

    def is_near_joint_pose(self, target, tolerance=0.06):
        current = self.get_current_joint_map(timeout=5.0)

        if current is None:
            return False

        for joint_name, target_pos in target.items():
            if joint_name not in current:
                return False

            err = abs(math.atan2(math.sin(target_pos - current[joint_name]),math.cos(target_pos - current[joint_name])))

            if err > tolerance:
                return False

        return True

    #--------- Arm movement functions ---------#

    def move_to_cartesian_pose(self, target):
        if self.check_stop_requested():
            return False, "Stopped by user"

        traj = self.plan_to_target(target["x"],target["y"],target["z"],target["qx"],target["qy"],target["qz"],target["qw"],)

        if traj is None:
            return False, f"Planning failed for {target['label']}"

        if self.check_stop_requested():
            return False, "Stopped by user"

        if not self.execute_trajectory(traj):
            return False, f"Execution failed for {target['label']}"

        if self.check_stop_requested():
            return False, "Stopped by user"

        finished = self.wait_until_trajectory_finished(traj,tolerance=0.008,timeout=25.0,)
        if self.check_stop_requested():
            return False, "Stopped by user"

        if finished:
            return True, f"Moved to {target['label']}"

        return False, f"Robot may not be fully settled at {target['label']}"

    def move_to_joint_pose(self, name, target):
        if self.check_stop_requested():
            return False, "Stopped by user"

        if self.is_near_joint_pose(target):
            return True, f"Already near {name}"

        traj = self.plan_to_joint_positions(target)

        if traj is None:
            return False, f"Planning failed for {name}"

        if self.check_stop_requested():
            return False, "Stopped by user"

        if not self.execute_trajectory(traj):
            return False, f"Execution failed for {name}"

        if self.check_stop_requested():
            return False, "Stopped by user"

        finished = self.wait_until_trajectory_finished(traj, timeout=25.0)

        if self.check_stop_requested():
            return False, "Stopped by user"

        if not finished:
            return False, f"Robot may not be fully settled at {name}"

        return True, f"Moved to {name}"

    def move_to_rest(self):
        if self.check_stop_requested():
            return False, "Stopped by user"

        if self.is_near_joint_pose(REST_APPROACH):
            return True, "Arm already at rest"

        return self.move_to_joint_pose("rest", REST_APPROACH)


    #--------- Stop and Reset functions ---------#

    def deactivate_trajectory_controller(self, timeout=0.5):
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("switch_controller service not available")
            return False

        req = SwitchController.Request()
        req.deactivate_controllers = ["joint_trajectory_controller"]
        req.activate_controllers = []
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout.sec = 0
        req.timeout.nanosec = int(timeout * 1e9)
        future = self.switch_controller_client.call_async(req)
        result = self.wait_future(future, timeout=timeout)
        return result is not None and result.ok

    def run_reset(self):
        with self.command_lock:
            self.stop_requested = False

        ok = self.reset_moveit_control()

        if ok:
            return True, "MoveIt control reset"
        return False, "MoveIt control reset failed"

    def reset_moveit_control(self):
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("switch_controller service not available")
            return False

        def switch(activate, deactivate):
            req = SwitchController.Request()
            req.activate_controllers = activate
            req.deactivate_controllers = deactivate
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.activate_asap = True
            req.timeout.sec = 3
            future = self.switch_controller_client.call_async(req)
            result = self.wait_future(future, timeout=5.0)
            return result is not None and result.ok

        ok1 = switch([], ["joint_trajectory_controller"])
        time.sleep(0.5)

        ok2 = switch(["joint_trajectory_controller"], [])
        time.sleep(0.5)

        return ok1 and ok2


def main(args=None):
    rclpy.init(args=args)

    node = ArmManager()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    startup_attempts = 0

    def startup():
        nonlocal startup_attempts
        startup_attempts += 1

        if not node.wait_for_arm_ready(timeout=2.0):
            if startup_attempts <= 20:
                node.get_logger().info(f"Waiting for arm startup readiness (attempt {startup_attempts}/20)")
            return

        node.startup_timer.cancel()
        node.get_logger().info("Startup: moving arm to rest")
        node.start_command("go_rest", node.move_to_rest)

    node.startup_timer = node.create_timer(1.0, startup)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
