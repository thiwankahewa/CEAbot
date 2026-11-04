"""Bounded recovery of the observed latched base JOINT_FAULT.

Only ArmManager owns this worker. It never submits a motion goal; callers must
plan again after readiness returns. Other faults require operator attention.
"""

import math
import socket
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController

JOINT_FAULT = 0x00000010
SOFTWARE_STOP = 0x00800000  # Base.EMERGENCY_STOP_ACTIVATED, not EMERGENCY_LINE_ACTIVATED.


def feedback_summary(feedback):
    """Keep the fault banks and arm state in logs even when validation fails."""
    state_names = {
        0: "UNSPECIFIED", 1: "BASE_INITIALIZATION", 2: "IDLE",
        3: "INITIALIZATION", 4: "IN_FAULT", 5: "MAINTENANCE",
        6: "SERVOING_LOW_LEVEL", 7: "SERVOING_READY",
        8: "SERVOING_PLAYING_SEQUENCE", 9: "SERVOING_MANUALLY_CONTROLLED",
    }
    base = feedback.base
    actuator_faults = [
        f"joint_{i}:A=0x{actuator.fault_bank_a:08x},B=0x{actuator.fault_bank_b:08x}"
        for i, actuator in enumerate(feedback.actuators, 1)
        if actuator.fault_bank_a or actuator.fault_bank_b
    ]
    return (
        f"base_fault_a=0x{base.fault_bank_a:08x}, base_fault_b=0x{base.fault_bank_b:08x}; "
        f"actuator_faults=[{'; '.join(actuator_faults) or 'none'}] "
        f"({len(feedback.actuators)} actuators); "
        f"active_state={state_names.get(base.active_state, 'UNKNOWN')}({base.active_state})"
    )


def validate_feedback(feedback, allowed_states, allow_clear=False, allow_software_stop=False):
    """The ROS fault interface is a sum; inspect every bank before clearing."""
    base = feedback.base
    allowed_faults = {0, JOINT_FAULT} if allow_clear else {JOINT_FAULT}
    if allow_software_stop:
        # Only used after this worker successfully requested a software stop.
        # Never admit the physical emergency-line bit or actuator fault bits.
        allowed_faults |= {fault | SOFTWARE_STOP for fault in allowed_faults}
    if base.fault_bank_a not in allowed_faults or base.fault_bank_b:
        raise RuntimeError("Base fault is not the supported latched JOINT_FAULT; "
                           + feedback_summary(feedback))
    if len(feedback.actuators) != 7 or any(
        actuator.fault_bank_a or actuator.fault_bank_b
        for actuator in feedback.actuators
    ):
        raise RuntimeError("Actuator fault or incomplete actuator feedback; inspect Kinova Web App; "
                           + feedback_summary(feedback))
    if base.active_state not in allowed_states:
        raise RuntimeError("Arm is not in the expected servoing state; automatic reset refused; "
                           + feedback_summary(feedback))


class KortexFaultSession:
    """Short-lived TCP session with bounded socket and RPC timeouts."""

    def __init__(self, node):
        self.node = node
        self.transport = None
        self.session = None

    def __enter__(self):
        try:
            from kortex_api.TCPTransport import TCPTransport
            from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
            from kortex_api.SessionManager import SessionManager
            from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
            from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
            from kortex_api.autogen.messages import Base_pb2, Common_pb2, Session_pb2
        except ImportError as exc:
            raise RuntimeError("Automatic fault reset needs the Kinova Python Kortex SDK") from exc

        self.base_pb = Base_pb2
        self.single_level_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.low_level = Common_pb2.ARMSTATE_SERVOING_LOW_LEVEL
        self.ready = Common_pb2.ARMSTATE_SERVOING_READY
        self.in_fault = Common_pb2.ARMSTATE_IN_FAULT
        self.initialization = Common_pb2.ARMSTATE_INITIALIZATION
        self.options = RouterClientSendOptions()
        self.options.timeout_ms = 2000
        try:
            self.transport = TCPTransport()
            self.transport.sock.settimeout(3.0)
            router = RouterClient(self.transport, lambda error: None)
            self.transport.connect(
                self.node.get_parameter("kinova_robot_ip").value,
                int(self.node.get_parameter("kinova_port").value))
            self.session = SessionManager(router)
            info = Session_pb2.CreateSessionInfo()
            info.username = self.node.get_parameter("kinova_username").value
            info.password = self.node.get_parameter("kinova_password").value
            info.session_inactivity_timeout = 10000
            info.connection_inactivity_timeout = 2000
            self.session.CreateSession(info, options=self.options)
            self.base = BaseClient(router)
            self.cyclic = BaseCyclicClient(router)
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def feedback(self):
        return self.cyclic.RefreshFeedback(options=self.options)

    def single_level(self):
        info = self.base_pb.ServoingModeInformation()
        info.servoing_mode = self.base_pb.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(info, options=self.options)

    def clear_faults(self):
        self.base.ClearFaults(options=self.options)

    def apply_software_stop(self):
        self.base.ApplyEmergencyStop(options=self.options)

    def servoing_mode(self):
        return self.base.GetServoingMode(options=self.options).servoing_mode

    def __exit__(self, *args):
        if self.session is not None:
            try:
                self.session.CloseSession(options=self.options)
            except Exception:
                pass
        if self.transport is not None:
            # SDK disconnect() joins without a deadline. Unblock its receiver
            # first and bound cleanup, including a failed connect/CreateSession.
            self.transport.event.set()
            try:
                self.transport.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.transport.sock.close()
            if self.transport.tcp_transport_thread.is_alive():
                self.transport.tcp_transport_thread.join(timeout=2.0)


class AutomaticFaultRecovery:
    def __init__(self, node, session_factory=KortexFaultSession):
        self.node = node
        self.session_factory = session_factory
        for name, default in (
            ("auto_fault_reset", False), ("auto_fault_reset_max_attempts", 2),
            ("kinova_robot_ip", ""), ("kinova_port", 10000),
            ("kinova_username", "admin"), ("kinova_password", "admin"),
        ):
            node.declare_parameter(name, default)
        self.enabled = bool(node.get_parameter("auto_fault_reset").value)
        self.limit = max(0, int(node.get_parameter("auto_fault_reset_max_attempts").value))
        self.attempts = 0
        self.running = False
        self.blocked_reason = None
        self.detected_at = None
        self.worker = None
        self.timer = node.create_timer(0.5, self.tick, callback_group=node.cb_group)

    def reason(self):
        if self.running:
            return "Recovering latched Kinova fault; waiting for verified arm control"
        return self.blocked_reason

    def eligible(self):
        node = self.node
        return (self.enabled and not self.blocked_reason
                and node.hardware_fault == 16.0
                and node.hardware_fault_received_at is not None
                and time.monotonic() - node.hardware_fault_received_at <= node.joint_state_max_age
                and node.has_fresh_arm_state())

    def tick(self):
        node = self.node
        with node.command_lock:
            if self.running or node.stop_requested or not self.eligible():
                self.detected_at = None
                return
            if self.detected_at is None:
                self.detected_at = time.monotonic()
            if node.command_busy or time.monotonic() - self.detected_at < 1.0:
                return
            if self.attempts >= self.limit:
                self.blocked_reason = "Automatic Kinova fault reset budget exhausted; inspect Kinova Web App"
                node.get_logger().error(self.blocked_reason)
                return
            node.command_busy = True
            self.running = True
            self.attempts += 1
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def check_stop(self):
        if not rclpy.ok() or self.node.check_stop_requested():
            raise RuntimeError("Stopped by user or ROS shutdown during fault recovery")

    def wait_until(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_stop()
            if predicate():
                return
            time.sleep(0.05)
        raise RuntimeError(message() if callable(message) else message)

    def controller_states(self):
        node = self.node
        client = node.list_controllers_client
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Cannot verify controller state")
        future = client.call_async(ListControllers.Request())
        try:
            result = node.wait_future(future, timeout=2.0)
            if result is None:
                raise RuntimeError("Controller state query timed out")
            return {c.name: c.state for c in result.controller}
        finally:
            if not future.done():
                client.remove_pending_request(future)

    @staticmethod
    def require_no_command_controllers(states):
        if any(state == "active" and name != "joint_state_broadcaster"
               for name, state in states.items()):
            raise RuntimeError("Another command controller is active; automatic reset refused")

    def switch(self, activate):
        node = self.node
        if not activate:
            states = self.controller_states()
            if states.get("joint_trajectory_controller") == "inactive":
                self.require_no_command_controllers(states)
                return
        client = node.switch_controller_client
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("Controller switch service unavailable")
        if activate:
            self.check_stop()
        request = SwitchController.Request()
        request.activate_controllers = ["joint_trajectory_controller"] if activate else []
        request.deactivate_controllers = [] if activate else ["joint_trajectory_controller"]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout.sec = 3
        result = node.wait_future(client.call_async(request), timeout=5.0)
        if result is None or not result.ok:
            raise RuntimeError("Controller switch failed or timed out")
        states = self.controller_states()
        if states.get("joint_trajectory_controller") != ("active" if activate else "inactive"):
            raise RuntimeError("Trajectory controller did not reach the requested state")
        if not activate:
            self.require_no_command_controllers(states)

    def wait_for_single_level(self, api):
        mode = None

        def confirmed():
            nonlocal mode
            mode = api.servoing_mode()
            return mode == api.single_level_mode

        self.wait_until(confirmed, 3.0,
                        lambda: f"Single-level servoing was not confirmed before ClearFaults (mode={mode})")

    def prepare_fault_reset(self, api):
        """Use the installed Kortex driver's stop-before-clear reset sequence.

        hardware_interface.cpp sends ApplyEmergencyStop twice because one call
        was unreliable in its testing. Do not clear until its expected stop bit
        is visible, and reject any unrelated fault throughout the transition.
        """
        self.switch(False)
        self.wait_for_idle()
        validate_feedback(api.feedback(), (api.low_level, api.ready, api.in_fault), allow_clear=True)
        self.check_stop()
        self.node.get_logger().warn(
            "Preparing Kinova fault reset: applying the driver's software-stop sequence "
            "with trajectory control inactive")
        for _ in range(2):
            self.check_stop()
            api.apply_software_stop()
            self.check_stop()
            feedback = api.feedback()
            validate_feedback(feedback, (api.low_level, api.ready, api.in_fault),
                              allow_clear=True, allow_software_stop=True)

        last_feedback = None

        def stop_confirmed():
            nonlocal last_feedback
            last_feedback = api.feedback()
            validate_feedback(last_feedback, (api.low_level, api.ready, api.in_fault),
                              allow_clear=True, allow_software_stop=True)
            return bool(last_feedback.base.fault_bank_a & SOFTWARE_STOP)

        self.wait_until(
            stop_confirmed, 3.0,
            lambda: "Software stop was not confirmed; ClearFaults not sent; "
                    + (feedback_summary(last_feedback) if last_feedback else "no feedback"))
        self.node.get_logger().info("Kinova feedback after software stop: " + feedback_summary(last_feedback))

    def wait_for_fault_clear(self, api, allow_software_stop=False):
        last_feedback = None
        initialization_logged = False

        def cleared():
            nonlocal last_feedback, initialization_logged
            last_feedback = api.feedback()
            if last_feedback.base.active_state == api.initialization:
                # After ClearFaults the arm can initialize before becoming ready.
                # Admit this state only here, with every fault bank clear. It is
                # not permission to reactivate trajectory control yet.
                validate_feedback(last_feedback, (api.initialization,), allow_clear=True)
                if last_feedback.base.fault_bank_a != 0:
                    raise RuntimeError("Kinova fault during post-reset initialization; "
                                       + feedback_summary(last_feedback))
                if not initialization_logged:
                    self.node.get_logger().info(
                        "Kinova fault banks cleared; waiting for initialization to finish "
                        "before reactivating trajectory control")
                    initialization_logged = True
                return False
            validate_feedback(last_feedback, (api.low_level, api.ready, api.in_fault),
                              allow_clear=True, allow_software_stop=allow_software_stop)
            return (last_feedback.base.fault_bank_a == 0
                    and last_feedback.base.active_state in (api.low_level, api.ready))

        def timeout_reason():
            if last_feedback is None:
                return "No feedback received after ClearFaults"
            if last_feedback.base.fault_bank_a:
                reason = "Kinova fault remained after one ClearFaults request"
            else:
                reason = "Kinova fault banks cleared, but arm did not reach a ready servoing state"
            return reason + "; " + feedback_summary(last_feedback)

        self.wait_until(cleared, 10.0, timeout_reason)
        self.node.get_logger().info("Kinova feedback after ClearFaults: " + feedback_summary(last_feedback))

    def wait_for_idle(self):
        stable_since = None

        def idle():
            nonlocal stable_since
            node = self.node
            state = node.current_joint_state
            velocities = dict(zip(state.name, state.velocity)) if state else {}
            settled = (node.has_fresh_arm_state()
                       and node.pending_execution_send is None
                       and (node.pending_execution_result is None or node.pending_execution_result.done())
                       and all(name in velocities and math.isfinite(velocities[name])
                               and abs(velocities[name]) < 0.01 for name in node.arm_joint_names))
            if not settled:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = time.monotonic()
            return time.monotonic() - stable_since >= 0.5

        self.wait_until(idle, 15.0, "Previous goal or arm motion did not settle before fault reset")

    def recover(self):
        node = self.node
        self.check_stop()
        with self.session_factory(node) as api:
            feedback = api.feedback()
            node.get_logger().info("Kinova feedback before recovery: " + feedback_summary(feedback))
            validate_feedback(feedback, (api.low_level,))
            self.check_stop()
            node.cancel_active_execution()
            self.switch(False)
            self.wait_for_idle()
            validate_feedback(api.feedback(), (api.low_level,))
            self.check_stop()
            api.single_level()
            self.wait_for_single_level(api)
            self.check_stop()
            feedback = api.feedback()
            node.get_logger().info("Kinova feedback after servoing-mode change: " + feedback_summary(feedback))
            validate_feedback(feedback, (api.low_level, api.ready, api.in_fault), allow_clear=True)
            self.prepare_fault_reset(api)
            self.check_stop()
            api.clear_faults()

            self.wait_for_fault_clear(api, allow_software_stop=True)
            self.wait_for_idle()
            self.check_stop()
            activated_at = time.monotonic()
            self.switch(True)
            self.check_stop()

            def fresh_clear():
                return (node.hardware_fault == 0.0
                        and node.hardware_fault_received_at is not None
                        and node.hardware_fault_received_at > activated_at
                        and node.joint_state_received_at is not None
                        and node.joint_state_received_at > activated_at
                        and node.hardware_fault_reason() is None and node.has_fresh_arm_state())

            self.wait_until(fresh_clear, 5.0, "No fresh fault-free ROS feedback after reactivation")
            if not node.wait_for_controller_ready(timeout=5.0, stable_duration=0.5):
                raise RuntimeError("Arm control did not become ready after fault reset")
            self.check_stop()
            feedback = api.feedback()
            validate_feedback(feedback, (api.low_level,), allow_clear=True)
            if feedback.base.fault_bank_a != 0 or node.hardware_fault != 0.0:
                raise RuntimeError("Kinova fault returned after controller activation")
            self.wait_for_idle()
            self.check_stop()
            node.connection_epoch += 1
            node.execution_epoch = None

    def run(self):
        node = self.node
        node.get_logger().warn(f"Automatic Kinova fault reset {self.attempts}/{self.limit}: checking fault banks")
        try:
            self.recover()
            node.get_logger().info("Kinova fault cleared and arm control verified; waiting commands may replan")
        except Exception as exc:
            self.blocked_reason = f"Automatic Kinova fault reset stopped: {exc}"
            node.get_logger().error(self.blocked_reason)
            try:
                self.switch(False)
            except Exception as stop_exc:
                node.get_logger().error(f"Could not confirm controller deactivation: {stop_exc}")
        finally:
            with node.command_lock:
                self.running = False
                node.command_busy = False
                self.detected_at = None
