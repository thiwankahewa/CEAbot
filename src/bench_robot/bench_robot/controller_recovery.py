"""Restart a crashed standalone controller manager without restarting the scan."""

from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.logging import get_logger
from launch_ros.actions import Node


def recovering_controller_manager(manager_factory, on_ready, max_restarts=3, delay=5.0):
    """Create new process actions on each attempt; launch application nodes once.

    Wait for the previous spawner to exit before restarting, so an old spawner
    cannot load controllers into the next manager. A failed spawner never
    starts the application. Shutdown never schedules another attempt.
    """
    applications_started = False

    def attempt(number):
        manager = manager_factory()
        spawner = Node(
            package="controller_manager", executable="spawner",
            arguments=[
                "joint_state_broadcaster", "joint_trajectory_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
                "--service-call-timeout", "5", "--switch-timeout", "10",
            ],
            output="screen",
        )
        state = {"manager_exited": False, "spawner_exited": False, "scheduled": False}

        def maybe_restart(context):
            if (context.is_shutdown or state["scheduled"]
                    or not state["manager_exited"] or not state["spawner_exited"]):
                return []
            state["scheduled"] = True
            if number >= max_restarts:
                get_logger("arm_recovery").error("Arm controller restart limit reached. Scan cannot continue.")
                return []
            return [
                LogInfo(msg=f"Restarting arm controller manager in {delay}s "
                            f"(recovery {number + 1}/{max_restarts})."),
                TimerAction(period=delay, actions=attempt(number + 1)),
            ]

        def manager_exited(event, context):
            state["manager_exited"] = True
            return maybe_restart(context)

        def controllers_spawned(event, context):
            nonlocal applications_started
            state["spawner_exited"] = True
            if context.is_shutdown:
                return []
            if state["manager_exited"]:
                return maybe_restart(context)
            if event.returncode != 0:
                get_logger("arm_recovery").error("Arm controllers failed to activate; scan remains blocked.")
                return []
            if not applications_started:
                applications_started = True
                return list(on_ready)
            return [LogInfo(msg="Arm controllers reloaded. Waiting scans may now recover.")]

        return [
            RegisterEventHandler(OnProcessStart(target_action=manager, on_start=[spawner])),
            RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=manager_exited)),
            RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=controllers_spawned)),
            manager,
        ]

    return attempt(0)
