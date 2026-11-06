"""Stop-control regression tests without constructing nodes or moving hardware."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from arm_controlling.arm_manager import ArmManager, ArmMotionPaused
from arm_controlling.plant_view_scanner import PlantViewScanner
from bench_robot.auto_state_manager import AutoStateManager
from bench_robot.aruco_detector import ArucoManager
from bench_robot.bench_changer import BenchChanger
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint


def done(value):
    future = Future()
    future.set_result(value)
    return future


def trajectory(position):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['joint_1']
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=[position])]
    return result


class ArmHarness(ArmManager):
    def __init__(self):
        self.command_lock = threading.Lock()
        self.estop_active = False
        self.pause_epoch = 0
        self.motion_pause_epoch = None
        self.stop_requested = False
        self.command_busy = False
        self.active_execute_goal = None
        self.pending_execution_send = None
        self.pending_execution_result = None
        self.connection_epoch = 0
        self.require_controller_ready = Mock()
        self.prepare_rest_controller = Mock(return_value=(True, 'ready'))
        self.execute_client = Mock()
        self.logger = Mock()
        self.wait_future = Mock(side_effect=lambda future, **kwargs: future.result())
        self.plan_to_joint_positions = Mock(return_value=trajectory(1.0))

    def get_logger(self):
        return self.logger


class ScanHarness(PlantViewScanner):
    def __init__(self):
        self.scan_lock = threading.Lock()
        self.estop_active = False
        self.scan_stopped = False
        self.scan_paused = False
        self.scan_busy = False
        self.stop_in_progress = False
        self.pub_auto_state_cmd = Mock()
        self.arm_stop_client = Mock()
        self.arm_stop_client.call_async.return_value = done(NS(success=True))
        self.wait_future = Mock(side_effect=lambda future, **kw: future.result())
        self.call_arm_go_rest = Mock(return_value=(True, 'rest'))
        self.logger = Mock()

    def get_logger(self):
        return self.logger


class StateHarness(AutoStateManager):
    def __init__(self):
        self.estop_active = False
        self.deferred_command = None
        self.run_cancelled = False
        self.mode = 'auto'
        self.auto_state = 'individual_plant_scan'
        self.publish_state = Mock()
        self.set_state = Mock()


class StopControlsTest(unittest.TestCase):
    @patch('arm_controlling.plant_view_scanner.rclpy.ok', return_value=True)
    def test_estop_race_does_not_lose_return_to_rest(self, _):
        scan = ScanHarness()
        scan.go_rest_client = Mock()
        scan.go_rest_client.call_async.side_effect = [
            done(NS(success=False, message='E-stop active')),
            done(NS(success=True, message='rest')),
        ]
        with patch('arm_controlling.plant_view_scanner.time.sleep'):
            self.assertEqual(PlantViewScanner.call_arm_go_rest(
                scan, cancel_on_scan_stop=True), (True, 'rest'))
        self.assertEqual(scan.go_rest_client.call_async.call_count, 2)

    @patch('arm_controlling.arm_manager.rclpy.ok', return_value=True)
    def test_no_motion_is_submitted_while_estop_is_held(self, _):
        arm = ArmHarness()
        arm.estop_active = True
        def release(_seconds):
            arm.execute_client.send_goal_async.assert_not_called()
            arm.cb_estop(NS(data=False))
        with patch('arm_controlling.arm_manager.time.sleep', side_effect=release):
            self.assertTrue(arm.wait_for_estop_release())

    @patch('arm_controlling.plant_view_scanner.rclpy.ok', return_value=True)
    def test_scan_worker_must_exit_before_rest(self, _):
        scan = ScanHarness()
        scan.scan_busy = True
        def finish(_seconds):
            scan.call_arm_go_rest.assert_not_called()
            self.assertTrue(scan.scan_stopped)
            scan.scan_busy = False
        with patch('arm_controlling.plant_view_scanner.time.sleep', side_effect=finish):
            self.assertTrue(scan.cb_stop_scan(None, NS()).success)
        scan.call_arm_go_rest.assert_called_once()

    def test_failed_rest_never_advances_to_next_row(self):
        node = NS(rest_pending=True, rest_pending_action='advance',
                  rest_future=done(NS(success=False, message='E-stop active')),
                  get_logger=Mock(return_value=Mock()), request_arm_rest=Mock(),
                  advance_to_next_goal=Mock())
        self.assertFalse(ArucoManager.process_pending_rest(node))
        node.advance_to_next_goal.assert_not_called()
        node.request_arm_rest.assert_called_once_with('advance')

    def test_bench_change_pause_extends_motion_deadline_not_sensor_freshness(self):
        node = NS(estop_started_at=None, now_s=Mock(side_effect=[10.0, 110.0]),
                  stop=Mock(), phase_start_time=5.0, exit_marker_loss_start_time=8.0,
                  last_tof_time=9.0)
        BenchChanger.cb_estop(node, NS(data=True))
        BenchChanger.cb_estop(node, NS(data=False))
        self.assertEqual(node.phase_start_time, 105.0)
        self.assertEqual(node.exit_marker_loss_start_time, 108.0)
        self.assertEqual(node.last_tof_time, 9.0)

    def test_estop_cancels_current_goal_only_once_and_preserves_stop(self):
        arm = ArmHarness()
        goal = arm.active_execute_goal = Mock()
        arm.stop_requested = True
        arm.cb_estop(NS(data=True))
        arm.cb_estop(NS(data=True))
        goal.cancel_goal_async.assert_called_once()
        self.assertEqual(arm.pause_epoch, 1)
        arm.cb_estop(NS(data=False))
        self.assertTrue(arm.stop_requested)

    def test_estop_blocks_new_motion_and_reset_commands(self):
        arm = ArmHarness()
        arm.estop_active = True
        for callback in (arm.cb_go_rest, arm.cb_pose_1, arm.cb_move_to_pose,
                         arm.cb_execute_planned_trajectory, arm.cb_plan_to_pose,
                         arm.cb_reset_moveit_control):
            result = callback(None, NS())
            self.assertFalse(result.success)
            self.assertIn('E-stop', result.message)
        arm.execute_client.send_goal_async.assert_not_called()

    def test_goal_submission_race_is_blocked(self):
        arm = ArmHarness()
        arm.motion_pause_epoch = 0
        arm.cb_estop(NS(data=True))
        arm.cb_estop(NS(data=False))
        with self.assertRaises(ArmMotionPaused):
            arm.send_execution_goal(NS())
        arm.execute_client.send_goal_async.assert_not_called()

    @patch('arm_controlling.arm_manager.rclpy.ok', return_value=True)
    def test_interrupted_goal_replans_same_endpoint_instead_of_replaying(self, _):
        arm = ArmHarness()
        original = trajectory(1.0)
        replanned = arm.plan_to_joint_positions.return_value
        handles = [Mock(accepted=True), Mock(accepted=True)]
        for handle in handles:
            handle.get_result_async.return_value = done(NS(result=NS(
                error_code=NS(val=MoveItErrorCodes.SUCCESS))))
        arm.execute_client.send_goal_async.side_effect = [done(handle) for handle in handles]
        interrupted = False

        def wait(future, **kwargs):
            nonlocal interrupted
            if kwargs.get('check_connection') and not interrupted:
                interrupted = True
                arm.cb_estop(NS(data=True))
                arm.cb_estop(NS(data=False))
                arm.check_arm_connection()
            return future.result()

        arm.wait_future.side_effect = wait
        self.assertTrue(arm.execute_trajectory(original))
        sent = arm.execute_client.send_goal_async.call_args_list
        self.assertEqual(len(sent), 2)
        self.assertIs(sent[0].args[0].trajectory, original)
        self.assertIs(sent[1].args[0].trajectory, replanned)
        arm.plan_to_joint_positions.assert_called_once_with({'joint_1': 1.0})
        handles[0].cancel_goal_async.assert_called()

    @patch('arm_controlling.arm_manager.rclpy.ok', return_value=True)
    def test_normal_stop_wins_over_paused_motion(self, _):
        arm = ArmHarness()
        arm.estop_active = True
        arm.stop_requested = True
        self.assertFalse(arm.execute_trajectory(trajectory(1.0)))
        arm.execute_client.send_goal_async.assert_not_called()

    @patch('arm_controlling.plant_view_scanner.rclpy.ok', return_value=True)
    def test_normal_stop_cancels_scan_before_rest_and_never_resumes_it(self, _):
        scan = ScanHarness()
        def rest(**kwargs):
            self.assertTrue(scan.scan_stopped)
            self.assertTrue(scan.stop_in_progress)
            scan.arm_stop_client.call_async.assert_called_once()
            return True, 'rest'
        scan.call_arm_go_rest.side_effect = rest
        result = scan.cb_stop_scan(None, NS())
        self.assertTrue(result.success)
        self.assertIn('returned to rest', result.message)
        self.assertEqual(scan.pub_auto_state_cmd.publish.call_args.args[0].data, 'stop')
        scan.cb_estop(NS(data=False))
        scan.cb_resume_scan(None, NS())
        self.assertTrue(scan.scan_stopped)
        self.assertFalse(scan.stop_in_progress)

    @patch('arm_controlling.plant_view_scanner.rclpy.ok', return_value=True)
    def test_normal_stop_waits_for_estop_release_before_rest(self, _):
        scan = ScanHarness()
        scan.estop_active = True
        def release(_seconds):
            scan.call_arm_go_rest.assert_not_called()
            scan.cb_estop(NS(data=False))
        with patch('arm_controlling.plant_view_scanner.time.sleep', side_effect=release):
            self.assertTrue(scan.cb_stop_scan(None, NS()).success)
        scan.call_arm_go_rest.assert_called_once()

    def test_failed_arm_stop_does_not_send_rest(self):
        scan = ScanHarness()
        scan.arm_stop_client.call_async.return_value = done(NS(success=False))
        self.assertFalse(scan.cb_stop_scan(None, NS()).success)
        scan.call_arm_go_rest.assert_not_called()

    def test_duplicate_stop_does_not_launch_another_rest(self):
        scan = ScanHarness()
        scan.stop_in_progress = True
        self.assertFalse(scan.cb_stop_scan(None, NS()).success)
        scan.arm_stop_client.call_async.assert_not_called()

    def test_scanner_estop_does_not_change_manual_pause_or_cancel_state(self):
        scan = ScanHarness()
        scan.scan_paused = True
        scan.cb_estop(NS(data=True))
        self.assertFalse(scan.scan_stopped)
        scan.cb_estop(NS(data=False))
        self.assertTrue(scan.scan_paused)

    def test_estop_defers_next_scan_stage_until_release(self):
        state = StateHarness()
        state.cb_estop(NS(data=True))
        state.cb_state_cmd(NS(data='individual_plant_scan'))
        state.set_state.assert_not_called()
        state.cb_estop(NS(data=False))
        state.set_state.assert_called_once_with('individual_plant_scan', reason='cmd')

    def test_stop_discards_deferred_stage_and_rejects_late_completion(self):
        state = StateHarness()
        state.cb_estop(NS(data=True))
        state.cb_state_cmd(NS(data='individual_plant_scan'))
        state.cb_state_cmd(NS(data='stop'))
        self.assertEqual(state.auto_state, 'idle')
        state.cb_estop(NS(data=False))
        state.cb_state_cmd(NS(data='bench_tracking_f'))
        state.set_state.assert_not_called()
        state.cb_new_scan(NS(data=[1, 11, 12]))
        state.cb_state_cmd(NS(data='bench_tracking_f'))
        state.set_state.assert_called_once_with('bench_tracking_f', reason='cmd')


if __name__ == '__main__':
    unittest.main()
