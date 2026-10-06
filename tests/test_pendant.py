"""Pendant interlocks and worker behavior; never connect to a physical robot."""

import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import create_autospec, Mock, patch

with patch("os.makedirs"):
    from xarm.wrapper import XArmAPI

from pendant_controller import (
    COMMAND_DURATION, DemoArm, FEEDBACK_TIMEOUT, HOME_MODE, HOME_STEP_DEG, HOME_TOLERANCE_DEG,
    INPUT_LEASE, PendantWorker, RobotController, capture_pose, export_poses, rounded_home, velocity_vector,
)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.arm = create_autospec(XArmAPI, instance=True)
        self.arm.connected = True
        self.arm.axis = 7
        self.arm.mode = 0
        self.arm.version_number = (2, 5, 1)
        self.arm.tcp_offset = [0.0] * 6
        self.arm.world_offset = [0.0] * 6
        self.arm.get_servo_angle.return_value = (0, [0.0] * 7)
        self.arm.get_position.return_value = (0, [345, 125, 432, 180, 0, 0])
        self.arm.get_state.return_value = (0, 2)
        self.arm.get_cmdnum.return_value = (0, 0)
        self.arm.get_err_warn_code.return_value = (0, [0, 0])
        self.arm.is_joint_limit.return_value = (0, False)
        for name in ("motion_enable", "set_state", "set_joint_maxacc", "set_tcp_maxacc",
                     "vc_set_joint_velocity", "vc_set_cartesian_velocity", "set_servo_angle", "clean_warn"):
            getattr(self.arm, name).return_value = 0
        self.arm.set_mode.side_effect = self.set_mode
        self.clock = Mock(return_value=100.0)
        self.factory = Mock(return_value=self.arm)
        self.c = RobotController(self.factory, self.clock)

    def set_mode(self, mode):
        self.arm.mode = mode
        return 0

    def connect(self):
        self.c.connect("192.168.1.197")

    def enable(self, mode="Joint"):
        self.connect()
        self.c.enable(mode)

    def assert_no_velocity(self):
        self.arm.vc_set_joint_velocity.assert_not_called()
        self.arm.vc_set_cartesian_velocity.assert_not_called()

    def test_constructor_does_not_connect_or_import_sdk(self):
        self.factory.assert_not_called()

    def test_connect_and_disconnect_are_read_only(self):
        self.connect()
        self.c.disconnect()
        self.arm.motion_enable.assert_not_called()
        self.arm.set_mode.assert_not_called()
        self.arm.set_state.assert_not_called()
        self.assert_no_velocity()
        self.arm.disconnect.assert_called_once()

    def test_enable_firmware_251_does_not_send_motion(self):
        self.enable()
        self.assertTrue(self.c.enabled)
        self.arm.set_mode.assert_called_once_with(4)
        self.arm.set_joint_maxacc.assert_called_once_with(10, is_radian=False)
        self.assert_no_velocity()

    def test_all_joint_axes_have_bounded_single_axis_velocity(self):
        self.enable()
        for axis in range(7):
            self.c.jog("Joint", axis, -1, 3)
            vector = [0.0] * 7
            vector[axis] = -3.0
            self.arm.vc_set_joint_velocity.assert_called_with(
                vector, is_radian=False, is_sync=True, duration=COMMAND_DURATION)

    def test_cartesian_base_and_tool_axis_mapping(self):
        self.connect()
        for mode, tool in (("TCP Base", False), ("TCP Tool", True)):
            self.c.enable(mode)
            for axis in range(6):
                self.c.jog(mode, axis, 1, 5 if axis < 3 else 3)
                vector = [0.0] * 6
                vector[axis] = 5 if axis < 3 else 3
                self.arm.vc_set_cartesian_velocity.assert_called_with(
                    vector, is_radian=False, is_tool_coord=tool, duration=COMMAND_DURATION)
            self.c.stop()
        self.arm.set_tcp_maxacc.assert_called_with(20)

    def test_release_sends_zero_velocity_and_stop_disarms(self):
        self.enable()
        self.c.jog("Joint", 0, 1, 3)
        self.c.release()
        self.arm.vc_set_joint_velocity.assert_called_with(
            [0.0] * 7, is_radian=False, is_sync=True, duration=COMMAND_DURATION)
        self.assertTrue(self.c.enabled)
        self.assertFalse(self.c.active)
        self.c.stop()
        self.arm.set_state.assert_called_with(4)
        with self.assertRaises(RuntimeError):
            self.c.jog("Joint", 0, 1, 3)

    def test_malformed_and_excessive_speeds_are_rejected(self):
        self.enable()
        invalid = [("Joint", 0, 1, v) for v in (0, -1, 11, float("nan"), float("inf"))]
        invalid += [("TCP Base", 0, 1, 21), ("TCP Tool", 3, 1, 11),
                    ("Joint", 7, 1, 3), ("Joint", -1, 1, 3), ("Joint", 1.0, 1, 3),
                    ("Joint", 0, 0, 3), ("Invalid", 0, 1, 3)]
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.c.jog(*args)
        self.assert_no_velocity()

    def test_fault_prevents_enable_even_when_getter_return_code_is_zero(self):
        for status in ([23, 0], [0, 14], [0, 11]):
            self.arm.get_err_warn_code.return_value = (0, status)
            self.connect()
            with self.assertRaises(RuntimeError):
                self.c.enable("Joint")
            self.c.disconnect()
        self.arm.motion_enable.assert_not_called()
        self.arm.clean_error.assert_not_called()
        self.arm.clean_warn.assert_not_called()

    def test_busy_paused_or_queued_controller_is_not_enabled(self):
        self.connect()
        for state, queued in ((1, 0), (3, 0), (2, 1)):
            self.arm.get_state.return_value = (0, state)
            self.arm.get_cmdnum.return_value = (0, queued)
            with self.assertRaises(RuntimeError):
                self.c.enable("Joint")
        self.arm.motion_enable.assert_not_called()

    def test_old_firmware_is_rejected_before_motor_enable(self):
        self.arm.version_number = (1, 7, 9)
        self.connect()
        with self.assertRaisesRegex(RuntimeError, "1.8.0"):
            self.c.enable("Joint")
        self.arm.motion_enable.assert_not_called()

    def test_rotated_world_rejects_base_jog(self):
        self.arm.world_offset = [0, 0, 0, 0, 0, 30]
        self.connect()
        with self.assertRaisesRegex(RuntimeError, "zero world rotation"):
            self.c.enable("TCP Base")
        self.arm.motion_enable.assert_not_called()

    def test_stale_or_invalid_feedback_never_sends_velocity(self):
        self.enable()
        self.clock.return_value += FEEDBACK_TIMEOUT + 0.01
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.c.jog("Joint", 0, 1, 3)
        self.assert_no_velocity()
        self.arm.get_position.return_value = (0, [float("nan")] * 6)
        with self.assertRaises(ValueError):
            self.c.poll()

    def test_mode_changes_and_faults_reject_further_motion(self):
        self.enable()
        self.arm.mode = 0
        with self.assertRaisesRegex(RuntimeError, "mode changed"):
            self.c.poll()
        with self.assertRaises(RuntimeError):
            self.c.jog("Joint", 0, 1, 3)
        self.arm.mode = 4
        self.arm.get_err_warn_code.return_value = (0, [0, 14])
        with self.assertRaises(RuntimeError):
            self.c.poll()
        self.assert_no_velocity()

    def test_cancel_between_enable_steps_stops_without_preparing_motion(self):
        self.connect()
        permit = Mock(side_effect=[True, False])
        with self.assertRaisesRegex(RuntimeError, "canceled"):
            self.c.enable("Joint", permitted=permit)
        self.arm.set_mode.assert_not_called()
        self.arm.set_state.assert_called_once_with(4)
        self.assert_no_velocity()

    def test_canceled_jog_is_never_sent(self):
        self.enable()
        self.c.jog("Joint", 0, 1, 3, permitted=lambda: False)
        self.assert_no_velocity()

    def test_warning_14_recovery_is_explicit_once_and_idle_only(self):
        self.connect()
        self.arm.get_err_warn_code.side_effect = [(0, [0, 14]), (0, [0, 0])]
        self.c.clear_warning_14()
        self.arm.clean_warn.assert_called_once()
        self.arm.motion_enable.assert_not_called()
        self.arm.clean_error.assert_not_called()
        self.assert_no_velocity()

    def test_recovery_rejects_other_faults_and_active_robot(self):
        self.connect()
        for status, state in (([1, 14], 2), ([0, 11], 2), ([0, 14], 1)):
            self.arm.get_err_warn_code.return_value = (0, status)
            self.arm.get_state.return_value = (0, state)
            with self.assertRaises(RuntimeError):
                self.c.clear_warning_14()
        self.arm.clean_warn.assert_not_called()

    def test_stop_failure_still_disconnects(self):
        self.enable()
        self.arm.set_state.return_value = -1
        with self.assertRaises(RuntimeError):
            self.c.disconnect()
        self.arm.disconnect.assert_called_once()
        self.assertFalse(self.c.enabled)

    def test_failed_connection_cleans_up(self):
        self.arm.connected = False
        with self.assertRaises(RuntimeError):
            self.connect()
        self.arm.disconnect.assert_called_once()
        self.assertIsNone(self.c.arm)

    def prepare_home(self):
        self.arm.get_servo_angle.return_value = (0, [7.7, -23.2, 10, 49.3, 2.5, 68.7, 4.2])
        self.connect()
        self.c.set_home()

    def test_home_chooses_only_zero_or_positive_45_and_ties_select_zero(self):
        self.assertEqual(rounded_home([7.7, -23.2, 10, 49.3, 2.5, 68.7, 4.2]), [0, 0, 0, 45, 0, 45, 0])
        self.assertEqual(rounded_home([22.5, -22.5, 67.5, -67.5, 180, -180, 0]), [0, 0, 45, 0, 45, 0, 0])
        with self.assertRaises(ValueError):
            rounded_home([float("nan")] * 7)

    def test_set_home_only_reads_and_checks_limits(self):
        self.prepare_home()
        self.assertEqual(self.c.home_target, [0, 0, 0, 45, 0, 45, 0])
        self.arm.motion_enable.assert_not_called()
        self.arm.set_servo_angle.assert_not_called()
        self.arm.set_state.assert_not_called()

    def test_invalid_home_keeps_previous_target(self):
        self.prepare_home()
        previous = self.c.home_target[:]
        self.arm.is_joint_limit.return_value = (0, True)
        with self.assertRaisesRegex(RuntimeError, "joint limit"):
            self.c.set_home()
        self.assertEqual(self.c.home_target, previous)
        self.arm.motion_enable.assert_not_called()

    def test_home_limit_warning_even_with_success_code_blocks_movement(self):
        self.prepare_home()
        self.arm.get_err_warn_code.side_effect = [(0, [0, 0]), (0, [0, 14])]
        with self.assertRaises(RuntimeError):
            self.c.enable(HOME_MODE)
        self.arm.motion_enable.assert_not_called()
        self.arm.set_servo_angle.assert_not_called()

    def test_home_requires_a_target_and_blocks_large_travel(self):
        self.connect()
        with self.assertRaisesRegex(RuntimeError, "Set home"):
            self.c.enable(HOME_MODE)
        self.c.home_target = [225] + [0] * 6
        with self.assertRaisesRegex(RuntimeError, "180 degrees"):
            self.c.enable(HOME_MODE)
        self.arm.motion_enable.assert_not_called()

    def test_home_is_bounded_nonblocking_joint_position_motion(self):
        self.prepare_home()
        self.c.enable(HOME_MODE)
        self.arm.set_mode.assert_called_once_with(0)
        self.assertFalse(self.c.home_step(3))
        kwargs = self.arm.set_servo_angle.call_args.kwargs
        self.assertEqual({k: v for k, v in kwargs.items() if k != "angle"},
                         dict(speed=3.0, mvacc=10.0, relative=False, is_radian=False, wait=False, radius=-1))
        initial = self.c.feedback["joints"]
        self.assertAlmostEqual(max(abs(a - b) for a, b in zip(kwargs["angle"], initial)), HOME_STEP_DEG)
        for a, b, home in zip(kwargs["angle"], initial, self.c.home_target):
            self.assertLessEqual(abs(a - home), abs(b - home))
        self.assert_no_velocity()

    def test_home_never_queues_multiple_segments(self):
        self.prepare_home()
        self.c.enable(HOME_MODE)
        self.c.home_step(3)
        self.c.home_step(3)
        self.arm.set_servo_angle.assert_called_once()
        target = self.c.home_segment[:]
        self.arm.get_servo_angle.return_value = (0, target)
        self.arm.get_state.return_value = (0, 1)
        self.clock.return_value += 0.1
        self.c.poll()
        self.c.home_step(3)
        self.arm.set_servo_angle.assert_called_once()  # Reached angles still need an idle controller.
        self.arm.get_state.return_value = (0, 2)
        self.arm.get_cmdnum.return_value = (0, 1)
        self.c.poll()
        self.c.home_step(3)
        self.arm.set_servo_angle.assert_called_once()
        self.arm.get_cmdnum.return_value = (0, 0)
        self.c.poll()
        self.c.home_step(3)
        self.assertEqual(self.arm.set_servo_angle.call_count, 2)

    def test_release_stops_home_and_retains_original_target_for_resume(self):
        self.prepare_home()
        home = self.c.home_target[:]
        self.c.enable(HOME_MODE)
        self.c.home_step(3)
        self.c.release()
        self.arm.set_state.assert_called_with(4)
        self.assertFalse(self.c.enabled)
        self.assertIsNone(self.c.home_segment)
        self.assertEqual(self.c.home_target, home)
        self.c.enable(HOME_MODE)
        self.c.home_step(3)
        self.assertEqual(self.c.home_target, home)

    def test_home_completion_sends_no_more_joint_commands(self):
        self.prepare_home()
        self.arm.get_servo_angle.return_value = (0, self.c.home_target[:])
        self.c.enable(HOME_MODE)
        self.assertTrue(self.c.home_step(3))
        self.arm.set_servo_angle.assert_not_called()

    def test_cancel_before_segment_and_stale_feedback_block_home(self):
        self.prepare_home()
        self.c.enable(HOME_MODE)
        self.c.home_step(3, permitted=lambda: False)
        self.clock.return_value += FEEDBACK_TIMEOUT + 0.01
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.c.home_step(3)
        self.arm.set_servo_angle.assert_not_called()

    def test_disconnect_clears_home(self):
        self.prepare_home()
        self.c.disconnect()
        self.assertIsNone(self.c.home_target)


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.worker = PendantWorker(demo=True)
        self.addCleanup(self.close)

    def close(self):
        self.worker.stop(close=True)
        if self.worker.ident is not None:
            self.worker.join(2)
            self.assertFalse(self.worker.is_alive())

    def until(self, predicate, timeout=2):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.worker.lock:
                snapshot = dict(self.worker.snapshot)
            if predicate(snapshot):
                return snapshot
            time.sleep(0.01)
        self.fail(f"Worker did not reach expected state: {self.worker.view()}")

    def start_enabled(self):
        self.worker.start()
        self.worker.request("connect", "demo")
        self.until(lambda s: s["connected"])
        self.worker.request("enable", "Joint")
        return self.until(lambda s: s["enabled"])

    def test_stop_discards_pending_enable_and_old_holds(self):
        self.worker.request("connect", "demo")
        self.worker.request("enable", "Joint")
        self.worker.stop()
        self.worker.start()
        time.sleep(0.06)
        self.assertIsNone(self.worker.controller.arm)
        self.worker.hold(0, "Joint", 0, 1, 3)
        self.assertIsNone(self.worker.held)

    def test_missing_gui_heartbeat_expires_jog(self):
        snapshot = self.start_enabled()
        self.worker.hold(snapshot["generation"], "Joint", 0, 1, 3)
        self.until(lambda s: s["active"])
        result = self.until(lambda s: not s["active"], timeout=INPUT_LEASE + 1)
        self.assertFalse(result["enabled"])
        self.assertGreater(result["generation"], snapshot["generation"])
        after = self.worker.controller.arm.joints[0]
        self.assertGreater(after, 7.7)
        time.sleep(0.1)
        self.assertAlmostEqual(after, self.worker.controller.arm.get_servo_angle()[1][0])

    def test_fault_during_poll_disarms_worker_and_requests_stop(self):
        self.start_enabled()
        self.worker.controller.arm.get_err_warn_code = lambda: (0, [23, 0])
        self.until(lambda s: not s["enabled"])
        self.assertEqual(self.worker.controller.arm.state, 4)

    def test_link_loss_disarms_even_without_a_held_input(self):
        self.start_enabled()
        self.worker.controller.arm.connected = False
        result = self.until(lambda s: not s["enabled"])
        self.assertTrue(result["has_session"])
        self.assertFalse(result["connected"])

    def test_expired_heartbeat_cannot_be_revived_by_a_late_ui_tick(self):
        snapshot = self.start_enabled()
        self.worker.hold(snapshot["generation"], "Joint", 0, 1, 3)
        self.until(lambda s: s["active"])
        result = self.until(lambda s: not s["enabled"])
        self.worker.hold(snapshot["generation"], "Joint", 0, 1, 3)
        time.sleep(0.05)
        self.assertIsNone(self.worker.held)
        self.assertNotEqual(result["generation"], snapshot["generation"])

    def test_released_input_remains_enabled_but_never_moves_again_by_itself(self):
        snapshot = self.start_enabled()
        self.worker.hold(snapshot["generation"], "Joint", 0, 1, 3)
        self.until(lambda s: s["active"])
        self.worker.release()
        result = self.until(lambda s: not s["active"])
        self.assertTrue(result["enabled"])
        position = self.worker.controller.arm.get_servo_angle()[1][0]
        time.sleep(INPUT_LEASE + 0.05)
        self.assertAlmostEqual(position, self.worker.controller.arm.get_servo_angle()[1][0])

    def test_stop_cancels_enable_during_blocking_connect(self):
        entered, proceed = threading.Event(), threading.Event()
        arm = DemoArm("demo")
        original = arm.connect

        def connect(**kwargs):
            entered.set()
            proceed.wait(1)
            original(**kwargs)

        arm.connect = connect
        arm.motion_enable = Mock(return_value=0)
        self.worker.controller.factory = lambda ip: arm
        self.worker.start()
        self.worker.request("connect", "demo")
        self.assertTrue(entered.wait(1))
        self.worker.request("enable", "Joint")
        self.worker.stop()
        proceed.set()
        self.until(lambda s: s["connected"])
        time.sleep(0.05)
        arm.motion_enable.assert_not_called()
        self.assertFalse(self.worker.controller.enabled)

    def prepare_home(self):
        self.worker.start()
        self.worker.request("connect", "demo")
        self.until(lambda s: s["connected"])
        self.worker.request("set_home")
        return self.until(lambda s: s.get("home_target") is not None)

    def renew_home(self, generation, seconds=0.2):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.worker.hold(generation, HOME_MODE, 0, 1, 3)
            time.sleep(0.03)

    def test_home_release_stops_and_second_press_resumes_same_target(self):
        snapshot = self.prepare_home()
        target = snapshot["home_target"]
        self.renew_home(snapshot["generation"])
        self.until(lambda s: s["active"])
        self.worker.release()
        released = self.until(lambda s: not s["enabled"])
        self.assertEqual(self.worker.controller.arm.state, 4)
        stopped = self.worker.controller.arm.joints[:]
        time.sleep(0.1)
        self.assertEqual(stopped, self.worker.controller.arm.get_servo_angle()[1])
        self.assertEqual(released["home_target"], target)
        self.renew_home(released["generation"])
        self.until(lambda s: s["active"])
        self.assertEqual(self.worker.controller.home_target, target)
        self.assertLess(self.worker.controller.arm.joints[0], stopped[0])

    def test_home_heartbeat_expiry_disarms_without_losing_target(self):
        snapshot = self.prepare_home()
        self.worker.hold(snapshot["generation"], HOME_MODE, 0, 1, 3)
        self.until(lambda s: s["active"])
        stopped = self.until(lambda s: not s["enabled"])
        self.assertEqual(stopped["home_target"], snapshot["home_target"])
        self.assertEqual(self.worker.controller.arm.state, 4)

    def test_home_release_during_enable_prevents_joint_motion(self):
        snapshot = self.prepare_home()
        arm = self.worker.controller.arm
        entered, proceed = threading.Event(), threading.Event()

        def enable(**kwargs):
            entered.set()
            proceed.wait(1)
            return 0

        arm.motion_enable = enable
        arm.set_servo_angle = Mock(return_value=0)
        self.worker.hold(snapshot["generation"], HOME_MODE, 0, 1, 3)
        self.assertTrue(entered.wait(1))
        self.worker.release()
        proceed.set()
        self.until(lambda s: not s["enabled"] and s["generation"] != snapshot["generation"])
        arm.set_servo_angle.assert_not_called()
        self.assertEqual(arm.state, 4)

    def test_home_runs_multiple_segments_to_completion_in_demo(self):
        arm = DemoArm("demo")
        arm.joints = [1.1, 0.7, -0.6, 44.8, 1.0, 45.3, -0.8]
        arm.set_servo_angle = Mock(wraps=arm.set_servo_angle)
        self.worker.controller.factory = lambda ip: arm
        snapshot = self.prepare_home()
        target = [0, 0, 0, 45, 0, 45, 0]
        self.assertEqual(snapshot["home_target"], target)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.worker.hold(snapshot["generation"], HOME_MODE, 0, 1, 3)
            time.sleep(0.03)
            current, messages = self.worker.view()
            if any("Home reached" in message for message in messages):
                break
        else:
            self.fail(f"Demo home did not complete: {self.worker.view()}")
        self.assertGreaterEqual(arm.set_servo_angle.call_count, 3)
        self.assertLessEqual(max(abs(a - b) for a, b in zip(arm.joints, target)), HOME_TOLERANCE_DEG)
        self.assertEqual(current["home_target"], target)
        self.assertEqual(arm.state, 4)
        self.assertFalse(current["enabled"])


class PoseTests(unittest.TestCase):
    def snapshot(self):
        controller = RobotController(DemoArm)
        controller.connect("demo")
        return dict(controller.feedback, connected=True, active=False)

    def test_capture_and_export_preserve_units_offsets_and_demo_marker(self):
        snapshot = self.snapshot()
        pose = capture_pose(snapshot, "Contact start", "demo", demo=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.json"
            export_poses(path, [pose])
            result = json.loads(path.read_text())
        self.assertEqual(result["format"], "xarm7-taught-poses")
        self.assertTrue(result["poses"][0]["demo"])
        self.assertEqual(result["poses"][0]["tcp_mm_deg"], snapshot["tcp"])
        self.assertEqual(len(result["poses"][0]["joints_deg"]), 7)

    def test_capture_rejects_stale_moving_or_faulted_feedback(self):
        for updates in (dict(received=0), dict(active=True), dict(state=1),
                        dict(error=23), dict(warning=14), dict(connected=False)):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                capture_pose(dict(self.snapshot(), **updates), "Pose", "demo")


class KeyboardTests(unittest.TestCase):
    """Exercise real event handlers without requiring a display server."""

    def setUp(self):
        from teach_pendant import TeachPendant
        self.app = object.__new__(TeachPendant)
        self.app.root = Mock()
        self.app.worker = Mock()
        self.app.key_releases = {}
        self.app.keys_down = set()
        self.app.held = None
        self.app.snapshot = dict(enabled=True, jog_mode="Joint", generation=0)
        self.app.mode = Mock(get=lambda: "Joint")
        self.app.axis = Mock(get=lambda: 0)
        self.app.joint_speed = Mock(get=lambda: "3")
        self.app.editing = Mock(return_value=False)

    def key(self, name):
        self.app.key_press(SimpleNamespace(keysym=name))

    def test_repeat_does_not_restart_input_after_stop(self):
        self.key("Left")
        self.app.stop()
        self.key("Left")
        self.assertIsNone(self.app.held)
        self.app.worker.hold.assert_called_once()
        self.app.finish_key_up("Left")
        self.key("Left")
        self.assertIsNotNone(self.app.held)

    def test_opposite_key_does_not_latch_original_direction(self):
        self.key("Left")
        self.key("Right")
        self.app.finish_key_up("Left")
        self.key("Right")  # Auto-repeat is not a fresh press.
        self.assertIsNone(self.app.held)
        self.app.finish_key_up("Right")
        self.assertFalse(self.app.keys_down)

    def test_editing_a_field_does_not_jog(self):
        self.app.editing.return_value = True
        self.key("Right")
        self.app.worker.hold.assert_not_called()

    def test_focus_loss_disables_jogging(self):
        self.app.closing = False
        self.app.enabling = False
        self.app.has_control_focus = Mock(return_value=False)
        self.key("Right")
        self.app.check_focus()
        self.assertIsNone(self.app.held)
        self.app.worker.stop.assert_called_once()


class DesktopSmokeTests(unittest.TestCase):
    def test_demo_widgets_and_connection(self):
        import tkinter as tk
        from teach_pendant import TeachPendant
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Desktop display unavailable: {error}")
        app = None
        try:
            app = TeachPendant(root, demo=True)
            root.update_idletasks()
            self.assertEqual(len(app.jog_buttons), 14)
            self.assertFalse(app.snapshot["connected"])
            app.connect()
            deadline = time.monotonic() + 2
            while not app.snapshot["connected"] and time.monotonic() < deadline:
                root.update()
                time.sleep(0.01)
            self.assertTrue(app.snapshot["connected"])
            self.assertFalse(app.snapshot["enabled"])
            app.mode.set("TCP Base")
            app.mode_changed()
            root.update_idletasks()
            self.assertEqual(app.rows[6].winfo_manager(), "")
            app.mode.set("Joint")
            app.mode_changed()
            root.update_idletasks()
            self.assertEqual(app.rows[6].winfo_manager(), "pack")
            # A ttk popdown is built by Tcl and has no matching Python widget object.
            combo = next(w for w in self.walk_widgets(root) if w.winfo_class() == "TCombobox")
            root.tk.call("ttk::combobox::Post", str(combo))
            popup = str(root.tk.call("ttk::combobox::PopdownWindow", str(combo)))
            root.tk.call("focus", "-force", popup + ".f.l")
            root.update_idletasks()
            self.assertTrue(app.editing())
            self.assertFalse(app.has_control_focus())
            app.check_focus()
            app.refresh()
            root.tk.call("ttk::combobox::Unpost", str(combo))
        finally:
            if app:
                app.worker.stop(close=True)
                app.worker.join(2)
            root.destroy()

    def walk_widgets(self, widget):
        for child in widget.winfo_children():
            yield child
            yield from self.walk_widgets(child)


class FocusTests(unittest.TestCase):
    def setUp(self):
        from teach_pendant import TeachPendant
        self.app = object.__new__(TeachPendant)
        self.app.root = Mock()
        self.app.root._w = "."
        self.app.root.focus_get.side_effect = KeyError("popdown")
        self.app.root.focus_displayof.side_effect = KeyError("popdown")
        self.app.worker = Mock()
        self.app.closing = self.app.enabling = False
        self.app.held = (0, "Joint", 0, 1, 3, "mouse")
        self.app.snapshot = dict(enabled=True, jog_mode="Joint", generation=0)
        self.path = ".controls.mode.popdown.f.l"
        self.top = ".controls.mode.popdown"
        self.widget_class = "Listbox"

        def call(command, operation, *args):
            if command == "focus":
                return self.path
            if operation == "toplevel":
                return self.top
            if operation == "class":
                return self.widget_class
            raise AssertionError((command, operation, args))

        self.app.root.tk.call.side_effect = call

    def test_combobox_popdown_is_handled_without_resolving_python_widget(self):
        self.assertTrue(self.app.editing())
        self.assertFalse(self.app.has_control_focus())
        self.app.check_focus()
        self.assertIsNone(self.app.held)
        self.app.worker.stop.assert_called_once()
        self.app.root.focus_get.assert_not_called()
        self.app.root.focus_displayof.assert_not_called()

    def test_regular_button_in_main_window_keeps_control_focus(self):
        self.path, self.top, self.widget_class = ".controls.button", ".", "TButton"
        self.assertFalse(self.app.editing())
        self.assertTrue(self.app.has_control_focus())
        self.app.check_focus()
        self.app.worker.stop.assert_not_called()

    def test_no_focus_or_destroyed_tcl_widget_fails_closed(self):
        import tkinter as tk
        self.path = ""
        self.assertFalse(self.app.has_control_focus())
        self.assertTrue(self.app.editing())
        self.app.root.tk.call.side_effect = tk.TclError("bad window path name")
        self.assertFalse(self.app.has_control_focus())
        self.assertTrue(self.app.editing())
        self.app.check_focus()
        self.app.worker.stop.assert_called_once()

    def test_popdown_does_not_break_periodic_refresh(self):
        snapshot = dict(self.app.snapshot, connected=True, active=True, has_session=True,
                        received=time.monotonic(), home_target=None)
        self.app.worker.view.return_value = (snapshot, [])
        for name in ("connect_button", "ip_entry", "disconnect_button", "enable_button",
                     "set_home_button", "home_button", "home_text", "status", "telemetry",
                     "tcp_text", "offset_text"):
            setattr(self.app, name, Mock())
        self.app.jog_buttons, self.app.row_values = [], []
        self.app.refresh()
        self.assertIsNone(self.app.held)
        self.app.root.after.assert_called_once_with(40, self.app.refresh)


if __name__ == "__main__":
    unittest.main()
