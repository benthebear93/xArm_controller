"""Verify motion behavior without hardware using mocks based on the real SDK API."""

import contextlib
import io
import unittest
from unittest.mock import create_autospec, patch

# Suppress only the SDK's creation of a log directory in the home directory on import.
# Use API signatures from the installed SDK and replace connections with mocks below.
with patch("os.makedirs"):
    from xarm.wrapper import XArmAPI

import cartesian_motion
import joint_motion


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.arm = create_autospec(XArmAPI, instance=True)
        self.arm.connected = True
        self.arm.axis = 7
        self.arm.version_number = (2, 7, 103)
        self.arm.world_offset = [0.0] * 6
        self.arm.tcp_offset = [0.0] * 6
        self.arm.get_err_warn_code.return_value = (0, [0, 0])
        self.arm.get_state.return_value = (0, 2)
        self.arm.get_cmdnum.return_value = (0, 0)
        self.joints = [10.0, -30.0, 20.0, 60.0, -10.0, 40.0, 90.0]
        self.pose = [350.0, -120.0, 280.0, 179.0, 15.0, -40.0]
        self.aligned_pose = self.pose[:3] + [180.0, 0.0, 0.0]
        self.ik_joints = [15.0, -20.0, 30.0, 75.0, -25.0, 45.0, 90.0]
        self.arm.get_servo_angle.return_value = (0, self.joints)
        self.arm.get_position.return_value = (0, self.pose)
        self.arm.get_inverse_kinematics.return_value = (0, self.ik_joints)
        self.arm.get_forward_kinematics.return_value = (0, self.aligned_pose)
        self.arm.is_joint_limit.return_value = (0, False)
        for name in ("motion_enable", "set_mode", "set_state", "set_servo_angle", "set_position", "clean_warn"):
            getattr(self.arm, name).return_value = 0
        stack = contextlib.ExitStack()
        self.addCleanup(stack.close)
        self.factory = stack.enter_context(patch("robot_common.create_arm", return_value=self.arm))
        stack.enter_context(patch("robot_common.time.sleep"))
        stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def run_example(self, module, *args):
        with patch("sys.argv", [module.__name__, *args]):
            module.main()

    def assert_no_enable_or_move(self):
        self.arm.motion_enable.assert_not_called()
        self.arm.set_mode.assert_not_called()
        self.arm.set_state.assert_not_called()
        self.arm.set_servo_angle.assert_not_called()
        self.arm.set_position.assert_not_called()

    def test_joint_motion_points_tcp_downward_at_current_xyz_using_ik(self):
        self.arm.get_position.side_effect = [(0, self.pose), (0, self.aligned_pose)]
        self.run_example(joint_motion)
        self.factory.assert_called_once_with("192.168.1.197")
        self.arm.get_inverse_kinematics.assert_called_once_with(
            [350.0, -120.0, 280.0, 180.0, 0.0, 0.0],
            input_is_radian=False, return_is_radian=False,
            limited=False, ref_angles=self.joints,
        )
        self.arm.set_servo_angle.assert_called_once_with(
            angle=self.ik_joints,
            speed=3.0, mvacc=10.0, relative=False, is_radian=False,
            wait=True, timeout=30,
        )
        self.arm.set_position.assert_not_called()
        self.arm.disconnect.assert_called_once()
        calls = [entry[0] for entry in self.arm.mock_calls]
        self.assertLess(calls.index("get_forward_kinematics"), calls.index("motion_enable"))

    def test_cartesian_moves_only_x_five_millimeters_and_preserves_orientation(self):
        self.run_example(cartesian_motion)
        self.arm.set_position.assert_called_once_with(
            x=355.0, y=-120.0, z=280.0, roll=179.0, pitch=15.0, yaw=-40.0,
            speed=5.0, mvacc=20.0, relative=False, is_radian=False,
            wait=True, timeout=30, radius=-1,
        )
        self.assertEqual(self.pose[0], 350.0)  # Preserve the original array returned by the SDK.
        self.arm.set_servo_angle.assert_not_called()
        self.arm.disconnect.assert_called_once()

    def test_both_dry_runs_only_read(self):
        for module in (joint_motion, cartesian_motion):
            with self.subTest(module=module.__name__):
                self.run_example(module, "--dry-run")
                self.assert_no_enable_or_move()
                self.arm.clean_warn.assert_not_called()

    def test_ip_override(self):
        self.run_example(joint_motion, "--dry-run", "--ip", "192.168.1.198")
        self.factory.assert_called_once_with("192.168.1.198")

    def test_fault_prevents_initialization(self):
        self.arm.get_err_warn_code.return_value = (0, [23, 0])
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()
        self.arm.disconnect.assert_called_once()

    def test_warning_14_requires_explicit_recovery(self):
        self.arm.get_err_warn_code.return_value = (0, [0, 14])
        with self.assertRaisesRegex(RuntimeError, "command has no solution"):
            self.run_example(joint_motion, "--dry-run")
        self.arm.clean_warn.assert_not_called()
        self.assert_no_enable_or_move()

    def test_warning_14_recovery_clears_once_and_only_previews(self):
        self.arm.get_err_warn_code.side_effect = [
            (0, [0, 14]), (0, [0, 0]), (0, [0, 0]), (0, [0, 0]),
        ]
        self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
        self.arm.clean_warn.assert_called_once_with()
        self.arm.get_inverse_kinematics.assert_called_once()
        self.assert_no_enable_or_move()
        self.arm.disconnect.assert_called_once()

    def test_warning_recovery_requires_dry_run(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                self.run_example(joint_motion, "--clear-warning-14")
        self.assertEqual(raised.exception.code, 2)
        self.factory.assert_not_called()

    def test_warning_recovery_never_clears_errors_or_other_warnings(self):
        for status in ([23, 14], [0, 11], [0, 12], [0, 13]):
            with self.subTest(status=status):
                self.arm.get_err_warn_code.return_value = (0, status)
                with self.assertRaises(RuntimeError):
                    self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
                self.arm.clean_warn.assert_not_called()
                self.arm.clean_error.assert_not_called()
                self.assert_no_enable_or_move()

    def test_warning_recovery_does_not_clear_while_busy(self):
        self.arm.get_err_warn_code.return_value = (0, [0, 14])
        for state, queued in ((1, 0), (3, 0), (2, 1)):
            with self.subTest(state=state, queued=queued):
                self.arm.get_state.return_value = (0, state)
                self.arm.get_cmdnum.return_value = (0, queued)
                with self.assertRaises(RuntimeError):
                    self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
                self.arm.clean_warn.assert_not_called()
                self.assert_no_enable_or_move()

    def test_persistent_warning_is_not_cleared_in_a_loop(self):
        self.arm.get_err_warn_code.return_value = (0, [0, 14])
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
        self.arm.clean_warn.assert_called_once_with()
        self.arm.get_inverse_kinematics.assert_not_called()
        self.assert_no_enable_or_move()

    def test_failed_warning_clear_aborts_without_motion(self):
        self.arm.get_err_warn_code.return_value = (0, [0, 14])
        self.arm.clean_warn.return_value = -1
        with self.assertRaisesRegex(RuntimeError, "Clear pre-existing"):
            self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
        self.arm.get_inverse_kinematics.assert_not_called()
        self.assert_no_enable_or_move()

    def test_ik_warning_is_rejected_even_when_sdk_returns_success(self):
        self.arm.get_err_warn_code.side_effect = [(0, [0, 0]), (0, [0, 14])]
        with self.assertRaisesRegex(RuntimeError, "controller found no solution"):
            self.run_example(joint_motion)
        self.arm.clean_warn.assert_not_called()
        self.arm.is_joint_limit.assert_not_called()
        self.assert_no_enable_or_move()

    def test_new_ik_warning_is_not_cleared_after_recovery(self):
        self.arm.get_err_warn_code.side_effect = [(0, [0, 14]), (0, [0, 0]), (0, [0, 14])]
        with self.assertRaisesRegex(RuntimeError, "controller found no solution"):
            self.run_example(joint_motion, "--dry-run", "--clear-warning-14")
        self.arm.clean_warn.assert_called_once_with()
        self.assert_no_enable_or_move()

    def test_controller_warning_after_fk_prevents_motor_enable(self):
        self.arm.get_err_warn_code.side_effect = [(0, [0, 0]), (0, [0, 0]), (0, [0, 14])]
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()

    def test_controller_error_before_motor_enable_prevents_motion(self):
        self.arm.get_err_warn_code.side_effect = [
            (0, [0, 0]), (0, [0, 0]), (0, [0, 0]), (0, [23, 0]),
        ]
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()

    def test_firmware_251_uses_legacy_ik_arguments(self):
        self.arm.version_number = (2, 5, 1)
        self.run_example(joint_motion, "--dry-run")
        self.arm.get_inverse_kinematics.assert_called_once_with(
            self.aligned_pose, input_is_radian=False, return_is_radian=False,
        )
        self.assert_no_enable_or_move()

    def test_busy_or_paused_robot_is_not_resumed(self):
        for state, queued in ((1, 0), (3, 0), (2, 1)):
            with self.subTest(state=state, queued=queued):
                self.arm.get_state.return_value = (0, state)
                self.arm.get_cmdnum.return_value = (0, queued)
                with self.assertRaises(RuntimeError):
                    self.run_example(joint_motion)
                self.assert_no_enable_or_move()

    def test_wrong_axis_count_prevents_initialization(self):
        self.arm.axis = 6
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()

    def test_failed_position_read_prevents_cartesian_move(self):
        self.arm.get_position.return_value = (-1, self.pose)
        with self.assertRaises(RuntimeError):
            self.run_example(cartesian_motion)
        self.arm.set_position.assert_not_called()
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_joint_motion_failure_requests_stop(self):
        self.arm.set_servo_angle.return_value = -1
        with self.assertRaises(RuntimeError):
            self.run_example(joint_motion)
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_cartesian_motion_timeout_requests_stop(self):
        self.arm.set_position.return_value = 100
        with self.assertRaises(RuntimeError):
            self.run_example(cartesian_motion)
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_keyboard_interrupt_requests_stop(self):
        self.arm.set_position.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.run_example(cartesian_motion)
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_connect_failure_still_disconnects(self):
        self.arm.connect.side_effect = OSError("connection failed")
        with self.assertRaises(OSError):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()
        self.arm.disconnect.assert_called_once()

    def test_ik_failure_prevents_motor_enable(self):
        self.arm.get_inverse_kinematics.return_value = (1, [])
        with self.assertRaisesRegex(RuntimeError, "inverse kinematics"):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()
        self.arm.disconnect.assert_called_once()

    def test_invalid_ik_values_prevent_motor_enable(self):
        for angles in ([0.0] * 6, [0.0] * 6 + [float("nan")], [float("inf")] * 7):
            with self.subTest(angles=angles):
                self.arm.get_inverse_kinematics.return_value = (0, angles)
                with self.assertRaisesRegex(RuntimeError, "finite"):
                    self.run_example(joint_motion)
                self.assert_no_enable_or_move()

    def test_joint_limit_failure_prevents_motor_enable(self):
        for result in ((0, True), (0, None), (-1, False)):
            with self.subTest(result=result):
                self.arm.is_joint_limit.return_value = result
                with self.assertRaises(RuntimeError):
                    self.run_example(joint_motion)
                self.assert_no_enable_or_move()

    def test_incorrect_fk_pose_prevents_motor_enable(self):
        for pose in (
            [360.0, -120.0, 280.0, 180.0, 0.0, 0.0],
            [350.0, -120.0, 280.0, 0.0, 0.0, 0.0],
            [350.0, -120.0, 280.0, float("nan"), 0.0, 0.0],
        ):
            with self.subTest(pose=pose):
                self.arm.get_forward_kinematics.return_value = (0, pose)
                with self.assertRaises(RuntimeError):
                    self.run_example(joint_motion)
                self.assert_no_enable_or_move()

    def test_rotated_world_frame_is_rejected_before_ik(self):
        self.arm.world_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 45.0]
        with self.assertRaisesRegex(RuntimeError, "world frame"):
            self.run_example(joint_motion)
        self.arm.get_inverse_kinematics.assert_not_called()
        self.assert_no_enable_or_move()

    def test_already_aligned_tcp_does_not_enable_motors(self):
        for rpy in ([180.0, 0.0, 0.0], [-180.0, 0.0, 0.0], [0.0, 180.0, 180.0]):
            with self.subTest(rpy=rpy):
                self.arm.get_position.return_value = (0, self.pose[:3] + rpy)
                self.run_example(joint_motion)
                self.arm.get_inverse_kinematics.assert_not_called()
                self.assert_no_enable_or_move()

    def test_upward_tcp_is_not_treated_as_already_aligned(self):
        self.arm.get_position.return_value = (0, self.pose[:3] + [0.0, 0.0, 0.0])
        self.run_example(joint_motion, "--dry-run")
        self.arm.get_inverse_kinematics.assert_called_once()
        self.assert_no_enable_or_move()

    def test_upward_final_orientation_is_rejected(self):
        self.arm.get_position.side_effect = [
            (0, self.pose), (0, self.pose[:3] + [0.0, 0.0, 0.0]),
        ]
        with self.assertRaisesRegex(RuntimeError, "Final TCP does not match"):
            self.run_example(joint_motion)
        self.arm.set_state.assert_called_with(4)

    def test_large_joint_branch_change_is_rejected(self):
        self.arm.get_inverse_kinematics.return_value = (0, [200.0] + self.joints[1:])
        with self.assertRaisesRegex(RuntimeError, "joint travel"):
            self.run_example(joint_motion)
        self.assert_no_enable_or_move()

    def test_motion_timeout_allows_slow_reorientation(self):
        self.arm.get_inverse_kinematics.return_value = (0, [190.0] + self.joints[1:])
        self.arm.get_position.side_effect = [(0, self.pose), (0, self.aligned_pose)]
        self.run_example(joint_motion)
        timeout = self.arm.set_servo_angle.call_args.kwargs["timeout"]
        self.assertGreater(timeout, 180.0 / 3.0)

    def test_stale_joint_plan_is_not_executed(self):
        moved = [12.0] + self.joints[1:]
        self.arm.get_servo_angle.side_effect = [(0, self.joints), (0, moved)]
        with self.assertRaisesRegex(RuntimeError, "moved during planning"):
            self.run_example(joint_motion)
        self.arm.set_servo_angle.assert_not_called()
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_wrong_final_tcp_pose_is_reported_as_failure(self):
        with self.assertRaisesRegex(RuntimeError, "Final TCP does not match"):
            self.run_example(joint_motion)
        self.arm.set_servo_angle.assert_called_once()
        self.arm.set_state.assert_called_with(4)
        self.arm.disconnect.assert_called_once()

    def test_orientation_error_uses_rotations_not_euler_component_differences(self):
        for rpy, expected in (
            ([0.0, 0.0, 0.0], 0.0),
            ([180.0, 180.0, 180.0], 0.0),
            ([0.0, 0.0, 360.0], 0.0),
            ([180.0, 0.0, 0.0], 180.0),
            ([0.0, 90.0, 0.0], 90.0),
        ):
            with self.subTest(rpy=rpy):
                self.assertAlmostEqual(joint_motion.orientation_error_deg(rpy), expected)

    def test_orientation_error_relative_to_downward_target(self):
        target = [180.0, 0.0, 0.0]
        for rpy, expected in (
            ([180.0, 0.0, 0.0], 0.0),
            ([-180.0, 0.0, 0.0], 0.0),
            ([0.0, 180.0, 180.0], 0.0),
            ([179.8, 0.0, 0.0], 0.2),
            ([180.0, 0.0, 30.0], 30.0),
            ([0.0, 0.0, 0.0], 180.0),
        ):
            with self.subTest(rpy=rpy):
                self.assertAlmostEqual(joint_motion.orientation_error_deg(rpy, target), expected)


if __name__ == "__main__":
    unittest.main()
