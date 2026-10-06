"""Orient the pusher downward (TCP RPY = 180, 0, 0 deg) using a joint move."""

import math

from robot_common import (
    MOTION_TIMEOUT,
    check,
    parse_args,
    position_motion,
    read_result,
    require_clear_status,
    robot_session,
    run_cli,
)

TARGET_RPY_DEG = (180.0, 0.0, 0.0)
SPEED_DEG_S = 3.0
ACCEL_DEG_S2 = 10.0
POSITION_TOLERANCE_MM = 1.0
ORIENTATION_TOLERANCE_DEG = 1.0
MAX_JOINT_TRAVEL_DEG = 180.0


def vector(values, size, label):
    """Reject missing, malformed, or non-finite controller results."""
    if values is None or len(values) != size or not all(math.isfinite(v) for v in values):
        raise RuntimeError(f"Expected {size} finite {label} values: {values}")
    return list(values)


def rotation_matrix(rpy):
    """Return Rz(yaw) Ry(pitch) Rx(roll) as a flat row-major matrix."""
    roll, pitch, yaw = map(math.radians, rpy)
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def orientation_error_deg(rpy, target_rpy=(0.0, 0.0, 0.0)):
    """Shortest rotation angle between orientations, including equivalent Euler angles."""
    actual = rotation_matrix(rpy)
    target = rotation_matrix(target_rpy)
    # The Frobenius inner product equals trace(R_target.T @ R_actual).
    trace = sum(a * b for a, b in zip(actual, target))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def verify_pose(pose, target, label):
    pose = vector(pose, 6, label)
    position_error = math.dist(pose[:3], target[:3])
    rotation_error = orientation_error_deg(pose[3:], target[3:])
    print(
        f"{label} error: position={position_error:.3f} mm, "
        f"orientation={rotation_error:.3f} deg"
    )
    if position_error > POSITION_TOLERANCE_MM or rotation_error > ORIENTATION_TOLERANCE_DEG:
        raise RuntimeError(f"{label} does not match the requested TCP pose")


def solve_alignment_ik(arm, target_pose, current):
    options = dict(input_is_radian=False, return_is_radian=False)
    if tuple(arm.version_number) >= (2, 7, 103):
        options.update(limited=False, ref_angles=current)
    else:
        print("Firmware older than 2.7.103: using legacy IK without reference-angle support.")
    code, angles = arm.get_inverse_kinematics(target_pose, **options)
    # SDK 1.18.4 can normalize a controller warning to API code 0 for a getter.
    # Check the controller explicitly before trusting any returned joint values.
    error, warning = read_result(arm.get_err_warn_code(), "Read status after IK")
    if code != 0 or error or warning:
        detail = (
            f"Solve inverse kinematics failed: SDK return code {code}, "
            f"error={error}, warning={warning}; target TCP (mm, deg)={target_pose}. "
            "No motion command was sent."
        )
        if warning == 14:
            detail += (
                " The controller found no solution for this position and orientation. "
                "The pose may be unreachable or the solver may need a different starting posture. "
                "Review the target before retrying; the script will not clear this new warning."
            )
        raise RuntimeError(detail)
    return vector(angles, 7, "IK joint angle")


def main():
    args = parse_args(__doc__)
    # Plan and validate without enabling motors, including during a normal run.
    with robot_session(args.ip, dry_run=True, clear_warning_14=args.clear_warning_14) as arm:
        current = vector(read_result(
            arm.get_servo_angle(is_radian=False, is_real=True), "Read current joint angles"
        ), 7, "joint angle")
        current_pose = vector(read_result(
            arm.get_position(is_radian=False), "Read current TCP pose"
        ), 6, "TCP pose")
        world_offset = vector(arm.world_offset, 6, "world offset")
        print(f"Current J1-J7 (deg): {current}")
        print(f"Current TCP [x,y,z,roll,pitch,yaw] (mm, deg): {current_pose}")
        print(f"World offset (mm, deg): {world_offset}")
        print(f"Configured TCP offset (mm, deg): {arm.tcp_offset}")
        if orientation_error_deg(world_offset[3:]) > 0.001:
            raise RuntimeError(
                "This example requires a world frame with no rotation relative to the base. "
                "Review the configured world offset in Studio; offsets were not changed."
            )

        # Preserve the final TCP XYZ; joint interpolation does not preserve XYZ en route.
        target_pose = current_pose[:3] + list(TARGET_RPY_DEG)
        print(f"Target TCP [x,y,z,roll,pitch,yaw] (mm, deg): {target_pose}")
        if orientation_error_deg(current_pose[3:], target_pose[3:]) <= ORIENTATION_TOLERANCE_DEG:
            print("TCP already matches the downward target within tolerance; no motion command was sent.")
            return

        target = solve_alignment_ik(arm, target_pose, current)
        delta = [goal - start for goal, start in zip(target, current)]
        print(f"Target J1-J7 (deg): {target}")
        print(f"Joint changes (deg): {delta}")
        limited = read_result(arm.is_joint_limit(target, is_radian=False), "Check joint limits")
        if limited is not False:
            raise RuntimeError("The IK target exceeds joint limits or could not be validated")
        travel = max(abs(value) for value in delta)
        if travel > MAX_JOINT_TRAVEL_DEG:
            raise RuntimeError(
                f"The IK solution requires {travel:.1f} deg of joint travel, exceeding "
                f"{MAX_JOINT_TRAVEL_DEG:.1f} deg. Review the IK branch and starting posture."
            )
        predicted = read_result(arm.get_forward_kinematics(
            target, input_is_radian=False, return_is_radian=False,
        ), "Check target forward kinematics")
        require_clear_status(arm)
        verify_pose(predicted, target_pose, "Predicted TCP")

        # Reorientation may take longer than the original 3-degree example.
        timeout = max(
            MOTION_TIMEOUT, travel / SPEED_DEG_S + SPEED_DEG_S / ACCEL_DEG_S2 + 10.0
        )
        print(f"Joint speed: {SPEED_DEG_S} deg/s; acceleration: {ACCEL_DEG_S2} deg/s^2")
        print(f"Motion timeout: {timeout:.1f} s")
        print("Joint motion preserves TCP XYZ only at the endpoint; review the swept workspace.")
        if args.dry_run:
            print("Dry run: no motion command was sent.")
            return

        with position_motion(arm):
            # Refuse a stale plan if the arm was moved while the target was calculated.
            latest = vector(read_result(
                arm.get_servo_angle(is_radian=False, is_real=True), "Recheck current joints"
            ), 7, "joint angle")
            if max(abs(a - b) for a, b in zip(latest, current)) > 1.0:
                raise RuntimeError("The robot moved during planning; run the example again")
            check(arm.set_servo_angle(
                angle=target, speed=SPEED_DEG_S, mvacc=ACCEL_DEG_S2,
                relative=False, is_radian=False, wait=True, timeout=timeout,
            ), "Align TCP using joint motion")
            actual = read_result(
                arm.get_servo_angle(is_radian=False, is_real=True), "Read joint angles after motion"
            )
            actual_pose = read_result(
                arm.get_position(is_radian=False), "Read TCP pose after motion"
            )
            print(f"Final J1-J7 (deg): {actual}")
            print(f"Final TCP (mm, deg): {actual_pose}")
            verify_pose(actual_pose, target_pose, "Final TCP")
            print("Motion complete: TCP matches the downward pushing orientation.")


if __name__ == "__main__":
    run_cli(main)
