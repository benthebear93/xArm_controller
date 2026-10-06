"""Move the TCP linearly by 5 mm along base-frame +X while keeping its orientation."""

from robot_common import MOTION_TIMEOUT, check, parse_args, read_result, robot_session, run_cli

DELTA_X_MM = 5.0
SPEED_MM_S = 5.0
ACCEL_MM_S2 = 20.0


def main():
    args = parse_args(__doc__)
    with robot_session(args.ip, args.dry_run, clear_warning_14=args.clear_warning_14) as arm:
        current = read_result(arm.get_position(is_radian=False), "Read current TCP pose")
        if len(current) != 6:
            raise RuntimeError(f"Expected 6 TCP pose values: {current}")
        target = list(current)
        target[0] += DELTA_X_MM  # Change only x in mm; preserve y/z/roll/pitch/yaw.
        print(f"Current TCP [x,y,z,roll,pitch,yaw] (mm, deg): {current}")
        print(f"Target TCP [x,y,z,roll,pitch,yaw] (mm, deg): {target}")
        if args.dry_run:
            print("Dry run: no motion command was sent.")
            return

        # set_position plans a straight TCP path in the base coordinate frame.
        check(arm.set_position(
            x=target[0], y=target[1], z=target[2],
            roll=target[3], pitch=target[4], yaw=target[5],
            speed=SPEED_MM_S,
            mvacc=ACCEL_MM_S2,
            relative=False,
            is_radian=False,
            wait=True,
            timeout=MOTION_TIMEOUT,
            radius=-1,
        ), "Move TCP linearly")
        actual = read_result(arm.get_position(is_radian=False), "Read TCP pose after motion")
        print(f"Motion complete, TCP (mm, deg): {actual}")


if __name__ == "__main__":
    run_cli(main)
