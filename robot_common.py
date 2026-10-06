"""Connection management and error handling for the xArm7 examples."""

import argparse
from contextlib import contextmanager
import sys
import time

ROBOT_IP = "192.168.1.197"
MOTION_TIMEOUT = 30  # seconds


def parse_args(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--ip", default=ROBOT_IP, help="Robot IP address (default: %(default)s)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Connect and display current/target values without enabling motors or moving",
    )
    parser.add_argument(
        "--clear-warning-14", action="store_true",
        help="Clear an existing 'command has no solution' warning once; requires --dry-run",
    )
    args = parser.parse_args()
    if args.clear_warning_14 and not args.dry_run:
        parser.error("--clear-warning-14 requires --dry-run; validate the target without moving first")
    return args


def check(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation} failed: SDK return code {code}")


def read_result(result, operation):
    code, value = result
    check(code, operation)
    return value


def create_arm(ip):
    # Import lazily so --help works without loading the SDK or creating its log directory.
    from xarm.wrapper import XArmAPI

    return XArmAPI(ip, is_radian=False, do_not_open=True)


def status_message(error, warning):
    message = f"Check the robot status: error={error}, warning={warning}"
    if warning == 14:
        message += (
            "; warning 14 means 'command has no solution'. "
            "To clear a pre-existing warning and recheck without moving, "
            "use --dry-run --clear-warning-14. Clearing it does not make a target reachable."
        )
    return message


def require_clear_status(arm):
    error, warning = read_result(arm.get_err_warn_code(), "Read error and warning codes")
    if error or warning:
        raise RuntimeError(status_message(error, warning))


def require_idle(arm):
    state = read_result(arm.get_state(), "Read robot state")
    queued = read_result(arm.get_cmdnum(), "Read queued command count")
    if state not in (0, 2, 4) or queued != 0:
        raise RuntimeError(f"The robot must be idle: state={state}, queued={queued}")


@contextmanager
def position_motion(arm):
    """Enable position control and request a stop if motion or verification fails."""
    require_clear_status(arm)
    require_idle(arm)
    try:
        check(arm.motion_enable(enable=True), "Enable motors")
        check(arm.set_mode(0), "Set position control mode")
        check(arm.set_state(0), "Prepare for motion")
        time.sleep(0.5)
        yield arm
    except (Exception, KeyboardInterrupt):
        if arm.connected:
            # Disconnecting alone does not stop an active motion.
            try:
                check(arm.set_state(4), "Request stop")
            except Exception as stop_error:
                print(f"Stop request failed: {stop_error}", file=sys.stderr)
        raise


@contextmanager
def robot_session(ip, dry_run=False, clear_warning_14=False):
    if clear_warning_14 and not dry_run:
        raise ValueError("Warning recovery is allowed only in a dry-run session")
    arm = create_arm(ip)
    try:
        arm.connect()
        if not arm.connected:
            raise RuntimeError(f"Failed to connect to the robot: {ip}")
        time.sleep(0.5)  # Allow time to receive the first status report.
        if arm.axis != 7:
            raise RuntimeError(f"An xArm7 is required; the connected robot has {arm.axis} axes")
        error, warning = read_result(arm.get_err_warn_code(), "Read error and warning codes")
        if error or (warning and not (warning == 14 and clear_warning_14)):
            raise RuntimeError(status_message(error, warning))
        require_idle(arm)
        if warning == 14:
            # Recovery is explicit, limited to this warning, and never enables motion.
            check(arm.clean_warn(), "Clear pre-existing controller warning 14")
            require_clear_status(arm)
            require_idle(arm)
            print("Cleared pre-existing warning 14 once; rechecking the target without motion.")
        if dry_run:
            yield arm
        else:
            with position_motion(arm):
                yield arm
    finally:
        arm.disconnect()


def run_cli(main):
    try:
        main()
    except KeyboardInterrupt:
        print("Execution interrupted by the user.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
