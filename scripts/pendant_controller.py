"""Single-owner robot worker and bounded velocity jogging for the desktop pendant."""

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from robot_common import ROBOT_IP, check, create_arm, read_result

MODES = {"Joint": 4, "TCP Base": 5, "TCP Tool": 5}
HOME_MODE = "Home"
HOME_STEP_DEG = 0.5  # At most one small, unblended position command in flight.
HOME_TOLERANCE_DEG = 0.05
COMMAND_DURATION = 0.2  # Controller-side timeout; never send indefinite velocity.
INPUT_LEASE = 0.15  # UI must renew a held input while its window has focus.
FEEDBACK_TIMEOUT = 0.4


def finite_vector(value, length, label):
    result = [float(v) for v in value]
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f"Invalid {label}")
    return result


def velocity_vector(mode, axis, direction, speed):
    if mode not in MODES:
        raise ValueError("Select a valid jog mode")
    size = 7 if mode == "Joint" else 6
    if type(axis) is not int or not 0 <= axis < size or direction not in (-1, 1):
        raise ValueError("Invalid jog axis or direction")
    speed = float(speed)
    maximum = 20.0 if mode != "Joint" and axis < 3 else 10.0
    if not math.isfinite(speed) or not 0 < speed <= maximum:
        raise ValueError(f"Jog speed must be greater than zero and at most {maximum}")
    vector = [0.0] * size
    vector[axis] = direction * speed
    return vector


def rounded_home(joints):
    """Choose 0 or +45 degrees for each joint; an exact 22.5-degree tie selects zero."""
    return [0.0 if angle <= 22.5 else 45.0
            for angle in finite_vector(joints, 7, "home source angles")]


class RobotController:
    """Used only by the worker thread. Construction never connects to hardware."""

    def __init__(self, factory=create_arm, clock=time.monotonic):
        self.factory, self.clock = factory, clock
        self.arm = None
        self.ip = ROBOT_IP
        self.enabled = False
        self.active = False
        self.mode = "Joint"
        self.owns_control = False
        self.feedback = {}
        self.home_target = None
        self.home_segment = None
        self.home_segment_sent = 0.0
        self.home_segment_deadline = 0.0

    @property
    def connected(self):
        return self.arm is not None and self.arm.connected

    def connect(self, ip):
        if self.arm is not None:
            raise RuntimeError("Disconnect before connecting again")
        self.ip = ip
        self.arm = self.factory(ip)
        try:
            self.arm.connect(timeout=1)
            if not self.connected:
                raise RuntimeError(f"Could not connect to {ip}")
            self.arm.set_timeout(0.5)  # Returns the timeout, not an SDK status code.
            if self.arm.axis != 7:
                raise RuntimeError("This pendant requires an xArm7")
            self.poll()
        except Exception:
            self.arm.disconnect()
            self.arm = None
            self.feedback = {}
            raise

    def poll(self):
        if not self.connected:
            raise RuntimeError("Robot is disconnected")
        started = self.clock()
        joints = finite_vector(read_result(self.arm.get_servo_angle(is_radian=False),
                                          "Read joints"), 7, "joint feedback")
        tcp = finite_vector(read_result(self.arm.get_position(is_radian=False),
                                       "Read TCP"), 6, "TCP feedback")
        state = read_result(self.arm.get_state(), "Read state")
        queued = read_result(self.arm.get_cmdnum(), "Read queue")
        error, warning = read_result(self.arm.get_err_warn_code(), "Read faults")
        self.feedback = dict(joints=joints, tcp=tcp, state=state, queued=queued,
                             error=error, warning=warning, robot_mode=self.arm.mode,
                             firmware=".".join(map(str, self.arm.version_number)),
                             tcp_offset=finite_vector(self.arm.tcp_offset, 6, "TCP offset"),
                             world_offset=finite_vector(self.arm.world_offset, 6, "world offset"),
                             received=started)
        if self.enabled:
            self.require_ready()
        return self.feedback

    def require_fresh(self):
        if not self.connected or not self.feedback:
            raise RuntimeError("Connect to the robot first")
        if self.clock() - self.feedback["received"] > FEEDBACK_TIMEOUT:
            raise RuntimeError("Robot feedback is stale; jogging is disabled")

    def require_clear(self):
        self.require_fresh()
        f = self.feedback
        if f["error"] or f["warning"]:
            raise RuntimeError(f"Controller error={f['error']}, warning={f['warning']}")

    def require_idle(self):
        if self.feedback["state"] not in (0, 2, 4) or self.feedback["queued"]:
            raise RuntimeError("Robot must be idle with an empty command queue")

    def require_ready(self):
        self.require_clear()
        if self.feedback["state"] not in (0, 1, 2):
            raise RuntimeError("Robot stopped or paused; enable jogging again after inspection")
        expected_mode = 0 if self.mode == HOME_MODE else MODES[self.mode]
        if self.feedback["robot_mode"] != expected_mode:
            raise RuntimeError("Robot control mode changed; jogging is disabled")

    def check_home_limits(self, target):
        limited = read_result(self.arm.is_joint_limit(target, is_radian=False), "Check home joint limits")
        # Getters can return success even when a fresh controller warning was raised.
        self.poll()
        self.require_clear()
        self.require_idle()
        if limited:
            raise RuntimeError("Rounded home exceeds a joint limit; home was not changed")

    def set_home(self):
        if self.active:
            raise RuntimeError("Release motion controls and wait until idle before setting home")
        self.poll()
        self.require_clear()
        self.require_idle()
        target = rounded_home(self.feedback["joints"])
        self.check_home_limits(target)
        self.home_target = target
        return target[:]

    def enable(self, mode, permitted=lambda: True):
        if mode not in MODES and mode != HOME_MODE:
            raise ValueError("Select a valid jog mode")
        if self.enabled:
            raise RuntimeError("Stop before enabling another jog session")
        self.poll()
        self.require_clear()
        self.require_idle()
        if mode != HOME_MODE and tuple(self.arm.version_number) < (1, 8, 0):
            raise RuntimeError("Timed velocity control requires firmware 1.8.0 or newer")
        if mode == HOME_MODE:
            if self.home_target is None:
                raise RuntimeError("Set home from the current angles before homing")
            self.check_home_limits(self.home_target)
            if max(abs(a - b) for a, b in zip(self.home_target, self.feedback["joints"])) > 180:
                raise RuntimeError("Home requires more than 180 degrees of joint travel; review the target")
        # Base jogging is only offered when base and configured world axes agree.
        if mode == "TCP Base" and any(abs(v) > 0.001 for v in self.feedback["world_offset"][3:]):
            raise RuntimeError("TCP Base jogging requires zero world rotation; inspect controller offsets")
        self.mode = mode
        robot_mode = 0 if mode == HOME_MODE else MODES[mode]
        operations = [
            (lambda: self.arm.motion_enable(enable=True), "Enable motors"),
            (lambda: self.arm.set_mode(robot_mode), "Set control mode"),
            ((lambda: self.arm.set_joint_maxacc(10, is_radian=False)) if mode in ("Joint", HOME_MODE)
             else (lambda: self.arm.set_tcp_maxacc(20)), "Set jog acceleration"),
            (lambda: self.arm.set_state(0), "Prepare jogging"),
        ]
        try:
            for operation, label in operations:
                if not permitted():
                    raise RuntimeError("Enable request canceled")
                self.owns_control = True
                check(operation(), label)
            # set_mode() does not update the SDK's cached mode; wait for its report.
            deadline = self.clock() + 1.0
            while self.arm.mode != robot_mode:
                if not permitted() or self.clock() >= deadline:
                    raise RuntimeError("Control mode was not confirmed")
                time.sleep(0.02)
            if not permitted():
                raise RuntimeError("Enable request canceled")
            self.enabled = True
            self.poll()
        except Exception:
            self.stop()
            raise

    def send_velocity(self, vector):
        if self.mode == "Joint":
            check(self.arm.vc_set_joint_velocity(vector, is_radian=False, is_sync=True,
                                                duration=COMMAND_DURATION), "Joint velocity")
        else:
            check(self.arm.vc_set_cartesian_velocity(
                vector, is_radian=False, is_tool_coord=self.mode == "TCP Tool",
                duration=COMMAND_DURATION), "TCP velocity")

    def jog(self, mode, axis, direction, speed, permitted=lambda: True):
        vector = velocity_vector(mode, axis, direction, speed)
        if not self.enabled or mode != self.mode:
            raise RuntimeError("Enable the selected jog mode first")
        self.require_ready()
        if permitted():
            self.active = True  # Also stop if the command raises after reaching the robot.
            self.send_velocity(vector)

    def release(self):
        if self.mode == HOME_MODE:
            self.stop()
            return
        if self.active:
            self.send_velocity([0.0] * (7 if self.mode == "Joint" else 6))
            self.active = False

    def home_step(self, speed, permitted=lambda: True):
        """Advance one bounded joint-position segment. Return True when home is reached."""
        velocity_vector("Joint", 0, 1, speed)
        if not self.enabled or self.mode != HOME_MODE or self.home_target is None:
            raise RuntimeError("Home control is not enabled")
        self.require_ready()
        current = self.feedback["joints"]
        idle = self.feedback["state"] in (0, 2) and self.feedback["queued"] == 0
        if self.home_segment is not None:
            reached = max(abs(a - b) for a, b in zip(self.home_segment, current)) <= HOME_TOLERANCE_DEG
            if idle and reached and self.feedback["received"] > self.home_segment_sent:
                self.home_segment = None
            elif self.clock() > self.home_segment_deadline:
                raise RuntimeError("Home segment did not finish; inspect the robot before retrying")
            else:
                return False  # Never queue another segment behind an unfinished one.
        if not idle:
            return False
        delta = [a - b for a, b in zip(self.home_target, current)]
        distance = max(map(abs, delta))
        if distance <= HOME_TOLERANCE_DEG:
            return True
        scale = min(1.0, HOME_STEP_DEG / distance)
        target = [a + scale * d for a, d in zip(current, delta)]
        if permitted():
            self.active = True
            self.home_segment = target
            self.home_segment_sent = self.clock()
            self.home_segment_deadline = self.clock() + min(distance, HOME_STEP_DEG) / float(speed) + 5.0
            check(self.arm.set_servo_angle(angle=target, speed=float(speed), mvacc=10.0,
                                          relative=False, is_radian=False, wait=False, radius=-1),
                  "Home joint segment")
        return False

    def stop(self, force=False):
        self.enabled = self.active = False
        self.home_segment = None
        if self.connected and (self.owns_control or force):
            check(self.arm.set_state(4), "Software stop")
            self.owns_control = False

    def disconnect(self):
        try:
            self.stop()
        finally:
            if self.arm is not None:
                self.arm.disconnect()
            self.arm = None
            self.feedback = {}
            self.owns_control = False
            self.home_target = None

    def clear_warning_14(self):
        if self.enabled:
            raise RuntimeError("Stop jogging before clearing warning 14")
        self.poll()
        self.require_fresh()
        self.require_idle()
        if self.feedback["error"] or self.feedback["warning"] != 14:
            raise RuntimeError("Only warning 14 without a controller error can be cleared")
        check(self.arm.clean_warn(), "Clear existing warning 14 once")
        self.poll()
        self.require_clear()


class PendantWorker(threading.Thread):
    """Own all SDK calls; prioritize stops and discard obsolete input generations."""

    def __init__(self, demo=False):
        super().__init__(name="xarm-pendant", daemon=True)
        self.controller = RobotController(factory=DemoArm if demo else create_arm)
        self.demo = demo
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.commands = deque()
        self.messages = deque(maxlen=100)
        self.generation = 0
        self.held = None
        self.stop_pending = False
        self.closing = False
        self.snapshot = {"connected": False, "has_session": False, "enabled": False,
                         "active": False, "generation": 0}

    def request(self, action, *args):
        with self.lock:
            self.commands.append((self.generation, action, args))
        self.wake.set()

    def stop(self, close=False, force=True):
        with self.lock:
            self.generation += 1
            self.held = None
            self.commands.clear()
            self.stop_pending = "force" if (force and not close) or self.stop_pending == "force" else "owned"
            self.closing = self.closing or close
        self.wake.set()

    def hold(self, generation, mode, axis, direction, speed):
        with self.lock:
            home_ready = (mode == HOME_MODE and self.snapshot.get("home_target") is not None
                          and self.snapshot.get("connected"))
            if (generation != self.generation or not (self.snapshot["enabled"] or home_ready)
                    or self.stop_pending or self.closing):
                return
            now = time.monotonic()
            if self.held and now >= self.held[-1]:
                # A recovered GUI must not revive an expired input without re-enabling.
                self.generation += 1
                self.held = None
                self.commands.clear()
                self.stop_pending = "owned"
            else:
                self.held = (generation, mode, axis, direction, speed, now + INPUT_LEASE)
        self.wake.set()

    def release(self):
        with self.lock:
            if self.held and self.held[1] == HOME_MODE:
                # Cancel even a home enable that is currently blocked in an SDK call.
                self.generation += 1
                self.commands.clear()
                self.stop_pending = self.stop_pending or "owned"
            self.held = None
        self.wake.set()

    def view(self):
        with self.lock:
            result = deepcopy(self.snapshot)
            messages = list(self.messages)
            self.messages.clear()
        return result, messages

    def permitted(self, generation, held=None):
        with self.lock:
            return (generation == self.generation and not self.stop_pending and not self.closing
                    and (held is None or (self.held is not None and self.held[:5] == held[:5]
                                          and time.monotonic() < self.held[-1])))

    def publish(self):
        c = self.controller
        with self.lock:
            self.snapshot = dict(deepcopy(c.feedback), connected=c.connected, enabled=c.enabled,
                                 has_session=c.arm is not None, active=c.active,
                                 jog_mode=c.mode, home_target=deepcopy(c.home_target),
                                 generation=self.generation)

    def log(self, message):
        with self.lock:
            self.messages.append(message)

    def run(self):
        c = self.controller
        next_poll = 0.0
        next_send = 0.0
        try:
            while True:
                self.wake.wait(0.02)
                self.wake.clear()
                with self.lock:
                    stop, closing = self.stop_pending, self.closing
                    self.stop_pending = False
                    command = self.commands.popleft() if self.commands and not stop else None
                    held = self.held
                try:
                    if stop:
                        c.stop(force=stop == "force")
                        self.log("Software stop requested. Jogging disabled.")
                    if closing:
                        break
                    if command:
                        generation, action, args = command
                        if self.permitted(generation):
                            if action == "enable":
                                c.enable(*args, permitted=lambda: self.permitted(generation))
                                self.log("Jogging enabled. Hold a jog control to move.")
                            elif action == "set_home":
                                target = c.set_home()
                                self.log(f"Home J1-J7 (deg): {target}. Review the swept workspace before homing.")
                            elif action in ("connect", "disconnect", "clear_warning_14"):
                                getattr(c, action)(*args)
                                self.log(f"{action.replace('_', ' ').capitalize()} completed.")
                            else:
                                raise ValueError("Unknown worker action")
                    # Recheck the latest lease after blocking SDK operations.
                    with self.lock:
                        held = self.held
                        expired = held is not None and time.monotonic() >= held[-1]
                        if expired:
                            self.held = None
                            self.commands.clear()
                            self.generation += 1
                    if expired:
                        c.stop()
                        self.log("Held input expired. Enable jogging again before moving.")
                    elif c.active and (held is None or not self.permitted(held[0], held)):
                        c.release()
                    if c.enabled and not c.connected:
                        raise RuntimeError("Connection lost; jogging is disabled")
                    if c.connected and time.monotonic() >= next_poll:
                        c.poll()
                        next_poll = time.monotonic() + 0.1
                    if held and held[1] == HOME_MODE and self.permitted(held[0], held):
                        velocity_vector("Joint", 0, 1, held[4])
                        permit_home = lambda: self.permitted(held[0], held)
                        if not c.enabled or c.mode != HOME_MODE:
                            if c.active:
                                raise RuntimeError("Release jogging before starting home")
                            c.stop()
                            c.enable(HOME_MODE, permitted=permit_home)
                        if c.home_step(held[4], permitted=permit_home):
                            c.stop()
                            with self.lock:
                                self.held = None
                                self.generation += 1
                            self.log("Home reached (within 0.05 deg). Motion disabled.")
                    elif held and held[1] != HOME_MODE and c.enabled and time.monotonic() >= next_send:
                        c.jog(*held[1:5], permitted=lambda: self.permitted(held[0], held))
                        next_send = time.monotonic() + 0.05
                except Exception as error:
                    self.log(str(error))
                    with self.lock:
                        self.generation += 1
                        self.held = None
                        self.commands.clear()
                    try:
                        c.stop()
                    except Exception as stop_error:
                        self.log(f"Stop could not be confirmed: {stop_error}. Use the physical E-stop.")
                    next_poll = time.monotonic() + 0.5
                self.publish()
        finally:
            try:
                c.disconnect()
            except Exception as error:
                self.log(f"Disconnect: {error}")
            self.publish()


def capture_pose(snapshot, name, ip, demo=False):
    if not name.strip():
        raise ValueError("Enter a pose name")
    if (not snapshot.get("connected") or snapshot.get("active")
            or time.monotonic() - snapshot.get("received", 0) > FEEDBACK_TIMEOUT):
        raise ValueError("Pose capture requires fresh feedback and released jog controls")
    if snapshot["state"] not in (0, 2, 4) or snapshot["queued"] or snapshot["error"] or snapshot["warning"]:
        raise ValueError("Pose capture requires an idle robot without faults")
    return dict(name=name.strip(), captured_at=datetime.now(timezone.utc).isoformat(),
                robot_ip=ip, demo=demo, joints_deg=list(snapshot["joints"]),
                tcp_mm_deg=list(snapshot["tcp"]), tcp_offset_mm_deg=list(snapshot["tcp_offset"]),
                world_offset_mm_deg=list(snapshot["world_offset"]))


def export_poses(path, poses):
    data = {"format": "xarm7-taught-poses", "version": 1, "poses": poses}
    # Serialize before opening the destination; reject NaN/Infinity in an export.
    content = json.dumps(data, indent=2, allow_nan=False) + "\n"
    Path(path).write_text(content, encoding="utf-8")


class DemoArm:
    """Local UI exercise only: independent joint/TCP values, no FK or collision model."""

    axis = 7
    version_number = (2, 5, 1)

    def __init__(self, ip):
        self.connected = False
        self.mode, self.state = 0, 2
        self.tcp_offset, self.world_offset = [0.0] * 6, [0.0] * 6
        self.joints = [7.7, -23.2, 10.0, 49.3, 2.5, 68.7, 4.2]
        self.tcp = [345.3, 124.6, 431.6, 180.0, 0.0, 0.0]
        self.velocity, self.deadline, self.last = [], 0.0, time.monotonic()
        self.tool = False
        self.joint_goal = None
        self.joint_speed = 3.0

    def connect(self, **kwargs):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def set_timeout(self, timeout):
        return timeout

    def update(self):
        now = time.monotonic()
        dt = max(0.0, min(now, self.deadline) - self.last)
        if self.mode == 0 and self.joint_goal is not None and self.state != 4:
            delta = [a - b for a, b in zip(self.joint_goal, self.joints)]
            distance = max(map(abs, delta))
            scale = min(1.0, self.joint_speed * (now - self.last) / distance) if distance else 1.0
            self.joints = [a + scale * d for a, d in zip(self.joints, delta)]
            self.state = 2 if scale == 1.0 else 1
            if scale == 1.0:
                self.joint_goal = None
        if self.velocity and self.state != 4:
            values = self.joints if self.mode == 4 else self.tcp
            velocity = list(self.velocity)
            if self.tool and self.mode == 5:
                # Rotate translation from current tool axes into the base frame.
                r, p, y = map(math.radians, self.tcp[3:])
                cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
                rotation = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                            (-sp, cp * sr, cp * cr))
                velocity[:3] = [sum(a * b for a, b in zip(row, velocity[:3])) for row in rotation]
            # Demo angular readouts are illustrative; this is not a dynamics/kinematics model.
            for i, speed in enumerate(velocity):
                values[i] += speed * dt
            self.state = 1 if now < self.deadline and any(velocity) else 2
        self.last = now

    def get_servo_angle(self, **kwargs):
        self.update()
        return 0, self.joints[:]

    def get_position(self, **kwargs):
        self.update()
        return 0, self.tcp[:]

    def get_state(self):
        self.update()
        return 0, self.state

    def get_cmdnum(self):
        return 0, 0

    def get_err_warn_code(self):
        return 0, [0, 0]

    def motion_enable(self, **kwargs):
        return 0

    def set_mode(self, mode):
        self.mode = mode
        return 0

    def set_state(self, state):
        self.update()
        self.state = state
        if state == 4:
            self.velocity = []
            self.joint_goal = None
        return 0

    def set_joint_maxacc(self, *args, **kwargs):
        return 0

    def set_tcp_maxacc(self, *args, **kwargs):
        return 0

    def is_joint_limit(self, joint, **kwargs):
        return 0, False  # Demo has no physical joint-limit model.

    def set_servo_angle(self, *, angle, speed, **kwargs):
        self.update()
        self.velocity = []
        self.joint_goal, self.joint_speed = list(angle), speed
        self.state = 1
        return 0

    def vc_set_joint_velocity(self, speeds, *, duration, **kwargs):
        self.update()
        self.velocity, self.deadline = list(speeds), time.monotonic() + duration
        return 0

    def vc_set_cartesian_velocity(self, speeds, *, duration, is_tool_coord, **kwargs):
        self.tool = is_tool_coord
        return self.vc_set_joint_velocity(speeds, duration=duration)
