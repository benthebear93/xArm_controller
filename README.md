# xArm7 Python Control Examples

Robot IP: **192.168.1.197**. These examples use the official [UFACTORY xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

| File | Behavior |
| --- | --- |
| `joint_motion.py` | Use inverse kinematics and a joint move to **point the pusher downward at TCP RPY [180, 0, 0] degrees**, keeping the current TCP XYZ at the endpoint |
| `cartesian_motion.py` | Move the TCP linearly by **5 mm along base-frame +X**, preserving Y/Z and orientation |
| `robot_common.py` | Connection management, position control initialization, SDK return code checks, and cleanup |
| `teach_pendant.py` | Tkinter desktop pendant: hold-to-jog/home controls, feedback, and taught-pose export |
| `pendant_controller.py` | SDK worker, timed velocity jogging, bounded joint-position homing, and offline demo |
| `realsense_viewer.py` | Show the RealSense RGB feed at 640 × 480, 30 FPS |
| `vendor/xarm_python_sdk-1.18.4-py3-none-any.whl` | SDK wheel downloaded from the official PyPI release, including its license |

Targets are calculated from the current position. Repeating the Cartesian example adds another 5 mm of displacement; the joint example aligns orientation and skips motion if already aligned within tolerance.
The examples do not move to a home position or return to the starting position.

## Desktop Teach Pendant

The pendant uses Python's Tkinter and the existing SDK. Run it from a graphical
desktop terminal. It starts **disconnected**, and connecting only reads robot
feedback. The default robot address is `192.168.1.197`.

```bash
# Exercise the UI without connecting to any robot
.venv/bin/python teach_pendant.py --demo

# Open the real-robot pendant; connection and jogging require separate UI actions
.venv/bin/python teach_pendant.py --ip 192.168.1.197
```

Tkinter is available in this workspace's `.venv`; no additional Python packages
are required. For another Python installation, check with `python -m tkinter`.
On Ubuntu with system Python, a missing Tkinter module can be installed using
`sudo apt install python3-tk`. A `couldn't connect to display` message means the
process needs a working graphical desktop session.

1. Start with `--demo` and click **Connect** to practice the controls.
2. For the real robot, stop other robot controllers, inspect tool/payload/TCP
   configuration, clear the swept workspace, and keep the physical E-stop accessible.
3. Click **Connect** and inspect the joint/TCP feedback, offsets, and fault codes.
4. Select **Joint**, **TCP Base**, or **TCP Tool**, then click **Enable Jog**.
   This enables motors, selects velocity control, and sets acceleration; it does
   not send a nonzero velocity until a jog control is pressed.
5. Hold an axis's **− / +** button, or select an axis and hold **Left / Right**.
   Release to send zero velocity. Only one axis can be jogged at a time.
6. **STOP**, **Esc**, or **Space** requests controller state 4 and disables
   jogging. After inspecting the robot, explicitly click **Enable Jog** to resume.

The STOP button stays visible when the rest of the window is scrolled. Arrow-key
jogging is ignored while editing text fields. Leaving a jog button releases that
input; losing application focus, changing jog mode, closing the app, a fault,
or expired input disables jogging. Disconnect/close requests a stop when the
pendant has taken control. A connection used only for inspection can be closed
without changing robot state. Manual STOP requests a stop even in inspection mode.
Opening a mode dropdown also suspends held motion. Its Tk-owned `popdown` window
is checked by its Tcl path, avoiding Python 3.11's widget-lookup `KeyError` when
focus moves into the dropdown.

| Control | Default speed | Maximum allowed by this app | Acceleration set on Enable Jog |
| --- | --- | --- | --- |
| Joint J1–J7 | 3 deg/s | 10 deg/s | 10 deg/s² joint acceleration |
| TCP X/Y/Z | 5 mm/s | 20 mm/s | 20 mm/s² translational TCP acceleration |
| TCP Rx/Ry/Rz | 3 deg/s | 10 deg/s | Existing controller angular acceleration |

Speed edits take effect on the **next press**, so a held input cannot jump to a
new speed. These caps are conservative application limits, not certified safety
limits. The acceleration settings are controller parameters and can affect later
clients; the pendant does not save them to flash or restore prior values.

**TCP Base** commands translation or rotation about base axes; it rejects a
nonzero configured world rotation. **TCP Tool** uses the current tool axes.
At downward RPY `[180, 0, 0]`, tool +Z points opposite base +Z. Linear jogging
commands zero angular velocity, maintaining the current orientation. Rx/Ry/Rz
are angular velocities about frame axes, **not** direct roll/pitch/yaw edits.
The live TCP panel separately displays the controller's XYZ and Euler RPY values.
Existing TCP/world offsets are displayed and never changed by the pendant.

The worker uses `vc_set_joint_velocity()` in mode **4** and
`vc_set_cartesian_velocity()` in mode **5**, with **duration=0.2 seconds on every
velocity command**, including zero velocity. It refreshes a held command about
every 50 ms, and requires the GUI to renew its input within 150 ms. Expired input
requires an explicit re-enable; a recovered GUI cannot automatically resume it.
Feedback older than 400 ms blocks jogging. Robot I/O runs on a worker thread,
keeping the UI separate from SDK calls. A software stop cannot be guaranteed
over a broken connection; command expiry and physical deceleration are not an
instantaneous or safety-rated stop. No collision clearance or table-height limit
is calculated by this app.

Timed velocity commands require firmware **1.8.0 or newer**, which includes your
**2.5.1** controller. The SDK documents modes, units, and the positive-duration
timeout in its [official API source](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/xarm/wrapper/xarm_api.py).
The pendant rejects older firmware rather than sending indefinite velocity.
It does not stream `set_servo_cartesian()` commands or provide autonomous pushing
trajectories.

### Hold-to-Home Joint Motion

Home is a user-defined joint posture, not the robot's factory zero or calibration
procedure. It does not use `move_gohome()` or align the TCP orientation.

1. Connect, release motion controls, and wait until the robot is idle.
2. Click **Set Home from Current**. Each measured joint angle is replaced by the
   nearer of **0° or +45°**. Exactly 22.5° selects 0°. Negative angles select 0°;
   no negative or 90° home targets are generated. For example,
   `[7.7, -23.2, 10, 49.3, 2.5, 68.7, 4.2]` becomes
   `[0, 0, 0, 45, 0, 45, 0]` degrees.
3. Inspect the displayed **Home J1–J7** target and the full swept workspace.
   Setting home only reads feedback and checks controller joint limits; it does
   not enable motors or move the robot. A failed check keeps the previous target.
4. Hold **Hold to Home** with the mouse to move toward that target. This button
   enables joint position control directly; **Enable Jog** is not required.
   It uses the **Joint** speed field (default 3 deg/s, maximum 10 deg/s) and
   10 deg/s² joint acceleration.
5. Release the button or move the pointer outside it to request a stop. The arm
   remains where it stops. A new press continues from the measured current angles
   toward the **same saved target**, without recalculating home. The target is
   kept until **Set Home from Current** is clicked again or the robot disconnects.
   It is not saved across application restarts.

Homing uses `set_servo_angle(..., wait=False, radius=-1)` in **mode 0**.
Each segment changes any joint by at most **0.5°**, with all joints interpolated
toward the target. Only one segment is outstanding: the worker waits for fresh
feedback showing an idle controller, an empty command queue, and the segment
endpoint before issuing another. This produces small point-to-point moves rather
than a continuous blended trajectory. Completion tolerance is **0.05° per joint**.

Releasing home requests `set_state(4)`, cancels pending activation, and clears
the held input. Focus loss, STOP/Esc/Space, stale feedback, controller faults,
and input heartbeat expiry also stop/disarm homing. A recovered UI cannot restart
an expired home hold; press the button again deliberately. Before moving, the
target is rechecked against controller joint limits; travel exceeding 180° on any
joint is rejected. Joint motion can change both TCP position and orientation.

Position commands have no velocity-command `duration` watchdog. If communication
is lost, the last accepted segment can still finish; another segment is not
sent. Normal stopping also has communication latency and physical deceleration,
so releasing a button cannot guarantee an instantaneous stop at the exact
release pose. Keep the physical E-stop accessible and inspect the swept workspace.

**Taught poses:** release all jog controls, wait for the robot to become idle,
enter a name, and click **Capture current pose**. **Export JSON** writes joint
angles (deg), TCP pose and offsets (mm/deg), timestamp, robot IP, and a demo flag.
Captured poses remain in memory until exported. This version records poses; it
does not replay them or automatically align the pusher. Use the separate
`joint_motion.py` preview/alignment workflow before opening the pendant if needed.

**Warning 14:** after stopping, the explicit **Clear warning 14** button clears
only that existing warning, once, with an idle controller and no error. It does
not enable jogging or clear controller errors. Any new error/warning during
jogging disables control and remains visible for inspection.

**Demo limitations:** demo mode is entirely local and never imports or connects
the robot SDK. It exercises UI input, timing, status, and pose export using
independent joint/TCP values. It has no FK, IK, collision, or dynamics model;
angular readouts are illustrative. Demo success does not validate a physical path.

## View the RealSense Camera

Install the camera dependencies in the existing environment and start the viewer:

```bash
uv pip install --python .venv/bin/python -r requirements-realsense.txt
.venv/bin/python realsense_viewer.py
```

Press `q` or `Esc` in the video window, close the window, or press `Ctrl+C` in
the terminal to exit. This script only connects to the camera and does not
control the robot. Run it from a desktop terminal with USB camera access.
If it reports `No device connected`, check the USB connection and device permissions,
and close any other program using the camera.

Uses the [official RealSense Python/OpenCV API example](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py)
as an API reference.

## Run the Examples

SDK 1.18.4 is installed in this workspace's `.venv`. The following commands move the physical robot.
Clear the robot's surroundings, keep the emergency stop accessible, and stop other control programs before running one example at a time.

```bash
# Point the pusher downward (joint speed: 3 deg/s, acceleration: 10 deg/s^2)
.venv/bin/python joint_motion.py

# TCP: +5 mm along X (speed: 5 mm/s, acceleration: 20 mm/s^2)
.venv/bin/python cartesian_motion.py
```

To display current and target values without moving, use the commands below. This option still requires a robot connection but does not enable motors, change control modes, or send motion commands.

```bash
.venv/bin/python joint_motion.py --dry-run
.venv/bin/python cartesian_motion.py --dry-run
```

Use `--ip 192.168.1.197` to specify the robot address. Adjust Cartesian displacement, motion speeds, and accelerations using the constants at the top of each example.

## Set the Downward Pushing Orientation

`joint_motion.py` targets the current TCP XYZ with roll/pitch/yaw set to **180, 0, 0 degrees** using `TARGET_RPY_DEG`. TCP +X follows base +X, while TCP +Y and +Z oppose base +Y and +Z. For a robot mounted upright on a horizontal surface, TCP +Z therefore points downward. This assumes the pusher extends along the configured TCP +Z axis; the physical pusher and TCP offset must agree. The configured TCP may differ from the flange frame because of the existing tool offset.

The script reads current joint angles and the TCP pose, asks the controller for inverse kinematics (IK), checks joint limits, and verifies the result with forward kinematics (FK). Only after those checks does it enable motion and call `set_servo_angle()` in mode `0`. It prints the target joint angles and the change for each joint. No Cartesian motion command is used.

Preview the target before executing the move:

```bash
.venv/bin/python joint_motion.py --dry-run
```

After checking the target and clearing the full swept workspace, execute it:

```bash
.venv/bin/python joint_motion.py
```

- **Only the endpoint preserves TCP XYZ.** During joint interpolation, the TCP and the rest of the arm can travel through other positions. IK/FK and joint-limit checks do not establish a collision-free path.
- Alignment at the current position may be unreachable. IK failure aborts the run before motor activation; the script does not silently change the target position.
- The world frame must have no rotation relative to the physical base. A rotated world offset is rejected, and no offsets are changed.
- On firmware `2.7.103` or newer, current joint angles are supplied as the IK reference, with `limited=False`. On older firmware, including `2.5.1`, the script uses legacy IK without these unsupported options and reports the limitation. A reference does not guarantee the closest IK solution.
- Any single joint change greater than `MAX_JOINT_TRAVEL_DEG` (180 degrees) is rejected for review instead of allowing an unexpected full turn.
- The timeout is at least 30 seconds and increases with the largest joint change at the configured speed. A 180-degree change at 3 deg/s is allowed more than 60 seconds.
- FK and final measured TCP pose must match the target within 1 mm of position and 1 degree of orientation. Orientation is compared against the requested downward rotation, accounting for equivalent Euler-angle representations such as roll +180 and -180 degrees.
- The script rechecks joint angles immediately before sending the move and aborts if they changed by more than 1 degree during planning. Keep other control programs stopped throughout the run.

## Controller Warning 14: Command Has No Solution

According to the [official SDK code reference](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api_code.md#controller-warn-code), controller warning `14` means the command has no solution. This is distinct from controller error `14`, which refers to servo motor 4.

If this warning is reported at connection time, before the current TCP and target are printed, the script has stopped before attempting a new IK solution. The log alone does not establish which earlier command caused it or whether the new target is reachable.

To clear only an existing warning 14 once and recheck the target without moving:

```bash
.venv/bin/python joint_motion.py --dry-run --clear-warning-14
```

This option requires `--dry-run`. It only calls `clean_warn()` when the robot has no controller error, has no queued commands, is idle, and reports warning 14. It rechecks the controller afterward. It does not enable motors, change motion modes, clear controller errors, or retry warning clearing in a loop. A plain `--dry-run` does not clear warnings.

The script also reads controller status immediately after IK: SDK 1.18.4 can normalize a warning into a successful API return code for a getter, so returned joint values alone are not sufficient validation. If warning 14 returns during IK, the target XYZ and orientation did not yield a solution from the controller. The script stops without moving and leaves the new warning visible. Clearing the warning does not resolve an unreachable pose; review the printed current/target poses and the starting posture before retrying.

Only after a successful preview and workspace review, run the motion separately:

```bash
.venv/bin/python joint_motion.py
```

## Coordinate Frames and Motion Behavior

- Joint and orientation angles are in degrees (deg); positions are in millimeters (mm). The examples explicitly set `is_radian=False`.
- The joint example computes joint targets for the current TCP XYZ and RPY [180, 0, 0] degrees, then passes the IK solution to `set_servo_angle()`.
- The Cartesian example reads the TCP pose with `get_position()`, increases only X by 5 mm, and passes all six pose values to `set_position()`. According to the [official API source documentation](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/xarm/wrapper/xarm_api.py), this command operates in the base coordinate frame.
- TCP refers to the tool center point configured in the controller. The examples use the existing TCP and world/base offsets. Check the world offset rotation if you intend to move along the physical base's +X axis. This differs from moving along the tool's local +X axis.
- Both examples use position control mode `0` and `wait=True` to wait for completion. Cartesian motion has a 30-second timeout; joint alignment extends it as needed for the planned joint travel.
- On an SDK error or `Ctrl+C`, a session that has begun preparing for motion requests a stop before disconnecting. A software stop command may not reach the robot if communication is lost.
- Execution stops if the robot reports an error or warning, is moving or paused, or has queued commands. The sole warning-recovery option is `--dry-run --clear-warning-14`, described above. Errors are not cleared automatically.
- A successful SDK return code does not guarantee collision clearance in the physical workspace. In particular, joint motion does not make the TCP follow a straight line.

Your PC must be able to reach the robot at `192.168.1.197`. For example, it can use a `192.168.1.x/24` address on the same network, with an address different from the robot's.

The SDK creates `~/.UFACTORY/log/xarm/sdk` when first loaded. Run the examples in a regular terminal with write access to your home directory. This workspace's sandbox restricts writes to the home directory, so the tests below mock that directory creation.

## Install in a New Environment

Use Python 3.10 or later. The SDK wheel is included, so installing the package does not require internet access.

```bash
uv venv .venv
uv pip install --python .venv/bin/python --no-index --find-links vendor -r requirements.txt
```

In a new environment without `uv`, use the standard `venv` and `pip` tools:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --no-index --find-links vendor -r requirements.txt
```

Package source: [PyPI xarm-python-sdk 1.18.4](https://pypi.org/project/xarm-python-sdk/1.18.4/).
The SHA-256 hash is pinned in `requirements.txt`.

## Test Without a Robot

```bash
.venv/bin/python -m unittest discover -s tests -v
```

The tests replace SDK connection objects with mocks, so they do not connect to or move a robot. They do not validate physical reachability, singularities, or collision clearance on the actual robot.
Pendant tests also exercise the local demo worker, command expiry, stop handling,
keyboard input, and pose export. The desktop smoke test opens only the demo; it
is skipped automatically when a graphical display is unavailable.
