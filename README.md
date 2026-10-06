# xArm7 Controller

Python tools and examples for controlling the xArm7 robot using the [UFACTORY SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

Default IP: `192.168.1.197` · Python 3.11 · SDK 1.18.4

## Setup

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python --no-index --find-links vendor -r requirements.txt
```

## Motion Examples

| Script | Motion | Speed |
| --- | --- | --- |
| `joint_motion.py` | Point the pusher downward at TCP RPY `[180, 0, 0]°` using a joint move | 3 deg/s |
| `cartesian_motion.py` | Move TCP +5 mm along base X, keeping orientation | 5 mm/s |

```bash
# Preview without moving
.venv/bin/python joint_motion.py --dry-run

# Move the robot
.venv/bin/python joint_motion.py
.venv/bin/python cartesian_motion.py
```

Both scripts accept `--dry-run` and `--ip`. Change the constants in each script to adjust targets, speed, and acceleration. Units are mm and degrees.

Joint alignment preserves TCP XYZ only at the endpoint. Each Cartesian run adds another 5 mm. Keep the workspace clear and the physical E-stop accessible.

To clear an existing warning 14 and preview again:

```bash
.venv/bin/python joint_motion.py --dry-run --clear-warning-14
```

## Teach Pendant

Run from a graphical desktop terminal with Tkinter available.

```bash
.venv/bin/python teach_pendant.py --demo  # Offline demo
.venv/bin/python teach_pendant.py         # Real robot
```

- **Jog:** Connect → select Joint / TCP Base / TCP Tool → Enable Jog. Hold a ± button or use ← / → for the selected axis.
- **Home:** Set Home from Current chooses the nearer of 0° or +45° for each joint (22.5° → 0°). Hold to Home moves toward that target; release to stop, press again to continue. The target resets on disconnect.
- **Poses:** Capture current pose → Export JSON.
- **Stop:** STOP, Esc, or Space. Software stopping depends on communication; use the physical E-stop when needed.

Default jog speeds: joints and TCP rotation **3 deg/s**, TCP translation **5 mm/s**. Edit speeds in the UI. Releasing a held control requests a stop; losing window focus disables motion.

## RealSense Viewer

```bash
uv pip install --python .venv/bin/python -r requirements-realsense.txt
.venv/bin/python realsense_viewer.py
```

RGB preview at 640 × 480, 30 FPS. Press `q` or `Esc` to close.

## Tests

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Tests use mocks and the local demo, without connecting to a robot. The GUI test is skipped when no display is available.
