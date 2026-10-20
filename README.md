# xArm7 Controller

Python tools for xArm7 motion, a desktop teach pendant, RealSense capture,
and flange-mounted ChArUco calibration.

Python 3.11 · SDK 1.18.4 · Default robot IP: `192.168.1.197` (`--ip` to override).
Run commands from the repository root. Motion uses mm/degrees; calibration transforms use meters.

## Install

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python --no-index --find-links vendor -r requirements.txt

# Camera capture and calibration
uv pip install --python .venv/bin/python -r requirements-realsense.txt

# Optional 3D viewer
uv pip install --python .venv/bin/python -r requirements-viser.txt
```

The bundled SDK wheel supports offline SDK installation. Optional dependencies
require a package index or a local cache. The teach pendant requires Tkinter and
a graphical desktop; camera previews also need a desktop.

## Motion and teach pendant

```bash
# Preview the downward TCP orientation without moving
.venv/bin/python scripts/joint_motion.py --dry-run

# Move: downward TCP orientation / +5 mm along base X
.venv/bin/python scripts/joint_motion.py
.venv/bin/python scripts/cartesian_motion.py

# Desktop pendant: offline demo / real robot
.venv/bin/python scripts/teach_pendant.py --demo
.venv/bin/python scripts/teach_pendant.py
```

Motion examples accept `--ip` and `--dry-run`. Edit their constants to change
targets, speed, or acceleration. Defaults are 3°/s for joints and 5 mm/s for TCP
translation. Joint alignment preserves TCP XYZ only at the endpoint; each
Cartesian run adds another 5 mm. To explicitly clear a pre-existing warning 14,
use `joint_motion.py --dry-run --clear-warning-14`.

In the pendant, connect, select Joint / TCP Base / TCP Tool, enable jog, and hold
a ± button or use ← / → for the selected axis. **Set Home from Current** chooses
0° or +45° per joint (22.5° → 0°); hold **Home** to approach it. Release to stop.
Capture poses and export JSON while stationary. STOP, Esc, Space, or lost window
focus stops/disables jogging. Keep the workspace clear and the physical E-stop accessible.

## Camera and calibration

```bash
.venv/bin/python scripts/realsense_viewer.py
.venv/bin/python scripts/realsense_viewer.py --depth --headless --frames 90 \
  --snapshot output/realsense/camera.png

# Default: check camera, robot, and path without enabling motion
.venv/bin/python scripts/auto_calibrate.py

# Move, capture, stop, and solve
.venv/bin/python scripts/auto_calibrate.py --execute

# Check a fixed calibration on ten new poses; add --execute to move
.venv/bin/python scripts/auto_verify.py
```

The camera viewer defaults to 640 × 480 at 30 FPS; calibration uses 1280 × 720.
Use `--serial` to select a camera. Depth preview is unaligned color visualization,
not metric depth output. Close other camera programs before calibration.

Automatic calibration defaults to a 34-pose wide sweep and a return along the
latest captured session's path for the same robot. `--current-pose` skips the
return. `--execute` starts motion after checks, without another prompt. The path
checks do not establish obstacle clearance. Automatic verification instead starts
at the current pose with ten local targets and keeps the calibration fixed.

See [CALIBRATION.md](CALIBRATION.md) for board measurements, manual capture,
solver options, return behavior, stopping, and independent verification.

## Read ArUco positions

```bash
.venv/bin/python scripts/read_aruco_positions.py \
  --calibration output/calibration/run01/calibration.json
```

Reads centers of IDs 0–3 from the [40 mm marker sheet](assets/aruco_4x4_50_ids_0-7_40mm_A4.pdf)
in robot base XYZ, in mm. Specify your saved calibration with `--calibration`;
use `--once` for one frame or `--headless` for continuous terminal output.
`--marker-size-mm` must match the measured outer black square. No robot connection
or depth stream is used. The camera serial and resolution must match the calibration.

The latest frame is saved to `output/aruco/positions.json`; missing IDs have no
retained position. Keep the ChArUco board out of view because it reuses these IDs
at a different size. Keep the calibrated camera fixed.

## 3D viewer

```bash
.venv/bin/python scripts/viser_robot.py          # Live robot feedback
.venv/bin/python scripts/viser_robot.py --demo   # Offline demo

# Plan without a camera or motion, then preview without a robot connection
.venv/bin/python scripts/auto_calibrate.py --plan-only --session output/calibration/preview
.venv/bin/python scripts/viser_robot.py --offline --play \
  --plan output/calibration/preview/motion_plan.json
```

Open **http://localhost:8080**. The viewer only reads robot state; it never enables
motors or clears faults. Blue shows planned joints; playback timing is approximate
and stops at the last pose. The bundled model is approximate; with a nonzero world
offset, the SDK TCP marker is hidden. Feedback is saved to `output/viser/robot_state.json`.
Model attribution and license: [SOURCE.md](assets/robots/xarm7/SOURCE.md).

## Repository

- `scripts/`: commands and shared geometry, camera, robot, and JSON helpers.
- `assets/`: board configuration, printable targets, robot meshes, and licenses.
- `vendor/`: pinned SDK wheel.

Generated `output/`, environments, caches, local tests, and hardware prototypes
are excluded from Git. A fresh clone contains no captured sessions or calibration
results. Asset paths resolve relative to the repository; output paths are relative
to the working directory. For local development tests:

```bash
PYTHONPATH=scripts .venv/bin/python -m unittest discover -s tests -v
```
