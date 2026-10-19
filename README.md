# xArm7 Controller

Python tools and examples for controlling the xArm7 robot using the [UFACTORY SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

Default IP: `192.168.1.197` · Python 3.11 · SDK 1.18.4

## Setup

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python --no-index --find-links vendor -r requirements.txt
```

The repository includes source, dependency lists, the offline SDK wheel,
and reusable board/robot assets. Local `output/` files (captures,
calibration results, motion plans, previews, and generated PDFs), environments,
caches, logs, local credentials, local development `tests/`, and unreleased
`hardware/` prototypes are excluded from Git. Generate the outputs
referenced below on your own machine; they are not included in a fresh clone.

## Layout

```text
scripts/       Python commands and shared modules
assets/        Calibration board configuration and robot models
vendor/        SDK wheel for offline installation
output/        Local generated data (ignored)
tests/         Local development tests (ignored)
hardware/      Unreleased prototypes (ignored)
```

Run the commands below from the repository root. Script imports resolve within
`scripts/`; board and robot model paths resolve from the repository location.
Relative output and session paths remain relative to your working directory.

## Motion Examples

| Script | Motion | Speed |
| --- | --- | --- |
| `scripts/joint_motion.py` | Point the pusher downward at TCP RPY `[180, 0, 0]°` using a joint move | 3 deg/s |
| `scripts/cartesian_motion.py` | Move TCP +5 mm along base X, keeping orientation | 5 mm/s |

```bash
# Preview without moving
.venv/bin/python scripts/joint_motion.py --dry-run

# Move the robot
.venv/bin/python scripts/joint_motion.py
.venv/bin/python scripts/cartesian_motion.py
```

Both scripts accept `--dry-run` and `--ip`. Change the constants in each script to adjust targets, speed, and acceleration. Units are mm and degrees.

Joint alignment preserves TCP XYZ only at the endpoint. Each Cartesian run adds another 5 mm. Keep the workspace clear and the physical E-stop accessible.

To clear an existing warning 14 and preview again:

```bash
.venv/bin/python scripts/joint_motion.py --dry-run --clear-warning-14
```

## Teach Pendant

Run from a graphical desktop terminal with Tkinter available.

```bash
.venv/bin/python scripts/teach_pendant.py --demo  # Offline demo
.venv/bin/python scripts/teach_pendant.py         # Real robot
```

- **Jog:** Connect → select Joint / TCP Base / TCP Tool → Enable Jog. Hold a ± button or use ← / → for the selected axis.
- **Home:** Set Home from Current chooses the nearer of 0° or +45° for each joint (22.5° → 0°). Hold to Home moves toward that target; release to stop, press again to continue. The target resets on disconnect.
- **Poses:** Capture current pose → Export JSON.
- **Stop:** STOP, Esc, or Space. Software stopping depends on communication; use the physical E-stop when needed.

Default jog speeds: joints and TCP rotation **3 deg/s**, TCP translation **5 mm/s**. Edit speeds in the UI. Releasing a held control requests a stop; losing window focus disables motion.

## Viser Robot Posture

```bash
uv pip install --python .venv/bin/python -r requirements-viser.txt
.venv/bin/python scripts/viser_robot.py
```

Open **http://localhost:8080** to see live xArm7 joint feedback and the SDK TCP pose. This viewer only reads robot state; it does not enable motors, clear faults, or send motion commands. Updates continue while the robot moves. The interface marks lost feedback as stale and supports freezing the display with **Live updates**.

The bundled Menagerie model is an approximate visualization. The display reports its flange difference from the SDK after removing the TCP offset. With a nonzero world offset, the SDK TCP marker is hidden to avoid displaying it in the wrong frame. The latest feedback is saved to `output/viser/robot_state.json`.

```bash
# Show a demo pose without connecting to hardware.
.venv/bin/python scripts/viser_robot.py --demo --port 8081

# Optionally add animated blue planned poses alongside live feedback.
.venv/bin/python scripts/viser_robot.py --plan output/calibration/auto01/motion_plan.json

# Check all calibration poses using controller IK/FK, without enabling motion or using a camera.
.venv/bin/python scripts/auto_calibrate.py --plan-only --session output/calibration/preview01

# Replay that plan with no robot connection.
.venv/bin/python scripts/viser_robot.py --offline --plan output/calibration/preview01/motion_plan.json --play --port 8081 --snapshot output/viser/preview_state.json
```

The playback controls provide **Play**, **Pose**, **Playback speed**, and **Restart preview**. Blue shows the interpolated planned joints; white shows the saved start pose in offline mode. Playback stops at the last pose. Restart resets the preview immediately; it does not simulate an unchecked return motion. Timing approximates joint motion and is not the controller's exact trajectory. Controller checks cover IK, joint limits, joint excursion, and sampled flange position/orientation bounds; they do not establish obstacle clearance or camera visibility of the mounted board.

Model origin and licensing: [model source notes](assets/robots/xarm7/SOURCE.md).

For a visibly wider reach/lift sweep (robot **base +X 220 mm, upper +Z 70.5 mm, Y ±80 mm**), plan and preview without motion:

```bash
.venv/bin/python scripts/auto_calibrate.py --plan-only --pattern wide --session output/calibration/preview_wide01
.venv/bin/python scripts/viser_robot.py --offline --play --port 8083 --plan output/calibration/preview_wide01/motion_plan.json
```

The wide pattern now adds lower-left, lower-center and lower-right views: BASE Z −100 mm and lower Y ±150 mm from the start. It has 34 capture poses and 71 waypoints with translation and tilt separated. Configure these with `--drop-mm` and `--lower-side-mm`; `--drop-mm 0` restores the original 25 capture poses. `--pattern wide` also works with `--dry-run` and automatic collection. Its joint excursion limit is 90° (local: 30°); controller path checks and image margin guards remain enabled. Base axes are not calibrated camera axes, and obstacle clearance/full-path image visibility are not established by these checks.

Generate a local plan with `scripts/auto_calibrate.py --plan-only --pattern wide --session output/calibration/preview_wide02`, then preview it with `scripts/viser_robot.py --offline --play --port 8084 --plan output/calibration/preview_wide02/motion_plan.json`. The pose slider selects a waypoint; **Restart preview** plays the full sequence.

The 25-pose automatic sweep is a local bootstrap, not full image coverage. To inspect the current RGB image and propose a 3×3 grid at three camera distances (without robot motion):

```bash
.venv/bin/python scripts/plan_camera_coverage.py --capture --output output/calibration/coverage_review
```

Open `coverage_review.png` in that output directory. Its 27 outlines are camera-space acquisition goals, **not a robot trajectory**. See [the coverage workflow](CALIBRATION.md#카메라-영상으로-촬영-위치거리-검토) for preliminary calibration and required robot checks.

## RealSense Viewer

```bash
uv pip install --python .venv/bin/python -r requirements-realsense.txt
.venv/bin/python scripts/realsense_viewer.py
```

RGB preview at 640 × 480, 30 FPS. Press `q` or `Esc` to close.

The viewer prints the camera model, serial, firmware, USB connection, and active stream profiles. It works with the connected D455; use `--serial` to select a camera when more than one is connected.

```bash
# RGB and depth side by side (depth is not aligned to RGB).
.venv/bin/python scripts/realsense_viewer.py --depth

# Test USB/frame reception without opening a window, and save the last image.
.venv/bin/python scripts/realsense_viewer.py --headless --depth --frames 90 \
  --snapshot output/realsense/d455_test.png

# RGB at the calibration program's default resolution.
.venv/bin/python scripts/realsense_viewer.py --width 1280 --height 720
```

`--headless` stops after 90 frames unless `--frames` is specified. With `--depth --snapshot`, a separate `_depth.png` is also saved; this is a color visualization, not metric depth data. The camera is released when the viewer exits, so close it before starting calibration.

## Calibration Board

The [board generator](assets/calibration/generate_charuco.py) creates `output/pdf/charuco_a4_30mm.pdf` and matching JSON metadata locally: 6 columns x 9 rows, 30 mm squares, 22.5 mm markers, `DICT_4X4_50`, IDs 0-26, legacy off. Run it in a Python environment with OpenCV, NumPy and ReportLab. Print at **100% / Actual Size**, measure a square, and attach to a rigid flat board on the robot; keep the external camera fixed.

```python
import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard((6, 9), 0.030, 0.0225, dictionary)  # meters
board.setLegacyPattern(False)
detector = cv2.aruco.CharucoDetector(board)
```

[XS13xx flange drawing](assets/calibration/xarm13xx_end_flange.png): 4 × M6 holes on PCD 50 mm, threaded depth 10 mm. Select bolt length for the bracket/washer stack without bottoming out. [Official dimensions](https://docs.xarm.ufactory.cc/4.robotic_electrical_interface.html#_4-1-end-flange).

## Flange-mounted Board Calibration

[`scripts/calibrate_eye_to_hand.py`](scripts/calibrate_eye_to_hand.py) calibrates a **fixed external RealSense RGB camera** using a ChArUco board rigidly mounted on the xArm flange. It reads robot poses while you move between captures with Studio or the teach pendant. See [the calibration guide (한국어)](CALIBRATION.md) for setup, pose selection, camera intrinsics, independent verification, and transform conventions.

The default pattern is **`calib.io_CHARUCO_210x297_9x6_30_22_DICT_4X4.pdf`** in the project root: 6 columns × 9 rows, nominally **30 mm squares / 22 mm markers**. The default board JSON currently uses the measured printed target dimensions, **27.3 mm squares / 20.3 mm markers**; measure your own print and supply a matching `--board` JSON. This also differs from the locally generated 22.5 mm marker board above.

```bash
# Capture 20-30 stopped poses: Space/S saves, Q/Esc quits.
.venv/bin/python scripts/calibrate_eye_to_hand.py collect --session output/calibration/run01

# Offline calculation; writes output/calibration/run01/calibration.json.
.venv/bin/python scripts/calibrate_eye_to_hand.py solve --session output/calibration/run01
```

Results include `T_base_camera` and `T_flange_board`, with translations in **meters**. Existing TCP offsets are removed mathematically; the world offset must be zero. Camera/board mounts must remain unchanged throughout a session. No motion or motor-enable commands are sent by the calibration program.

For **automatic robot motion**, use [`scripts/auto_calibrate.py`](scripts/auto_calibrate.py). It generates 25 local poses around the starting flange pose, checks IK/FK and sampled joint paths before enabling motors, captures each stopped pose, and runs the calibration solver. Defaults: 25 mm position radius, 12° tilt, 3°/s joint speed. This is not an obstacle/collision planner; allow clearance for the arm and mounted board.

```bash
# Inspect a plan without enabling motors or moving.
.venv/bin/python scripts/auto_calibrate.py --dry-run --session output/calibration/auto01

# Move, capture, stop, and solve. Starts motion without another prompt.
.venv/bin/python scripts/auto_calibrate.py --session output/calibration/auto01
```

Space/Q/Esc in the preview or Ctrl+C in the terminal requests a stop. Use `--headless` without a desktop. See [automatic calibration instructions](CALIBRATION.md#자동-이동촬영계산) for limits, stopping behavior, and camera options.

If the camera's factory distortion model cannot be used directly by OpenCV, automatic collection fits camera intrinsics from the captured ChArUco views before solving hand-eye calibration. No extra flag is required; `--fit-intrinsics` can also force this with a compatible factory model.

## Tests

Tests are kept locally for development and are not included in this distribution.
If you have the local `tests/` directory, run:

```bash
PYTHONPATH=scripts .venv/bin/python -m unittest discover -s tests -v
```

Tests use mocks and the local demo, without connecting to a robot. The GUI test is skipped when no display is available.
