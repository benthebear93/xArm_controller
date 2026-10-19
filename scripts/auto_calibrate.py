"""Move xArm through a bounded local or wide sweep and capture ChArUco views.

Start with a rigid flange-mounted board visible to the fixed camera. --dry-run
performs camera/robot/IK/FK checks and saves a motion plan WITHOUT enabling motion.
Normal execution moves the robot. This is not an obstacle/collision planner.
Neither pattern guarantees image-wide coverage or calibrated camera depth diversity.
--pattern wide reaches in robot base +X, lifts in +Z and sweeps sideways in Y.
Use scripts/plan_camera_coverage.py to review coverage goals after an initial calibration.
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import time
from types import SimpleNamespace

import cv2
import numpy as np

from calibration_core import (
    DEFAULT_BOARD, BoardDetector, InsufficientCornersError, UnsupportedCameraModelError, camera_arrays, check_excitation, finite_array,
    flange_from_tcp, pose_error, pose_to_transform, read_json,
    validate_transform, write_json,
)
from calibrate_eye_to_hand import (
    RealSenseCamera, assert_stationary, capture_sample, choose_camera, overlay,
    positive, read_robot, save_sample, session_path, solve, utc_now,
)
from joint_motion import solve_alignment_ik
from robot_common import (
    ROBOT_IP, check, position_motion, read_result, require_clear_status,
    require_idle, robot_session, run_cli,
)

PATH_STEP_DEG = 1.0
MAX_JOINT_EXCURSION_DEG = 30.0
POLL_SECONDS = 0.05
TRACKING_MIN_CORNERS = 6  # Presence monitoring only; saved captures still require 12.


def transform_to_pose(value):
    """Meters/rotation matrix to the SDK's mm and fixed XYZ Euler degrees."""
    value = validate_transform(value)
    r = value[:3, :3]
    cp = np.hypot(r[0, 0], r[1, 0])
    pitch = np.arctan2(-r[2, 0], cp)
    if cp > 1e-8:
        roll, yaw = np.arctan2(r[2, 1], r[2, 2]), np.arctan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = np.arctan2(-r[1, 2], r[1, 1]), 0.0
    return np.r_[1000 * value[:3, 3], np.degrees([roll, pitch, yaw])].tolist()


def generate_targets(seed_flange, radius_mm=25, tilt_deg=12):
    """25 reproducible poses, including the seed; no cumulative position drift.

    Rotation axes and translation directions are expressed in the SEED flange
    frame. The rotation angle is bounded by tilt_deg, not per-Euler-component.
    """
    seed = validate_transform(seed_flange)
    targets = [seed.copy()]
    for ring in range(3):
        # Opposing x/y tilts first: even the first five samples constrain two axes.
        for i in (0, 4, 2, 6, 1, 5, 3, 7):
            # Offset the translated ring to keep its orientations distinct even
            # at the minimum translation radius and with small measurement noise.
            phase = 2 * np.pi * (i + (0.5 if ring == 2 else 0)) / 8
            axis = np.array([np.cos(phase), np.sin(phase), 0.0])
            angle = np.radians(tilt_deg * (0.6 if ring == 1 else 1.0))
            target = seed.copy()
            target[:3, :3] = seed[:3, :3] @ cv2.Rodrigues(axis * angle)[0]
            # Translate from the first non-seed sample; do not spend 17 views
            # rotating in one location. Both signs of local depth are included.
            direction = np.array([np.cos(phase), np.sin(phase), (-1) ** (i + ring) * .7])
            direction /= np.linalg.norm(direction)
            distance = radius_mm * (0.5, 0.75, 1.)[ring]
            target[:3, 3] += seed[:3, :3] @ direction * distance / 1000
            targets.append(target)
    return targets


def generate_wide_targets(seed_flange, forward_mm=220, lift_mm=150, side_mm=80, tilt_deg=12,
                          drop_mm=100, lower_side_mm=150):
    """Reach/lift sweep plus three tilted views at each of three lower locations.

    Translations use ROBOT BASE axes: +X forward, +Z up, +/-Y sideways.
    These are not calibrated camera axes. Orientations stay near the seed.
    """
    seed = validate_transform(seed_flange)
    targets = [seed.copy()]
    # Approach in three stages before doing the higher/lateral poses.
    for fraction in (1 / 3, 2 / 3, 1.):
        target = seed.copy()
        target[0, 3] += fraction * forward_mm / 1000
        targets.append(target)
    anchors = [
        (1., -1., .27), (1., 1., .27),
        # Captures 11-19 previously clipped the top of the D455 image. Lower
        # only these three anchors: at lift=150 mm, side tops drop 30 mm and
        # the center top drops 79.5 mm. Keep their XY and all rotations intact.
        (.82, 1., .47), (.82, 0., .47), (.82, -1., .47),
        (.55, -1., .13), (.55, 1., .13),
    ]
    offsets = [np.array([x * forward_mm, y * side_mm, z * lift_mm]) for x, y, z in anchors]
    if drop_mm > 0:
        # Visit the two lower corners and lower center explicitly, rather than
        # calling the low end of an entirely positive-Z sweep "lower".
        offsets.extend(np.array([.82 * forward_mm, y, -drop_mm])
                       for y in (-lower_side_mm, 0., lower_side_mm))
    for shift in offsets:
        for axis in ([1., 0, 0], [-1., 0, 0], [0, 1., 0]):
            target = seed.copy()
            target[:3, 3] += shift / 1000
            target[:3, :3] = seed[:3, :3] @ cv2.Rodrigues(np.array(axis) * np.radians(tilt_deg))[0]
            targets.append(target)
    return targets


def joint_excursion_limit(args):
    return 90. if args.pattern == "wide" else MAX_JOINT_EXCURSION_DEG


def wide_waypoints(targets, step_mm=40.):
    """Separate large translations from tilts, preserving the capture goals.

    Short Cartesian staging points constrain the controller's joint-interpolated
    arcs. They are still FK checked and never counted as extra calibration views.
    """
    result = [(targets[0], True)]
    previous = targets[0]
    for target in targets[1:]:
        distance, angle = pose_error(target, previous)
        if distance > 1e-6:
            steps = max(1, int(np.ceil(distance / step_mm)))
            for step in range(1, steps + 1):
                if step == steps and angle < 1e-6:
                    continue
                stage = previous.copy()
                stage[:3, 3] += (target[:3, 3] - previous[:3, 3]) * step / steps
                result.append((stage, False))
        result.append((target, True))
        previous = target
    return result


def require_full_board(image, detection, detector, margin_px=35):
    """Reject an already clipped starting view, using the pattern's outer boundary.

    This planar homography is a visibility guard, not calibrated robot geometry.
    Keeping only enough inner corners for PnP does not establish visibility.
    """
    objects, pixels = detector.points(detection)
    homography, inliers = cv2.findHomography(objects[:, :2], pixels, cv2.RANSAC, 2.)
    if homography is None or inliers is None or np.count_nonzero(inliers) < 12:
        raise ValueError("Cannot establish the full board boundary from detected corners")
    cols, rows = detector.board.getChessboardSize()
    square = detector.board.getSquareLength()
    outer = np.array([[[0, 0], [cols * square, 0],
                       [cols * square, rows * square], [0, rows * square]]], dtype=np.float32)
    boundary = cv2.perspectiveTransform(outer, homography)[0]
    if not np.isfinite(boundary).all() or not cv2.isContourConvex(boundary):
        raise ValueError("Invalid projected board boundary")
    lower, upper = boundary.min(axis=0), boundary.max(axis=0)
    height, width = image.shape[:2]
    if np.any(lower < margin_px) or np.any(upper > [width - 1 - margin_px, height - 1 - margin_px]):
        raise ValueError(
            f"Full board needs >= {margin_px:g} px image margin; "
            f"estimated pattern bounds {np.round(lower, 1).tolist()} to {np.round(upper, 1).tolist()}. "
            "Pattern is clipped or too close to the image boundary at this pose."
        )
    return boundary


def record_capture_visibility(image, record, detector):
    """ChArUco permits partial views; record boundary warnings, not fatal errors.

    Required corner count, ID validity and non-collinearity still apply. Capture
    already checks stationarity and, when intrinsics are available, reprojection.
    """
    detector.points(record["detection"])
    try:
        require_full_board(image, record["detection"], detector, margin_px=10)
        record["full_pattern_margin_ok"] = True
    except ValueError as exc:
        record["full_pattern_margin_ok"] = False
        record["visibility_warning"] = str(exc)


def check_starting_view(image, detection, detector):
    """Require the full pattern with 10 px clearance; 35 px is advisory.

    Homography estimates vary between frames, so the recommended framing margin
    must not become a brittle stop condition after returning to a known seed.
    """
    boundary = require_full_board(image, detection, detector, margin_px=10)
    height, width = image.shape[:2]
    margin = float(min(boundary[:, 0].min(), boundary[:, 1].min(),
                       width - 1 - boundary[:, 0].max(), height - 1 - boundary[:, 1].max()))
    result = {"minimum_margin_px": 10, "recommended_margin_px": 35,
              "estimated_margin_px": margin, "pattern_boundary_px": boundary.tolist(),
              "recommended_margin_ok": margin >= 35}
    if margin < 35:
        print(f"Starting view accepted: estimated margin {margin:.1f} px >= 10 px minimum; "
              "35 px is recommended, not required.", flush=True)
    return result


def check_offsets(seed, current):
    for key in ("tcp_offset_mm_deg", "world_offset_mm_deg"):
        if not np.allclose(seed[key], current[key], atol=1e-6, rtol=0):
            raise RuntimeError("Robot coordinate offsets changed; stopping automatic calibration")


def check_envelope(flange, seed_flange, args):
    distance, angle = pose_error(flange, seed_flange)
    if args.pattern == "wide":
        shift = 1000 * (np.asarray(flange)[:3, 3] - np.asarray(seed_flange)[:3, 3])
        side = max(args.side_mm, args.lower_side_mm if args.drop_mm > 0 else 0)
        low = np.array([-20, -side - 20, -args.drop_mm - 20])
        high = np.array([args.forward_mm + 20, side + 20, args.lift_mm + 20])
        if np.any(shift < low) or np.any(shift > high) or angle > args.tilt_deg + 3:
            raise ValueError(f"Path leaves wide BASE-frame envelope: {np.round(shift, 1).tolist()} mm / {angle:.1f} deg")
        return
    # Includes the sampled joint-interpolated path, not just requested endpoints.
    if distance > args.radius_mm + 5 or angle > args.tilt_deg + 3:
        raise ValueError(f"Path leaves local flange envelope: {distance:.1f} mm / {angle:.1f} deg")


def forward_flange(arm, joints, seed):
    pose = read_result(arm.get_forward_kinematics(
        list(joints), input_is_radian=False, return_is_radian=False,
    ), "Forward kinematics")
    require_clear_status(arm)
    return flange_from_tcp(pose, seed["tcp_offset_mm_deg"], seed["world_offset_mm_deg"])


def plan_motion(arm, seed, args, tick=lambda: None):
    """Validate all endpoint IKs and sampled joint paths before enabling motors."""
    seed_joints = finite_array(seed["joints_deg"], (7,), "seed joints")
    seed_flange = validate_transform(seed["T_base_flange"])
    offset = pose_to_transform(seed["tcp_offset_mm_deg"])
    # Confirm this controller's FK uses the same configured TCP convention.
    initial_fk = forward_flange(arm, seed_joints, seed)
    if any(a > b for a, b in zip(pose_error(initial_fk, seed_flange), (1.0, 0.5))):
        raise RuntimeError("FK and measured flange pose disagree; check robot/offset configuration")
    previous = seed_joints
    plan = []
    targets = (generate_wide_targets(seed_flange, args.forward_mm, args.lift_mm, args.side_mm, args.tilt_deg,
                                     args.drop_mm, args.lower_side_mm)
               if args.pattern == "wide" else generate_targets(seed_flange, args.radius_mm, args.tilt_deg))
    waypoints = wide_waypoints(targets) if args.pattern == "wide" else [(t, True) for t in targets]
    limit = joint_excursion_limit(args)
    for index, (flange, capture) in enumerate(waypoints):
        tick()
        tcp = transform_to_pose(flange @ offset)
        joints = seed_joints if index == 0 else np.asarray(solve_alignment_ik(arm, tcp, previous.tolist()))
        joints = finite_array(joints, (7,), "IK joints")
        if np.max(np.abs(joints - seed_joints)) > limit:
            raise ValueError(f"Pose {index + 1}: IK changes a joint by >{limit:g} deg from start")
        limited = read_result(arm.is_joint_limit(joints.tolist(), is_radian=False), "Joint limits")
        require_clear_status(arm)
        if limited is not False:
            raise ValueError(f"Pose {index + 1}: joint limit check failed")
        actual = forward_flange(arm, joints, seed)
        if any(a > b for a, b in zip(pose_error(actual, flange), (1.0, 0.5))):
            raise ValueError(f"Pose {index + 1}: IK/FK does not match the requested pose")
        steps = max(1, int(np.ceil(np.max(np.abs(joints - previous)) / PATH_STEP_DEG)))
        for fraction in np.linspace(0, 1, steps + 1):
            tick()
            intermediate = previous + fraction * (joints - previous)
            check_envelope(forward_flange(arm, intermediate, seed), seed_flange, args)
        plan.append({"index": index + 1, "joints_deg": joints.tolist(),
                     "tcp_pose_mm_deg": tcp, "T_base_flange": flange.tolist(),
                     "capture": capture, "path_checks": steps + 1})
        print(f"Plan {index + 1:02d}/{len(waypoints)} ({'capture' if capture else 'transit'}): "
              f"TCP {np.round(tcp, 2).tolist()}", flush=True)
        previous = joints
    check_excitation([np.asarray(item["T_base_flange"]) for item in plan])
    assert_stationary(seed, read_robot(arm))
    return plan


class Preview:
    """Nonblocking camera/keyboard watchdog during robot motion."""

    def __init__(self, camera, detector, camera_model, headless=False):
        self.camera, self.detector, self.camera_model = camera, detector, camera_model
        self.headless = headless
        self.window = "Automatic calibration - SPACE / Q / Esc: STOP"
        self.status = "Planning local sweep"
        self.last_frame = self.last_board = time.monotonic()
        self.last_number = None
        self.last_corner_count = None
        self.opened = False

    def __enter__(self):
        if not self.headless:
            # Some cv2 wheels point Qt at a fonts directory they do not ship.
            if not Path(os.environ.get("QT_QPA_FONTDIR", "/nonexistent")).is_dir():
                for directory in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/truetype"):
                    if Path(directory).is_dir():
                        os.environ["QT_QPA_FONTDIR"] = directory
                        break
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            self.opened = True
        try:
            self.refresh()
        except BaseException:
            if self.opened:
                cv2.destroyWindow(self.window)
            raise
        return self

    def refresh(self):
        """Called only while stopped; require a board before starting a move."""
        for attempt in range(1, 4):
            image, timestamp = self.camera.fresh()
            detection = self.detector.detect_for_capture(image)
            try:
                self.detector.points(detection, minimum=TRACKING_MIN_CORNERS)
                break
            except InsufficientCornersError as exc:
                if attempt == 3:
                    raise InsufficientCornersError(f"Preview failed after 3 fresh frames: {exc}") from exc
        self.last_frame = self.last_board = time.monotonic()
        self.last_number = timestamp["frame_number"]
        self.last_corner_count = len(detection["ids"])
        self.draw(image, detection)

    def draw(self, image, detection):
        if not self.headless:
            # No solvePnP in the motion polling path; keep stop monitoring responsive.
            cv2.imshow(self.window, overlay(image, detection, self.detector, None,
                                            [self.status, "SPACE / Q / Esc: STOP"]))

    def tick(self):
        result = self.camera.poll()
        if result is not None:
            image, timestamp = result
            if timestamp["frame_number"] != self.last_number:
                self.last_number = timestamp["frame_number"]
                self.last_frame = time.monotonic()
                detection = self.detector.detect(image)
                self.last_corner_count = len(detection["ids"])
                try:
                    self.detector.points(detection, minimum=TRACKING_MIN_CORNERS)
                    self.last_board = time.monotonic()
                except ValueError:
                    pass
                self.draw(image, detection)
        if not self.headless:
            if (cv2.waitKey(1) & 0xFF in (32, ord("q"), 27)
                    or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1):
                raise KeyboardInterrupt("Stop requested in preview")
        if time.monotonic() - self.last_frame > 1.0:
            raise RuntimeError("Camera frames stale for >1 second")
        if time.monotonic() - self.last_board > 1.0:
            raise RuntimeError(f"ChArUco detection lost for >1 second: tracking requires "
                               f">={TRACKING_MIN_CORNERS} valid non-collinear corners; "
                               f"last frame had {self.last_corner_count}")

    def __exit__(self, *_):
        if self.opened:
            cv2.destroyWindow(self.window)


def moving_feedback(arm, seed, args):
    if not arm.connected:
        raise RuntimeError("Robot disconnected during automatic motion")
    require_clear_status(arm)
    state = read_result(arm.get_state(), "Motion state")
    queued = read_result(arm.get_cmdnum(), "Queued commands")
    if state not in (0, 1, 2) or arm.mode != 0 or queued > 1:
        raise RuntimeError(f"Unexpected robot state/mode/queue: {state}/{arm.mode}/{queued}")
    joints = finite_array(read_result(arm.get_servo_angle(is_radian=False, is_real=True),
                                      "Motion joint feedback"), (7,), "actual joints")
    pose = read_result(arm.get_position(is_radian=False), "Motion TCP feedback")
    current = {"tcp_offset_mm_deg": arm.tcp_offset, "world_offset_mm_deg": arm.world_offset}
    check_offsets(seed, current)
    flange = flange_from_tcp(pose, current["tcp_offset_mm_deg"], current["world_offset_mm_deg"])
    check_envelope(flange, seed["T_base_flange"], args)
    if np.max(np.abs(joints - np.asarray(seed["joints_deg"]))) > joint_excursion_limit(args) + 0.5:
        raise RuntimeError("Actual joints exceeded the planned excursion limit")
    return state, queued, joints


def move_to(arm, target, expected_previous, seed, args, tick):
    """One non-blended joint command at a time; poll until confirmed stationary."""
    tick()
    current = read_robot(arm)
    check_offsets(seed, current)
    if np.max(np.abs(np.asarray(current["joints_deg"]) - expected_previous)) > 0.5:
        raise RuntimeError("Robot moved away from the planned starting joints")
    if arm.mode != 0 or read_result(arm.get_state(), "Pre-move state") not in (0, 2):
        raise RuntimeError("Robot was stopped/paused or changed mode; motion will not resume")
    goal = np.asarray(target["joints_deg"])
    travel = float(np.max(np.abs(goal - np.asarray(current["joints_deg"]))))
    if travel > 0.05:
        check(arm.set_servo_angle(angle=goal.tolist(), is_radian=False, relative=False,
                                 speed=args.speed, mvacc=args.acceleration, wait=False, radius=-1),
              "Move to calibration pose")
        started = time.monotonic()
        timeout = max(15.0, 2 * travel / args.speed + 5)
        stable = 0
        while True:
            tick()
            state, queued, joints = moving_feedback(arm, seed, args)
            finished = state != 1 and queued == 0 and np.max(np.abs(joints - goal)) <= 0.1
            stable = stable + 1 if finished else 0
            if stable >= 3 and time.monotonic() - started >= 0.2:
                break
            if time.monotonic() - started > timeout:
                raise RuntimeError("Motion timed out before reaching calibration pose")
            time.sleep(POLL_SECONDS)
    measured = read_robot(arm)
    check_offsets(seed, measured)
    if any(a > b for a, b in zip(pose_error(measured["T_base_flange"], target["T_base_flange"]), (1.0, 0.5))):
        raise RuntimeError("Measured flange pose disagrees with the calibration target")


def execute_plan(arm, camera, detector, pose_camera, seed, plan, session, args, preview):
    """Keep all enabled operations inside the stop-on-error context."""
    assert_stationary(seed, read_robot(arm))
    preview.refresh()
    with position_motion(arm):
        assert_stationary(seed, read_robot(arm))
        previous = np.asarray(seed["joints_deg"])
        for target in plan:
            preview.status = f"Pose {target['index']}/{len(plan)} - saved {len(session['samples'])}"
            # A stopped/paused robot is never silently re-enabled between poses.
            move_to(arm, target, previous, seed, args, preview.tick)
            previous = np.asarray(target["joints_deg"])
            if not target.get("capture", True):
                continue
            image, record = capture_sample(arm, camera, detector, pose_camera,
                                           session["samples"], args.settle)
            record_capture_visibility(image, record, detector)
            record["planned_pose_index"] = target["index"]
            save_sample(args.session, session, image, record)
            print(f"Saved image {len(session['samples'])} at waypoint {target['index']}/{len(plan)} "
                  f"({len(record['detection']['ids'])} corners)"
                  + (" — partial/edge view accepted" if not record["full_pattern_margin_ok"] else ""), flush=True)
            preview.refresh()
            preview.tick()
        check(arm.set_state(4), "Stop after automatic collection")


def plan_document(seed, plan, args, fit_intrinsics=None):
    wide = args.pattern == "wide"
    side = max(args.side_mm, args.lower_side_mm if args.drop_mm > 0 else 0)
    return {
        "schema_version": 1, "created_utc": utc_now(), "robot_ip": args.ip,
        "purpose": "wide_base_frame_sweep" if wide else "local_bootstrap_sweep", "image_coverage_checked": False,
        "pose_pattern": "forward_lift_lower_corners_v3_lowered_top" if wide else "early_opposed_tilts_with_3d_translation_v2",
        "capture_pose_count": sum(t.get("capture", True) for t in plan),
        "waypoint_count": len(plan),
        "translation_frame": "robot_base" if wide else "seed_flange",
        "wide_offsets_mm": {"forward_base_x": args.forward_mm, "lift_base_z": args.lift_mm,
                            "side_base_y": args.side_mm, "drop_base_z": args.drop_mm,
                            "lower_side_base_y": args.lower_side_mm} if wide else None,
        "upper_capture_lift_mm": .47 * args.lift_mm if wide else None,
        "camera_depth_coverage_checked": False,
        "fit_intrinsics": fit_intrinsics,
        "seed": seed, "radius_mm": args.radius_mm, "tilt_deg": args.tilt_deg,
        "speed_deg_s": args.speed, "acceleration_deg_s2": args.acceleration,
        "flange_path_limits": ({"base_offset_min_mm": [-20, -side - 20, -args.drop_mm - 20],
                                "base_offset_max_mm": [args.forward_mm + 20, side + 20, args.lift_mm + 20],
                                "angle_deg": args.tilt_deg + 3} if wide else
                               {"radius_mm": args.radius_mm + 5, "angle_deg": args.tilt_deg + 3}),
        "max_joint_excursion_deg": joint_excursion_limit(args),
        "path_sample_step_deg": PATH_STEP_DEG,
        "collision_checked": False, "targets": plan,
    }


def plan_start_return(arm, document, args, tick=lambda: None):
    """Retrace a saved joint path from a waypoint or a verified point on a segment."""
    if document["robot_ip"] != args.ip:
        raise ValueError("Start plan belongs to a different robot IP")
    seed = document["seed"]
    current = read_robot(arm)
    check_offsets(seed, current)
    seed_joints = finite_array(seed["joints_deg"], (7,), "preset joints")
    if (np.max(np.abs(np.asarray(current["joints_deg"]) - seed_joints)) <= .1
            and all(a <= b for a, b in zip(
                pose_error(current["T_base_flange"], seed["T_base_flange"]), (1., .5)))):
        return current, [], args
    # Use the original path envelope, even if the next collection's settings differ.
    options = ["--ip", args.ip, "--radius-mm", str(document["radius_mm"]),
               "--tilt-deg", str(document["tilt_deg"]), "--speed", str(args.speed),
               "--acceleration", str(args.acceleration)]
    if document["purpose"] == "wide_base_frame_sweep":
        offsets = document["wide_offsets_mm"]
        options += ["--pattern", "wide"]
        for flag, key in (("forward-mm", "forward_base_x"), ("lift-mm", "lift_base_z"),
                          ("side-mm", "side_base_y"), ("drop-mm", "drop_base_z"),
                          ("lower-side-mm", "lower_side_base_y")):
            options += ["--" + flag, str(offsets[key])]
    elif document["purpose"] != "local_bootstrap_sweep":
        raise ValueError("Expected an original calibration sweep plan")
    return_args = parse_args(options)
    targets = document["targets"]
    joints = np.asarray([finite_array(t["joints_deg"], (7,), "saved joints") for t in targets])
    if not len(targets) or np.max(np.abs(joints[0] - seed_joints)) > .01:
        raise ValueError("Saved path does not begin at its start preset")
    nearest = int(np.argmin(np.max(np.abs(joints - current["joints_deg"]), axis=1)))
    waypoint_matches = (np.max(np.abs(joints[nearest] - current["joints_deg"])) <= .5
                        and all(a <= b for a, b in zip(pose_error(current["T_base_flange"],
                                            targets[nearest]["T_base_flange"]), (1., .5))))
    if waypoint_matches:
        returning = list(reversed(targets[:nearest + 1]))
    else:
        # Non-blended joint moves follow segments in joint space. A stop may be
        # far from either endpoint while still lying precisely on that path.
        candidates = []
        for index, (start, end) in enumerate(zip(joints[:-1], joints[1:])):
            direction = end - start
            squared_length = float(direction @ direction)
            if squared_length < 1e-12:
                continue
            fraction = float(np.clip((np.asarray(current["joints_deg"]) - start)
                                     @ direction / squared_length, 0., 1.))
            projected = start + fraction * direction
            error = float(np.max(np.abs(projected - current["joints_deg"])))
            candidates.append((error, index, fraction, projected))
        best = min(candidates, key=lambda item: item[0]) if candidates else None
        if best is None or best[0] > .1:
            raise ValueError("Current joints do not match a saved waypoint or joint-path segment "
                             "(segment tolerance 0.1 deg); automatic return refused. "
                             "Review the current pose in the pendant.")
        error, index, fraction, projected = best
        flange = forward_flange(arm, projected, seed)
        if any(a > b for a, b in zip(pose_error(current["T_base_flange"], flange), (1., .5))):
            raise ValueError("Current flange disagrees with FK of the matched return segment")
        matched = {"joints_deg": projected.tolist(), "T_base_flange": flange.tolist(),
                   "tcp_pose_mm_deg": transform_to_pose(flange @ pose_to_transform(seed["tcp_offset_mm_deg"])),
                   "matched_segment": [index + 1, index + 2], "segment_fraction": fraction,
                   "joint_path_error_deg": error}
        returning = [matched, *reversed(targets[:index + 1])]
        print(f"Stopped on segment {index + 1}->{index + 2} at {fraction:.1%}; "
              f"joint-path error {error:.6f} deg. Retracing toward waypoint {index + 1}.", flush=True)
    route = [dict(t, index=i + 1, capture=False) for i, t in enumerate(returning)]
    previous = np.asarray(current["joints_deg"])
    for target in route:
        tick()
        goal = np.asarray(target["joints_deg"])
        if np.max(np.abs(goal - seed_joints)) > joint_excursion_limit(return_args):
            raise ValueError("Saved return joint excursion exceeds the allowed limit")
        limited = read_result(arm.is_joint_limit(goal.tolist(), is_radian=False), "Return joint limits")
        require_clear_status(arm)
        if limited is not False:
            raise ValueError("Return waypoint fails joint limits")
        actual = forward_flange(arm, goal, seed)
        if any(a > b for a, b in zip(pose_error(actual, target["T_base_flange"]), (1., .5))):
            raise ValueError("Saved return waypoint disagrees with controller FK")
        steps = max(1, int(np.ceil(np.max(np.abs(goal - previous)) / PATH_STEP_DEG)))
        for fraction in np.linspace(0, 1, steps + 1):
            tick()
            check_envelope(forward_flange(arm, previous + fraction * (goal - previous), seed),
                           seed["T_base_flange"], return_args)
        target["path_checks"] = steps + 1
        previous = goal
    assert_stationary(current, read_robot(arm))
    print(f"Start preset TCP: {np.round(seed['tcp_pose_mm_deg'], 2).tolist()}", flush=True)
    print(f"Return: retrace {len(route)} waypoints at {args.speed:g} deg/s; "
          "obstacles and board swept volume are not modeled.", flush=True)
    return current, route, return_args


def prepare_start(arm, camera, detector, pose_camera, args):
    """Return to an explicit saved seed; dry-run validates only, with no enable."""
    document = read_json(args.start_plan)
    with Preview(camera, detector, pose_camera, args.headless) as preview:
        current, route, return_args = plan_start_return(arm, document, args, preview.tick)
        if not route:
            print("Already at the selected calibration start pose.", flush=True)
            return True
        saved = plan_document(document["seed"], route, return_args)
        saved.update(purpose="return_to_calibration_start", current=current,
                     source_plan=str(args.start_plan), start_only=True)
        write_json(args.session / "return_plan.json", saved)
        if args.dry_run:
            print("Dry run: return path checked; no motion enabled. "
                  "Capture sweep and starting camera view will be checked after actual return.", flush=True)
            return False
        assert_stationary(current, read_robot(arm))
        with position_motion(arm):
            previous = np.asarray(current["joints_deg"])
            for target in route:
                preview.status = f"Return to start {target['index']}/{len(route)}"
                move_to(arm, target, previous, document["seed"], return_args, preview.tick)
                previous = np.asarray(target["joints_deg"])
            check(arm.set_state(4), "Stop at calibration start")
        assert_stationary(document["seed"], read_robot(arm))
    return True


def run(args):
    if session_path(args.session).exists():
        raise ValueError("Session already contains captures; choose a new --session")
    if args.plan_only:
        # Planning for Viser does not need a camera or board detection.
        with robot_session(args.ip, dry_run=True) as arm:
            seed = read_robot(arm)
            plan = plan_motion(arm, seed, args)
            write_json(args.session / "motion_plan.json", plan_document(seed, plan, args))
        print(f"Plan only: no motion enabled. Saved {args.session / 'motion_plan.json'}")
        return
    board_config = read_json(args.board)
    detector = BoardDetector(board_config)
    with RealSenseCamera(args) as camera, robot_session(args.ip, dry_run=True) as arm:
        seed = read_robot(arm)
        camera_config = choose_camera(camera.metadata, args.intrinsics)
        fit_camera_intrinsics = args.fit_intrinsics
        try:
            camera_arrays(camera_config)
        except UnsupportedCameraModelError:
            fit_camera_intrinsics = True
            print(f"Factory distortion model '{camera_config['distortion_model']}' is not OpenCV-compatible. "
                  "Automatically enabling ChArUco intrinsic fitting after collection.", flush=True)
        if fit_camera_intrinsics:
            # Do not reject frames using factory intrinsics that will be replaced.
            # Robot stability and ChArUco geometry checks still apply to every capture.
            pose_camera = None
            print("Camera intrinsics will be fitted from the captured views; "
                  "reprojection checks run after fitting.", flush=True)
        else:
            pose_camera = camera_config
        if args.start_plan:
            if not prepare_start(arm, camera, detector, pose_camera, args):
                return
            seed = read_robot(arm)
        # Fail before motor enable if the initial board cannot be captured.
        initial_image, initial_sample = capture_sample(arm, camera, detector, pose_camera, [], args.settle)
        args.session.mkdir(parents=True, exist_ok=True)
        # Save a concrete diagnostic even when visibility rejects the starting pose.
        if not cv2.imwrite(str(args.session / "initial_view.png"), initial_image):
            raise RuntimeError("Could not save initial_view.png")
        try:
            visibility = check_starting_view(initial_image, initial_sample["detection"], detector)
        except ValueError as exc:
            raise ValueError(f"Starting view check failed before collection motion: {exc} "
                             "Give the starting board more image margin; see initial_view.png.") from exc
        write_json(args.session / "initial_view.json", {
            "detection": initial_sample["detection"], "robot": initial_sample["robot"],
            "camera_timestamp": initial_sample["camera_timestamp"], "visibility": visibility,
        })
        print(f"Initial board: {len(initial_sample['detection']['ids'])} corners", flush=True)
        print((f"Wide BASE-frame sweep: +X {args.forward_mm:g} mm, upper captures +Z {.47 * args.lift_mm:g} mm, "
               f"Y +/-{args.side_mm:g} mm; lower corners Z -{args.drop_mm:g} mm, Y +/-{args.lower_side_mm:g} mm"
               if args.pattern == "wide" else
               f"Local flange sweep: 25 poses; radius {args.radius_mm:g} mm")
              + f"; tilt {args.tilt_deg:g} deg; speed {args.speed:g} deg/s", flush=True)
        print("IK/FK checks do not model obstacles or the board's swept volume.", flush=True)
        with Preview(camera, detector, pose_camera, args.headless) as preview:
            plan = plan_motion(arm, seed, args, preview.tick)
            args.session.mkdir(parents=True, exist_ok=True)
            write_json(args.session / "motion_plan.json", plan_document(seed, plan, args, fit_camera_intrinsics))
            if args.dry_run:
                if not cv2.imwrite(str(args.session / "initial_view.png"), initial_image):
                    raise RuntimeError("Could not save initial_view.png")
                print(f"Dry run passed. No motion enabled. Plan: {args.session / 'motion_plan.json'}")
                return
            session = {
                "schema_version": 1, "setup": "eye_to_hand", "translation_unit": "m",
                "created_utc": utc_now(), "robot_ip": args.ip, "board": board_config,
                "camera": camera_config, "samples": [], "collection": f"automatic_{args.pattern}_sweep",
                "fit_intrinsics": fit_camera_intrinsics,
                "status": "collecting",
            }
            write_json(session_path(args.session), session)
            try:
                execute_plan(arm, camera, detector, pose_camera, seed, plan, session, args, preview)
                session["status"] = "collection_complete"
            except (Exception, KeyboardInterrupt) as exc:
                session.update(status="aborted", abort_reason=str(exc) or type(exc).__name__)
                # execute_plan's context has already requested a stop. Preserve a
                # last view to diagnose the failed pose without masking its error.
                try:
                    image, timestamp = camera.fresh()
                    if cv2.imwrite(str(args.session / "abort_view.png"), image):
                        write_json(args.session / "abort_view.json", {
                            "reason": session["abort_reason"], "camera_timestamp": timestamp,
                            "detection": detector.detect_for_capture(image),
                        })
                except Exception:
                    pass
                raise
            finally:
                write_json(session_path(args.session), session)
    # The arm is stopped/disconnected before numerical calibration starts.
    print("Collection complete; solving calibration offline.", flush=True)
    solve(SimpleNamespace(
        session=args.session, intrinsics=None, fit_intrinsics=fit_camera_intrinsics,
        exclude=[], max_reprojection_px=2.0, method="park", output=None,
        max_translation_rms_mm=3.0, max_rotation_rms_deg=1.0,
    ))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=ROBOT_IP)
    parser.add_argument("--start-plan", type=Path,
                        help="Use this saved motion_plan.json's seed; retrace its waypoints to return first")
    parser.add_argument("--serial", help="RealSense serial number")
    parser.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    parser.add_argument("--session", type=Path, default=Path("output/calibration") / datetime.now().strftime("auto_%Y%m%d_%H%M%S"))
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--pattern", choices=("local", "wide"), default="local")
    parser.add_argument("--forward-mm", type=positive, default=220., help="Wide sweep: robot BASE +X reach")
    parser.add_argument("--lift-mm", type=positive, default=150., help="Wide sweep: BASE Z envelope scale; upper captures use 47%% of this")
    parser.add_argument("--side-mm", type=positive, default=80., help="Wide sweep: robot BASE +/-Y span")
    parser.add_argument("--drop-mm", type=float, default=100., help="Wide sweep: lower row below seed in BASE Z (0 disables)")
    parser.add_argument("--lower-side-mm", type=positive, default=150., help="Wide sweep: lower corners at BASE Y +/- this distance")
    parser.add_argument("--radius-mm", type=positive, default=25.0)
    parser.add_argument("--tilt-deg", type=positive, default=12.0)
    parser.add_argument("--speed", type=positive, default=3.0, help="Joint speed, deg/s")
    parser.add_argument("--acceleration", type=positive, default=10.0, help="Joint acceleration, deg/s^2")
    parser.add_argument("--settle", type=positive, default=0.8)
    parser.add_argument("--intrinsics", type=Path)
    parser.add_argument("--fit-intrinsics", action="store_true",
                        help="Fit camera intrinsics from captured views (automatic for unsupported factory distortion)")
    parser.add_argument("--headless", action="store_true", help="No preview window; Ctrl+C requests stop")
    parser.add_argument("--dry-run", action="store_true", help="Read/plan only, without enabling motors or moving")
    parser.add_argument("--plan-only", action="store_true", help="Check and save a robot motion plan for Viser; no camera or motion")
    args = parser.parse_args(argv)
    if args.start_plan and args.plan_only:
        parser.error("--start-plan requires camera monitoring; use --dry-run instead of --plan-only")
    if not 5 <= args.radius_mm <= 50 or not 10 <= args.tilt_deg <= 20:
        parser.error("Local sweep requires radius 5-50 mm and tilt 10-20 deg")
    if not (50 <= args.forward_mm <= 300 and 20 <= args.lift_mm <= 200 and 20 <= args.side_mm <= 150):
        parser.error("Wide sweep requires forward 50-300 mm, lift 20-200 mm and side 20-150 mm")
    if not (np.isfinite(args.drop_mm) and 0 <= args.drop_mm <= 150 and 20 <= args.lower_side_mm <= 250):
        parser.error("Wide lower row requires drop 0-150 mm and lower-side 20-250 mm")
    if args.speed > 5 or args.acceleration > 20 or args.settle < 0.3:
        parser.error("Require speed <=5 deg/s, acceleration <=20 deg/s^2, settle >=0.3 s")
    if args.intrinsics and args.fit_intrinsics:
        parser.error("Choose --intrinsics or --fit-intrinsics, not both")
    if min(args.width, args.height, args.fps) <= 0:
        parser.error("Stream dimensions and fps must be positive")
    return args


def main(argv=None):
    run(parse_args(argv))


if __name__ == "__main__":
    run_cli(main)
