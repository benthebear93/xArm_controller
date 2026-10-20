"""Calibrate a fixed RealSense RGB camera against an xArm flange-mounted ChArUco board.

The robot connection is read-only. Move between poses using Studio/teach_pendant.
Run --help or read CALIBRATION.md for the complete procedure.
"""

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np

from calibration_core import (
    BoardDetector, DEFAULT_BOARD, InsufficientCornersError, METHODS, Projection, camera_arrays,
    fit_intrinsics, residuals, solve_eye_to_hand, summarize_residuals,
)
from geometry import finite_array, flange_from_tcp, inverse, pose_error, validate_transform
from io_utils import read_json, utc_now, write_json
from realsense_camera import RealSenseCamera, add_camera_arguments, require_same_camera
from robot_common import (
    ROBOT_IP, positive, read_result, require_clear_status, require_idle, robot_session, run_cli,
)


def read_robot(arm):
    require_clear_status(arm)
    require_idle(arm)
    tcp_offset = finite_array(arm.tcp_offset, (6,), "TCP offset").tolist()
    world_offset = finite_array(arm.world_offset, (6,), "world offset").tolist()
    pose = finite_array(read_result(arm.get_position(is_radian=False), "Read TCP pose"),
                        (6,), "TCP pose").tolist()
    joints = finite_array(read_result(
        arm.get_servo_angle(is_radian=False, is_real=True), "Read actual joints",
    ), (7,), "joint angles").tolist()
    base_flange = flange_from_tcp(pose, tcp_offset, world_offset)
    require_clear_status(arm)
    require_idle(arm)
    return {
        "tcp_pose_mm_deg": pose, "tcp_offset_mm_deg": tcp_offset,
        "world_offset_mm_deg": world_offset, "joints_deg": joints,
        "T_base_flange": base_flange.tolist(),
        "host_monotonic_s": time.monotonic(), "host_utc": utc_now(),
    }


def assert_stationary(reference, current):
    for key in ("tcp_offset_mm_deg", "world_offset_mm_deg"):
        if not np.allclose(reference[key], current[key], atol=1e-6, rtol=0):
            raise ValueError("Robot coordinate offsets changed during capture")
    position, rotation = pose_error(reference["T_base_flange"], current["T_base_flange"])
    joint_delta = np.max(np.abs(np.asarray(reference["joints_deg"]) - current["joints_deg"]))
    if position > 0.3 or rotation > 0.1 or joint_delta > 0.1:
        raise ValueError(
            f"Robot moved during capture ({position:.3f} mm, {rotation:.3f} deg, "
            f"joint {joint_delta:.3f} deg); stop and retry"
        )


def capture_sample(arm, camera, detector, camera_model, previous, settle):
    first = read_robot(arm)
    checks = [first]
    for _ in range(4):
        time.sleep(settle / 4)
        current = read_robot(arm)
        assert_stationary(first, current)
        checks.append(current)
    for attempt in range(1, 4):
        if attempt > 1:
            time.sleep(.15)
            before_retry = read_robot(arm)
            assert_stationary(first, before_retry)
            checks.append(before_retry)
        image, timestamp = camera.fresh()
        last = read_robot(arm)
        assert_stationary(first, last)
        checks.append(last)
        detection = detector.detect_for_capture(image)
        try:
            detector.points(detection)
            break
        except InsufficientCornersError as exc:
            if attempt == 3:
                raise InsufficientCornersError(f"Capture failed after 3 fresh frames: {exc}") from exc
            print(f"Capture attempt {attempt}/3: {exc}; retrying at the same stationary pose", flush=True)
    for sample in previous:
        distance, angle = pose_error(last["T_base_flange"], sample["robot"]["T_base_flange"])
        if distance < 5 and angle < 3:
            raise ValueError("Duplicate pose: change position by >=5 mm or orientation by >=3 deg")
    record = {
        "captured_utc": utc_now(), "robot": last, "stationary_checks": checks,
        "camera_timestamp": timestamp, "detection": detection, "detection_attempts": attempt,
    }
    if camera_model is not None:
        pose, rms = detector.estimate(detection, camera_model)
        if rms > 2:
            raise ValueError(f"Reprojection error {rms:.2f} px exceeds 2 px; check focus/board")
        record.update(T_camera_board=pose.tolist(), reprojection_rms_px=rms)
    return image, record


def overlay(image, detection, detector, camera, status):
    result = image.copy()
    if detection["ids"]:
        cv2.aruco.drawDetectedCornersCharuco(
            result, np.asarray(detection["corners"], np.float32).reshape(-1, 1, 2),
            np.asarray(detection["ids"], np.int32).reshape(-1, 1),
        )
    total_corners = len(detector.board.getChessboardCorners())
    lines = [f"Markers: {detection['marker_count']}   Corners: {len(detection['ids'])}/{total_corners}"]
    if camera is not None:
        try:
            pose, rms = detector.estimate(detection, camera)
            axes = np.array([[0, 0, 0], [.06, 0, 0], [0, .06, 0], [0, 0, .06]])
            pixels = Projection(camera).project(axes @ pose[:3, :3].T + pose[:3, 3])
            pixels = np.round(pixels).astype(int)
            for point, color in zip(pixels[1:], ((0, 0, 255), (0, 255, 0), (255, 0, 0))):
                cv2.line(result, tuple(pixels[0]), tuple(point), color, 2)
            lines[0] += f"   Reprojection: {rms:.2f} px"
        except (ValueError, cv2.error):
            pass
    lines.extend(status)
    for i, line in enumerate(lines):
        location = (12, 26 + 27 * i)
        size, baseline = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(result, (8, location[1] - size[1] - 4),
                      (min(result.shape[1] - 1, 16 + size[0]), location[1] + baseline + 3),
                      (0, 0, 0), cv2.FILLED)
        cv2.putText(result, line, location, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 1, cv2.LINE_AA)
    return result


def choose_camera(factory, intrinsics_path):
    if intrinsics_path is None:
        return factory
    custom = read_json(intrinsics_path)
    if custom["image_size"] != factory["image_size"]:
        raise ValueError("Custom intrinsics image_size does not match the color stream")
    if custom.get("serial", factory["serial"]) != factory["serial"]:
        raise ValueError("Custom intrinsics belong to a different camera serial")
    camera_arrays(custom)
    return dict(factory, matrix=custom["matrix"], dist_coeffs=custom["dist_coeffs"],
                distortion_model=custom["distortion_model"], source=str(intrinsics_path))


def session_path(directory):
    return Path(directory) / "session.json"


def new_session(robot_ip, board, camera, **metadata):
    return {
        "schema_version": 1, "setup": "eye_to_hand", "translation_unit": "m",
        "created_utc": utc_now(), "robot_ip": robot_ip,
        "board": board, "camera": camera, "samples": [], **metadata,
    }


def save_sample(directory, session, image, record):
    """Persist an image first, then atomically reference it in the session."""
    directory = Path(directory)
    (directory / "images").mkdir(parents=True, exist_ok=True)
    sample_id = max((s["id"] for s in session["samples"]), default=0) + 1
    image_path = Path("images") / f"{sample_id:04d}.png"
    while (directory / image_path).exists():
        sample_id += 1
        image_path = Path("images") / f"{sample_id:04d}.png"
    if not cv2.imwrite(str(directory / image_path), image):
        raise RuntimeError(f"Cannot write {image_path}")
    record = dict(record, id=sample_id, image=image_path.as_posix())
    session["samples"].append(record)
    write_json(session_path(directory), session)
    return record


def load_session(directory):
    result = read_json(session_path(directory))
    if result.get("schema_version") != 1 or result.get("setup") != "eye_to_hand":
        raise ValueError("Unsupported calibration session format")
    if result.get("translation_unit") != "m":
        raise ValueError("Session transform translations must be in meters")
    return result


def collect(args):
    config = read_json(args.board)
    directory = Path(args.session)
    if session_path(directory).exists() and not args.resume:
        raise ValueError("Session exists: use --resume without moving the camera/board mount, or a new --session")
    if args.resume and not session_path(directory).exists():
        raise ValueError("No existing session to resume")
    with RealSenseCamera(args) as camera:
        camera_config = choose_camera(camera.metadata, args.intrinsics)
        try:
            Projection(camera_config)
            pose_camera = camera_config
        except ValueError as exc:
            print(f"Camera pose preview disabled: {exc}")
            pose_camera = None
        # Homography interpolation keeps stored observations independent of factory intrinsics.
        detector = BoardDetector(config)
        if args.resume:
            session = load_session(directory)
            if (session["board"] != config or session["camera"] != camera_config
                    or session["robot_ip"] != args.ip):
                raise ValueError("Cannot resume: board, camera parameters, or robot IP changed")
        else:
            session = new_session(args.ip, config, camera_config)
        with robot_session(args.ip, dry_run=True) as arm:
            read_robot(arm)
            write_json(session_path(directory), session)
            (directory / "images").mkdir(exist_ok=True)
            window = "Eye-to-hand: SPACE/S capture, Q/Esc quit"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            status = "Move with Studio/pendant, stop, then SPACE/S. Aim for 20-30 views."
            print(f"Session: {directory.resolve()}\nRead-only robot connection; {status}")
            try:
                while True:
                    frame, _ = camera.read()
                    detection = detector.detect(frame)
                    view = overlay(frame, detection, detector, pose_camera,
                                   [f"Saved: {len(session['samples'])}", status])
                    cv2.imshow(window, view)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if key not in (ord("s"), 32):
                        continue
                    try:
                        frame, record = capture_sample(
                            arm, camera, detector, pose_camera, session["samples"], args.settle,
                        )
                    except (ValueError, RuntimeError, cv2.error) as exc:
                        status = f"Rejected: {exc}"
                        print(status)
                        continue
                    record = save_sample(directory, session, frame, record)
                    status = f"Saved #{record['id']}: {len(record['detection']['ids'])} corners"
                    print(status)
            finally:
                cv2.destroyAllWindows()
    print(f"Saved {len(session['samples'])} samples. Next: solve --session {directory}")


def check_board(args):
    config = read_json(args.board)
    detector = BoardDetector(config)
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError(f"Cannot read image: {args.image}; render a PDF to PNG first")
    detection = detector.detect(image)
    detector.points(detection)
    print(f"Detected {detection['marker_count']} markers, {len(detection['ids'])} ChArUco corners")
    print(f"Columns/rows={config['squares_columns_rows']}, square={config['square_size_mm']} mm, "
          f"marker={config['marker_size_mm']} mm, dictionary={config['dictionary']}")
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), overlay(image, detection, detector, None, [])):
            raise RuntimeError(f"Cannot write {args.output}")


def prepare_observations(args, session, camera):
    detector = BoardDetector(session["board"])
    selected = [s for s in session["samples"] if s["id"] not in args.exclude]
    unknown = set(args.exclude) - {s["id"] for s in session["samples"]}
    if unknown:
        raise ValueError(f"Unknown excluded sample IDs: {sorted(unknown)}")
    if args.fit_intrinsics:
        camera = fit_intrinsics(detector, selected, camera)
    Projection(camera)
    observations, rejected = [], []
    for sample in selected:
        try:
            pose, rms = detector.estimate(sample["detection"], camera)
            if rms > args.max_reprojection_px:
                raise ValueError(f"Reprojection RMS {rms:.3f} px exceeds limit")
            robot = sample["robot"]
            # Reconstruct from raw SDK values to verify offset and unit conventions.
            flange = flange_from_tcp(robot["tcp_pose_mm_deg"], robot["tcp_offset_mm_deg"],
                                     robot["world_offset_mm_deg"])
            if not np.allclose(flange, validate_transform(robot["T_base_flange"]), atol=1e-7):
                raise ValueError("Stored robot transform disagrees with raw TCP pose/offset")
            observations.append((sample["id"], flange, pose, rms))
        except (ValueError, cv2.error) as exc:
            rejected.append({"id": sample["id"], "reason": str(exc)})
    return observations, rejected, camera


def solve(args):
    session = load_session(args.session)
    camera = choose_camera(session["camera"], args.intrinsics)
    observations, rejected, camera = prepare_observations(args, session, camera)
    ids = [o[0] for o in observations]
    base_flange, camera_board = [o[1] for o in observations], [o[2] for o in observations]
    base_camera, flange_board, errors, excitation = solve_eye_to_hand(
        base_flange, camera_board, "park" if args.method == "joint" else args.method,
    )
    optimization = None
    if args.method == "joint":
        from joint_calibration import refine_eye_to_hand

        selected = {sample["id"]: sample for sample in session["samples"]}
        base_camera, flange_board, optimization = refine_eye_to_hand(
            [selected[sample_id] for sample_id in ids], session["board"], camera, base_camera, flange_board,
        )
        errors = residuals(base_flange, camera_board, base_camera, flange_board)
    metrics = summarize_residuals(errors)
    acceptable = passes_residual_limits(metrics, args)
    result = {
        "schema_version": 1, "setup": "eye_to_hand", "created_utc": utc_now(),
        "session": str(Path(args.session).resolve()), "translation_unit": "m",
        "convention": "T_a_b maps a point in frame b into frame a: p_a = R_a_b @ p_b + t_a_b",
        "closure": "T_base_flange @ T_flange_board = T_base_camera @ T_camera_board",
        "board_frame": "Top-left OUTER board corner; x right, y down, z into printed board",
        "camera_frame": "RealSense color optical frame (not depth optical/device frame)",
        "T_base_camera": base_camera.tolist(), "T_camera_base": inverse(base_camera).tolist(),
        "T_flange_board": flange_board.tolist(), "T_board_flange": inverse(flange_board).tolist(),
        "board": session["board"], "camera": camera, "method": args.method,
        "opencv_version": cv2.__version__,
        "solver_backend": "scipy_least_squares" if optimization else (
            "opencv" if hasattr(cv2, "calibrateHandEye") else "numpy_park"),
        "optimization": optimization,
        "sample_ids": ids, "excluded_sample_ids": args.exclude, "rejected_samples": rejected,
        "excitation": excitation, "residuals": metrics,
        "quality": {
            "passed_training_residual_limits": acceptable,
            "max_translation_rms_mm": args.max_translation_rms_mm,
            "max_rotation_rms_deg": args.max_rotation_rms_deg,
            "independent_new_capture_verified": False,
            "note": "Training residuals are not absolute accuracy. Verify on newly captured poses.",
        },
        "per_sample": sample_residuals(observations, errors),
    }
    output = Path(args.output) if args.output else Path(args.session) / "calibration.json"
    if output.resolve() == session_path(args.session).resolve():
        raise ValueError("Output must not overwrite session.json")
    write_json(output, result)
    print(f"Used {len(ids)} views; rejected {len(rejected)}; excluded {len(args.exclude)}")
    print_metrics(metrics)
    print("T_base_camera (translations in meters):\n", np.array2string(base_camera, precision=8))
    print(f"Result: {output.resolve()}")
    if not acceptable:
        raise ValueError("Saved diagnostic result FAILED residual limits; inspect per_sample and recapture")
    print("Training residual limits passed. Run verify on a separate collection of new poses.")


def print_metrics(metrics):
    print(f"Closure translation RMS/max: {metrics['translation_rms_mm']:.3f} / "
          f"{metrics['translation_max_mm']:.3f} mm")
    print(f"Closure rotation RMS/max: {metrics['rotation_rms_deg']:.3f} / "
          f"{metrics['rotation_max_deg']:.3f} deg")


def passes_residual_limits(metrics, args):
    return (metrics["translation_rms_mm"] <= args.max_translation_rms_mm
            and metrics["rotation_rms_deg"] <= args.max_rotation_rms_deg)


def sample_residuals(observations, errors):
    return [
        {"id": observation[0], "reprojection_rms_px": observation[3],
         "translation_error_mm": error[0], "rotation_error_deg": error[1]}
        for observation, error in zip(observations, errors)
    ]


def verify(args):
    if args.fit_intrinsics:
        raise ValueError("Verification must not refit camera intrinsics")
    session = load_session(args.session)
    calibration = read_json(args.calibration)
    if (calibration.get("setup") != "eye_to_hand" or calibration.get("translation_unit") != "m"
            or calibration["board"] != session["board"]):
        raise ValueError("Calibration setup/units/board do not match the verification session")
    if Path(calibration["session"]).resolve() == Path(args.session).resolve():
        raise ValueError("Use a separate session of NEW poses for independent verification")
    require_same_camera(session["camera"], calibration["camera"])
    observations, rejected, _ = prepare_observations(args, session, calibration["camera"])
    report = {
        "schema_version": 1, "created_utc": utc_now(), "session": str(Path(args.session).resolve()),
        "calibration": str(Path(args.calibration).resolve()), "calibration_refit": False,
        "source_calibration": session.get("verification", {}).get("source_calibration", str(args.calibration)),
        "sample_ids": [o[0] for o in observations], "excluded_sample_ids": args.exclude,
        "rejected_samples": rejected, "passed": False,
        "limits": {"translation_rms_mm": args.max_translation_rms_mm,
                   "rotation_rms_deg": args.max_rotation_rms_deg},
        "note": "New-pose consistency check; not a measurement of absolute workspace accuracy.",
    }
    report_path = Path(args.session) / "verification.json"
    if len(observations) < 3:
        report["failure_reason"] = "Verification needs at least 3 new, valid poses"
        write_json(report_path, report)
        raise ValueError("Verification needs at least 3 new, valid poses")
    base_camera = validate_transform(calibration["T_base_camera"])
    flange_board = validate_transform(calibration["T_flange_board"])
    errors = residuals([o[1] for o in observations], [o[2] for o in observations],
                       base_camera, flange_board)
    metrics = summarize_residuals(errors)
    passed = passes_residual_limits(metrics, args)
    report.update(passed=passed, residuals=metrics, per_sample=sample_residuals(observations, errors))
    if not passed:
        report["failure_reason"] = "Independent verification FAILED residual limits"
    write_json(report_path, report)
    print(f"Verified {len(errors)} poses; rejected {len(rejected)}")
    for observation, error in zip(observations, errors):
        print(f"  #{observation[0]}: {error[0]:.3f} mm, {error[1]:.3f} deg")
    print_metrics(metrics)
    print(f"Verification report: {report_path}")
    if not passed:
        raise ValueError("Independent verification FAILED residual limits")
    print("Independent verification passed residual limits.")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collect_parser = commands.add_parser("collect", help="Preview RGB and save stationary paired samples")
    collect_parser.add_argument("--ip", default=ROBOT_IP)
    add_camera_arguments(collect_parser)
    collect_parser.add_argument("--settle", type=positive, default=0.6, help="Stationary observation duration, seconds")
    collect_parser.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    collect_parser.add_argument("--intrinsics", type=Path, help="OpenCV color camera intrinsics JSON")
    collect_parser.add_argument("--session", type=Path, default=Path("output/calibration") / datetime.now().strftime("%Y%m%d_%H%M%S"))
    collect_parser.add_argument("--resume", action="store_true", help="Append ONLY if camera and mount are unchanged")
    collect_parser.set_defaults(func=collect)

    check_parser = commands.add_parser("check-board", help="Check board configuration against a PNG/JPEG, offline")
    check_parser.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    check_parser.add_argument("--image", type=Path, required=True)
    check_parser.add_argument("--output", type=Path, help="Optional annotated PNG")
    check_parser.set_defaults(func=check_board)

    for name, handler in (("solve", solve), ("verify", verify)):
        command = commands.add_parser(name, help="Solve calibration offline" if name == "solve" else "Check calibration against a separate session")
        command.add_argument("--session", type=Path, required=True)
        command.add_argument("--exclude", type=int, nargs="*", default=[], help="Sample IDs to exclude explicitly")
        command.add_argument("--max-reprojection-px", type=positive, default=2.0)
        command.add_argument("--max-translation-rms-mm", type=positive, default=3.0)
        command.add_argument("--max-rotation-rms-deg", type=positive, default=1.0)
        command.set_defaults(func=handler, fit_intrinsics=False)
        if name == "solve":
            command.add_argument("--method", choices=["joint", *METHODS], default="joint",
                                 help="Default: joint pixel optimization with fixed intrinsics; Park initializes the fit")
            command.add_argument("--intrinsics", type=Path)
            command.add_argument("--fit-intrinsics", action="store_true", help="Fit OpenCV intrinsics from these ChArUco views")
            command.add_argument("--output", type=Path)
        else:
            command.add_argument("--calibration", type=Path, required=True)
    args = parser.parse_args(argv)
    if getattr(args, "settle", 0.6) < 0.3:
        parser.error("--settle must be at least 0.3 seconds")
    if getattr(args, "fit_intrinsics", False) and getattr(args, "intrinsics", None):
        parser.error("Choose --intrinsics or --fit-intrinsics, not both")
    args.func(args)


if __name__ == "__main__":
    run_cli(main)
