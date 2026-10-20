"""Review camera coverage and propose image-grid / nearer-board poses, without motion.

Input is an RGB snapshot and its exact camera metadata. Output targets are in
camera coordinates until a preliminary eye-to-hand calibration is supplied.
These are acquisition goals, NOT a collision-checked robot motion plan.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from calibration_core import BoardDetector, DEFAULT_BOARD, Projection
from geometry import inverse, transform, validate_transform
from io_utils import read_json, utc_now, write_json
from realsense_camera import RealSenseCamera, add_camera_arguments
from robot_common import run_cli


def board_geometry(board):
    width, height = np.asarray(board["squares_columns_rows"]) * board["square_size_mm"] / 1000
    corners = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]])
    # Sample the full patterned boundary, not just detected interior corners.
    boundary = np.concatenate([a + np.linspace(0, 1, 20)[:, None] * (b - a)
                               for a, b in zip(corners, np.roll(corners, -1, axis=0))])
    return np.array([width / 2, height / 2, 0]), boundary


def camera_points(pose, points):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]


def analyze(image, camera, board):
    projection = Projection(camera)
    if list(image.shape[1::-1]) != list(camera["image_size"]):
        raise ValueError("Image resolution does not match camera metadata")
    detector = BoardDetector(board)
    detection = detector.detect(image)
    pose, rms = detector.estimate(detection, camera)
    if rms > 2:
        raise ValueError(f"Board pose reprojection error too large: {rms:.2f} px")
    center, boundary = board_geometry(board)
    center_3d = camera_points(pose, center)
    center_px = projection.project(center_3d)[0]
    outline = projection.project(camera_points(pose, boundary))
    full = bool(np.all(outline >= 0) and np.all(outline < projection.size))
    return {
        "detected_corners": len(detection["ids"]),
        "total_corners": len(detector.board.getChessboardCorners()),
        "center_px": center_px.tolist(), "center_normalized": (center_px / projection.size).tolist(),
        "center_camera_m": center_3d.tolist(), "pattern_fully_visible": full,
        "pattern_bounds_px": [outline.min(axis=0).tolist(), outline.max(axis=0).tolist()],
        "reprojection_rms_px": rms, "T_camera_board": pose.tolist(),
        "pose_note": "Single-view factory-intrinsics estimate; not a hand-eye calibration.",
    }, detection, projection


def propose(observation, projection, board, depth_factors=(1., .85, .70), tilt_deg=15, margin_px=35):
    """27 goals: 3x3 screen positions at three depths, varied pitch/yaw/roll.

    A board-center ray sets the location. Rotation is about that center, so a
    tilted board does not drift to another image cell. Clipped goals are reported
    explicitly rather than silently dropped or shrunk toward the seed pose.
    """
    seed = validate_transform(observation["T_camera_board"])
    center, boundary = board_geometry(board)
    seed_center = camera_points(seed, center)
    if not 0 < min(depth_factors) <= max(depth_factors) <= 1.5:
        raise ValueError("Depth factors must be positive and <= 1.5")
    goals = []
    for layer, factor in enumerate(depth_factors):
        depth = float(seed_center[2] * factor)
        for row, y in enumerate((.29, .5, .71)):
            # Serpentine reduces large left-right jumps between rows.
            columns = (0, 1, 2) if row % 2 == 0 else (2, 1, 0)
            for column in columns:
                uv = np.array([(.3, .5, .7)[column], y]) * projection.size
                ray = projection.rays([uv])[0]
                target_center = ray * depth / ray[2]
                angles = np.radians(tilt_deg) * np.array([
                    (row - 1) * .7, (column - 1) * .7, (layer - 1) * .35])
                # Face the camera before adding tilt to avoid oblique edge views.
                normal = target_center / np.linalg.norm(target_center)
                x_axis = seed[:3, 0] - normal * np.dot(seed[:3, 0], normal)
                if np.linalg.norm(x_axis) < 1e-6:
                    raise ValueError("Cannot construct a camera-facing board orientation")
                x_axis /= np.linalg.norm(x_axis)
                face_camera = np.column_stack([x_axis, np.cross(normal, x_axis), normal])
                rotation = face_camera @ cv2.Rodrigues(angles)[0]
                pose = transform(rotation, target_center - rotation @ center)
                outline = projection.project(camera_points(pose, boundary))
                inside = bool(np.all(outline >= margin_px) and
                              np.all(outline <= projection.size - margin_px))
                goals.append({
                    "index": len(goals) + 1, "row": row, "column": column, "depth_layer": layer,
                    "center_px": uv.tolist(), "depth_factor": factor, "depth_m": depth,
                    "toward_camera_mm": float(1000 * (seed_center[2] - depth)),
                    "board_center_displacement_mm": float(1000 * np.linalg.norm(target_center - seed_center)),
                    "T_camera_board": pose.tolist(), "pattern_outline_px": outline.tolist(),
                    "image_margin_passed": inside,
                })
    return goals


def map_to_robot(goals, calibration):
    """Coordinate conversion only; never imply IK, path or collision validation."""
    if calibration.get("setup") != "eye_to_hand" or calibration.get("translation_unit") != "m":
        raise ValueError("Expected an eye-to-hand calibration with meter translations")
    if not calibration.get("quality", {}).get("passed_training_residual_limits", False):
        raise ValueError("Preliminary calibration did not pass its residual limits")
    base_camera = validate_transform(calibration["T_base_camera"])
    board_flange = inverse(calibration["T_flange_board"])
    return [dict(g, T_base_flange=(base_camera @ validate_transform(g["T_camera_board"]) @ board_flange).tolist())
            for g in goals]


def render_review(image, observation, detection, goals, destination):
    h, w = image.shape[:2]
    panels = []
    for layer in range(3):
        canvas = (image.astype(float) * .42).astype(np.uint8)
        selected = [g for g in goals if g["depth_layer"] == layer]
        for goal in selected:
            color = (80, 235, 120) if goal["image_margin_passed"] else (60, 80, 255)
            outline = np.round(goal["pattern_outline_px"]).astype(np.int32)
            cv2.polylines(canvas, [outline], True, color, 2, cv2.LINE_AA)
            u, v = np.round(goal["center_px"]).astype(int)
            cv2.drawMarker(canvas, (u, v), color, cv2.MARKER_CROSS, 14, 2)
            cv2.putText(canvas, f'{goal["index"]:02d}', (u + 12, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2, cv2.LINE_AA)
        for point in detection["corners"]:
            cv2.circle(canvas, tuple(np.round(point).astype(int)), 3, (0, 220, 255), -1)
        header = np.full((70, w, 3), 25, dtype=np.uint8)
        goal = selected[0]
        label = (f'Layer {layer + 1}: depth {goal["depth_m"] * 100:.0f} cm | '
                 f'toward camera {goal["toward_camera_mm"]:.0f} mm | GOALS ONLY - no robot path checked')
        cv2.putText(header, label, (20, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(header, 'Yellow dots: current detections. Green: pattern inside 35px margin. Red: clipped.',
                    (20, 55), cv2.FONT_HERSHEY_SIMPLEX, .55, (160, 200, 230), 1, cv2.LINE_AA)
        panels.append(np.vstack([header, canvas]))
    if not cv2.imwrite(str(destination), np.vstack(panels)):
        raise RuntimeError(f"Cannot write {destination}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--camera", type=Path)
    parser.add_argument("--capture", action="store_true", help="Capture a fresh RGB frame and metadata; no robot connection")
    add_camera_arguments(parser)
    parser.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    parser.add_argument("--calibration", type=Path, help="Optional preliminary calibration.json for robot-frame goals")
    parser.add_argument("--output", type=Path, default=Path("output/calibration/coverage_review"))
    args = parser.parse_args(argv)
    if args.capture:
        if args.image or args.camera:
            parser.error("Use --capture or --image plus --camera, not both")
        args.output.mkdir(parents=True, exist_ok=True)
        args.image, args.camera = args.output / "current.png", args.output / "camera.json"
        with RealSenseCamera(args) as camera_device:
            image, timestamp = camera_device.fresh()
            if not cv2.imwrite(str(args.image), image):
                raise RuntimeError("Could not save camera snapshot")
            write_json(args.camera, camera_device.metadata)
            write_json(args.output / "timestamp.json", timestamp)
    elif args.image is None or args.camera is None:
        parser.error("Provide --capture or both --image and --camera")
    camera, board = read_json(args.camera), read_json(args.board)
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError(f"Cannot read {args.image}")
    calibration = read_json(args.calibration) if args.calibration else None
    if calibration is not None:
        if (calibration["camera"].get("serial") != camera.get("serial")
                or calibration["camera"]["image_size"] != camera["image_size"]
                or calibration["board"] != board):
            raise ValueError("Camera serial/resolution or board differs from preliminary calibration")
        camera = calibration["camera"]
    observation, detection, projection = analyze(image, camera, board)
    goals = propose(observation, projection, board)
    if calibration is not None:
        goals = map_to_robot(goals, calibration)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "created_utc": utc_now(), "kind": "camera_coverage_goals_not_motion_plan",
              "image": str(args.image.resolve()), "camera": camera, "board": board,
              "observation": observation, "goals": goals,
              "robot_frame_available": calibration is not None,
              "robot_reachability_checked": False, "collision_checked": False,
              "note": "Full patterned board margin only; paper, mount, cables and obstacles are not modeled."}
    write_json(args.output / "coverage_goals.json", report)
    render_review(image, observation, detection, goals, args.output / "coverage_review.png")
    print(f"Current: {observation['detected_corners']}/{observation['total_corners']} corners, "
          f"center={np.round(observation['center_px'], 1)}, full pattern={observation['pattern_fully_visible']}")
    print(f"Board center depth: {observation['center_camera_m'][2]:.3f} m; reprojection RMS "
          f"{observation['reprojection_rms_px']:.3f} px (single-view estimate)")
    for layer in range(3):
        group = [g for g in goals if g["depth_layer"] == layer]
        print(f"Layer {layer + 1}: {group[0]['depth_m']:.3f} m, "
              f"{sum(g['image_margin_passed'] for g in group)}/9 fit image; "
              f"board-center movement {min(g['board_center_displacement_mm'] for g in group):.0f}"
              f"-{max(g['board_center_displacement_mm'] for g in group):.0f} mm")
    print(f"Review: {args.output / 'coverage_review.png'}")
    print("No robot connected or moved. Robot IK, workspace and path checks remain required.")


if __name__ == "__main__":
    run_cli(main)
