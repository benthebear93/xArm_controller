"""Selection and new-pose geometry for automatic verification; no hardware IO."""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from calibration_core import Projection
from geometry import pose_error, validate_transform
from io_utils import read_json


def load_reference(path, robot_ip, board, destination):
    path = Path(path).resolve()
    calibration = read_json(path)
    if (calibration.get("setup") != "eye_to_hand" or calibration.get("translation_unit") != "m"
            or calibration.get("board") != board):
        raise ValueError("Verification calibration setup/units/board do not match")
    if not calibration.get("quality", {}).get("passed_training_residual_limits"):
        raise ValueError("Automatic verification requires a calibration that passed training residual limits")
    if calibration.get("board_geometry"):
        raise ValueError("Automatic verification supports the fixed planar board model only")
    for key in ("T_base_camera", "T_flange_board"):
        validate_transform(calibration[key])
    Projection(calibration["camera"])
    source = Path(calibration["session"]).resolve()
    if source == Path(destination).resolve():
        raise ValueError("Verification must use a NEW session")
    session = read_json(source / "session.json")
    if session.get("robot_ip") != robot_ip or session.get("board") != board:
        raise ValueError("Calibration training session belongs to a different robot or board")
    ids = set(calibration["sample_ids"])
    samples = [s for s in session["samples"] if s["id"] in ids]
    if len(samples) != len(ids) or not samples:
        raise ValueError("Calibration training poses are missing; cannot check verification pose independence")
    return calibration, [validate_transform(s["robot"]["T_base_flange"]) for s in samples]


def latest_calibration(directory, robot_ip):
    candidates = []
    for path in Path(directory).glob("*/calibration*.json"):
        calibration = read_json(path)
        if (calibration.get("setup") != "eye_to_hand"
                or not calibration.get("quality", {}).get("passed_training_residual_limits")):
            continue
        source = Path(calibration["session"]) / "session.json"
        if source.exists() and read_json(source).get("robot_ip") == robot_ip:
            candidates.append((datetime.fromisoformat(calibration["created_utc"]).timestamp(), str(path), path))
    if not candidates:
        raise ValueError("No successful calibration found; specify --calibration PATH")
    return max(candidates)[2]


def require_new_pose(pose, training):
    # Same tolerance is applied to planned goals and measured capture poses.
    if any(distance < 10 and angle < 5 for distance, angle in (pose_error(pose, old) for old in training)):
        raise ValueError("Verification pose is too close to a training pose (<10 mm and <5 deg); "
                         "choose a different start pose or sweep dimensions")


def generate_verification_targets(seed_flange, args):
    """Ten new local (default) or wide locations, plus a stationary seed.

    Local targets stay 0.8 * radius from the current seed. Wide targets use
    interior fractions of the requested sweep. The seed is NOT a capture.
    """
    seed = validate_transform(seed_flange)
    targets = [seed.copy()]
    # (forward fraction, lateral fraction, vertical fraction). Negative Z uses drop.
    locations = [( .25, -.35, .10), (.48, .45, .22), (.70, -.50, .32),
                 ( .92,  .25, .12), (.62, .80, .38), (.40, -.75, .18),
                 ( .75, -.55,-.45), (.60, .05,-.70), (.90,  .55,-.60), (.45, .35,-.30)]
    for index, (x, y, z) in enumerate(locations):
        target = seed.copy()
        phase = 2*np.pi*index/10 + np.pi/10
        if args.pattern == "wide":
            side = args.lower_side_mm if z < 0 and args.drop_mm > 0 else args.side_mm
            height = z * (args.drop_mm if z < 0 else args.lift_mm)
            shift = np.array([x*args.forward_mm, y*side, height]) / 1000
        else:
            direction = np.array([np.cos(phase), np.sin(phase), (-1)**index*.5])
            direction *= .8*args.radius_mm / (1000*np.linalg.norm(direction))
            shift = seed[:3, :3] @ direction
        target[:3, 3] += shift
        tilt_phase = phase + np.pi/2
        axis = np.array([np.cos(tilt_phase), np.sin(tilt_phase), 0.])
        target[:3, :3] = seed[:3, :3] @ cv2.Rodrigues(axis*np.radians(.8*args.tilt_deg))[0]
        targets.append(target)
    return targets
