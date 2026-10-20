"""Inspect saved calibration frames and motion diversity; never connect to hardware.

Diagnostic hand-eye fitting deliberately reports failed quality/excitation checks
and does NOT export transforms that could be mistaken for a usable calibration.
"""

import argparse
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

from calibration_core import check_excitation, park_eye_to_hand, residuals, summarize_residuals
from geometry import average_transform, inverse, pose_error
from io_utils import read_json, write_json
from plan_camera_coverage import analyze, board_geometry, camera_points
from robot_common import run_cli


def review(directory, output):
    directory, output = Path(directory), Path(output)
    session = read_json(directory / "session.json")
    samples = session["samples"]
    if not samples:
        raise ValueError("Session has no saved images")
    output.mkdir(parents=True, exist_ok=True)
    poses, observations, details, tiles = [], [], [], []
    for sample in samples:
        image = cv2.imread(str(directory / sample["image"]))
        if image is None:
            raise ValueError(f"Missing image for sample {sample['id']}")
        observation, detection, projection = analyze(image, session["camera"], session["board"])
        poses.append(np.asarray(sample["robot"]["T_base_flange"]))
        observations.append(np.asarray(observation["T_camera_board"]))
        details.append(dict(id=sample["id"], stored_corner_count=len(sample["detection"]["ids"]), **observation))
        _, boundary = board_geometry(session["board"])
        outline = projection.project(camera_points(observations[-1], boundary))
        cv2.polylines(image, [np.round(outline).astype(np.int32)], True,
                      (80, 220, 80) if observation["pattern_fully_visible"] else (30, 60, 255), 2)
        for point in detection["corners"]:
            cv2.circle(image, tuple(np.round(point).astype(int)), 3, (0, 220, 255), -1)
        x, y = np.round(observation["center_px"]).astype(int)
        x0 = int(np.clip(x - 160, 0, max(0, image.shape[1] - 320)))
        y0 = int(np.clip(y - 140, 0, max(0, image.shape[0] - 280)))
        crop = cv2.resize(image[y0:y0 + 280, x0:x0 + 320], (320, 280))
        tile = np.zeros((330, 320, 3), np.uint8)
        tile[50:] = crop
        cv2.putText(tile, f"#{sample['id']} | {len(detection['ids'])}/{observation['total_corners']} corners", (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(tile, f"pattern fully in frame: {observation['pattern_fully_visible']}", (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (180, 200, 230), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    montage = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
    if not cv2.imwrite(str(output / "frames_review.png"), montage):
        raise RuntimeError("Could not write frame review")
    rejection = []
    try:
        excitation = check_excitation(poses)
    except ValueError as exc:
        excitation = {"passed": False, "reason": str(exc)}
        rejection.append(str(exc))
    if len(samples) < 12 and session.get("fit_intrinsics"):
        rejection.append(f"Only {len(samples)} images; intrinsic fitting requires at least 12")
    diagnostic = {}
    if len(poses) >= 3:
        # Raw numerical fit is useful for diagnosis, but is never promoted to a
        # calibration when the production solver's diversity checks fail.
        try:
            base_camera = park_eye_to_hand(poses, observations)
            flange_board = average_transform([inverse(b) @ base_camera @ c for b, c in zip(poses, observations)])
            diagnostic = summarize_residuals(residuals(poses, observations, base_camera, flange_board))
            if diagnostic["translation_rms_mm"] > 3 or diagnostic["rotation_rms_deg"] > 1:
                rejection.append("Diagnostic hand-eye residuals exceed 3 mm / 1 degree")
            diagnostic["usable_for_motion_planning"] = False
            diagnostic["note"] = "Fixed factory intrinsics; diagnostic only, no calibrated transforms exported."
        except ValueError as exc:
            diagnostic = {"error": str(exc), "usable_for_motion_planning": False}
    pairs = [pose_error(a, b) for a, b in combinations(poses, 2)]
    centers = np.asarray([d["center_px"] for d in details])
    depth = [d["center_camera_m"][2] for d in details]
    report = {"session": str(directory.resolve()), "status": session.get("status"),
              "abort_reason": session.get("abort_reason"), "saved_samples": len(samples),
              "calibration_file_present": (directory / "calibration.json").exists(),
              "intrinsic_fit_minimum_views": 12, "production_solve_blockers": rejection,
              "max_relative_rotation_deg": max((p[1] for p in pairs), default=0),
              "board_center_px_min": centers.min(axis=0).tolist(),
              "board_center_px_max": centers.max(axis=0).tolist(),
              "estimated_depth_m_min_max": [min(depth), max(depth)],
              "full_pattern_frames": sum(d["pattern_fully_visible"] for d in details),
              "excitation": excitation, "diagnostic_only_fit": diagnostic, "samples": details}
    write_json(output / "review.json", report)
    print({k: v for k, v in report.items() if k not in ("samples", "excitation")})
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    review(args.session, args.output or args.session / "review")


if __name__ == "__main__":
    run_cli(main)
