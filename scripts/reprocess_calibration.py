"""Re-detect saved images with measured board geometry into a NEW session.

No hardware access. The source session and images are never changed. Run the
usual offline solve command on the resulting session afterwards.
"""

import argparse
import copy
from pathlib import Path
import shutil

import cv2

from calibrate_eye_to_hand import load_session
from calibration_core import BoardDetector, DEFAULT_BOARD
from io_utils import read_json, utc_now, write_json
from robot_common import run_cli


def reprocess(source, output, board_path=DEFAULT_BOARD):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError("Choose a new output directory; existing data will not be overwritten")
    session = load_session(source)
    board = read_json(board_path)
    detector = BoardDetector(board)
    # Validate all source files before creating the destination.
    paths = []
    for sample in session["samples"]:
        path = (source / sample["image"]).resolve()
        if not path.is_relative_to(source) or not path.is_file():
            raise ValueError(f"Invalid source image for sample {sample['id']}: {path}")
        paths.append(path)
    result = copy.deepcopy(session)
    result.update(board=board, samples=[], status="reprocessed", collection="offline_reprocessed")
    result.pop("abort_reason", None)
    audit = {"source_session": str(source), "source_status": session.get("status"),
             "created_utc": utc_now(), "original_board": session["board"],
             "measured_board": board, "samples": [], "rejected_samples": []}
    (output / "images").mkdir(parents=True)
    for sample, path in zip(session["samples"], paths):
        image = cv2.imread(str(path))
        if image is None or list(image.shape[1::-1]) != session["camera"]["image_size"]:
            raise ValueError(f"Unreadable or wrong-resolution image: {path}")
        detection = detector.detect_for_capture(image)
        try:
            detector.points(detection)
        except ValueError as exc:
            audit["rejected_samples"].append({"id": sample["id"], "reason": str(exc)})
            continue
        record = copy.deepcopy(sample)
        # These derived values refer to the old board geometry.
        for key in ("T_camera_board", "reprojection_rms_px", "full_pattern_margin_ok", "visibility_warning"):
            record.pop(key, None)
        record["detection"] = detection
        record["image"] = f"images/{sample['id']:04d}.png"
        record["redetected_utc"] = utc_now()
        shutil.copy2(path, output / record["image"])
        result["samples"].append(record)
        audit["samples"].append({"id": sample["id"], "original_corners": len(sample["detection"]["ids"]),
                                  "redetected_corners": len(detection["ids"])})
    result["reprocessing"] = audit
    write_json(output / "session.json", result)
    write_json(output / "reprocessing.json", audit)
    print(f"Reprocessed {len(result['samples'])}/{len(session['samples'])} saved views; "
          f"rejected {len(audit['rejected_samples'])}. Output: {output}")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    args = parser.parse_args()
    reprocess(args.session, args.output, args.board)


if __name__ == "__main__":
    run_cli(main)
