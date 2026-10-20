"""Read centers of the 40 mm ArUco IDs 0-3 in the calibrated robot BASE frame.

Uses RGB square-marker PnP and the saved camera-to-base transform; no robot
connection or motion commands. Show the standalone markers from
assets/aruco_4x4_50_ids_0-7_40mm_A4.pdf. Remove the calibration ChArUco board
from view because it reuses these IDs with a different physical marker size.
"""

import argparse
from collections import Counter
from pathlib import Path
import time
from types import SimpleNamespace

import cv2
import numpy as np

from calibration_core import Projection
from geometry import finite_array, validate_transform
from io_utils import read_json, utc_now, write_json
from realsense_camera import RealSenseCamera, require_same_camera
from robot_common import positive, run_cli

DEFAULT_OUTPUT = Path("output/aruco/positions.json")
MARKER_IDS = (0, 1, 2, 3)
MARKER_SIZE_MM = 40.0
DICTIONARY = "DICT_4X4_50"
MAX_REPROJECTION_PX = 2.0
PRINT_INTERVAL_S = .5


def marker_points(size_mm):
    if not np.isfinite(size_mm) or size_mm <= 0:
        raise ValueError("Marker size must be positive and finite")
    half = size_mm / 2000
    # IPPE_SQUARE order: decoded top-left, top-right, bottom-right, bottom-left.
    # Origin is the black square's center; +x right, +y up, +z out of the print.
    return np.array([[-half, half, 0], [half, half, 0],
                     [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


class MarkerReader:
    """Pure frame processing, also usable from another Python program."""

    def __init__(self, calibration, marker_size_mm=MARKER_SIZE_MM):
        if calibration.get("setup") != "eye_to_hand" or calibration.get("translation_unit") != "m":
            raise ValueError("Expected an eye-to-hand calibration with translations in meters")
        if not calibration.get("quality", {}).get("passed_training_residual_limits"):
            raise ValueError("The selected calibration failed its residual limits")
        self.base_camera = validate_transform(calibration["T_base_camera"])
        self.camera = calibration["camera"]
        self.projection = Projection(self.camera)
        self.size_mm = marker_size_mm
        self.object_points = marker_points(marker_size_mm)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def check_camera(self, metadata):
        require_same_camera(metadata, self.camera)

    def estimate(self, corners):
        pixels = finite_array(corners, (4, 2), "marker corners")
        ideal, matrix, distortion = self.projection.pnp_inputs(pixels)
        count, rotations, translations, _ = cv2.solvePnPGeneric(
            self.object_points, np.ascontiguousarray(ideal), matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        candidates = []
        if count:
            for rotation, translation in zip(rotations, translations):
                if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                    continue
                rotation, translation = cv2.solvePnPRefineLM(
                    self.object_points, ideal, matrix, distortion, rotation.copy(), translation.copy(),
                )
                points = self.object_points @ cv2.Rodrigues(rotation)[0].T + translation.reshape(3)
                if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
                    continue
                error = self.projection.project(points) - pixels
                rms = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
                if np.isfinite(rms):
                    candidates.append((rms, translation.reshape(3)))
        if not candidates:
            raise ValueError("No valid marker pose in front of the camera")
        rms, camera_center = min(candidates, key=lambda item: item[0])
        if rms > MAX_REPROJECTION_PX:
            raise ValueError(f"Reprojection RMS {rms:.2f} px exceeds {MAX_REPROJECTION_PX:g} px")
        base_center = self.base_camera[:3, :3] @ camera_center + self.base_camera[:3, 3]
        return {
            "position_base_mm": (1000 * base_center).tolist(),
            "position_camera_mm": (1000 * camera_center).tolist(),
            "center_px": self.projection.project(camera_center[None])[0].tolist(),
            "corners_px": pixels.tolist(), "reprojection_rms_px": rms,
        }

    def process(self, image):
        if image is None or image.ndim not in (2, 3):
            raise ValueError("Expected a grayscale or BGR image")
        if [image.shape[1], image.shape[0]] != self.camera["image_size"]:
            raise ValueError("Image resolution differs from the calibrated camera; do not resize frames")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        ids = [] if ids is None else np.asarray(ids).ravel().tolist()
        counts = Counter(ids)
        markers, rejected = [], []
        for marker_id in MARKER_IDS:
            if counts[marker_id] > 1:
                rejected.append({"id": marker_id, "reason": "Duplicate ID in this frame; position is ambiguous"})
                continue
            if not counts[marker_id]:
                continue
            try:
                marker = self.estimate(np.asarray(corners[ids.index(marker_id)]).reshape(4, 2))
                markers.append(dict(id=marker_id, **marker))
            except (ValueError, cv2.error) as exc:
                rejected.append({"id": marker_id, "reason": str(exc)})
        found = {m["id"] for m in markers}
        # Never carry positions forward from an earlier frame after an ID is lost.
        return {"markers": markers, "missing_ids": [i for i in MARKER_IDS if i not in found],
                "rejected_markers": rejected}


def draw_positions(image, report):
    result = image.copy()
    rows = {m["id"]: m for m in report["markers"]}
    cv2.putText(result, "Marker centers in ROBOT BASE (mm) | q / Esc: quit", (15, 26),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2, cv2.LINE_AA)
    for row, marker_id in enumerate(MARKER_IDS):
        marker = rows.get(marker_id)
        color = (50, 230, 50) if marker else (0, 180, 255)
        label = f"ID {marker_id}: not available"
        if marker:
            x, y, z = marker["position_base_mm"]
            label = f"ID {marker_id}: X {x:.1f}  Y {y:.1f}  Z {z:.1f} mm"
            polygon = np.rint(marker["corners_px"]).astype(np.int32)
            center = tuple(np.rint(marker["center_px"]).astype(int))
            cv2.polylines(result, [polygon], True, color, 2)
            cv2.drawMarker(result, center, color, cv2.MARKER_CROSS, 12, 2)
            cv2.putText(result, str(marker_id), center, cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
        cv2.putText(result, label, (15, 54 + 25*row), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2, cv2.LINE_AA)
    return result


def print_positions(report):
    rows = {m["id"]: m for m in report["markers"]}
    labels = []
    for marker_id in MARKER_IDS:
        if marker_id in rows:
            xyz = rows[marker_id]["position_base_mm"]
            labels.append(f"ID {marker_id}: ({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) mm")
        else:
            labels.append(f"ID {marker_id}: --")
    print(" | ".join(labels), flush=True)
    for rejected in report["rejected_markers"]:
        print(f"  ID {rejected['id']}: {rejected['reason']}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True, help="Saved eye-to-hand calibration JSON")
    parser.add_argument("--marker-size-mm", type=positive, default=MARKER_SIZE_MM,
                        help="Measured outer black-square side, excluding white margin (default: 40)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Latest-frame JSON output")
    parser.add_argument("--headless", action="store_true", help="Print/save without a preview window; Ctrl+C to stop")
    parser.add_argument("--once", action="store_true", help="Read one frame, print/save it, and exit without a window")
    args = parser.parse_args(argv)
    if args.output.resolve() == args.calibration.resolve():
        parser.error("Output must not overwrite the calibration")
    if args.output.suffix.lower() != ".json":
        parser.error("--output must be a .json file")
    calibration = read_json(args.calibration)
    reader = MarkerReader(calibration, args.marker_size_mm)
    width, height = reader.camera["image_size"]
    camera_options = SimpleNamespace(serial=reader.camera["serial"], width=width, height=height,
                                     fps=reader.camera.get("fps", 30))
    print(f"Calibration: {args.calibration}\n{DICTIONARY}; IDs 0-3; side {args.marker_size_mm:g} mm", flush=True)
    print("Coordinates: marker CENTER, robot BASE XYZ in mm. RGB PnP; no depth stream or robot connection.", flush=True)
    print("Keep the calibration ChArUco board out of view: it reuses these IDs at a different size.", flush=True)
    print(f"Latest-frame JSON: {args.output}", flush=True)
    window = "ArUco IDs 0-3 - robot base XYZ (mm)"
    window_open = False
    try:
        with RealSenseCamera(camera_options) as camera:
            reader.check_camera(camera.metadata)
            if not (args.headless or args.once):
                cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
                window_open = True
            last_output = -float("inf")
            while True:
                image, timestamp = camera.read()
                report = dict(schema_version=1, updated_utc=utc_now(), frame_timestamp=timestamp,
                              calibration=str(args.calibration.resolve()), camera_serial=reader.camera["serial"],
                              coordinate_frame="robot_base", position_unit="mm", origin="marker_center",
                              method="rgb_square_pnp", dictionary=DICTIONARY, marker_size_mm=args.marker_size_mm,
                              requested_ids=list(MARKER_IDS), **reader.process(image))
                # Save every processed frame, including frames with missing markers.
                write_json(args.output, report)
                now = time.monotonic()
                if now - last_output >= PRINT_INTERVAL_S or args.once:
                    print_positions(report)
                    last_output = now
                if args.once:
                    break
                if window_open:
                    cv2.imshow(window, draw_positions(image, report))
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                    if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
    finally:
        if window_open:
            cv2.destroyWindow(window)


if __name__ == "__main__":
    run_cli(main)
