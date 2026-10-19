"""ChArUco geometry and eye-to-hand calibration; no hardware connections.

T_a_b maps coordinates from b to a. All transform translations use meters.
For a fixed camera and a flange-mounted board: B_i Y = X C_i, where
B_i = T_base_flange, Y = T_flange_board, X = T_base_camera,
and C_i = T_camera_board.
"""

import json
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

from joint_motion import rotation_matrix

DEFAULT_BOARD = Path(__file__).resolve().parents[1] / "assets/calibration/calib_io_charuco.json"
METHODS = {
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
}
if not hasattr(cv2, "calibrateHandEye"):
    # Some OpenCV 5 wheels omit the Python hand-eye binding. The NumPy Park
    # implementation below keeps this project's existing environment usable.
    METHODS = {"park": cv2.CALIB_HAND_EYE_PARK}


class UnsupportedCameraModelError(ValueError):
    """Valid camera coefficients with a projection model OpenCV cannot use."""


class InsufficientCornersError(ValueError):
    """A fresh stationary frame may recover a transient detection shortage."""


def read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, data):
    """Replace a JSON file atomically, never writing non-standard NaN values."""
    path = Path(path)
    payload = json.dumps(data, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)


def finite_array(values, shape, label):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"Expected finite {label} with shape {shape}")
    return result


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    return validate_transform(result)


def validate_transform(value):
    value = finite_array(value, (4, 4), "rigid transform")
    rotation = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError("Invalid rigid transform (rotation or homogeneous last row)")
    return value


def inverse(value):
    value = validate_transform(value)
    rotation = value[:3, :3].T
    return transform(rotation, -rotation @ value[:3, 3])


def pose_to_transform(pose_mm_deg):
    pose = finite_array(pose_mm_deg, (6,), "xArm XYZ/RPY")
    return transform(np.asarray(rotation_matrix(pose[3:])).reshape(3, 3), pose[:3] / 1000)


def flange_from_tcp(pose_mm_deg, tcp_offset_mm_deg, world_offset_mm_deg):
    world = pose_to_transform(world_offset_mm_deg)
    if not np.allclose(world, np.eye(4), atol=1e-8):
        raise ValueError(
            "Calibration requires world offset [0,0,0,0,0,0] to use the physical robot base. "
            "Review the world offset in Studio; this program does not change offsets."
        )
    return pose_to_transform(pose_mm_deg) @ inverse(pose_to_transform(tcp_offset_mm_deg))


def pose_error(actual, expected):
    """Translation distance in mm and shortest orientation difference in degrees."""
    actual, expected = validate_transform(actual), validate_transform(expected)
    cosine = (np.trace(actual[:3, :3].T @ expected[:3, :3]) - 1) / 2
    return (float(1000 * np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
            float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))


def average_transform(values):
    u, _, vt = np.linalg.svd(np.sum([t[:3, :3] for t in values], axis=0))
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return transform(rotation, np.mean([t[:3, 3] for t in values], axis=0))


def make_board(config):
    columns, rows = config["squares_columns_rows"]
    square, marker = config["square_size_mm"], config["marker_size_mm"]
    if (not isinstance(columns, int) or not isinstance(rows, int)
            or min(columns, rows) < 3 or not 0 < marker < square
            or not np.isfinite([square, marker]).all()):
        raise ValueError("Invalid ChArUco board dimensions")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["dictionary"]))
    ids = np.asarray(config["marker_ids"], dtype=np.int32)
    if len(ids) != columns * rows // 2 or len(np.unique(ids)) != len(ids):
        raise ValueError("Incorrect or duplicate board marker IDs")
    board = cv2.aruco.CharucoBoard(
        (columns, rows), square / 1000, marker / 1000, dictionary, ids=ids,
    )
    board.setLegacyPattern(config["legacy_pattern"])
    return board


def camera_arrays(camera):
    """Only use coefficients with the OpenCV forward Brown-Conrady convention."""
    matrix = finite_array(camera["matrix"], (3, 3), "camera matrix")
    distortion = np.asarray(camera["dist_coeffs"], dtype=np.float64).reshape(-1)
    if (len(distortion) not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all()
            or min(matrix[0, 0], matrix[1, 1]) <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or not np.allclose([matrix[0, 1], matrix[1, 0]], 0)):
        raise ValueError("Invalid OpenCV camera intrinsics")
    if len(camera["image_size"]) != 2 or min(camera["image_size"]) <= 0:
        raise ValueError("Invalid camera image_size")
    model = camera["distortion_model"]
    if model not in ("none", "brown_conrady", "opencv") and np.any(distortion != 0):
        raise UnsupportedCameraModelError(
            f"RealSense distortion model '{model}' cannot be passed directly to OpenCV. "
            "Collect diverse views, then solve with --fit-intrinsics, or supply "
            "--intrinsics with independently calibrated OpenCV intrinsics."
        )
    if model == "none" and np.any(distortion != 0):
        raise ValueError("Distortion model 'none' must have zero coefficients")
    return matrix, distortion


class BoardDetector:
    def __init__(self, config, camera=None):
        self.board = make_board(config)
        self.camera = camera
        parameters = cv2.aruco.CharucoParameters()
        if camera is not None:
            parameters.cameraMatrix, parameters.distCoeffs = camera_arrays(camera)
        self.detector = cv2.aruco.CharucoDetector(self.board, parameters)

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, markers, marker_ids = self.detector.detectBoard(gray)
        return {
            "corners": [] if corners is None else corners.reshape(-1, 2).tolist(),
            "ids": [] if ids is None else ids.reshape(-1).tolist(),
            "marker_count": 0 if marker_ids is None else len(marker_ids),
        }

    def detect_for_capture(self, image):
        """Extra detection pass while stationary; retain original-image coordinates.

        Small distant markers sometimes decode after 2x interpolation. This does
        not add information or relax minimum corner/reprojection requirements.
        The fast motion watchdog continues to use detect().
        """
        result = self.detect(image)
        if self.camera is not None or len(result["ids"]) == len(self.board.getChessboardCorners()):
            return result
        enlarged = self.detect(cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
        if len(enlarged["ids"]) <= len(result["ids"]):
            return result
        # OpenCV resize maps pixel centers: original = (resized + .5)/2 - .5.
        enlarged["corners"] = ((np.asarray(enlarged["corners"]) + .5) / 2 - .5).tolist()
        enlarged["detection_scale"] = 2
        enlarged["native_corner_count"] = len(result["ids"])
        return enlarged

    def points(self, detection, minimum=12):
        ids = np.asarray(detection["ids"], dtype=np.int32).reshape(-1)
        if len(ids) == 0 and len(detection["corners"]) == 0:
            raise InsufficientCornersError(f"Only 0 ChArUco corners; need at least {minimum}")
        corners = finite_array(detection["corners"], (len(ids), 2), "image corners")
        points = self.board.getChessboardCorners()
        if len(np.unique(ids)) != len(ids) or np.any(ids < 0) or np.any(ids >= len(points)):
            raise ValueError("Invalid or collinear ChArUco corner IDs")
        if len(ids) < minimum:
            raise InsufficientCornersError(f"Only {len(ids)} ChArUco corners; need at least {minimum}")
        if self.board.checkCharucoCornersCollinear(ids):
            raise ValueError("Invalid or collinear ChArUco corner IDs")
        return np.asarray(points[ids], dtype=np.float32), corners.astype(np.float32)

    def estimate(self, detection, camera, minimum=12):
        obj, img = self.points(detection, minimum)
        matrix, distortion = camera_arrays(camera)
        ok, rvec, tvec = cv2.solvePnP(obj, img, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError("Could not estimate the board pose")
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, matrix, distortion, rvec, tvec)
        pose = transform(cv2.Rodrigues(rvec)[0], tvec)
        if np.min((pose[:3, :3] @ obj.T + tvec.reshape(3, 1))[2]) <= 0:
            raise ValueError("Board pose is behind the camera")
        projected = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))
        if not np.isfinite(rms):
            raise ValueError("Non-finite board reprojection error")
        return pose, rms


def fit_intrinsics(detector, samples, camera):
    if len(samples) < 12:
        raise ValueError("Camera intrinsic fitting requires at least 12 diverse views")
    point_pairs = [detector.points(sample["detection"]) for sample in samples]
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        [p[0] for p in point_pairs], [p[1] for p in point_pairs],
        tuple(camera["image_size"]), None, None,
    )
    result = dict(camera, matrix=matrix.tolist(), dist_coeffs=distortion.reshape(-1).tolist(),
                  distortion_model="opencv", source="charuco_fit", fit_rms_px=float(rms))
    camera_arrays(result)
    return result


def check_excitation(poses):
    """Reject translation-only/single-axis data that cannot constrain calibration."""
    rotations = [cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0].reshape(3)
                 for a, b in combinations(poses, 2)]
    singular = np.linalg.svd(np.asarray(rotations), compute_uv=False)
    max_angle = float(np.degrees(max(np.linalg.norm(r) for r in rotations)))
    if max_angle < 15 or singular[1] < np.radians(5) or singular[1] < 0.1 * singular[0]:
        raise ValueError(
            "Insufficient rotation diversity: tilt the board around at least two "
            "non-parallel axes, with 15-30 degrees or more of total orientation variation."
        )
    return {"max_relative_rotation_deg": max_angle,
            "rotation_singular_values": singular.tolist()}


def residuals(base_flange, camera_board, base_camera, flange_board):
    return [pose_error(b @ flange_board, base_camera @ c)
            for b, c in zip(base_flange, camera_board)]


def summarize_residuals(errors):
    values = np.asarray(errors)
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "translation_max_mm": float(np.max(values[:, 0])),
        "rotation_rms_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "rotation_max_deg": float(np.max(values[:, 1])),
    }


def park_eye_to_hand(base_flange, camera_board):
    """Solve A X = X D by the Park-Martin rotation-log/SVD method.

    From B_i Y = X C_i: A = B_i inv(B_j), D = C_i inv(C_j).
    log(R_A) = R_X log(R_D), then (R_A-I)t_X = R_X t_D - t_A.
    """
    motions = []
    for i, j in combinations(range(len(base_flange)), 2):
        a = base_flange[i] @ inverse(base_flange[j])
        d = camera_board[i] @ inverse(camera_board[j])
        alpha = cv2.Rodrigues(a[:3, :3])[0].reshape(3)
        beta = cv2.Rodrigues(d[:3, :3])[0].reshape(3)
        motions.append((a, d, alpha, beta))
    covariance = sum(np.outer(alpha, beta) for _, _, alpha, beta in motions)
    u, singular, vt = np.linalg.svd(covariance)
    if singular[1] < 1e-10:
        raise ValueError("Robot/camera relative rotations do not constrain a hand-eye solution")
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    lhs = np.vstack([a[:3, :3] - np.eye(3) for a, _, _, _ in motions])
    rhs = np.concatenate([rotation @ d[:3, 3] - a[:3, 3] for a, d, _, _ in motions])
    translation, _, rank, _ = np.linalg.lstsq(lhs, rhs, rcond=None)
    if rank < 3:
        raise ValueError("Robot rotations do not constrain camera translation")
    return transform(rotation, translation)


def solve_eye_to_hand(base_flange, camera_board, method="park"):
    if len(base_flange) != len(camera_board) or len(base_flange) < 8:
        raise ValueError("Need at least 8 matched robot/board poses; 20-30 are recommended")
    base_flange = [validate_transform(p) for p in base_flange]
    camera_board = [validate_transform(p) for p in camera_board]
    excitation = check_excitation(base_flange)
    # Invert robot poses for eye-to-hand. OpenCV's nominal cam2gripper output
    # now represents T_base_camera, NOT T_flange_camera.
    flange_base = [inverse(pose) for pose in base_flange]
    if hasattr(cv2, "calibrateHandEye"):
        rotation, translation = cv2.calibrateHandEye(
            [p[:3, :3] for p in flange_base], [p[:3, 3] for p in flange_base],
            [p[:3, :3] for p in camera_board], [p[:3, 3] for p in camera_board],
            method=METHODS[method],
        )
        base_camera = transform(rotation, translation)
    elif method == "park":
        base_camera = park_eye_to_hand(base_flange, camera_board)
    else:
        raise ValueError("This OpenCV build only supports the NumPy Park solver; use --method park")
    flange_board = average_transform([b @ base_camera @ c
                                     for b, c in zip(flange_base, camera_board)])
    errors = residuals(base_flange, camera_board, base_camera, flange_board)
    return base_camera, flange_board, errors, excitation
