"""Joint camera/base and board/flange fitting with FIXED intrinsics and flat board.

No hardware access, no deformation fitting, and no per-view pose optimization.
Inverse-Brown observations are deprojected by the RealSense SDK once; optimization
uses ideal pixel coordinates. Saved camera metadata retains its original model.
"""

import cv2
import numpy as np

from calibration_core import BoardDetector, Projection
from geometry import inverse, transform, validate_transform


def refine_eye_to_hand(samples, board, camera, base_camera, flange_board):
    from scipy.optimize import least_squares

    detector = BoardDetector(board)
    projection = Projection(camera)
    objects, observed, robot = [], [], []
    for sample in samples:
        obj, pixels = detector.points(sample["detection"])
        pixels, matrix, distortion = projection.pnp_inputs(pixels)
        objects.append(obj.astype(float))
        observed.append(pixels)
        robot.append(validate_transform(sample["robot"]["T_base_flange"]))
    if len(samples) < 8:
        raise ValueError("Joint calibration requires at least 8 matched poses")
    observed = np.vstack(observed)

    def pack(pose):
        return np.r_[cv2.Rodrigues(pose[:3, :3])[0].ravel(), pose[:3, 3]]

    def unpack(parameters):
        return (transform(cv2.Rodrigues(parameters[:3])[0], parameters[3:6]),
                transform(cv2.Rodrigues(parameters[6:9])[0], parameters[9:12]))

    def predict(parameters):
        x, y = unpack(parameters)
        camera_base = inverse(x)
        points = []
        for b, obj in zip(robot, objects):
            pose = camera_base @ b @ y
            points.append(obj @ pose[:3, :3].T + pose[:3, 3])
        points = np.vstack(points)
        pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, distortion)[0].reshape(-1, 2)
        return pixels, points[:, 2]

    def residual(parameters):
        predicted, depth = predict(parameters)
        return np.r_[(predicted-observed).ravel(), np.minimum(depth-.05, 0)*1e4]

    initial = np.r_[pack(base_camera), pack(flange_board)]
    initial_pixels, _ = predict(initial)
    result = least_squares(residual, initial, x_scale="jac", loss="linear", max_nfev=250,
                           ftol=1e-9, xtol=1e-9, gtol=1e-7)
    predicted, depth = predict(result.x)
    if not result.success or not np.isfinite(result.x).all() or np.min(depth) <= .05:
        raise ValueError(f"Joint calibration did not converge to a valid solution: {result.message}")
    rms = lambda pixels: float(np.sqrt(np.mean(np.sum((pixels-observed)**2, axis=1))))
    if rms(predicted) > rms(initial_pixels) + 1e-7:
        raise ValueError("Joint calibration increased the pixel objective")
    x, y = unpack(result.x)
    details = dict(backend="scipy_least_squares", parameters=12, intrinsics_fixed=True,
                   board_model="flat", loss="linear", converged=True, nfev=result.nfev,
                   objective_coordinates="sdk_undistorted_ideal_pixels" if projection.inverse_brown else "image_pixels",
                   initial_prediction_rms_px=rms(initial_pixels), prediction_rms_px=rms(predicted),
                   message=result.message)
    return x, y, details
