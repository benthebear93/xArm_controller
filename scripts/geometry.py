"""Rigid transforms: T_a_b maps frame b into frame a; translations are meters."""

import numpy as np

from robot_common import rotation_matrix


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
