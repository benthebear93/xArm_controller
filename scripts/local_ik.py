"""Bounded, reference-seeded IK using a caller-supplied, read-only FK function."""

import cv2
import numpy as np

from geometry import finite_array, pose_error, validate_transform


def solve_local_ik(forward, target, reference, seed, max_excursion_deg, tick=lambda: None):
    """Find a nearby joint solution without the firmware's unseeded legacy IK.

    All angles are degrees and transforms use meters. The small joint penalty
    keeps redundant solutions close to the previous waypoint. Bounds apply to
    the ORIGINAL seed, not just the previous waypoint. The caller must still
    check controller joint limits, endpoint FK, and the interpolated path.
    """
    from scipy.optimize import least_squares

    target = validate_transform(target)
    reference = finite_array(reference, (7,), "IK reference joints")
    seed = finite_array(seed, (7,), "IK seed joints")
    if not np.isfinite(max_excursion_deg) or max_excursion_deg <= 0:
        raise ValueError("Local IK requires a positive joint excursion limit")
    lower, upper = seed - max_excursion_deg, seed + max_excursion_deg
    if np.any(reference < lower) or np.any(reference > upper):
        raise ValueError("Local IK reference is outside the seed joint excursion limit")
    cached_joints, cached_residual = None, None
    queries = 0

    def residual(joints):
        nonlocal cached_joints, cached_residual, queries
        tick()
        if cached_joints is not None and np.array_equal(joints, cached_joints):
            return cached_residual.copy()
        queries += 1
        if queries > 400:
            raise ValueError("Local IK exceeded its FK query budget; no motion enabled")
        actual = validate_transform(forward(joints))
        position = 1000 * (actual[:3, 3] - target[:3, 3])
        rotation = np.degrees(cv2.Rodrigues(target[:3, :3].T @ actual[:3, :3])[0].ravel())
        value = np.r_[position, rotation, .001 * (joints - reference)]
        cached_joints, cached_residual = joints.copy(), value.copy()
        return value

    def jacobian(joints):
        base = residual(joints)
        columns = []
        for axis in range(7):
            # A fixed 0.1-degree probe avoids quantization in controller FK
            # responses; scipy's tiny default finite differences do not.
            step = min(.1, max_excursion_deg)
            if joints[axis] + step > upper[axis]:
                step = -step
            probe = joints.copy()
            probe[axis] += step
            columns.append((residual(probe) - base) / step)
        return np.column_stack(columns)

    def reached_target(joints):
        error = residual(joints)
        if np.linalg.norm(error[:3]) < .05 and np.linalg.norm(error[3:6]) < .02:
            raise StopIteration  # Accept controller precision without over-solving.

    result = least_squares(residual, reference, jac=jacobian, bounds=(lower, upper),
                           callback=reached_target, max_nfev=30,
                           ftol=1e-8, xtol=1e-6, gtol=1e-6)
    joints = finite_array(result.x, (7,), "local IK joints")
    tick()
    distance, angle = pose_error(forward(joints), target)
    # A callback stop (or the iteration limit) is acceptable only when an
    # independent final FK satisfies the physical pose tolerances.
    if distance > .3 or angle > .1:
        raise ValueError(f"Local IK did not converge within {max_excursion_deg:g} deg of the start "
                         f"(position error {distance:.3f} mm, rotation error {angle:.3f} deg); "
                         "no motion enabled")
    return joints
