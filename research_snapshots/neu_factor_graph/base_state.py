"""Private full-base reconstruction under a declared one-axis/home model.

No ROS adapter, calibration inference, input synchronizer or graph authority.
T_A_B maps B into A. Heading is atan2 of base +x in a fixed gravity/world
reference H. R_heading_base0 is the *complete* startup orientation in H;
its roll/pitch cannot be inferred from startup heading alone.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from backend import inv, rigid

# Floating-point algebra boundary, not a tuned localization/health threshold.
NUMERICAL_EPS = 1e-12


def yaw_rotation(angle):
    if not np.isfinite(angle):
        raise ValueError("nonfinite angle")
    return Rotation.from_euler("z", angle).as_matrix()


def moving_extrinsic(home, center, gimbal_delta):
    """T_B_S(gamma)=T_c Rz(gamma) inv(T_c) T_B_S(home)."""
    home = rigid(home)
    center = np.asarray(center, dtype=float)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("invalid yaw-axis center")
    joint = np.eye(4)
    joint[:3, :3] = yaw_rotation(gimbal_delta)
    joint[:3, 3] = center - joint[:3, :3] @ center
    return joint @ home


def normalized_sensor(raw_start, raw_current, home):
    """Explicit B0 numerical basis; equal raw/canonical frame names do not do this."""
    return rigid(home) @ inv(rigid(raw_start)) @ rigid(raw_current)


def from_measured_joint(raw_start, raw_current, home, center, gimbal_delta):
    """Geometry baseline only: requires an independent, time-matched joint angle."""
    return normalized_sensor(raw_start, raw_current, home) @ inv(
        moving_extrinsic(home, center, gimbal_delta))


def from_heading(raw_start, raw_current, home, center, heading,
                 *, R_heading_base0=None):
    """Recover full base pose, or retain a non-identifiable decomposition.

    All calibration/startup terms here are deterministic *conditioning inputs*.
    A solved algebraic pose is not a calibrated estimate or global localization.
    If startup orientation is unknown there is no silent level-start assumption.
    Joint angle inferred from raw pose and heading is not independent of them.
    """
    if R_heading_base0 is None:
        raise ValueError("full startup orientation in heading reference is required")
    reference = np.eye(4)
    reference[:3, :3] = np.asarray(R_heading_base0, dtype=float)
    reference = rigid(reference)[:3, :3]
    if not np.isfinite(heading):
        raise ValueError("nonfinite chassis heading")
    A = normalized_sensor(raw_start, raw_current, home)
    M = reference @ A[:3, :3] @ rigid(home)[:3, :3].T
    direction = np.array([np.cos(heading), np.sin(heading), 0.])
    normal = np.array([-np.sin(heading), np.cos(heading), 0.])
    a, b = normal @ M[:, 0], normal @ M[:, 1]
    constraint_norm = float(np.hypot(a, b))
    result = {"status": "ambiguous_joint_decomposition", "T_base0_base": None,
              "gimbal_delta_rad": None, "heading_constraint_norm": constraint_norm,
              "forward_projection_norm": None, "active_accept": False,
              "covariance": None}
    if constraint_norm < NUMERICAL_EPS:
        # Heading imposes no independent joint constraint. Do not pick gamma=0.
        return result
    gamma = float(np.arctan2(a, b))
    candidates = []
    # Collinearity gives two roots pi apart; directed heading must reject the
    # backward-facing root. A vertical base +x has no defined heading.
    for angle in (gamma, gamma + np.pi):
        orientation = M @ yaw_rotation(-angle)
        forward = orientation[:, 0]
        projected = float(np.linalg.norm(forward[:2]))
        if projected > NUMERICAL_EPS and direction @ forward > NUMERICAL_EPS:
            candidates.append((angle, projected))
    if len(candidates) != 1:
        return result
    angle, projected = candidates[0]
    angle = float(np.arctan2(np.sin(angle), np.cos(angle)))
    B = A @ inv(moving_extrinsic(home, center, angle))
    result.update({"status": "solved_conditioned", "T_base0_base": B,
                   "gimbal_delta_rad": angle, "forward_projection_norm": projected})
    return result
