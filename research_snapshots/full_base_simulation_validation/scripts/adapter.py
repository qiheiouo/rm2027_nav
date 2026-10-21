"""Research-only rigid-chain adapter. No ROS, TF publisher, graph or recovery.

T_A_B maps B coordinates into A. O has the heading reference axes (z up)
and the startup base origin; B0 retains the complete startup base attitude.
All calibration/reference values are conditioning inputs, not estimates.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class Status(str, Enum):
    SOLVED = "SOLVED"
    AMBIGUOUS = "AMBIGUOUS"
    ILL_CONDITIONED = "ILL_CONDITIONED"
    INVALID_TIME = "INVALID_TIME"
    INVALID_CALIBRATION = "INVALID_CALIBRATION"
    INVALID_INPUT = "INVALID_INPUT"


@dataclass(frozen=True)
class PoseSample:
    T_raw_sensor: np.ndarray
    stamp_ns: int
    clock_id: str
    raw_epoch: str
    raw_parent: str
    sensor_frame: str
    valid: bool = True


@dataclass(frozen=True)
class HeadingSample:
    yaw_rad: float
    stamp_ns: int
    clock_id: str
    heading_epoch: str  # boot ID + heading reset counter, supplied by caller
    reference_frame: str
    valid: bool = True
    online: bool = True


@dataclass(frozen=True)
class JointSample:
    angle_rad: float  # calibrated mechanical angle; + about joint-axis +z
    stamp_ns: int
    clock_id: str
    joint_frame: str
    calibration_id: str
    valid: bool = True


@dataclass(frozen=True)
class Calibration:
    T_base_joint: np.ndarray  # fixed axis frame, +z is rotation axis
    T_joint_sensor_home: np.ndarray  # moving joint frame at home -> sensor
    home_angle_rad: float
    calibration_id: str
    base_frame: str
    joint_frame: str
    sensor_frame: str  # FAST-LIO state IMU origin, NOT automatically lidar origin
    model: str = "single_revolute"


@dataclass(frozen=True)
class Startup:
    raw: PoseSample
    heading: HeadingSample
    R_reference_base0: np.ndarray  # full attitude, no default level assumption
    gravity_down_base0: np.ndarray  # normalized gravity direction in B0
    attitude_stamp_ns: int
    home_stamp_ns: int
    clock_id: str
    home_confirmed: bool
    home_joint_angle_rad: float
    calibration_id: str
    odom_frame: str  # research numerical basis, not a public TF authority


@dataclass(frozen=True)
class Policy:
    # Demonstration bounds, NOT new-car measured acceptance thresholds.
    max_skew_ns: int = 1_000_000
    numerical_eps: float = 1e-12
    rigid_tolerance: float = 1e-8
    reference_tolerance: float = 1e-6
    known_joint_heading_tolerance_rad: float = 1e-6
    min_constraint_derivative: float = 0.05
    min_forward_projection: float = 0.05
    max_heading_gain: float = 20.0


@dataclass
class Result:
    status: Status
    reason: str
    stamp_ns: int
    odom_frame: str
    base_frame: str
    calibration_id: str
    T_odom_base: Optional[np.ndarray] = None
    T_base0_base: Optional[np.ndarray] = None
    joint_angle_rad: Optional[float] = None
    diagnostics: dict = field(default_factory=dict)
    active_accept: bool = False
    covariance: None = None


def wrap(angle):
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def inv(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def startup_attitude_from_gravity(gravity_down_base0, heading_rad, *, eps=1e-12):
    """Explicit construction from calibrated unit gravity and azimuth at rest.

    This does not estimate gravity from accelerometer data or check whether a
    measured vector is actually gravity. Callers must supply a verified frame,
    sign, clock and static/attitude estimate. It is never invoked implicitly.
    """
    down = np.asarray(gravity_down_base0, dtype=float)
    if (down.shape != (3,) or not np.isfinite(down).all() or
            abs(np.linalg.norm(down) - 1.) > 1e-8 or not np.isfinite(heading_rad)):
        raise ValueError("known unit startup gravity and finite azimuth required")
    up = -down
    forward_horizontal = np.array([1., 0., 0.]) - up * up[0]
    length = np.linalg.norm(forward_horizontal)
    if length <= eps:
        raise ValueError("startup base forward axis is vertical; azimuth undefined")
    x = forward_horizontal / length
    y = np.cross(up, x)
    d = np.array([np.cos(heading_rad), np.sin(heading_rad), 0.])
    n = np.array([-d[1], d[0], 0.])
    return np.column_stack((d, n, [0., 0., 1.])) @ np.column_stack((x, y, up)).T


def _rotation(R, tol):
    R = np.asarray(R, dtype=float)
    if (R.shape != (3, 3) or not np.isfinite(R).all() or
            not np.allclose(R.T @ R, np.eye(3), atol=tol, rtol=0) or
            abs(np.linalg.det(R) - 1) > tol):
        raise ValueError("rotation must be finite SO(3), without normalization")
    return R


def _rigid(T, tol):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or
            not np.allclose(T[3], [0, 0, 0, 1], atol=tol, rtol=0)):
        raise ValueError("pose must be finite SE(3)")
    _rotation(T[:3, :3], tol)
    return T


def extrinsic(calibration, angle_rad):
    """E(q) = F Rz(q - q_home) K; callers must validate calibration first."""
    joint = np.eye(4)
    joint[:3, :3] = rz(angle_rad - calibration.home_angle_rad)
    return calibration.T_base_joint @ joint @ calibration.T_joint_sensor_home


def _timestamp(t):
    return isinstance(t, (int, np.integer)) and not isinstance(t, bool) and t > 0


def solve(raw, heading, calibration, startup, *, joint=None, policy=Policy()):
    """One stateless sample. No interpolation, rate propagation or branch prior.

    Pose fields are populated ONLY for SOLVED. Diagnostic candidates for a
    multi-root/tangent case never imply a usable odometry output.
    """
    result = Result(Status.INVALID_INPUT, "", raw.stamp_ns, startup.odom_frame,
                    calibration.base_frame, calibration.calibration_id)
    result.diagnostics["sources"] = dict(
        raw_stamp_ns=raw.stamp_ns, heading_stamp_ns=heading.stamp_ns,
        startup_raw_stamp_ns=startup.raw.stamp_ns, clock_id=startup.clock_id,
        raw_epoch=raw.raw_epoch, heading_epoch=heading.heading_epoch,
        heading_reference=heading.reference_frame)

    def reject(status, reason):
        result.status, result.reason = status, reason
        return result

    policy_values = [policy.numerical_eps, policy.rigid_tolerance,
                     policy.reference_tolerance, policy.known_joint_heading_tolerance_rad,
                     policy.min_constraint_derivative, policy.min_forward_projection,
                     policy.max_heading_gain]
    if (not _timestamp(policy.max_skew_ns) or
            not np.isfinite(policy_values).all() or min(policy_values) <= 0):
        return reject(Status.INVALID_INPUT, "invalid research policy")

    try:
        F = _rigid(calibration.T_base_joint, policy.rigid_tolerance)
        K = _rigid(calibration.T_joint_sensor_home, policy.rigid_tolerance)
        R0 = _rotation(startup.R_reference_base0, policy.rigid_tolerance)
        gravity = np.asarray(startup.gravity_down_base0, dtype=float)
        if (calibration.model != "single_revolute" or
                not all([calibration.calibration_id, calibration.base_frame,
                         calibration.joint_frame, calibration.sensor_frame,
                         startup.odom_frame]) or
                len({calibration.base_frame, calibration.joint_frame,
                     calibration.sensor_frame}) != 3 or
                not np.isfinite(calibration.home_angle_rad) or
                not np.isfinite(startup.home_joint_angle_rad) or
                startup.calibration_id != calibration.calibration_id or
                not startup.home_confirmed or
                abs(wrap(startup.home_joint_angle_rad - calibration.home_angle_rad)) >
                policy.reference_tolerance):
            raise ValueError("missing/mismatched calibration, single-axis model or home")
        if (gravity.shape != (3,) or not np.isfinite(gravity).all() or
                np.linalg.norm(gravity - R0.T @ [0., 0., -1.]) >
                policy.reference_tolerance):
            raise ValueError("full startup attitude and gravity reference disagree")
        if (np.linalg.norm(R0[:2, 0]) < policy.numerical_eps or
                not np.isfinite(startup.heading.yaw_rad) or
                abs(wrap(np.arctan2(R0[1, 0], R0[0, 0]) - startup.heading.yaw_rad)) >
                policy.reference_tolerance):
            raise ValueError("startup attitude does not match directed heading reference")
    except (ValueError, TypeError):
        return reject(Status.INVALID_CALIBRATION,
                      "invalid complete chain, confirmed home or startup reference")

    samples = [raw, heading, startup.raw, startup.heading]
    if joint is not None:
        samples.append(joint)
    if (not all(_timestamp(s.stamp_ns) for s in samples) or
            not _timestamp(startup.attitude_stamp_ns) or
            not _timestamp(startup.home_stamp_ns) or
            not startup.clock_id or
            any(s.clock_id != startup.clock_id for s in samples)):
        return reject(Status.INVALID_TIME, "missing/zero timestamp or clock mismatch")
    startup_times = [startup.raw.stamp_ns, startup.heading.stamp_ns,
                     startup.attitude_stamp_ns, startup.home_stamp_ns]
    current_times = [raw.stamp_ns, heading.stamp_ns]
    if joint is not None:
        current_times.append(joint.stamp_ns)
    result.diagnostics.update(
        heading_sensor_skew_ns=heading.stamp_ns - raw.stamp_ns,
        startup_skew_ns=max(startup_times) - min(startup_times),
        max_skew_ns=policy.max_skew_ns)
    if (max(startup_times) - min(startup_times) > policy.max_skew_ns or
            max(current_times) - min(current_times) > policy.max_skew_ns or
            raw.stamp_ns < startup.raw.stamp_ns or
            not raw.raw_epoch or raw.raw_epoch != startup.raw.raw_epoch or
            not heading.heading_epoch or
            heading.heading_epoch != startup.heading.heading_epoch):
        return reject(Status.INVALID_TIME, "skew, epoch change or pre-startup sample")

    if (not all(s.valid for s in samples) or not heading.online or
            not startup.heading.online or not np.isfinite(heading.yaw_rad) or
            not raw.raw_parent or raw.raw_parent != startup.raw.raw_parent or
            raw.sensor_frame != calibration.sensor_frame or
            startup.raw.sensor_frame != calibration.sensor_frame or
            not heading.reference_frame or
            heading.reference_frame != startup.heading.reference_frame):
        return reject(Status.INVALID_INPUT, "invalid input or numerical frame mismatch")
    if joint is not None and (joint.joint_frame != calibration.joint_frame or
                              joint.calibration_id != calibration.calibration_id or
                              not np.isfinite(joint.angle_rad)):
        return reject(Status.INVALID_CALIBRATION, "joint angle convention/calibration mismatch")
    try:
        raw0 = _rigid(startup.raw.T_raw_sensor, policy.rigid_tolerance)
        raw1 = _rigid(raw.T_raw_sensor, policy.rigid_tolerance)
    except (ValueError, TypeError):
        return reject(Status.INVALID_INPUT, "invalid raw full sensor pose")
    if raw.stamp_ns == startup.raw.stamp_ns and not np.allclose(
            raw0, raw1, atol=policy.rigid_tolerance, rtol=0):
        return reject(Status.INVALID_TIME, "different raw poses at identical source time")
    if (joint is not None and raw.stamp_ns == startup.raw.stamp_ns and
            abs(wrap(joint.angle_rad - calibration.home_angle_rad)) > policy.reference_tolerance):
        return reject(Status.INVALID_CALIBRATION, "startup joint contradicts confirmed home")

    D = np.eye(4)
    D[:3, :3] = R0
    E0 = F @ K
    T_odom_sensor = D @ E0 @ inv(raw0) @ raw1
    direction = np.array([np.cos(heading.yaw_rad), np.sin(heading.yaw_rad), 0.])
    normal = np.array([-direction[1], direction[0], 0.])
    Q = T_odom_sensor[:3, :3] @ K[:3, :3].T
    u = F[:3, :3].T @ [1., 0., 0.]
    U = Q @ [u[0], u[1], 0.]
    V = Q @ [u[1], -u[0], 0.]
    W = Q @ [0., 0., u[2]]
    a, b, c = float(normal @ U), float(normal @ V), float(normal @ W)
    rho = float(np.hypot(a, b))
    known_home = joint is None and raw.stamp_ns == startup.raw.stamp_ns
    mode = "known_joint" if joint else ("startup_home" if known_home else "heading_decomposition")
    result.diagnostics.update(mode=mode,
                              coefficients=[a, b, c], constraint_amplitude=rho,
                              candidates=[], uncertainty_model="not_provided")

    if joint is not None:
        roots = [wrap(joint.angle_rad - calibration.home_angle_rad)]
    elif known_home:
        roots = [0.0]
    elif rho <= policy.numerical_eps:
        if abs(c) <= policy.numerical_eps:
            return reject(Status.AMBIGUOUS, "heading constraint has a continuum of roots")
        return reject(Status.INVALID_INPUT, "no joint angle satisfies heading")
    elif abs(c) > rho + policy.numerical_eps:
        return reject(Status.INVALID_INPUT, "heading incompatible with complete rigid chain")
    else:
        phase = np.arctan2(b, a)
        cosine = float(np.clip(-c / rho, -1., 1.))
        # A tangent root should not split into two roots due to roundoff.
        if abs(1. - abs(cosine)) <= policy.numerical_eps:
            offset = 0. if cosine > 0 else np.pi
        else:
            offset = np.arccos(cosine)
        roots = [wrap(phase + offset), wrap(phase - offset)]
        if abs(wrap(roots[0] - roots[1])) <= policy.numerical_eps:
            roots = roots[:1]

    candidates = []
    for delta in roots:
        T = T_odom_sensor @ inv(extrinsic(calibration, calibration.home_angle_rad + delta))
        forward = T[:3, 0]
        projection = float(np.linalg.norm(forward[:2]))
        directed_projection = float(direction @ forward)
        if projection <= policy.numerical_eps or directed_projection <= policy.numerical_eps:
            continue
        residual = abs(wrap(np.arctan2(forward[1], forward[0]) - heading.yaw_rad))
        if residual > (policy.known_joint_heading_tolerance_rad if joint else
                       policy.reference_tolerance):
            continue
        derivative = float(abs(-a * np.sin(delta) + b * np.cos(delta)))
        gain = None if derivative <= policy.numerical_eps else directed_projection / derivative
        candidate = dict(joint_delta_rad=delta, forward_projection=projection,
                         constraint_derivative=derivative, heading_gain_rad_per_rad=gain,
                         heading_residual_rad=residual, T_odom_base=T.tolist())
        result.diagnostics["candidates"].append(candidate)
        candidates.append((delta, T, candidate))
    if not candidates:
        return reject(Status.INVALID_INPUT, "no directed nonvertical heading solution")
    if len(candidates) > 1:
        return reject(Status.AMBIGUOUS, "multiple directed-heading decompositions")
    delta, T, candidate = candidates[0]
    if (candidate["forward_projection"] < policy.min_forward_projection or
            (joint is None and not known_home and
             (candidate["constraint_derivative"] < policy.min_constraint_derivative or
              candidate["heading_gain_rad_per_rad"] is None or
              candidate["heading_gain_rad_per_rad"] > policy.max_heading_gain))):
        return reject(Status.ILL_CONDITIONED, "unique root exceeds declared sensitivity bounds")
    result.status, result.reason = Status.SOLVED, "unique conditioned rigid-chain solution"
    result.T_odom_base = T
    result.T_base0_base = inv(D) @ T
    result.joint_angle_rad = wrap(calibration.home_angle_rad + delta)
    return result
