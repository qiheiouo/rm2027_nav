"""Offline ideal fixed-axis omni kinematics; no ROS, hardware or controller IO."""

from copy import deepcopy
from dataclasses import dataclass
from itertools import combinations, product
import json
from numbers import Real
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


REL_TOL = 1e-9  # Fraction of each signed drive-speed bound.


class NumericalGeometryError(ValueError):
    """A syntactically valid geometry cannot be resolved at the declared precision."""

    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def array(value, shape, name):
    if value is None:
        raise ValueError(f"{name}: missing numeric value")
    # In particular reject unfilled MCU template entries instead of coercing null.
    if any(v is None or isinstance(v, (bool, np.bool_)) or not isinstance(v, Real)
           for v in np.asarray(value, dtype=object).flat):
        raise ValueError(f"{name}: missing or nonnumeric value (booleans/strings forbidden)")
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name}: expected finite array of shape {shape}")
    return result


def keys(value, expected, name):
    if not isinstance(value, dict) or set(value) != set(expected.split()):
        raise ValueError(f"{name}: required keys are {expected}")


def vectors(value, size, name):
    raw = np.asarray(value, dtype=object)
    result = array(value, raw.shape, name)
    if result.ndim < 1 or result.shape[-1] != size or not np.all(np.isfinite(result)):
        raise ValueError(f"{name}: expected finite (..., {size}) array")
    return result


def polytope_geometry(A, b, transform):
    """Positive-rhs halfspaces; invertible whitening is only a coordinate change."""
    planes = (A @ transform) / b[:, None]
    found = []
    for active in combinations(range(len(planes)), 3):
        block = planes[list(active)]
        if np.linalg.matrix_rank(block, tol=1e-11) < 3:
            continue
        candidate = np.linalg.solve(block, np.ones(3))
        if np.all(planes @ candidate <= 1. + REL_TOL):
            if not any(np.allclose(candidate, p, atol=1e-8, rtol=1e-9) for p in found):
                found.append(candidate)
    if len(found) < 4:
        raise ValueError("could not resolve a full-dimensional polytope numerically")
    points = np.array(found)
    hull = ConvexHull(points)
    return points @ transform.T, hull.simplices, float(hull.volume * abs(np.linalg.det(transform)))


@dataclass(frozen=True)
class Box:
    lower: np.ndarray
    upper: np.ndarray

    @property
    def corners(self):
        return np.array(list(product(*zip(self.lower, self.upper))))

    @property
    def volume(self):
        return float(np.prod(self.upper - self.lower))

    def contains(self, velocity):
        v = vectors(velocity, 3, "velocity")
        return np.all((v >= self.lower) & (v <= self.upper), axis=-1)

    def export(self):
        return dict(lower=self.lower.tolist(), upper=self.upper.tolist(), volume=self.volume)


class OmniModel:
    """omega_drive = J v_body; forward() returns an unweighted LS estimate.

    v_body = [vx m/s, vy m/s, wz rad/s] at base_link's origin. Config array
    order is drive order. Positive wheel rotation rolls along rolling_direction_xy.
    drive_sign states the sign of positive drive rotation relative to that wheel.
    """

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(), object_pairs_hook=unique_object))

    def __init__(self, config):
        keys(config, "schema_version name parameter_status source frame wheels study", "config")
        if type(config["schema_version"]) is not int or config["schema_version"] != 1 or config["frame"] != "base_link":
            raise ValueError("expected schema_version=1 and frame=base_link")
        if config["parameter_status"] not in ("synthetic", "unconfirmed", "mcu_confirmed"):
            raise ValueError("unknown parameter_status")
        for key in ("name", "source"):
            if not isinstance(config[key], str) or not config[key].strip():
                raise ValueError(f"{key} must be nonempty")
        wheels = config["wheels"]
        if not isinstance(wheels, list) or len(wheels) < 3:
            raise ValueError("at least three configured wheels are needed for a 3D region")
        names, rows, lower, upper = [], [], [], []
        for w in wheels:
            keys(w, "name position_xy_m rolling_direction_xy radius_m drive_per_wheel_ratio drive_sign speed_bounds_drive_rad_s", "wheel")
            if not isinstance(w["name"], str) or not w["name"] or w["name"] in names:
                raise ValueError("wheel names must be unique nonempty strings")
            names.append(w["name"])
            x, y = array(w["position_xy_m"], (2,), "position_xy_m")
            dx, dy = array(w["rolling_direction_xy"], (2,), "rolling_direction_xy")
            if not np.isclose(np.hypot(dx, dy), 1., atol=1e-8, rtol=0):
                raise ValueError("rolling_direction_xy must be a unit rolling vector, not an axle")
            radius = float(array(w["radius_m"], (), "radius_m"))
            ratio = float(array(w["drive_per_wheel_ratio"], (), "drive_per_wheel_ratio"))
            sign = float(array(w["drive_sign"], (), "drive_sign"))
            if radius <= 0 or ratio <= 0 or sign not in (-1., 1.):
                raise ValueError("radius and ratio must be positive; drive_sign must be +/-1")
            lo, hi = array(w["speed_bounds_drive_rad_s"], (2,), "speed bounds")
            if not lo < 0 < hi:
                raise ValueError("this harness requires finite bounds with lo < 0 < hi")
            rows.append(sign * ratio / radius * np.array([dx, dy, x * dy - y * dx]))
            lower.append(lo)
            upper.append(hi)
        keys(config["study"], "velocity_scale configured_box_lower configured_box_upper", "study")
        study = config["study"]
        self.scale = array(study["velocity_scale"], (3,), "velocity_scale")
        lo = array(study["configured_box_lower"], (3,), "configured_box_lower")
        hi = array(study["configured_box_upper"], (3,), "configured_box_upper")
        if np.any(self.scale <= 0) or not np.all((lo < 0) & (hi > 0)):
            raise ValueError("velocity_scale positive and configured box strictly contains zero")
        self.config = deepcopy(config)
        self.names = names
        self.J = np.array(rows)
        if not np.all(np.isfinite(self.J)):
            raise ValueError("derived J is nonfinite; check parameter magnitudes")
        self.lower, self.upper = np.array(lower), np.array(upper)
        # Scale both drive and chassis coordinates before testing geometry rank.
        normalized = self.J * self.scale / np.maximum(-self.lower, self.upper)[:, None]
        if not np.all(np.isfinite(normalized)):
            raise ValueError("normalized J is nonfinite; check numerical scales/units")
        _, singular, vt = np.linalg.svd(normalized, full_matrices=False)
        self.numerics = dict(scaled_singular_values=singular.tolist(),
                             rank_relative_cutoff=1e-10,
                             scaled_condition=(float(singular[0]/singular[-1]) if singular[-1] else None))
        if singular[-1] <= singular[0] * 1e-10:
            raise NumericalGeometryError("rank-deficient or ill-conditioned geometry: no resolved bounded 3D polytope", self.numerics)
        self.scaled_condition = float(singular[0] / singular[-1])
        self.K = np.linalg.pinv(self.J, rcond=1e-12)
        self.numerics.update(raw_inverse_identity_error=float(np.max(np.abs(self.K @ self.J - np.eye(3)))),
                             caution_ill_conditioned=self.scaled_condition > 1e6,
                             normalized_worst_noise_gain=float(1/singular[-1]))
        if not np.allclose(self.K @ self.J, np.eye(3), atol=1e-9, rtol=0):
            raise NumericalGeometryError("drive-coordinate LS inverse is numerically unresolved; check units/scales", self.numerics)
        self.A = np.vstack((self.J, -self.J))
        self.b = np.concatenate((self.upper, -self.lower))
        # Whiten the geometry rather than using Qhull jitter or truncating a weak
        # velocity direction. This preserves the physical polytope, including its
        # large extents near singularity; it does not fix noise amplification.
        self.geometry_transform = self.scale[:, None] * vt.T / singular
        self.vertices, self.triangles, self.volume = polytope_geometry(self.A, self.b, self.geometry_transform)
        self.bounding_box = Box(self.vertices.min(axis=0), self.vertices.max(axis=0))
        eye = np.eye(3)
        self.axis_box = Box(-self.ray_limit(-eye), self.ray_limit(eye))
        # Largest homothetic copy of this particular box inside P, not a
        # maximum-volume axis-aligned box over all aspect ratios/centres.
        corner_load = (self.axis_box.corners @ self.A.T / self.b).max()
        self.inner_scale = min(1., 1. / float(corner_load))
        self.inner_box = Box(self.axis_box.lower * self.inner_scale,
                             self.axis_box.upper * self.inner_scale)
        self.configured_box = Box(lo, hi)

    def inverse(self, velocity):
        """Body twist -> signed configured drive angular speeds, in rad/s."""
        return vectors(velocity, 3, "velocity") @ self.J.T

    def forward(self, drive_speeds):
        """Drive speeds -> LS body twist; check residual() for inconsistent feedback."""
        return vectors(drive_speeds, len(self.names), "drive_speeds") @ self.K.T

    def residual(self, drive_speeds):
        speeds = vectors(drive_speeds, len(self.names), "drive_speeds")
        return speeds - self.inverse(self.forward(speeds))

    def slack(self, velocity):
        """Signed halfspace slack, in drive rad/s; ordering: upper then lower."""
        return self.b - vectors(velocity, 3, "velocity") @ self.A.T

    def contains(self, velocity, tolerance=REL_TOL):
        return np.all(self.slack(velocity) / self.b >= -tolerance, axis=-1)

    def wheel_feasible(self, velocity, tolerance=REL_TOL):
        speeds = self.inverse(velocity)
        return np.all((speeds <= self.upper * (1 + tolerance)) &
                      (speeds >= self.lower * (1 + tolerance)), axis=-1)

    def ray_limit(self, direction):
        """Largest t >= 0 with t*direction in P; zero direction returns infinity."""
        load = vectors(direction, 3, "direction") @ self.A.T / self.b
        maximum = np.maximum(0., load.max(axis=-1))
        return np.divide(1., maximum, out=np.full_like(maximum, np.inf), where=maximum > 0)

    def radial_clip(self, velocity):
        """Feasible direction-preserving shrink through zero; not a QP projection."""
        v = vectors(velocity, 3, "velocity")
        return v * np.minimum(1., self.ray_limit(v))[..., None]

    def linear_constraints(self, mapping=None, offset=None):
        """Pull back v_body = mapping @ z + offset to C @ z <= d.

        Consumer chooses whether z is an MPC state, input or an affine reference.
        An acceleration input alone is NOT a velocity: include current velocity
        and dt explicitly in mapping/offset when that model is intended.
        """
        mapping = np.eye(3) if mapping is None else np.asarray(mapping, dtype=float)
        offset = np.zeros(3) if offset is None else array(offset, (3,), "offset")
        if mapping.ndim != 2 or mapping.shape[0] != 3 or not np.all(np.isfinite(mapping)):
            raise ValueError("mapping must be finite, shape (3, n)")
        return self.A @ mapping, self.b - self.A @ offset

    def export(self):
        return dict(
            schema_version=1, research_only=True, hardware_validated=False,
            config=self.config, twist_order=["vx", "vy", "wz"],
            twist_units=["m/s", "m/s", "rad/s"], drive_units="rad/s",
            wheel_order=self.names, inverse_J_drive=self.J.tolist(),
            forward_K_least_squares=self.K.tolist(), scaled_condition=self.scaled_condition,
            numerics=self.numerics,
            halfspaces=dict(A=self.A.tolist(), b=self.b.tolist(), relation="A @ v_body <= b",
                            row_order=[n + ":upper" for n in self.names] +
                                      [n + ":lower" for n in self.names],
                            relative_tolerance=REL_TOL),
            vertices=self.vertices.tolist(), hull_triangles=self.triangles.tolist(),
            volume=self.volume, volume_units="(m/s)^2 * (rad/s)",
            boxes=dict(bounding=self.bounding_box.export(), configured=self.configured_box.export(),
                       pure_axis=self.axis_box.export(), homothetic_inner=self.inner_box.export()),
            homothetic_inner_scale=self.inner_scale)
