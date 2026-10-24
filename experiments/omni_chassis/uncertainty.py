"""Synthetic bounded-parameter error studies, not a calibration posterior."""

from copy import deepcopy
import numpy as np

from model import array, vectors


def budget_values(budget):
    names = ("radius_relative", "position_component_m", "angle_rad", "speed_relative")
    if set(budget) != set(names):
        raise ValueError("uncertainty requires explicit radius, position, angle and speed budgets")
    r, p, a, s = [float(array(budget[n], (), n)) for n in names]
    if not (0 <= r < 1 and p >= 0 and 0 <= a <= np.pi and 0 <= s < 1):
        raise ValueError("invalid uncertainty bounds")
    return r, p, a, s


def row_derivatives(model):
    """Analytic physical J derivatives; radius in m, position in m, angle in rad."""
    radius, x_grad, y_grad, angle = [], [], [], []
    for i, w in enumerate(model.config["wheels"]):
        x, y = w["position_xy_m"]
        dx, dy = w["rolling_direction_xy"]
        k = w["drive_sign"] * w["drive_per_wheel_ratio"] / w["radius_m"]
        radius.append(-model.J[i] / w["radius_m"])
        x_grad.append(k*np.array([0., 0., dy]))
        y_grad.append(k*np.array([0., 0., -dx]))
        angle.append(k*np.array([-dy, dx, x*dx+y*dy]))
    return dict(radius_m=np.array(radius), position_x_m=np.array(x_grad),
                position_y_m=np.array(y_grad), angle_rad=np.array(angle))


def utilization_upper_bound(model, commands, budget):
    """Sufficient, nonlinear finite-error bound over every declared parameter box.

    If this value <=1, all actual wheel-speed inequalities hold within this
    ideal model for ALL errors in the budget. The triangle-inequality bound is
    conservative and does not require a probability distribution or small angle.
    """
    er, ep, ea, es = budget_values(budget)
    v = vectors(commands, 3, "commands")
    speed = model.inverse(v)
    direction_error = 2*np.sin(ea/2)
    translation = np.linalg.norm(v[..., :2], axis=-1)
    yaw = abs(v[..., 2])
    excess = []
    for i, w in enumerate(model.config["wheels"]):
        k = w["drive_per_wheel_ratio"] / w["radius_m"]
        # Accepted unit-vector inputs may carry floating-point norm roundoff.
        norm = np.linalg.norm(w["rolling_direction_xy"])
        angle_error = norm*direction_error
        lever_error = np.linalg.norm(w["position_xy_m"])*angle_error + norm*np.sqrt(2)*ep
        excess.append((er*abs(speed[..., i]) + k*(angle_error*translation + lever_error*yaw)) / (1-er))
    excess = np.stack(excess, axis=-1)
    bounds = np.concatenate((excess, excess), axis=-1)
    nominal = v @ model.A.T
    return np.maximum(0., ((nominal+bounds)/(model.b*(1-es))).max(axis=-1))


def robust_ray_research(model, commands, budget):
    v = vectors(commands, 3, "commands")
    return v / np.maximum(1., utilization_upper_bound(model, v, budget))[..., None]


def perturb_config(config, budget, rng):
    er, ep, ea, es = budget_values(budget)
    changed = deepcopy(config)
    changed["parameter_status"] = "synthetic"
    changed["source"] = "Hypothetical bounded perturbation; not a measured MCU configuration."
    for w in changed["wheels"]:
        w["radius_m"] *= float(1+rng.uniform(-er, er))
        w["position_xy_m"] = (np.array(w["position_xy_m"])+rng.uniform(-ep, ep, 2)).tolist()
        angle = rng.uniform(-ea, ea)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        w["rolling_direction_xy"] = (rotation @ w["rolling_direction_xy"]).tolist()
        w["speed_bounds_drive_rad_s"] = (np.array(w["speed_bounds_drive_rad_s"])*
                                          (1+rng.uniform(-es, es, 2))).tolist()
    return changed
