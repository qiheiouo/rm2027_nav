#!/usr/bin/env python3
"""Stage-two synthetic experiments: optimization, error bounds and numerical limits."""

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import platform
import tempfile
from time import perf_counter_ns

import numpy as np
import scipy
from scipy.optimize import minimize

from model import OmniModel, polytope_geometry, unique_object, array, keys
from optimization import CommandProjector, maximum_box, tinympc_candidate
from run import ROOT, proportion, rigid_body_drive_speeds
from uncertainty import perturb_config, robust_ray_research, utilization_upper_bound, budget_values


METHODS = ("naive_axis", "conservative_box", "maximum_box", "ray", "euclidean", "weighted")


def validate_protocol(p):
    keys(p, "schema_version description comparison runtime sensitivity conditioning regression gates", "protocol")
    if type(p["schema_version"]) is not int or p["schema_version"] != 1:
        raise ValueError("research protocol schema_version must be 1")
    required_counts = {"comparison": ("samples_per_seed", "boundary_directions"),
        "runtime": ("single_calls", "repeats", "batch_size", "batch_repeats"),
        "sensitivity": ("draws_per_family_level", "directions"),
        "regression": ("layouts_per_seed", "commands_per_layout", "oracle_every", "oracle_commands")}
    for group, fields in required_counts.items():
        for field in fields:
            v = p[group][field]
            if type(v) is not int or v < 1:
                raise ValueError(f"{group}.{field} must be a positive integer")
    if p["regression"]["oracle_commands"] > p["regression"]["commands_per_layout"]:
        raise ValueError("oracle_commands exceeds commands_per_layout")
    for group in ("comparison", "regression"):
        seeds = p[group]["seeds"]
        if not seeds or any(type(s) is not int or s < 0 for s in seeds) or len(set(seeds)) != len(seeds):
            raise ValueError(f"{group}: distinct nonnegative integer seeds required")
    for group in ("runtime", "sensitivity"):
        if type(p[group]["seed"]) is not int or p[group]["seed"] < 0:
            raise ValueError(f"{group}: nonnegative integer seed required")
    steps = p["comparison"]["boundary_steps"]
    if not steps or np.any(array(steps, (len(steps),), "boundary_steps") <= 0):
        raise ValueError("positive finite boundary steps required")
    if float(array(p["comparison"]["domain_expansion"], (), "domain_expansion")) < 1:
        raise ValueError("common domain expansion must be at least 1")
    if not p["sensitivity"]["levels"]:
        raise ValueError("uncertainty levels cannot be empty")
    for level in p["sensitivity"]["levels"]: budget_values(level)
    for name, bounds in p["regression"]["ranges"].items():
        lo, hi = array(bounds, (2,), name)
        if lo > hi or (name in ("radius_m", "drive_per_wheel_ratio", "speed_capacity_drive_rad_s") and lo <= 0):
            raise ValueError(f"invalid random parameter range: {name}")
    lo, hi = p["regression"]["ranges"]["wheel_count"]
    if type(lo) is not int or type(hi) is not int or lo < 3:
        raise ValueError("wheel_count range must contain integers >=3")
    if not p["conditioning"]["exponents"] or any(type(e) is not int or e < 0 for e in p["conditioning"]["exponents"]):
        raise ValueError("nonnegative conditioning exponents required")
    if float(array(p["conditioning"]["normalized_drive_noise"], (), "normalized_drive_noise")) <= 0:
        raise ValueError("positive finite diagnostic noise amplitude required")
    for name, v in p["gates"].items():
        if float(array(v, (), name)) <= 0: raise ValueError("positive finite numerical gates required")


def summary(values):
    x = np.asarray(values, dtype=float)
    return dict(mean=float(x.mean()), median=float(np.median(x)),
                p95=float(np.quantile(x, .95)), p99=float(np.quantile(x, .99)),
                minimum=float(x.min()), maximum=float(x.max()))


def physical_violation(model, commands):
    speed = rigid_body_drive_speeds(model, np.asarray(commands).reshape(-1, 3))
    utilization = np.maximum(speed/model.upper, speed/model.lower)
    return np.maximum(0., utilization.max(axis=1)-1)


def make_methods(model, weights):
    started = perf_counter_ns()
    boxes = {mode: maximum_box(model, mode) for mode in ("origin", "free", "contains_zero")}
    box_ms = (perf_counter_ns()-started)/1e6
    metric = np.diag(weights/model.scale**2)
    setup_ms, methods = {}, {}
    for name in METHODS:
        started = perf_counter_ns()
        methods[name] = CommandProjector(model, name, metric, boxes["contains_zero"]["box"])
        setup_ms[name] = (perf_counter_ns()-started)/1e6
    return methods, boxes, metric, dict(all_three_box_solves_ms=box_ms, projector_setup_ms=setup_ms)


def boundary_metrics(model, methods, metric, protocol):
    rng = np.random.default_rng(8201)
    directions = rng.normal(size=(protocol["boundary_directions"], 3))*model.scale
    edge = directions*model.ray_limit(directions)[:, None]
    probes = np.vstack((edge, model.vertices, np.zeros((1, 3))))
    perturb = rng.normal(size=probes.shape)
    perturb /= np.linalg.norm(perturb, axis=1)[:, None]
    out = {}
    for name, method in methods.items():
        projected = method.project(probes)
        item = dict(max_violation=float(physical_violation(model, projected).max()),
                    zero_distortion_scaled=float(np.linalg.norm(method.project(np.zeros(3))/model.scale)),
                    idempotence_scaled=float(np.max(abs((method.project(projected)-projected)/model.scale))),
                    local_gains={})
        W = metric if name == "weighted" else np.eye(3)
        for step in protocol["boundary_steps"]:
            delta = perturb*model.scale*step
            changed = method.project(probes+delta)-projected
            gain = np.sqrt(np.einsum("ij,jk,ik->i", changed, W, changed) /
                           np.einsum("ij,jk,ik->i", delta, W, delta))
            item["local_gains"][str(step)] = float(gain.max())
        out[name] = item
    return out


def benchmark(methods, samples, protocol):
    rng = np.random.default_rng(protocol["seed"])
    samples = samples[rng.permutation(len(samples))]
    times = {name: [] for name in METHODS}
    for method in methods.values():
        method.project(samples[:8])
    for _ in range(protocol["repeats"]):
        for name in rng.permutation(METHODS):
            for command in samples[:protocol["single_calls"]]:
                started = perf_counter_ns()
                methods[name].project(command)
                times[name].append((perf_counter_ns()-started)/1000)
    out = {}
    size = min(protocol["batch_size"], len(samples))
    for name in METHODS:
        batches = []
        for _ in range(protocol["batch_repeats"]):
            started = perf_counter_ns()
            methods[name].project(samples[:size])
            batches.append((perf_counter_ns()-started)/1000/size)
        out[name] = dict(single_command_us=summary(times[name]),
                         batch_amortized_us_per_command=summary(batches), batch_size=size,
                         single_calls_per_repeat=min(protocol["single_calls"], len(samples)))
    return out


def comparison(model, protocol, runtime):
    methods, boxes, metric, setup = make_methods(model, np.array(protocol["scaled_metric_weights"]))
    lower = np.minimum(model.bounding_box.lower, model.configured_box.lower)*protocol["domain_expansion"]
    upper = np.maximum(model.bounding_box.upper, model.configured_box.upper)*protocol["domain_expansion"]
    sources = [np.random.default_rng(s).uniform(lower, upper, (protocol["samples_per_seed"], 3))
               for s in protocol["seeds"]]
    source = np.concatenate(sources)
    feasible = model.contains(source)
    results, all_outputs = {}, {}
    for name, method in methods.items():
        projected = method.project(source)
        all_outputs[name] = projected
        diff = projected-source
        violation = physical_violation(model, projected)
        scaled = np.linalg.norm(diff/model.scale, axis=1)
        cost = np.sqrt(np.einsum("ij,jk,ik->i", diff, metric, diff))
        if name == "naive_axis":
            A = np.vstack((model.A, np.eye(3), -np.eye(3)))
            b = np.r_[model.b, model.axis_box.upper, -model.axis_box.lower]
            _, _, volume = polytope_geometry(A, b, model.geometry_transform)
            retained = volume/model.volume
        elif name in ("conservative_box", "maximum_box"):
            retained = method.box.volume/model.volume
        else:
            retained = 1.
        norms = np.linalg.norm(source/model.scale, axis=1)*np.linalg.norm(projected/model.scale, axis=1)
        cosine = np.divide(np.einsum("ij,ij->i", source/model.scale, projected/model.scale),
                           norms, out=np.ones(len(source)), where=norms > 1e-15)
        item = dict(violation=proportion(int((violation > 1e-8).sum()), len(source)),
                    max_relative_violation=float(violation.max()),
                    raw_l2_distortion=summary(np.linalg.norm(diff, axis=1)),
                    scaled_l2_distortion=summary(scaled), weighted_distortion=summary(cost),
                    component_mean_abs_distortion=np.mean(abs(diff), axis=0).tolist(),
                    direction_error_degrees=summary(np.degrees(np.arccos(np.clip(cosine, -1, 1)))),
                    retained_feasible_volume_fraction=retained,
                    max_change_of_feasible_input_scaled=float(scaled[feasible].max(initial=0.)),
                    per_seed=[])
        for i, seed in enumerate(protocol["seeds"]):
            section = slice(i*protocol["samples_per_seed"], (i+1)*protocol["samples_per_seed"])
            item["per_seed"].append(dict(seed=seed, mean_scaled_distortion=float(scaled[section].mean()),
                                         violation_fraction=float(np.mean(violation[section] > 1e-8))))
        if method.quadratic:
            item["optimality_residual"] = method.quadratic.optimality_residual(source, projected)
        results[name] = item
    feasible_names = METHODS[1:]
    # Metric optimality is checked against all feasible candidate methods on the
    # SAME commands. The naive box is excluded because its outputs can violate P.
    for name, W in (("euclidean", np.eye(3)), ("weighted", metric)):
        errors = {n: np.einsum("ij,jk,ik->i", all_outputs[n]-source, W, all_outputs[n]-source)
                  for n in feasible_names}
        results[name]["max_cost_excess_over_feasible_alternatives"] = float(max(
            np.maximum(0., (errors[name]-errors[n])/(1+errors[n])).max() for n in feasible_names))
    export_boxes = {mode: {k: (v.export() if k == "box" else v) for k, v in item.items()}
                    for mode, item in boxes.items()}
    candidate = tinympc_candidate(model, "input")
    return dict(name=model.config["name"], source_count=len(source), common_domain_lower=lower.tolist(),
                common_domain_upper=upper.tolist(), metric=metric.tolist(),
                weighted_equivalent_to_euclidean=bool(np.allclose(metric, np.eye(3)*np.trace(metric)/3)), methods=results,
                boxes=export_boxes, setup=setup, runtime=benchmark(methods, source, runtime),
                boundary=boundary_metrics(model, methods, metric, protocol),
                tinympc_input_candidate=candidate.export())


def sensitivity(model, protocol, weights):
    methods, _, _, _ = make_methods(model, weights)
    rng = np.random.default_rng(protocol["seed"])
    direction = rng.normal(size=(protocol["directions"], 3))*model.scale
    edge = direction*model.ray_limit(direction)[:, None]
    source = np.vstack((edge*.9, edge*1.5))
    outputs = {name: method.project(source) for name, method in methods.items()}
    rows = []
    families = ("radius_relative", "position_component_m", "angle_rad", "speed_relative", "combined")
    for family in families:
        for level, complete_budget in enumerate(protocol["levels"]):
            budget = {k: (v if family == "combined" or family == k else 0.)
                      for k, v in complete_budget.items()}
            robust = robust_ray_research(model, source, budget)
            bound = utilization_upper_bound(model, source, budget)
            volume, conditions = [], []
            counts = {name: [] for name in METHODS}
            maximum = {name: 0. for name in METHODS}
            robust_max, upper_bound_excess = 0., 0.
            for _ in range(protocol["draws_per_family_level"]):
                actual = OmniModel(perturb_config(model.config, budget, rng))
                volume.append(actual.volume/model.volume)
                conditions.append(actual.scaled_condition)
                for name in METHODS:
                    violation = physical_violation(actual, outputs[name])
                    counts[name].append(float(np.mean(violation > 1e-8)))
                    maximum[name] = max(maximum[name], float(violation.max()))
                robust_max = max(robust_max, float(physical_violation(actual, robust).max()))
                # physical_violation clips utilization at 1; compare the exact
                # signed utilization here so interior commands are not distorted.
                actual_speed = rigid_body_drive_speeds(actual, source)
                actual_util = np.maximum(actual_speed/actual.upper, actual_speed/actual.lower).max(axis=1)
                upper_bound_excess = max(upper_bound_excess, float((actual_util-bound).max()))
            rows.append(dict(family=family, level=level, budget=budget, volume_ratio=summary(volume),
                             condition=summary(conditions),
                             nominal_method_violations={n: dict(fraction_over_draws=summary(counts[n]),
                                                               maximum_relative=maximum[n]) for n in METHODS},
                             robust_ray_max_relative_violation=robust_max,
                             actual_minus_proven_utilization_bound_max=upper_bound_excess))
    return dict(name=model.config["name"], draws_per_family_level=protocol["draws_per_family_level"],
                sources_per_draw=len(source), results=rows)


def raw_matrix(config):
    result = []
    for w in config["wheels"]:
        x, y = w["position_xy_m"]
        dx, dy = w["rolling_direction_xy"]
        result.append(w["drive_sign"]*w["drive_per_wheel_ratio"]/w["radius_m"]*
                      np.array([dx, dy, x*dy-y*dx]))
    return np.array(result)


def conditioning(base, protocol):
    results = []
    for family in ("shrinking_yaw_lever", "parallel_directions"):
        for exponent in [*protocol["exponents"], None]:
            epsilon = 10.**(-exponent) if exponent is not None else 0.
            config = deepcopy(base.config)
            for w in config["wheels"]:
                if family == "shrinking_yaw_lever":
                    w["position_xy_m"] = (epsilon*np.array(w["position_xy_m"])).tolist()
                else:
                    angle = epsilon*np.arctan2(*w["rolling_direction_xy"][::-1])
                    w["rolling_direction_xy"] = [float(np.cos(angle)), float(np.sin(angle))]
            J = raw_matrix(config)
            d = np.maximum(-base.lower, base.upper)
            N = J*base.scale/d[:, None]
            _, singular, _ = np.linalg.svd(N, full_matrices=False)
            item = dict(family=family, epsilon=epsilon, normalized_singular_values=singular.tolist(),
                        normalized_worst_noise_gain=(float(1/singular[-1]) if singular[-1] else None),
                        worst_scaled_velocity_error_for_declared_noise=(float(protocol["normalized_drive_noise"]/singular[-1]) if singular[-1] else None))
            v = np.array([.31, -.27, .41], dtype=np.float32)
            N32 = N.astype(np.float32)
            estimated, _, rank32, _ = np.linalg.lstsq(N32, N32@v, rcond=np.finfo(np.float32).eps*max(N.shape))
            item.update(float32_resolved_rank=int(rank32), float32_scaled_roundtrip_error=float(np.max(abs(estimated-v))))
            try:
                model = OmniModel(config)
                rng = np.random.default_rng(8301)
                u = rng.normal(size=(64, 3))*model.scale
                output = CommandProjector(model, "euclidean").project(u)
                item.update(accepted=True, numerics=model.numerics, volume=model.volume,
                            vertex_relative_violation=float(physical_violation(model, model.vertices).max()),
                            projection_relative_violation=float(physical_violation(model, output).max()))
            except (ValueError, RuntimeError) as error:
                item.update(accepted=False, reason=str(error),
                            numerics=getattr(error, "diagnostics", None))
            results.append(item)
    return dict(base_config=base.config["name"], normalized_drive_noise=protocol["normalized_drive_noise"],
                results=results)


def invalid_cases(base):
    cases = []
    fields = [("radius_m", 0), ("radius_m", -1), ("radius_m", None), ("radius_m", "0.05"),
              ("radius_m", True), ("radius_m", float("nan")), ("radius_m", float("inf")),
              ("drive_per_wheel_ratio", 0), ("drive_per_wheel_ratio", -2),
              ("drive_sign", 0), ("drive_sign", True),
              ("position_xy_m", [0]), ("position_xy_m", [None, 0]),
              ("rolling_direction_xy", [0, 0]), ("rolling_direction_xy", [1, 1]),
              ("speed_bounds_drive_rad_s", [40, -40]), ("speed_bounds_drive_rad_s", [0, 40]),
              ("speed_bounds_drive_rad_s", [-40, float("inf")])]
    for i, (field, value) in enumerate(fields):
        c = deepcopy(base)
        c["wheels"][0][field] = value
        cases.append((f"wheel_{i}_{field}", c))
    for field, value in (("schema_version", True), ("frame", "map"), ("wheels", []),
                         ("parameter_status", "hardware_go"), ("source", "")):
        c = deepcopy(base); c[field] = value
        cases.append((field, c))
    c = deepcopy(base); c["wheels"][1]["name"] = c["wheels"][0]["name"]
    cases.append(("duplicate_wheel_id", c))
    c = deepcopy(base); c["wheels"][0]["typo_radius"] = .1
    cases.append(("unknown_field", c))
    c = deepcopy(base); c["study"]["velocity_scale"][0] = 0
    cases.append(("invalid_coordinate_scale", c))
    c = deepcopy(base); c["study"]["configured_box_lower"][0] = 1
    cases.append(("box_excludes_zero", c))
    c = deepcopy(base)
    for w in c["wheels"]: w["rolling_direction_xy"] = [1, 0]
    cases.append(("rank_deficient", c))
    return cases


def validation(base):
    results = []
    for name, config in invalid_cases(base.config):
        try:
            OmniModel(config)
            results.append(dict(case=name, rejected=False))
        except (ValueError, TypeError) as error:
            results.append(dict(case=name, rejected=True, reason=str(error)))
    with tempfile.TemporaryDirectory() as directory:
        p = Path(directory)/"duplicate.json"
        p.write_text('{"schema_version":1,"schema_version":1}')
        try:
            OmniModel.load(p)
            results.append(dict(case="duplicate_json_key", rejected=False))
        except ValueError as error:
            results.append(dict(case="duplicate_json_key", rejected=True, reason=str(error)))
    return dict(results=results, all_rejected=all(r["rejected"] for r in results),
                undetectable_without_external_truth=["plausible wrong drive sign", "wheel ID permutation with unchanged names",
                    "RPM mislabeled as rad/s", "axle direction supplied as a unit rolling direction", "plausible wrong reference origin"])


def random_config(template, rng, ranges, name):
    c = deepcopy(template)
    c.update(name=name, parameter_status="synthetic", source="Seeded random parameter regression, not new-car priors.")
    c["wheels"] = []
    for i in range(int(rng.integers(ranges["wheel_count"][0], ranges["wheel_count"][1]+1))):
        angle = rng.uniform(*ranges["angle_rad"])
        c["wheels"].append(dict(name=str(i), position_xy_m=rng.uniform(*ranges["position_component_m"], 2).tolist(),
            rolling_direction_xy=[float(np.cos(angle)), float(np.sin(angle))],
            radius_m=float(rng.uniform(*ranges["radius_m"])),
            drive_per_wheel_ratio=float(rng.uniform(*ranges["drive_per_wheel_ratio"])),
            drive_sign=int(rng.choice([-1, 1])),
            speed_bounds_drive_rad_s=[-float(rng.uniform(*ranges["speed_capacity_drive_rad_s"])),
                                       float(rng.uniform(*ranges["speed_capacity_drive_rad_s"]))]))
    return c


def reference_projection(model, source, W):
    S = model.scale
    Q = S[:, None]*W*S
    u = source/S
    B = model.A*S/model.b[:, None]
    result = minimize(lambda z: .5*(z-u)@Q@(z-u), model.radial_clip(source)/S,
                      jac=lambda z: Q@(z-u), method="SLSQP",
                      constraints=[dict(type="ineq", fun=lambda z: 1-B@z, jac=lambda z: -B)],
                      options=dict(ftol=1e-11, maxiter=500))
    if not result.success or np.max(B@result.x-1) > 1e-8:
        raise RuntimeError(f"independent SLSQP reference unresolved: {result.message}")
    return result.x*S


def regression(template, protocol, weights, gates):
    maxima = dict(violation=0., optimality=0., oracle_metric_distance=0., box_dual_gap=0.,
                  inverse_error=0., independent_kinematics_error=0., minimum_box_gain_over_homothetic=1e100)
    failures, conditions, per_seed, count, oracle_count, certified_non_success_flags = [], [], [], 0, 0, 0
    started = perf_counter_ns()
    for seed in protocol["seeds"]:
        failed = 0
        for index in range(protocol["layouts_per_seed"]):
            # Independent per-layout streams prevent a failed layout from
            # changing all later parameter samples or command samples.
            rng = np.random.default_rng(np.random.SeedSequence([seed, index, 0]))
            config = random_config(template, rng, protocol["ranges"], f"random_{seed}_{index}")
            count += 1
            try:
                m = OmniModel(config)
                conditions.append(m.scaled_condition)
                box_result = maximum_box(m)
                certified_non_success_flags += int(not box_result["solver_success"])
                gain = box_result["box"].volume/m.inner_box.volume
                maxima["minimum_box_gain_over_homothetic"] = min(maxima["minimum_box_gain_over_homothetic"], gain)
                maxima["box_dual_gap"] = max(maxima["box_dual_gap"], box_result["log_volume_gap_bound"])
                command_rng = np.random.default_rng(np.random.SeedSequence([seed, index, 1]))
                u = command_rng.normal(size=(protocol["commands_per_layout"], 3))*m.scale*2
                error = float(np.max(abs((m.forward(m.inverse(u))-u)/m.scale)))
                maxima["inverse_error"] = max(maxima["inverse_error"], error)
                maxima["independent_kinematics_error"] = max(maxima["independent_kinematics_error"],
                    float(np.max(abs(m.inverse(u)-rigid_body_drive_speeds(m, u))/np.maximum(-m.lower, m.upper))))
                W = np.diag(weights/m.scale**2)
                for method in ("conservative_box", "maximum_box", "ray", "euclidean", "weighted"):
                    candidate = CommandProjector(m, method, W, box_result["box"])
                    v = candidate.project(u)
                    maxima["violation"] = max(maxima["violation"], float(physical_violation(m, v).max()))
                    if candidate.quadratic:
                        maxima["optimality"] = max(maxima["optimality"], candidate.quadratic.optimality_residual(u, v))
                        if index % protocol["oracle_every"] == 0:
                            for j in range(protocol["oracle_commands"]):
                                oracle = reference_projection(m, u[j], candidate.quadratic.W)
                                difference = v[j]-oracle
                                distance = np.sqrt(difference@candidate.quadratic.W@difference)/(1+np.linalg.norm(u[j]/m.scale))
                                maxima["oracle_metric_distance"] = max(maxima["oracle_metric_distance"], float(distance))
                                oracle_count += 1
                if gain < 1-1e-6:
                    raise RuntimeError("maximum zero-containing box smaller than homothetic feasible competitor")
            except (ValueError, RuntimeError) as error:
                failed += 1
                if len(failures) < 5:
                    failures.append(dict(seed=seed, index=index, reason=str(error), config=config))
        per_seed.append(dict(seed=seed, layouts=protocol["layouts_per_seed"], failures=failed))
        print(f"random regression seed {seed}: {protocol['layouts_per_seed']} layouts, failures={failed}", flush=True)
    total_failures = sum(s["failures"] for s in per_seed)
    passed = (not total_failures and maxima["violation"] <= gates["projection_relative_violation"] and
              maxima["optimality"] <= gates["optimality_residual"] and
              maxima["box_dual_gap"] <= gates["box_log_volume_dual_gap"] and
              maxima["inverse_error"] <= gates["regular_roundtrip_scaled"] and
              maxima["independent_kinematics_error"] <= gates["regular_roundtrip_scaled"] and
              maxima["oracle_metric_distance"] <= gates["oracle_metric_distance"])
    return dict(layouts=count, commands_per_layout=protocol["commands_per_layout"],
                independent_oracle_queries=oracle_count, maxima=maxima, condition=summary(conditions),
                per_seed=per_seed, failure_examples=failures, total_failures=total_failures, passed=passed,
                certified_box_solver_non_success_flags=certified_non_success_flags,
                elapsed_s=(perf_counter_ns()-started)/1e9)


def compact_evidence(result):
    """Keep decisive metrics, not raw samples or repeated per-draw summaries."""
    saved = {k: v for k, v in result.items() if k != "sensitivity"}
    if "sensitivity" in result:
        saved["sensitivity"] = []
        for item in result["sensitivity"]:
            rows = []
            for r in item["results"]:
                rows.append(dict(family=r["family"], level=r["level"], budget=r["budget"],
                    volume_ratio_min_median_max=[r["volume_ratio"][k] for k in ("minimum", "median", "maximum")],
                    nominal_method_violations={name: dict(mean_fraction=s["fraction_over_draws"]["mean"],
                        worst_draw_fraction=s["fraction_over_draws"]["maximum"], max_relative=s["maximum_relative"])
                        for name, s in r["nominal_method_violations"].items()},
                    robust_ray_max_relative_violation=r["robust_ray_max_relative_violation"],
                    actual_minus_proven_utilization_bound_max=r["actual_minus_proven_utilization_bound_max"]))
            saved["sensitivity"].append(dict(name=item["name"], draws_per_family_level=item["draws_per_family_level"],
                                             sources_per_draw=item["sources_per_draw"], results=rows))
    return saved


def evaluate_gates(result):
    gates = result["protocol"]["gates"]
    passed = []
    if "comparison" in result:
        for row in result["comparison"]:
            for name in METHODS[1:]:
                method, boundary = row["methods"][name], row["boundary"][name]
                passed.extend((method["max_relative_violation"] <= gates["projection_relative_violation"],
                    boundary["max_violation"] <= gates["projection_relative_violation"],
                    boundary["zero_distortion_scaled"] < 1e-12, boundary["idempotence_scaled"] < 1e-8))
                if name in ("ray", "euclidean", "weighted"):
                    passed.append(method["max_change_of_feasible_input_scaled"] < 1e-8)
                if "optimality_residual" in method:
                    passed.extend((method["optimality_residual"] <= gates["optimality_residual"],
                        method["max_cost_excess_over_feasible_alternatives"] <= gates["optimality_residual"],
                        max((v for s, v in boundary["local_gains"].items() if float(s) >= 1e-6), default=0.) <= 1+1e-5))
            for box in row["boxes"].values():
                passed.append(box["log_volume_gap_bound"] <= gates["box_log_volume_dual_gap"])
    if "sensitivity" in result:
        for row in result["sensitivity"]:
            passed.extend(r["robust_ray_max_relative_violation"] <= gates["projection_relative_violation"] and
                          r["actual_minus_proven_utilization_bound_max"] <= gates["projection_relative_violation"]
                          for r in row["results"])
    if "conditioning" in result:
        for row in result["conditioning"]["results"]:
            if row["epsilon"] == 0: passed.append(not row["accepted"])
            elif row["epsilon"] >= 1e-4: passed.append(row["accepted"])
            if row["accepted"]:
                passed.extend((row["vertex_relative_violation"] <= gates["projection_relative_violation"],
                               row["projection_relative_violation"] <= gates["projection_relative_violation"]))
    if "validation" in result: passed.append(result["validation"]["all_rejected"])
    if "regression" in result: passed.append(result["regression"]["passed"])
    return all(passed)


def plot_results(result, path):
    if "comparison" not in result:
        raise ValueError("comparison results required for plotting")
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/rm2027_omni_mpl")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.subplots_adjust(left=.07, right=.98, bottom=.17, top=.89, hspace=.5, wspace=.24)
    labels = ["Naive*", "Conservative", "Max box", "Ray", "Euclidean", "Weighted"]
    x = np.arange(len(METHODS))
    width = .18
    for i, row in enumerate(result["comparison"]):
        data = [row["methods"][n] for n in METHODS]
        values = ([100*m["retained_feasible_volume_fraction"] for m in data],
                  [m["weighted_distortion"]["mean"] for m in data],
                  [row["runtime"][n]["single_command_us"]["p99"] for n in METHODS],
                  [row["boundary"][n]["local_gains"][min(row["boundary"][n]["local_gains"],
                    key=lambda s: abs(np.log(float(s)/1e-6)))] for n in METHODS])
        for ax, y in zip(axes.flat, values):
            ax.bar(x+(i-1.5)*width, y, width=width, label=row["name"].replace("synthetic_", ""))
    settings = [("Retained feasible fixed-point volume", "percent of P"),
                ("Mean weighted command distortion", "dimensionless weighted norm"),
                ("Single-command runtime, p99", "microseconds (log scale)"),
                ("Observed local boundary gain at step 1e-6", "raw norm; weighted in W norm")]
    for ax, (title, ylabel) in zip(axes.flat, settings):
        ax.set(title=title, ylabel=ylabel, xticks=x, xticklabels=labels)
        ax.tick_params(axis="x", labelsize=8)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    axes[1, 0].set_yscale("log")
    handles, legend = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, legend, loc="lower center", bbox_to_anchor=(.5, .055), ncol=4)
    fig.text(.5, .02, "* Naive outputs can violate wheel limits. Synthetic Research only; no hardware/closed-loop claim.",
             ha="center", fontsize=10)
    fig.suptitle("Omni feasible-region stage 2 | common command samples | SYNTHETIC", fontsize=15)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=ROOT/"configs/stage2_protocol.json")
    parser.add_argument("--config", action="append", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/stage2")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--sections", nargs="+", choices=("comparison", "sensitivity", "conditioning", "validation", "regression"),
                        default=["comparison", "sensitivity", "conditioning", "validation", "regression"])
    args = parser.parse_args()
    try:
        protocol = json.loads(args.protocol.read_text(), object_pairs_hook=unique_object)
        validate_protocol(protocol)
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(f"invalid research protocol: {error}")
    paths = args.config or sorted((ROOT/"configs").glob("synthetic_*.json"))
    models = [OmniModel.load(p) for p in paths]
    weights = np.array(protocol["comparison"]["scaled_metric_weights"])
    if weights.shape != (3,) or not np.all(np.isfinite(weights) & (weights > 0)):
        parser.error("three positive finite metric weights required")
    args.output.mkdir(parents=True, exist_ok=True)
    result = dict(schema_version=2, research_only=True, hardware_validated=False, protocol=protocol,
                  environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                                   platform=platform.platform(), blas_threads=os.environ.get("OPENBLAS_NUM_THREADS")))
    base = OmniModel.load(ROOT/"configs/synthetic_symmetric4.json")
    for section in args.sections:
        print(f"starting {section}", flush=True)
        if section == "comparison":
            result[section] = [comparison(m, protocol[section], protocol["runtime"]) for m in models]
        elif section == "sensitivity":
            result[section] = [sensitivity(m, protocol[section], weights) for m in models]
        elif section == "conditioning":
            result[section] = conditioning(base, protocol[section])
        elif section == "validation":
            result[section] = validation(base)
        else:
            result[section] = regression(base.config, protocol[section], weights, protocol["gates"])
        # Save each completed independent section, including any failures.
        (args.output/"results.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    result["passed"] = evaluate_gates(result)
    (args.output/"results.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    (args.output/"evidence.json").write_text(json.dumps(compact_evidence(result), separators=(",", ":"), allow_nan=False)+"\n")
    if args.plot:
        plot_results(result, args.output/"comparison.png")
    print(f"stage2 gates: {'PASS' if result['passed'] else 'FAIL'}", flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
