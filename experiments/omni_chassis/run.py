#!/usr/bin/env python3
"""Reproducible config -> matrices, polytopes, box comparison and research gates."""

import argparse
from itertools import combinations
import json
from pathlib import Path
import platform
import subprocess

import numpy as np
import scipy

from model import OmniModel, REL_TOL


ROOT = Path(__file__).resolve().parent
BOX_NAMES = ("configured", "pure_axis", "homothetic_inner")


def rigid_body_drive_speeds(model, velocities):
    """Independent reference using 3D rigid-body point velocity, without J."""
    velocity = np.asarray(velocities)
    translation = np.column_stack((velocity[:, :2], np.zeros(len(velocity))))
    rotation = np.column_stack((np.zeros((len(velocity), 2)), velocity[:, 2]))
    speeds = []
    for w in model.config["wheels"]:
        point = np.array([*w["position_xy_m"], 0.])
        direction = np.array([*w["rolling_direction_xy"], 0.])
        local = translation + np.cross(rotation, point)
        speeds.append((local @ direction) / w["radius_m"] *
                      w["drive_sign"] * w["drive_per_wheel_ratio"])
    return np.array(speeds).T


def proportion(count, total):
    """Conditional fraction and Wilson 95% interval (Monte Carlo uncertainty)."""
    if not total:
        return dict(count=int(count), total=0, fraction=None, wilson95=None)
    p, z = count / total, 1.959963984540054
    denominator = 1 + z * z / total
    centre = (p + z * z / (2 * total)) / denominator
    half = z * np.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return dict(count=int(count), total=int(total), fraction=float(p),
                wilson95=[0. if count == 0 else max(0., float(centre - half)),
                          1. if count == total else min(1., float(centre + half))])


def run_experiment(model, samples_per_seed, seeds, boundary_samples=2048):
    boxes = dict(zip(BOX_NAMES, (model.configured_box, model.axis_box, model.inner_box)))
    # All methods see exactly the same uniform velocities in one enclosing box.
    domain_lo = np.minimum(model.bounding_box.lower, model.configured_box.lower)
    domain_hi = np.maximum(model.bounding_box.upper, model.configured_box.upper)
    totals = {name: dict(accepted=0, false_accept=0, false_reject=0) for name in boxes}
    feasible_total, mismatch, roundtrip, reference_error, clip_violation = 0, 0, 0., 0., 0.
    per_seed = []
    for seed in seeds:
        velocity = np.random.default_rng(seed).uniform(domain_lo, domain_hi, (samples_per_seed, 3))
        drive = rigid_body_drive_speeds(model, velocity)
        truth = np.all((drive <= model.upper * (1 + REL_TOL)) &
                       (drive >= model.lower * (1 + REL_TOL)), axis=1)
        feasible = int(truth.sum())
        feasible_total += feasible
        mismatch += int(np.count_nonzero(truth != model.contains(velocity)))
        roundtrip = max(roundtrip, float(np.max(np.abs((model.forward(model.inverse(velocity)) -
                                                      velocity) / model.scale))))
        reference_error = max(reference_error, float(np.max(np.abs(model.inverse(velocity) - drive) /
                                                            np.maximum(-model.lower, model.upper))))
        clipped = model.radial_clip(velocity)
        # Check clip output using the independent point-velocity reference too.
        clip_drive = rigid_body_drive_speeds(model, clipped)
        clip_util = np.maximum(clip_drive / model.upper, clip_drive / model.lower)
        clip_violation = max(clip_violation, float(clip_util.max() - 1))
        row = dict(seed=seed, feasible=feasible, boxes={})
        for name, box in boxes.items():
            accepted = box.contains(velocity)
            counts = dict(accepted=int(accepted.sum()),
                          false_accept=int(np.count_nonzero(accepted & ~truth)),
                          false_reject=int(np.count_nonzero(~accepted & truth)))
            for key, value in counts.items():
                totals[name][key] += value
            row["boxes"][name] = dict(**counts,
                false_accept_fraction=(counts["false_accept"] / counts["accepted"]
                                       if counts["accepted"] else None),
                false_reject_fraction=counts["false_reject"] / feasible if feasible else None)
        per_seed.append(row)
    count = samples_per_seed * len(seeds)
    summary = {}
    for name, box in boxes.items():
        c = totals[name]
        corner_drive = rigid_body_drive_speeds(model, box.corners)
        utilization = np.maximum(corner_drive / model.upper, corner_drive / model.lower)
        summary[name] = dict(
            box=box.export(), volume_over_polytope=box.volume / model.volume,
            max_corner_wheel_utilization=float(utilization.max()),
            false_accept=proportion(c["false_accept"], c["accepted"]),
            false_reject=proportion(c["false_reject"], feasible_total))

    # Fixed seed independent of MC seeds. Dimensionless directions avoid silently
    # treating a rad/s axis and an m/s axis as comparable Euclidean lengths.
    rng = np.random.default_rng(20261024)
    directions = np.vstack((rng.normal(size=(boundary_samples, 3)) * model.scale,
                            np.eye(3) * model.scale, -np.eye(3) * model.scale,
                            model.vertices))
    edge = directions * model.ray_limit(directions)[:, None]
    relative_slack = model.slack(edge) / model.b
    boundary = dict(
        count=len(edge),
        max_boundary_residual=float(np.max(np.abs(relative_slack.min(axis=1)))),
        max_vertex_violation=float(np.maximum(0., (-model.slack(model.vertices) / model.b).max())),
        inward_rejected=int(np.count_nonzero(~model.contains(edge * (1 - 1e-6)))),
        outward_accepted=int(np.count_nonzero(model.contains(edge * (1 + 1e-6)))),
        inner_box_corners_rejected=int(np.count_nonzero(~model.contains(model.inner_box.corners))))
    gates = dict(
        independent_kinematics=reference_error < 1e-10,
        roundtrip=roundtrip < 1e-10,
        wheel_polytope_agreement=mismatch == 0,
        radial_clip=clip_violation < 1e-9,
        boundaries=(boundary["max_boundary_residual"] < 1e-9 and
                    boundary["max_vertex_violation"] < 1e-9 and
                    boundary["inward_rejected"] == 0 and boundary["outward_accepted"] == 0),
        inner_box=(boundary["inner_box_corners_rejected"] == 0 and
                   totals["homothetic_inner"]["false_accept"] == 0))
    return dict(
        name=model.config["name"], parameter_status=model.config["parameter_status"],
        gates=gates, passed=all(gates.values()),
        volume=model.volume, vertex_count=len(model.vertices),
        scaled_condition=model.scaled_condition, inner_scale=model.inner_scale,
        monte_carlo=dict(samples_per_seed=samples_per_seed, seeds=seeds, total=count,
                         distribution="uniform in common enclosing box",
                         domain_lower=domain_lo.tolist(), domain_upper=domain_hi.tolist(),
                         feasible=proportion(feasible_total, count),
                         wheel_polytope_disagreements=mismatch,
                         max_reference_error_normalized=reference_error,
                         max_roundtrip_error_scaled=roundtrip,
                         max_radial_clip_violation_relative=clip_violation,
                         boxes=summary, per_seed=per_seed),
        boundary=boundary)


def slice_polygon(model, wz):
    rows = model.A[:, :2]
    rhs = model.b - model.A[:, 2] * wz
    points = []
    for i, j in combinations(range(len(rows)), 2):
        block = rows[[i, j]]
        if abs(np.linalg.det(block)) < 1e-10:
            continue
        point = np.linalg.solve(block, rhs[[i, j]])
        if np.all((rows @ point - rhs) / model.b <= REL_TOL):
            if not any(np.allclose(point, p, atol=1e-8) for p in points):
                points.append(point)
    if len(points) < 3:
        return np.empty((0, 2))
    points = np.array(points)
    angles = np.arctan2(*(points - points.mean(axis=0)).T[::-1])
    return points[np.argsort(angles)]


def plot(model, output):
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/rm2027_omni_mpl")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5))
    fig.subplots_adjust(left=.06, right=.99, bottom=.24, top=.82, wspace=.28)
    levels = (0., .5 * model.axis_box.upper[2], .9 * model.axis_box.upper[2])
    boxes = (model.configured_box, model.axis_box, model.inner_box)
    for ax, wz in zip(axes, levels):
        polygon = slice_polygon(model, wz)
        if len(polygon):
            ax.add_patch(Polygon(polygon, facecolor="#9dc8d8", edgecolor="#13566a",
                                 alpha=.7, label="wheel polytope slice"))
        for box, label, color, style in zip(boxes, BOX_NAMES,
                                          ("#db833b", "#a94059", "#417c3d"), ("--", ":", "-.",)):
            if box.lower[2] <= wz <= box.upper[2]:
                ax.add_patch(Rectangle(box.lower[:2], *(box.upper - box.lower)[:2],
                                       fill=False, edgecolor=color, linestyle=style, label=label))
        extent = np.maximum(abs(model.bounding_box.lower[:2]), model.bounding_box.upper[:2])
        extent = np.maximum(extent, np.maximum(abs(model.configured_box.lower[:2]),
                                               model.configured_box.upper[:2])) * 1.08
        ax.set(xlim=(-extent[0], extent[0]), ylim=(-extent[1], extent[1]),
               xlabel="vx [m/s]", ylabel="vy [m/s]", title=f"wz = {wz:.3g} rad/s")
        ax.set_aspect("equal")
        ax.grid(alpha=.2)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, fontsize=8, loc="lower center", ncol=4,
               bbox_to_anchor=(.5, .015))
    fig.suptitle(model.config["name"] + " | SYNTHETIC / RESEARCH ONLY"
                 if model.config["parameter_status"] == "synthetic" else
                 model.config["name"] + " | RESEARCH ONLY / NOT HARDWARE VALIDATED")
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, action="append", help="repeat for multiple configs")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs")
    parser.add_argument("--samples-per-seed", type=int, default=20000)
    parser.add_argument("--seeds", type=int, nargs="+", default=[11, 23, 47, 71, 101])
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if args.samples_per_seed < 1 or any(s < 0 for s in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error("positive sample count and distinct nonnegative seeds required")
    paths = args.config or sorted((ROOT / "configs").glob("synthetic_*.json"))
    # Validate the entire batch before creating any partial result directories.
    try:
        models = [OmniModel.load(p) for p in paths]
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    if len({m.config["name"] for m in models}) != len(models):
        parser.error("configuration names must be unique in a batch")
    if len({p.stem for p in paths}) != len(paths):
        parser.error("configuration filenames must have unique stems in a batch")
    try:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True))
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = None, None
    results = dict(schema_version=1, research_only=True, hardware_validated=False,
                   environment=dict(python=platform.python_version(), numpy=np.__version__,
                                    scipy=scipy.__version__, base_revision=revision, dirty=dirty),
                   experiments=[])
    args.output.mkdir(parents=True, exist_ok=True)
    for model in models:
        # Names are metadata; use the input filename for filesystem destinations.
        index = len(results["experiments"])
        directory = args.output / paths[index].stem
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "model.json").write_text(json.dumps(model.export(), indent=2, allow_nan=False) + "\n")
        result = run_experiment(model, args.samples_per_seed, args.seeds)
        results["experiments"].append(result)
        if args.plot:
            plot(model, directory / "slices.png")
        rates = result["monte_carlo"]["boxes"]["pure_axis"]
        fraction = rates["false_accept"]["fraction"]
        formatted_rate = f"{fraction:.2%}" if fraction is not None else "unobserved"
        print(f"{model.config['name']}: {'PASS' if result['passed'] else 'FAIL'}; "
              f"vertices={len(model.vertices)}, "
              f"pure-axis false accept={formatted_rate}, "
              f"inner volume/P={model.inner_box.volume/model.volume:.2%}")
    (args.output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
    return 0 if all(r["passed"] for r in results["experiments"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
