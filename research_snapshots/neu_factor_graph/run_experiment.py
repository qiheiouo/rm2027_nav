#!/usr/bin/env python3
"""Frozen minimal controls and existing-GICP connectivity; no threshold search."""
import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from backend import exp6, inv, log6, solve, validate

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/neu_localization_l1"))
from run_baseline import write_pcd, COLUMNS, BOOLS


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def trajectory(n=31, straight=False):
    t = np.linspace(0, 1, n)
    T = np.repeat(np.eye(4)[None], n, axis=0)
    if straight:
        T[:, :3, 3] = np.column_stack([2 + 3*t, np.zeros(n), np.zeros(n)])
    else:
        T[:, :3, 3] = np.column_stack([2 + 3*t, .7*np.sin(t*3), .12*np.sin(t*4)])
        T[:, :3, :3] = Rotation.from_euler("xyz", np.column_stack([
            .08*np.sin(t*4), .1*np.sin(t*3), .3+.5*t])).as_matrix()
    return T


def dataset(truth, name, perfect=False, weak=False):
    rng = np.random.default_rng(20261007)
    O = [np.eye(4)]
    odom_sigmas = np.array([.025, .025, .015, .003, .003, .004])
    map_sigmas = np.array([.12, .12, .08, .025, .025, .03])
    for i in range(1, len(truth)):
        noise = np.zeros(6) if perfect else rng.normal(size=6)*odom_sigmas
        if not perfect:
            noise += [.015, 0, 0, 0, 0, .001]
        O.append(O[-1] @ (inv(truth[i-1]) @ truth[i]) @ exp6(noise))
    context = {"session_id": name, "odom_epoch": "synthetic_epoch_0",
               "numerical_basis_revision": "synthetic_canonical_base0_v1",
               "map_revision": "synthetic_fixed_map_v1", "calibration_revision": "known_synthetic_v1",
               "input_kind": "synthetic", "frames": {"map":"map", "odom":"odom", "base":"base_link"}}
    data = {"context": context, "samples": [], "scan_factors": [],
            "odom_sqrt_information": np.diag(1/np.array([.04,.04,.03,.008,.008,.008])).tolist(),
            "odom_noise_model_origin": "fixed nominal incremental model; includes designed drift, not calibrated LIO covariance"}
    for i, T in enumerate(O):
        stamp = 100_000_000_000 + i*200_000_000
        data["samples"].append({"id": i, "timestamp_ns": stamp, "T_odom_base": T.tolist()})
        if i % 3 == 0:
            noise = np.zeros(6) if perfect else rng.normal(size=6)*map_sigmas
            L = np.diag(1/map_sigmas)
            if weak:
                noise[3:] = 0  # Fixed straight corridor axes for exact longitudinal nullspace.
                noise[0] = 0
                if i == 0:
                    noise[:] = 0  # One explicitly known full-pose synthetic start anchor.
                else:
                    L[0, 0] = 0  # No longitudinal information, not a tiny finite surrogate.
            measurement = truth[i]@exp6(noise)
            if weak and i > 0:
                # Unobserved coordinate is seed-propagated, never injected from GT.
                measurement[0,3] = (truth[0]@np.array(O[i]))[0,3]
            data["scan_factors"].append({"id": f"scan_{i}", "query_id": f"independent_query_{i}",
                "node_id": i, "timestamp_ns": stamp, "context": copy.deepcopy(context),
                "basin_status": "single_basin_assumed", "T_map_base": measurement.tolist(),
                "sqrt_information": L.tolist(),
                "noise_model_origin": "analytic independent measurement model; rank-5 corridor after first anchor" if weak else "analytic independent measurement model"})
    return data


def baseline(data):
    O, _, factors = validate(data)
    by_node = {i:S for i,S,_ in factors}
    X, Cs = [], []
    C = None
    for i, pose in enumerate(O):
        if i in by_node:
            C = by_node[i] @ inv(pose)
        if C is None:
            raise ValueError("comparison requires first-node global anchor")
        X.append(C @ pose)
        Cs.append(C.copy())
    return np.array(X), np.array(Cs)


def causal_graph(data):
    O, _, factors = validate(data)
    nodes = {i for i, _, _ in factors}
    X, Cs, runs = [], [], []
    current = None
    previous = None
    for i, odom in enumerate(O):
        if i in nodes:
            prefix = copy.deepcopy(data)
            prefix["samples"] = data["samples"][:i+1]
            prefix["scan_factors"] = [f for f in data["scan_factors"] if f["node_id"] <= i]
            if "odom_factors" in data:
                prefix["odom_factors"] = data["odom_factors"][:i]
            initial = None
            if previous is not None:
                count = len(previous)
                initial = np.concatenate([previous, current @ O[count:i+1]])
            result = solve(prefix, initial)
            previous = np.array(result["T_map_base"])
            current = np.array(result["T_map_odom"])[-1]
            runs.append({"node": i, **{k:result[k] for k in ["cost", "solver_success", "nfev", "optimality", "solve_ms"]}})
        X.append(current @ odom)
        Cs.append(current.copy())
    return np.array(X), np.array(Cs), runs


def errors(X, truth, Cs, scan_nodes):
    position = np.linalg.norm(X[:, :3, 3]-truth[:, :3, 3], axis=1)
    rotation = np.linalg.norm(log6(inv(truth)@X)[:, 3:], axis=1)
    changes = inv(Cs[scan_nodes[:-1]]) @ Cs[scan_nodes[1:]]
    return {"position_rmse_m": float(np.sqrt(np.mean(position**2))),
            "position_axis_rmse_m": np.sqrt(np.mean((X[:, :3, 3]-truth[:, :3, 3])**2, axis=0)).tolist(),
            "position_final_axis_error_m": (X[-1, :3, 3]-truth[-1, :3, 3]).tolist(),
            "position_final_m": float(position[-1]), "rotation_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(rotation**2)))),
            "correction_step_translation_rms_m": float(np.sqrt(np.mean(np.sum(changes[:, :3, 3]**2, axis=1)))),
            "correction_step_rotation_rms_deg": float(np.rad2deg(np.sqrt(np.mean(np.sum(log6(changes)[:, 3:]**2, axis=1)))))}


def compare(data, truth, output, name):
    save(output / f"{name}_input.json", data)
    base, Cbase = baseline(data)
    causal, Ccausal, runs = causal_graph(data)
    batch = solve(data)
    save(output / f"{name}_batch.json", batch)
    save(output / f"{name}_causal.json", {"T_map_base":causal.tolist(), "T_map_odom":Ccausal.tolist(), "prefix_runs":runs})
    nodes = [f["node_id"] for f in data["scan_factors"]]
    summary = {"samples": len(truth), "scan_factors": len(nodes),
               "single_correction": errors(base, truth, Cbase, nodes),
               "causal_prefix_graph": errors(causal, truth, Ccausal, nodes),
               "offline_full_batch": errors(np.array(batch["T_map_base"]), truth, np.array(batch["T_map_odom"]), nodes),
               "batch_cost": batch["cost"], "all_solvers_converged": all(r["solver_success"] for r in runs) and batch["solver_success"],
               "median_prefix_solve_ms": float(np.median([r["solve_ms"] for r in runs]))}
    reconstructed = np.array(batch["T_map_odom"]) @ np.array([s["T_odom_base"] for s in data["samples"]])
    summary["map_odom_reconstruction_max_abs"] = float(np.max(abs(reconstructed-np.array(batch["T_map_base"]))))
    return summary


def math_checks(data):
    rng = np.random.default_rng(7)
    twists = rng.normal(size=(30, 6))*.2
    twists[:2] = 0
    twists[1] = 1e-10
    transforms = exp6(twists)
    check = {"exp_log_max_abs": float(np.max(abs(log6(transforms)-twists))),
             "inverse_max_abs": float(np.max(abs(transforms@inv(transforms)-np.eye(4))))}
    relative = copy.deepcopy(data)
    relative["scan_factors"] = []
    check["no_map_status"] = solve(relative)["status"]
    check["no_map_has_global_output"] = solve(relative)["T_map_odom"] is not None
    rejected = []
    for name in ["ambiguous", "duplicate_query", "cross_epoch", "timestamp_mismatch"]:
        bad = copy.deepcopy(data)
        if name == "ambiguous": bad["scan_factors"][0]["basin_status"] = "ambiguous"
        if name == "duplicate_query": bad["scan_factors"].append(copy.deepcopy(bad["scan_factors"][0]))
        if name == "cross_epoch": bad["scan_factors"][0]["context"]["odom_epoch"] = "reset_epoch_1"
        if name == "timestamp_mismatch": bad["scan_factors"][0]["timestamp_ns"] += 1
        try:
            validate(bad)
        except ValueError:
            rejected.append(name)
    check["malformed_inputs_rejected"] = rejected
    assert check["exp_log_max_abs"] < 1e-10 and check["inverse_max_abs"] < 1e-10
    assert not check["no_map_has_global_output"] and len(rejected) == 4
    return check


def gicp_measurements(output):
    """One fixed synthetic room, independent map/query samples; same V defaults."""
    binary = ROOT / "build/l1_probe/registration_probe"
    if not binary.exists():
        raise FileNotFoundError("rebuild existing L1 probe before GICP connectivity experiment")
    rng = np.random.default_rng(42)

    def room(seed):
        r = np.random.default_rng(seed)
        surfaces = []
        for origin,u,v,count in [([0,-3,0],[8,0,0],[0,6,0],2000),
                ([0,-3,0],[8,0,0],[0,0,2.5],1000),([0,3,0],[8,0,0],[0,0,2.5],900),
                ([0,-3,0],[0,6,0],[0,0,2.5],800),([8,-3,0],[0,6,0],[0,0,2.5],800),
                ([5.8,-1.3,0],[.6,.2,0],[0,0,1.7],600),([1.0,1.4,0],[.7,-.1,0],[0,0,1.2],500)]:
            surfaces.append(np.array(origin)+r.random((count,1))*u+r.random((count,1))*v)
        return np.concatenate(surfaces)

    target = room(100)
    write_pcd(output / "map.pcd", target)
    truth = trajectory(16)
    data = dataset(truth, "existing_gicp_connectivity")
    data["scan_factors"] = []
    O = np.array([s["T_odom_base"] for s in data["samples"]])
    initial_correction = truth[0] @ inv(O[0])  # Declared known synthetic coarse prior.
    rows = []
    for i in range(0, len(truth), 3):
        # Registration sees points in numerical O, not falsely relabelled raw LIO R.
        query_world = room(200+i)
        local = (inv(truth[i])[:3,:3] @ query_world.T).T + inv(truth[i])[:3,3]
        local += rng.normal(0, .015, local.shape)
        source = (O[i,:3,:3] @ local.T).T + O[i,:3,3]
        source_path, diag_path, seed_path = output/f"source_{i}.pcd", output/f"diag_{i}.json", output/f"seed_{i}.txt"
        write_pcd(source_path, source)
        np.savetxt(seed_path, initial_correction, fmt="%.17g")
        args = [str(binary), str(output/"map.pcd"), str(source_path), "0", "0", "0", ".01", "1",
                ".25", ".20", "1.0", ".001", ".001", "30", "100", "100", ".50", "2.0", "1.0", str(diag_path), str(seed_path)]
        run = subprocess.run(args, check=True, capture_output=True, text=True, timeout=30)
        values = run.stdout.strip().splitlines()[-1].split(",")
        if len(values) != len(COLUMNS): raise RuntimeError("probe schema mismatch")
        row = {k:(v if k=="reason" else bool(int(v)) if k in BOOLS else float(v)) for k,v in zip(COLUMNS,values)}
        correction = np.array([[row[f"T{r}{c}"] for c in range(4)] for r in range(4)])
        S = correction @ O[i]
        rows.append({"node":i, "V_accept":row["algorithm_accept"], "probe_pose_gate":row["pose_gate_accept"],
                     "fitness":row["fitness"], "T_map_base":S.tolist()})
        if row["algorithm_accept"]:
            data["scan_factors"].append({"id":f"gicp_{i}", "query_id":f"independent_pcd_{i}", "node_id":i,
                "timestamp_ns":data["samples"][i]["timestamp_ns"], "context":copy.deepcopy(data["context"]),
                "basin_status":"single_basin_assumed", "T_map_base":S.tolist(),
                "sqrt_information":np.diag([12.5,12.5,12.5,33.333,33.333,33.333]).tolist(),
                "noise_model_origin":"fixed nominal demonstration weights; NOT fitness or T diagnostic Hessian"})
    save(output/"gicp_measurements.json", rows)
    return data, truth, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/neu_factor_graph")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    truth = trajectory()
    perfect = dataset(truth, "perfect", perfect=True)
    normal = dataset(truth, "drift_noise")
    alias = copy.deepcopy(normal)
    alias["context"]["session_id"] = "alias_12m"
    shift = np.eye(4); shift[0,3] = 12
    for f in alias["scan_factors"]:
        f["context"] = copy.deepcopy(alias["context"])
        f["T_map_base"] = (shift@np.array(f["T_map_base"])).tolist()
    straight = trajectory(straight=True)
    weak = dataset(straight, "weak_longitudinal", weak=True)
    summary = {"scope":"synthetic controls only; no new-car performance claim", "math_checks":math_checks(perfect), "cases":{}}
    for name, data, gt in [("perfect",perfect,truth), ("drift_noise",normal,truth), ("alias_12m",alias,truth), ("weak_longitudinal",weak,straight)]:
        summary["cases"][name] = compare(data, gt, args.output, name)
        print(name, json.dumps(summary["cases"][name], allow_nan=False), flush=True)
    gicp, gt, rows = gicp_measurements(args.output)
    summary["cases"]["gicp_connectivity"] = compare(gicp, gt, args.output, "gicp_connectivity")
    summary["gicp_measurements"] = rows
    # Compare trajectories against each other without using GT inside the solver.
    normal_batch = json.loads((args.output/"drift_noise_batch.json").read_text())
    alias_batch = json.loads((args.output/"alias_12m_batch.json").read_text())
    summary["alias_invariance"] = {
        "cost_absolute_difference":abs(alias_batch["cost"]-normal_batch["cost"]),
        "shifted_pose_max_abs_difference":float(np.max(abs(np.array(alias_batch["T_map_base"])-shift@np.array(normal_batch["T_map_base"]))))}
    save(args.output/"summary.json", summary)
    print("summary", args.output/"summary.json", flush=True)


if __name__ == "__main__":
    main()
