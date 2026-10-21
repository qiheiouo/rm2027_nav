#!/usr/bin/env python3
"""Finite L1 synthetic seed sweep; no active TF, ROS publishers or commands.

The PCL/basic-gate probe mirrors new-car bd6cf689; quality is a passive
counterfactual from T e374894a. Ground truth is kept out of probe inputs.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import numpy as np
import yaml

SOURCE_PIN = "bd6cf689d3d5aca5c8edd430ff760ca5ceb27496"
QUALITY_PIN = "e374894a7a713ea84b82de9e92479a37041f39ff"
PARAM_KEYS = [
    "map_leaf_size", "source_leaf_size", "max_correspondence_distance",
    "transformation_epsilon", "euclidean_fitness_epsilon", "max_iterations",
    "min_source_points", "min_target_points", "max_fitness_score",
    "max_translation_jump", "max_yaw_jump",
]
COLUMNS = [
    "source_points", "target_points", "converged", "fitness",
    "solve_ms", "probe_total_ms", "algorithm_accept", "pose_gate_accept",
    "reason", "translation_jump", "yaw_jump", "T_passive_overlap",
    "T_passive_min_eigenvalue", "T_passive_condition", "T_counterfactual_gate",
] + [f"T_passive_eigen{i}" for i in range(6)] + [f"T{r}{c}" for r in range(4) for c in range(4)]
BOOLS = {"converged", "algorithm_accept", "pose_gate_accept", "T_counterfactual_gate"}

def command(args, **kwargs):
    return subprocess.check_output(args, text=True, **kwargs).strip()

def read_pcd(path):
    s = path.read_text()
    payload = s.split("DATA ascii\n", 1)[1]
    return np.array([[float(v) for v in l.split()] for l in payload.splitlines() if l.strip()])

def write_pcd(path, pts):
    header = (
        "# Synthetic L1 diagnostic only. Never deploy.\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\nDATA ascii\n"
    )
    path.write_text(header + "".join(" ".join(f"{v:.8f}" for v in p)+"\n" for p in pts))

def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite(v) for v in value]
    return value

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--reuse-build", action="store_true")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Code identity: no cherry-picking of quality or recovery into the baseline.
    changed = command(["git", "diff", SOURCE_PIN, "--", "src"], cwd=root)
    if changed:
        raise SystemExit("Baseline source differs from fixed new-car pin")
    cfgpath = root/"src/rm_gicp_relocalization/config/gicp_3d.yaml"
    params = yaml.safe_load(cfgpath.read_text())["gicp_relocalization"]["ros__parameters"]
    quality_root = root/"build/l1_reference_quality"
    for rel in ["include/rm_gicp_relocalization/registration_quality.hpp", "src/registration_quality.cpp"]:
        p = quality_root/rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(command(
            ["git", "show", f"{QUALITY_PIN}:src/rm_gicp_relocalization/{rel}"], cwd=root)+"\n")
    build = root/"build/l1_probe"
    if not args.reuse_build:
        subprocess.run([
            "cmake", "-S", str(Path(__file__).parent), "-B", str(build),
            f"-DPROJECT_ROOT={root}", f"-DQUALITY_ROOT={quality_root}",
            "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_CXX_FLAGS=-pipe",
        ], check=True)
        subprocess.run(["cmake", "--build", str(build), "-j1"], check=True)
    fixture = root/"src/rm_map_tools/maps/phase2e_test/phase2j_3d_test.pcd"
    pts = read_pcd(fixture)
    inputs = root/"build/l1_inputs"
    inputs.mkdir(parents=True, exist_ok=True)
    double = inputs/"duplicate_12m.pcd"
    write_pcd(double, np.vstack([pts, pts+np.array([12., 0., 0.])]))
    sparse = inputs/"sparse_50.pcd"
    write_pcd(sparse, pts[::10][:50])
    floor = inputs/"planar_floor.pcd"
    write_pcd(floor, np.array([[x, y, 0.] for x in np.linspace(-3,3,31)
                            for y in np.linspace(-3,3,31)]))
    cases = []
    def add(name, group, seed, target=fixture, source=fixture, has_seed=True, age=0.):
        cases.append(dict(id=name, group=group, seed=list(seed), target=str(target),
                          source=str(source), has_seed=has_seed, age=age))
    add("identity", "D1_fixture", (0,0,0))
    for axis, values in [("x", [.5,1.,2.,3.]), ("y", [.5,1.,2.])]:
        for value in values:
            for sign in [-1,1]:
                seed = (sign*value,0,0) if axis=="x" else (0,sign*value,0)
                add(f"{axis}_{sign*value:+g}", "D1_fixture", seed)
    for value in [.25,.5,1.,math.pi]:
        for sign in [-1,1]:
            add(f"yaw_{sign*value:+.4f}", "D1_fixture", (0,0,sign*value))
    for i, seed in enumerate([(1.,1.,.5),(2.,2.,1.),(-1.,1.,-.5)]):
        add(f"joint_{i}", "D1_fixture", seed)
    add("wrong_far_xy", "D2_wrong_seed", (12,0,0))
    add("no_seed", "D2_wrong_seed", (0,0,0), has_seed=False)
    add("duplicate_correct", "D2_synthetic_duplicate", (0,0,0), target=double)
    add("duplicate_wrong_12m", "D2_synthetic_duplicate", (12,0,0), target=double)
    add("floor_identity", "D3_synthetic_degenerate", (0,0,0), target=floor, source=floor)
    add("sparse_50", "D3_synthetic_degenerate", (0,0,0), source=sparse)
    add("stale_pose_0p6s", "D3_synthetic_time_gate", (0,0,0), age=.6)
    add("future_pose_0p2s", "D3_synthetic_time_gate", (0,0,0), age=-.2)

    # Probe input has no GT, labels or oracle. Identity truth is only scored below.
    manifest = {
        "source_pin": SOURCE_PIN, "historical_quality_pin": QUALITY_PIN,
        "historical_quality_role": "passive_only_NOT_new_car_gate",
        "parameters": params,
        "fixture": {
            "bundle": "src/rm_map_tools/maps/phase2e_test/phase2j_3d_test.bundle.yaml",
            "pcd": str(fixture.relative_to(root)),
            "pcd_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
            "source_points_before_filter": len(pts),
        },
        "generated_inputs": "Deterministic translation, subsampling and grid in run_baseline.py; no randomness",
        "case_inputs": [dict(c, target=str(Path(c["target"]).relative_to(root)),
                              source=str(Path(c["source"]).relative_to(root))) for c in cases],
        "synthetic_oracle_tolerance": {"xy_m": .10, "yaw_rad": .10, "z_m": .10},
        "environment": {
            "ros_distro": os.environ.get("ROS_DISTRO","UNKNOWN"),
            "pcl": command(["pkg-config","--modversion","pcl_registration"]),
            "eigen": command(["pkg-config","--modversion","eigen3"]),
            "platform": platform.platform(), "python": sys.version.split()[0],
        },
        "limitations": [
            "Synthetic queries are not independent real MID360 episodes.",
            "ROS/PCL host versions differ from the original Humble target.",
            "No LIO, gimbal calibration, runtime recovery or motion exercised.",
            "probe_total_ms includes passive diagnostics; not production latency.",
        ],
    }
    (out/"manifest.json").write_text(json.dumps(finite(manifest),ensure_ascii=False,indent=2)+"\n")
    results=[]
    for case in cases:
        argv = [str(build/"registration_probe"), case["target"], case["source"]]
        argv += [str(v) for v in case["seed"]]
        argv += [str(case["age"]), str(int(case["has_seed"]))]
        argv += [str(params[k]) for k in PARAM_KEYS]
        run = subprocess.run(argv, capture_output=True, text=True, check=True, timeout=30)
        values = run.stdout.strip().splitlines()[-1].split(",")
        if len(values)!=len(COLUMNS):
            raise RuntimeError(f"Bad probe schema: {case['id']}")
        row = dict(case_id=case["id"],group=case["group"],
                   seed_x=case["seed"][0],seed_y=case["seed"][1],
                   seed_yaw=case["seed"][2])
        for k,v in zip(COLUMNS,values):
            row[k] = v if k=="reason" else (bool(int(v)) if k in BOOLS else float(v))
        row["accepted_chain"] = row["algorithm_accept"] and row["pose_gate_accept"]
        row["final_xy_error_m"] = math.hypot(row["T03"],row["T13"])
        row["final_yaw_error_rad"] = abs(math.atan2(row["T10"],row["T00"]))
        row["correct_synthetic_basin"] = (
            row["final_xy_error_m"] <= .10 and row["final_yaw_error_rad"] <= .10
            and abs(row["T23"]) <= .10
        )
        row["false_accept"] = row["accepted_chain"] and not row["correct_synthetic_basin"]
        if row["solve_ms"] == 0:
            row["final_xy_error_m"] = None
            row["final_yaw_error_rad"] = None
            row["correct_synthetic_basin"] = None
        results.append(row)
        print(case["id"],row["reason"],"xy=",row["final_xy_error_m"],
              "yaw=",row["final_yaw_error_rad"],f"false_accept={row['false_accept']}",flush=True)
    with (out/"results.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(results[0]),lineterminator="\n")
        writer.writeheader();writer.writerows(results)
    summary = dict(cases=len(results),
                   false_accept_cases=[r["case_id"] for r in results if r["false_accept"]],
                   rejected_cases=[r["case_id"] for r in results if not r["accepted_chain"]])
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(summary,ensure_ascii=False))

if __name__=="__main__":
    main()
