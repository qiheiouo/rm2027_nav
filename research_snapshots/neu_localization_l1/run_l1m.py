#!/usr/bin/env python3
"""L1-M bounded synthetic mechanisms; fixed gates, no global retrieval or ROS.

All clouds entering the probe use a single mathematical canonical odom basis.
Labels and ideal odometry are scoring-only. Partial masks are NOT MID360 scans.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import subprocess
import numpy as np
import yaml
from run_baseline import (SOURCE_PIN, QUALITY_PIN, PARAM_KEYS, COLUMNS, BOOLS,
                          command, read_pcd, write_pcd, finite)


def save(path, data):
    path.write_text(json.dumps(finite(data), ensure_ascii=False, indent=2)+"\n")


def plot_aliasing(out, points, rows):
    import os
    cache = Path(__file__).resolve().parents[2]/"build/l1m_matplotlib"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR",str(cache))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    wrong = rows["repeat_wrong12"]
    correct = rows["repeat_correct"]
    T = np.array([[wrong[f"T{r}{c}"] for c in range(4)] for r in range(4)])
    aligned = points@T[:3,:3].T+T[:3,3]
    fig, axes = plt.subplots(1,2,figsize=(10,3.8),layout="constrained")
    ax = axes[0]
    ax.scatter(points[:,0],points[:,1],s=8,color="#6b7280",label="Map: correct place")
    ax.scatter(points[:,0]+12,points[:,1],s=8,color="#b6bcc5",label="Map: repeated place")
    ax.scatter(aligned[:,0],aligned[:,1],s=12,marker="+",color="#c44136",label="Aligned query: wrong basin")
    ax.scatter([0,12],[0,0],marker="x",s=65,color=["#18735d","#c44136"])
    ax.annotate("",xy=(12,.3),xytext=(0,.3),
        arrowprops={"arrowstyle":"<->","color":"#27272a"})
    ax.text(6,.55,"12 m global error",ha="center",fontsize=9)
    ax.text(0,3.05,"Correct map place",ha="center",va="top",fontsize=8)
    ax.text(12,3.05,"Repeated map place\n+ aligned query (red)",ha="center",va="top",fontsize=8)
    ax.set(xlabel="Map x (m)",ylabel="Map y (m)",ylim=(-2.5,3.3))
    ax.set_title("Good local alignment at the wrong place",fontsize=11)
    ax.set_aspect("equal")
    ax = axes[1];x = np.arange(1,7)
    ax.plot(x,[correct[f"T_passive_eigen{i}"] for i in range(6)],"o-",color="#18735d",label="Correct basin")
    ax.plot(x,[wrong[f"T_passive_eigen{i}"] for i in range(6)],"x--",color="#c44136",label="Wrong basin (+12 m)")
    ax.axhline(.001,color="#6b7280",linestyle=":",label="Historical T min-eigen threshold")
    ax.set_yscale("log");ax.set(xlabel="Eigenvalue index",ylabel="Average point-to-plane information",
        xticks=x)
    ax.set_title("Local observability does not identify the place",fontsize=11)
    ax.legend(fontsize=8,loc="lower right");ax.grid(alpha=.2)
    fig.savefig(out/"aliasing.png",dpi=170)
    plt.close(fig)


def geometry_audit(root):
    cfg = yaml.safe_load((Path(__file__).parent/"new_car_cad_baseline.yaml").read_text())
    polygon = np.array(cfg["footprint_m"])
    axis = np.array(cfg["gimbal"]["yaw_axis_xy_m"])
    result = {
        "reference": "cad_wheel_center; ground base differs by +0.06649 m in Z",
        "footprint_span_xy_m": np.ptp(polygon, axis=0).tolist(),
        "footprint_max_radius_m": float(np.linalg.norm(polygon, axis=1).max()),
        "selected_research_primary": "A", "hardware_primary": "undecided",
        "unknown_rotation_and_IMU_origin_not_filled": True, "hypotheses": {},
    }
    for name, lidar in cfg["mid360"].items():
        p = np.array(lidar["cad_model_origin_xyz_m"])
        lever = p[:2]-axis
        reported = np.array(lidar["reported_gimbal_xyz_m"])
        ground = p+np.array([0.,0.,cfg["wheel"]["radius_m"]])
        # Pure yaw model at the wheel plane. This does not infer joint Z or RPY.
        checks = []
        for yaw in [0., math.pi/2, math.pi, -math.pi/2]:
            c,s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c,-s],[s,c]])
            at_yaw = axis+R@lever
            checks.append({"yaw_rad":yaw,"cad_origin_xy_m":at_yaw.tolist(),
                "ignoring_lever_home_to_current_error_m":float(np.linalg.norm(at_yaw-p[:2]))})
        assert abs(ground[2]-lidar["reported_ground_height_m"]) < 1e-12
        assert np.max(np.abs(lever-reported[:2])) <= 1.00001e-5
        result["hypotheses"][name] = {
            "ground_base_model_origin_xyz_m":ground.tolist(),
            "derived_yaw_axis_lever_xy_m":lever.tolist(),
            "reported_minus_derived_lever_xy_m":(reported[:2]-lever).tolist(),
            "lever_radius_m":float(np.linalg.norm(lever)),
            "yaw_model_checks":checks,
            "30ms_yaw_mismatch_at_3rad_s_lever_error_m":float(2*np.linalg.norm(lever)*math.sin(.03*3/2)),
            "interpretation":"CAD model origin only, not yet Livox or internal IMU origin",
        }
    # Coordinate-basis counterexample: a correct sensor-initial registration
    # combined with an odom->base pose in a different basis produces a wrong base.
    # This is a mathematical interface test, not a measured hardware transform.
    c,s = math.cos(.4), math.sin(.4)
    home = np.eye(4);home[:2,:2] = [[c,-s],[s,c]]
    home[:3,3] = [.13776,-.09151,.20761]
    raw_pose = np.eye(4);raw_pose[:3,3] = [.3,.1,0.]
    odom_sensor = home@raw_pose
    map_raw = np.eye(4)
    correct_map_odom = map_raw@np.linalg.inv(home)
    correct_sensor = correct_map_odom@odom_sensor
    mistaken_sensor = map_raw@odom_sensor
    result["cloud_basis_counterexample"] = {
        "home_is_synthetic_not_calibration":True,
        "home_B0_S0":home.tolist(), "raw_T_R_S":raw_pose.tolist(),
        "correct_map_odom":correct_map_odom.tolist(),
        "same_frame_name_but_wrong_basis_sensor_translation_error_m":float(
            np.linalg.norm(mistaken_sensor[:3,3]-correct_sensor[:3,3])),
        "same_frame_name_but_wrong_basis_yaw_error_rad":.4,
        "roundtrip_error":float(np.abs(correct_sensor-raw_pose).max()),
    }
    assert result["cloud_basis_counterexample"]["roundtrip_error"] < 1e-12
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reuse-build", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output.resolve();out.mkdir(parents=True, exist_ok=True)
    if command(["git","diff",SOURCE_PIN,"--","src"], cwd=root):
        raise SystemExit("Source no longer matches fixed new-car pin")
    params = yaml.safe_load((root/"src/rm_gicp_relocalization/config/gicp_3d.yaml").read_text())["gicp_relocalization"]["ros__parameters"]
    quality_root = root/"build/l1_reference_quality"
    if not args.reuse_build:
        for rel in ["include/rm_gicp_relocalization/registration_quality.hpp", "src/registration_quality.cpp"]:
            p = quality_root/rel;p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(command(["git","show",f"{QUALITY_PIN}:src/rm_gicp_relocalization/{rel}"],cwd=root)+"\n")
        subprocess.run(["cmake","-S",str(Path(__file__).parent),"-B",str(root/"build/l1_probe"),
            f"-DPROJECT_ROOT={root}",f"-DQUALITY_ROOT={quality_root}",
            "-DCMAKE_BUILD_TYPE=Release","-DCMAKE_CXX_FLAGS=-pipe"],check=True)
        subprocess.run(["cmake","--build",str(root/"build/l1_probe"),"-j1"],check=True)
    fixture = root/"src/rm_map_tools/maps/phase2e_test/phase2j_3d_test.pcd"
    points = read_pcd(fixture)
    inputs = root/"build/l1m_inputs";inputs.mkdir(parents=True, exist_ok=True)
    clouds = {"original":fixture}
    def cloud(name, pts):
        path = inputs/f"{name}.pcd";write_pcd(path,pts);clouds[name] = path
    cloud("duplicate", np.vstack([points, points+[12.,0.,0.]]))
    cloud("floor", np.array([[x,y,0.] for x in np.linspace(-3,3,31) for y in np.linspace(-3,3,31)]))
    cloud("sparse", points[::10][:50])
    # Fixed artificial partial observations, retaining multiple surfaces.
    cloud("partial0", points[points[:,2] <= 1.2])
    cloud("partial1", points[(points[:,2] >= .2) & (np.arange(len(points))%4 != 0)])
    cloud("partial2", points[(points[:,2] >= .4) & (np.arange(len(points))%4 != 1)])
    # Unique feature deliberately exists at only the correct place. Its lack
    # of correspondences at the wrong copy can be invisible to inlier fitness.
    landmark = np.array([[x,y,z] for x in np.linspace(-.25,.25,6)
        for y in np.linspace(-.25,.25,6) for z in np.linspace(2.8,3.2,5)])
    cloud("unique_query", np.vstack([points,landmark]))
    cloud("duplicate_unique", np.vstack([points,points+[12.,0.,0.],landmark]))
    cases = []
    def add(case_id, category, target, source, seed, age=0., has_seed=True):
        cases.append({"id":case_id,"category":category,"target":target,
            "source":source,"seed_xy_yaw":list(seed),"source_age_s":age,"has_seed":has_seed})
    add("correct_identity","correct_basin","original","original",(0,0,0))
    add("wrong_far12","wrong_seed","original","original",(12,0,0))
    add("wrong_yaw_pi","wrong_seed","original","original",(0,0,-math.pi))
    add("repeat_correct","repeated_structure","duplicate","original",(0,0,0))
    add("repeat_wrong12","repeated_structure","duplicate","original",(12,0,0))
    for i in range(3):
        for x,label in [(0,"seed0"),(12,"seed12")]:
            add(f"partial{i}_{label}","repeated_partial_sequence","duplicate",f"partial{i}",(x,0,0))
    add("unique_correct","unique_feature_partial_support","duplicate_unique","unique_query",(0,0,0))
    add("unique_wrong12","unique_feature_partial_support","duplicate_unique","unique_query",(12,0,0))
    add("floor_identity","degenerate_observation","floor","floor",(0,0,0))
    add("sparse50","degenerate_observation","original","sparse",(0,0,0))
    add("stale_correct","stale_pose","original","original",(0,0,0),age=.6)
    add("no_seed","wrong_seed","original","original",(0,0,0),has_seed=False)
    # Write fixed inputs and separate labels BEFORE running the registration.
    oracle = {c["id"]:{"T_map_odom":np.eye(4).tolist(),"tolerance":{
        "xy_m":.1,"yaw_rad":.1,"z_m":.1},"source":"synthetic known construction"} for c in cases}
    save(out/"labels.json",oracle)
    manifest = {"source_pin":SOURCE_PIN,"quality_pin":QUALITY_PIN,
        "quality_role":"passive counterfactual; NOT new-car acceptance",
        "parameters":params,"cases":cases,
        "clouds":{name:{"path":str(p.relative_to(root)),"sha256":hashlib.sha256(p.read_bytes()).hexdigest(),
                       "raw_points":len(read_pcd(p))} for name,p in clouds.items()},
        "families":4,"independent_real_new_car_episodes":0,
        "labels_are_not_probe_inputs":True,
        "fixed_seeds_not_candidate_retrieval":True,
        "temporal_method":"compare fixed hypotheses across 3 different artificial partial clouds; ideal canonical odometry used only after solve",
        "limitations":["not MID360 beam/occlusion/deskew simulation", "no hardware/ROS/TF publication", "no threshold tuning", "no true optimizer Hessian or per-iteration correspondence instrumentation"]}
    save(out/"manifest.json",manifest)
    rows = []
    diagnostics = out/"diagnostics";diagnostics.mkdir(exist_ok=True)
    for case in cases:
        argv = [str(root/"build/l1_probe/registration_probe"),str(clouds[case["target"]]),str(clouds[case["source"]])]
        argv += [str(v) for v in case["seed_xy_yaw"]]
        argv += [str(case["source_age_s"]),str(int(case["has_seed"]))]
        argv += [str(params[k]) for k in PARAM_KEYS]
        dump_path = diagnostics/f"{case['id']}.json";argv += [str(dump_path)]
        run = subprocess.run(argv,capture_output=True,text=True,check=True,timeout=30)
        values = run.stdout.strip().splitlines()[-1].split(",")
        assert len(values) == len(COLUMNS)
        row = {"case_id":case["id"],"category":case["category"]}
        for k,v in zip(COLUMNS,values):
            row[k] = v if k=="reason" else bool(int(v)) if k in BOOLS else float(v)
        row["accepted_chain"] = row["algorithm_accept"] and row["pose_gate_accept"]
        has_solution = row["solve_ms"] > 0 and row["converged"]
        row["xy_error_m"] = math.hypot(row["T03"],row["T13"]) if has_solution else None
        row["yaw_error_rad"] = abs(math.atan2(row["T10"],row["T00"])) if has_solution else None
        row["correct_basin"] = (row["xy_error_m"]<=.1 and row["yaw_error_rad"]<=.1 and abs(row["T23"])<=.1) if has_solution else None
        row["false_accept"] = row["accepted_chain"] and not row["correct_basin"]
        diag = json.loads(dump_path.read_text())
        for name in ["fitness_inliers","postfit_geometric_inliers_strict","T_normal_valid_inliers","unique_target_points"]:
            row[name] = diag[name]
        if diag["T_normal_valid_inliers"]:
            eig = np.linalg.eigvalsh(diag["T_information_average_JtJ"])
            assert np.allclose(eig,[row[f"T_passive_eigen{i}"] for i in range(6)],atol=1e-10,rtol=1e-9)
        if diag["fitness_inliers"]:
            assert abs(np.mean(diag["fitness_inlier_squared_residuals"])-row["fitness"]) < 1e-8
        rows.append(row)
        print(case["id"],row["reason"],"false_accept",row["false_accept"],"N",row["T_normal_valid_inliers"],flush=True)
    with (out/"results.csv").open("w",newline="") as f:
        writer = csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator="\n")
        writer.writeheader();writer.writerows(rows)
    indexed = {r["case_id"]:r for r in rows}
    # Guard the diagnostic extension against changing historical registration.
    old = {r["case_id"]:r for r in csv.DictReader((root/"experiments/neu_localization_l1/evidence/results.csv").open())}
    for new_id,old_id in [("correct_identity","identity"),("repeat_wrong12","duplicate_wrong_12m")]:
        for key in ["fitness","T03","T13","T23","T_passive_min_eigenvalue","T_passive_condition"]:
            assert abs(indexed[new_id][key]-float(old[old_id][key])) < 1e-10
    temporal = {}
    for label in ["seed0","seed12"]:
        rr = [indexed[f"partial{i}_{label}"] for i in range(3)]
        translations = np.array([[r["T03"],r["T13"],r["T23"]] for r in rr])
        yaw = np.array([math.atan2(r["T10"],r["T00"]) for r in rr])
        odom = [np.eye(4) for _ in rr]
        for i,T in enumerate(odom):
            T[:2,3] = [.2*i,.05*i]
        corrections = [np.array([[r[f"T{i}{j}"] for j in range(4)] for i in range(4)]) for r in rr]
        global_base = [G@O for G,O in zip(corrections,odom)]
        motion_errors = []
        for i in range(2):
            observed = np.linalg.inv(global_base[i])@global_base[i+1]
            expected = np.linalg.inv(odom[i])@odom[i+1]
            error = np.linalg.inv(expected)@observed
            motion_errors.append({"translation_m":float(np.linalg.norm(error[:3,3])),
                "yaw_rad":float(abs(math.atan2(error[1,0],error[0,0])))})
        temporal[label] = {"cases":[r["case_id"] for r in rr],
            "all_accepted":all(r["accepted_chain"] for r in rr),
            "all_T_counterfactual_pass":all(r["T_counterfactual_gate"] for r in rr),
            "all_wrong":all(not r["correct_basin"] for r in rr),
            "max_adjacent_map_odom_translation_change_m":float(np.linalg.norm(np.diff(translations,axis=0),axis=1).max()),
            "max_adjacent_yaw_change_rad":float(np.abs(np.diff(yaw)).max()),
            "mean_global_xy_error_m":float(np.mean([r["xy_error_m"] for r in rr])),
            "constant_translation_alias_innovation_cancels_exactly":True,
            "ideal_odom_base_xy_for_three_observations_m":[[0,0],[.2,.05],[.4,.1]],
            "relative_motion_innovation_errors":motion_errors,
            "scope":"post-process ideal-odometry consistency diagnostic; NOT a runtime temporal gate"}
    save(out/"temporal.json",temporal)
    save(out/"geometry_audit.json",geometry_audit(root))
    summary = {"decision":"Modify", "cases":len(rows), "families":4,
        "false_accept_cases":[r["case_id"] for r in rows if r["false_accept"]],
        "rejected_cases":[r["case_id"] for r in rows if not r["accepted_chain"]],
        "historical_numeric_parity_passed":True,"full_H_eigen_consistency_passed":True,
        "fitness_denominator_consistency_passed":True,"real_new_car_episodes_verified":0}
    save(out/"summary.json",summary)
    plot_aliasing(out,points,indexed)
    print(json.dumps(summary,ensure_ascii=False))


if __name__ == "__main__":
    main()
