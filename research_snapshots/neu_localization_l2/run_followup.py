#!/usr/bin/env python3
"""Preregistered small prior/K/perturbation challenge; still private Research.

Uses unchanged policy/gates. Eight perturbation configurations use TWO heldout
surface samples, not eight new physical scans. Does not add an acceptance rule.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import numpy as np
import yaml
from prototype import Prototype,pose,transform,save,normalize,descriptor,compare,yaw_of,wrap,read_pcd,write_pcd
from run_experiment import geometry,input_metadata,errors,mock_sensor


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--baseline-output",type=Path,default=Path("build/l2_full"))
    ap.add_argument("--reuse-perturbations",action="store_true",help="Only rescore saved perturbation outputs")
    ap.add_argument("--output",type=Path,required=True);args=ap.parse_args()
    root=Path(__file__).resolve().parents[2];config=yaml.safe_load(Path(__file__).with_name("config.yaml").read_text())
    inputs=root/"build/l2_inputs";work=root/"build/l2_followup";work.mkdir(parents=True,exist_ok=True)
    db=inputs/"database/database.json";database=json.loads(db.read_text())
    result={"stage":"Research","active_accept":False,"policy_unchanged":True,
        "database_revision":database["database_revision"],"map_revision":database["map_revision"],
        "recipe_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "hypotheses":["bounded prior can hide globally plausible basins even when locally successful",
                      "Top-K tie truncation must not yield a unique-location claim",
                      "SC coarse yaw is sensitive to lateral change even with unchanged observed geometry"],
        "prior_challenge":[],"K_boundary":[],"perturbations":[]}
    # Posthoc audit compares searches for exactly the same query; does not use
    # query truth to produce poses, nor promote a global winner.
    for query in json.loads((inputs/"queries.json").read_text()):
        name=Path(query).stem
        g=json.loads((args.baseline_output/f"sc_only_global_{name}.json").read_text())
        for mode in ["manual","configured"]:
            p=json.loads((args.baseline_output/f"sc_only_{mode}_{name}.json").read_text())
            assert all(g[k]==p[k] for k in ["query_id","source_timestamp_ns","database_revision","map_revision","basis_revision"])
            excluded=[]
            for basin in g["basins"]:
                T=np.asarray(basin["representative_pose"])
                if not any(np.linalg.norm(T[:2,3]-np.asarray(b["representative_pose"])[:2,3])<=config["quality"]["basin_xy_separation_m"] and
                    abs(wrap(yaw_of(T)-yaw_of(np.asarray(b["representative_pose"]))))<=config["quality"]["basin_yaw_separation_rad"] for b in p["basins"]):
                    excluded.append(basin["basin_id"])
            result["prior_challenge"].append({"query_id":name,"mode":mode,"prior_status":p["status"],
                "global_status":g["status"],"prior_basins":p["basin_count"],"global_basins":g["basin_count"],
                "globally_plausible_basins_absent_from_prior_result":excluded,
                "prior_single_has_global_competition":p["status"]=="prior_conditioned_single" and bool(excluded),
                "external_prior_truth_not_established_by_audit":True})
    for cap in [8,1]:
        cfg=copy.deepcopy(config);cfg["retrieval"].update(top_k=1,max_keyframes=cap)
        r=Prototype(db,cfg,work/f"K1_cap{cap}").query(inputs/"q_repeat_A.json")
        assert r["status"]==("ambiguous" if cap==8 else "incomplete_search") and not r["active_accept"]
        result["K_boundary"].append({"requested_K":1,"cap":cap,"status":r["status"],"candidate_count":r["candidate_count"],
            "selected_keyframes":r["searches"][0]["selected_keyframes"],"tie_group_truncated":r["searches"][0]["tie_group_truncated"],
            "places":[h["place_id"] for h in r["hypotheses"]],"active_accept":r["active_accept"]})
        if cap==1: save(work/"K1_truncated.json",r)
    parameters=[(.1,.15,.73),(.6,.15,.73),(1.6,.15,.73),(.1,.15,.73+math.pi/2)]
    for kind,place,origin,seed in [("corner","corner",0,7001),("repeat","repeat_A",12,7002)]:
        world=geometry(kind,seed);world+=np.random.default_rng(seed+10).normal(0,.006,world.shape)
        for i,(dx,dy,yaw) in enumerate(parameters):
            name=f"perturb_{kind}_{i}";points=transform(world,np.linalg.inv(pose(dx,dy,yaw)))
            meta=input_metadata(points,inputs,name,400_000_000_000+len(result["perturbations"])*100_000_000,.37)
            meta["query_id"]=name;save(inputs/f"{name}.json",meta)
            if args.reuse_perturbations: r=json.loads((work/f"{name}.json").read_text())
            else:
                r=Prototype(db,config,work/name).query(inputs/f"{name}.json");save(work/f"{name}.json",r)
            label={"T_map_query":pose(origin+dx,dy,yaw).tolist()}
            correct=[h for h in r["hypotheses"] if h["place_id"]==place]
            best=min(correct,key=lambda h:sum(errors(h["fine_pose_map_query"],label))) if correct else None
            rank=next((h["rank"] for h in r["searches"][0]["ranked_places"] if h["place_id"]==place),None)
            result["perturbations"].append({"query_id":name,"heldout_surface_seed":seed,"relative_pose_parameters":[dx,dy,yaw],
                "correct_place":place,"correct_rank":rank,"status":r["status"],"basin_count":r["basin_count"],
                "fine_scored_error_xy_z_yaw":errors(best["fine_pose_map_query"],label) if best else None,
                "correct_SC_distance":min(h["coarse_score"] for h in correct) if correct else None,
                "correct_SC_yaw_error_rad":min(abs(wrap(h["coarse_yaw_rad"]-yaw)) for h in correct) if correct else None,
                "correct_registration_quality_valid":any(h["registration_quality_valid"] for h in correct),
                "correct_eligible":any(h["local_eligible"] and max(errors(h["fine_pose_map_query"],label))<=.1 for h in correct),
                "query_wall_ms":r["query_wall_ms"],"active_accept":r["active_accept"]})
    # Swap mock mechanical A/B geometry while keeping the exact SAME observed
    # point set. This is a coordinate invariance check, not A/B coverage evidence.
    points=geometry("corner",8100);T_O_B=pose(1,-2,.6);invariance=[]
    cad=yaml.safe_load((root/"experiments/neu_localization_l1/new_car_cad_baseline.yaml").read_text())
    axis=np.asarray(cad["gimbal"]["yaw_axis_xy_m"])
    for sensor in ["A","B"]:
        origin=np.asarray(cad["mid360"][sensor]["cad_model_origin_xyz_m"])
        for angle in [0.,math.pi/2,math.pi]:
            T=mock_sensor(angle);T[:2,3]=axis+pose(yaw=angle)[:2,:2]@(origin[:2]-axis)
            T[2,3]=cad["mid360"][sensor]["reported_ground_height_m"]
            raw=transform(points,np.linalg.inv(T))
            meta={"normalization_status":"synthetic_exact","timestamp_ns":500_000_000_000,
                "transform_timestamp_ns":500_000_000_000,"source_frame":f"synthetic_{sensor}","basis_revision":database["basis_revision"],
                "T_odom_input":(T_O_B@T).tolist(),"T_odom_base":T_O_B.tolist()}
            normalized,_=normalize(raw,meta,config);error=float(np.abs(normalized-points).max())
            score=compare(descriptor(points,config),descriptor(normalized,config),config)[0][0]
            assert error<1e-12 and score<1e-12
            invariance.append({"sensor":sensor,"gimbal_yaw_rad":angle,"XYZ_error_m":error,"SC_distance":score})
    result["A_B_normalization"]={"checks":invariance,"hardware_calibration_proven":False,"real_sensor_visibility_proven":False}
    q=json.loads((inputs/"q_corner.json").read_text())
    points,_=normalize(read_pcd(inputs/q["cloud_path"]),q,config)
    T_B_I=np.linalg.inv(np.asarray(q["T_odom_base"]))@np.asarray(q["T_odom_input"])
    T_O_B=np.asarray(q["T_odom_base"]);pitch=.25
    Ry=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    T_O_B[:3,:3]=T_O_B[:3,:3]@Ry
    T_O_Q=pose(*T_O_B[:2,3],yaw=yaw_of(T_O_B),z=T_O_B[2,3]);T_Q_B=np.linalg.inv(T_O_Q)@T_O_B
    raw=transform(points,np.linalg.inv(T_Q_B@T_B_I));name="query_frame_tilted_math"
    write_pcd(inputs/f"{name}.pcd",raw)
    q.update(query_id=name,source_frame_id=name,cloud_path=f"{name}.pcd",T_odom_base=T_O_B.tolist(),T_odom_input=(T_O_B@T_B_I).tolist())
    save(inputs/f"{name}.json",q)
    normalized,_=normalize(read_pcd(inputs/q["cloud_path"]),q,config)
    r=Prototype(db,config,work/name).query(inputs/f"{name}.json")
    assert np.allclose(r["T_query_base"],T_Q_B,atol=1e-12) and not np.allclose(T_Q_B,np.eye(4))
    assert np.abs(normalized-points).max()<1e-7 and not r["active_accept"]
    result["query_frame_tilt"]={"base_pitch_rad":pitch,"T_query_base":r["T_query_base"],
        "normalized_points_error_m":float(np.abs(normalized-points).max()),"fine_pose_frame":r["fine_pose_frame"],
        "pose_gate_scope":r["pose_gate_scope"],"source_age_scope":r["source_age_scope"],
        "public_map_base_pose_validation":"NOT_RUN","active_accept":False}
    result["counts"]={"prior_singles_with_global_competition":sum(r["prior_single_has_global_competition"] for r in result["prior_challenge"]),
        "perturbation_configurations":8,"independent_heldout_surface_samples":2,
        "correct_eligible_perturbations":sum(r["correct_eligible"] for r in result["perturbations"])}
    save(args.output,result);print(json.dumps(result["counts"],ensure_ascii=False),flush=True)


if __name__=="__main__": main()
