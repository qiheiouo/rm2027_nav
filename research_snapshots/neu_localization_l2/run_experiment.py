#!/usr/bin/env python3
"""Bounded analytic holdout experiment; no real-car or field-effect claims.

Independent random surface samples form database and query clouds. Labels live
outside the prototype inputs and are read only by the scoring code below.
"""
from pathlib import Path
import argparse
import copy
import csv
import hashlib
import json
import math
import time
import numpy as np
import yaml
from prototype import (Prototype, build_database, normalize, descriptor, compare,
                       pose, transform, wrap, yaw_of, save, write_pcd, read_pcd)

MAP="toy-l2-v1";BASIS="synthetic-base-gravity-v1"


def surface(origin,u,v,count,rng):
    a=rng.uniform(0,1,(count,2))
    return np.asarray(origin)+a[:,0,None]*np.asarray(u)+a[:,1,None]*np.asarray(v)


def geometry(kind,seed):
    r=np.random.default_rng(seed)
    floor=surface([-3,-3,0],[6,0,0],[0,6,0],750,r)
    if kind=="floor": return floor
    specs=[([-3,-3,0],[0,6,0],[0,0,2.7],700),
           ([-3,3,0],[6,0,0],[0,0,3.1],700)]
    if kind=="corner":
        specs += [([1,-1,0],[1.2,0,0],[0,0,1.5],300),
                  ([1,-1,0],[0,1.4,0],[0,0,1.5],300)]
    elif kind=="repeat":
        specs += [([.7,-1.2,0],[1.1,0,0],[0,0,2.1],300),
                  ([.7,-1.2,0],[0,.8,0],[0,0,2.1],300)]
    elif kind=="shelves":
        specs=[([-3,-3,0],[6,0,0],[0,0,2.7],650),
               ([3,-3,0],[0,6,0],[0,0,1.8],600),
               ([-2.2,-1,0],[0,3.5,0],[0,0,1.6],350),
               ([-.4,-1.8,0],[0,3.8,0],[0,0,1.1],350),
               ([1.6,-1.5,0],[0,3.,0],[0,0,2.4],350)]
    elif kind=="irregular":
        specs=[([-3,-2,0],[1.,5,0],[0,0,2.2],650),
               ([-2,3,0],[5,-1,0],[0,0,3.4],650),
               ([.8,-1.8,0],[1.6,.8,0],[0,0,1.6],450),
               ([-1.2,-.8,0],[0,1.2,0],[0,0,2.8],300)]
    else: raise ValueError(kind)
    return np.vstack([floor]+[surface(*s,r) for s in specs])


def mock_sensor(gimbal):
    """Known simulation only: CAD A lever + INVENTED sensor RPY, not calibration."""
    axis=np.array([-.00194,-.00002]);lever=np.array([.13970,-.09149])
    p=axis+pose(yaw=gimbal)[:2,:2]@lever
    roll,pitch=.1,-.2
    Rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    Ry=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    T=pose(p[0],p[1],gimbal+.4,z=.20761);T[:3,:3]=T[:3,:3]@Ry@Rx
    return T


def input_metadata(points,output,name,stamp,gimbal):
    T_B_I=mock_sensor(gimbal);T_O_B=pose(1.,-2.,.6)
    raw=transform(points,np.linalg.inv(T_B_I));write_pcd(output/f"{name}.pcd",raw)
    return {"cloud_path":f"{name}.pcd","source_frame_id":name,"source_frame":"synthetic_lidar_A",
        "timestamp_ns":stamp,"receipt_timestamp_ns":stamp+10_000_000,"transform_timestamp_ns":stamp,
        "map_revision":MAP,"basis_revision":BASIS,"normalization_status":"synthetic_exact",
        "T_odom_input":(T_O_B@T_B_I).tolist(),"T_odom_base":T_O_B.tolist(),
        "synthetic_gimbal_yaw_rad":gimbal,"synthetic_sensor_RPY_not_hardware":True}


def fixture(output,config):
    output.mkdir(parents=True,exist_ok=True)
    definitions=[("corner",0,0,0.),("repeat_A",12,0,0.),("repeat_B",24,0,0.),
                 ("shelves",0,12,0.),("irregular",12,12,.45),("floor",24,12,0.)]
    manifest={"schema_version":1,"database_revision":"toy-db-v1","map_revision":MAP,
        "basis_revision":BASIS,"data_kind":"analytic_synthetic_holdout",
        "map_bundle":{"pcd":"per-keyframe analytic surfaces","occupancy":None,"hardware_valid":False},"keyframes":[]}
    for i,(place,x,y,yaw) in enumerate(definitions):
        kind="repeat" if place.startswith("repeat") else place
        # Identical analytic repeated structures: independent QUERY sampling is
        # the holdout; repeated database places deliberately share geometry.
        seed=111 if kind=="repeat" else 100+i
        points=geometry(kind,seed)
        meta=input_metadata(points,output,f"db_{place}",100_000_000_000+i*100_000_000,i*.37)
        meta.update(keyframe_id=f"kf_{place}",place_id=place,T_map_keyframe=pose(x,y,yaw).tolist())
        manifest["keyframes"].append(meta)
    save(output/"manifest.json",manifest)
    # Parameters registered in source before observing outcomes. Priors below
    # represent operator/config input, never an optimizer ground-truth argument.
    cases=[("q_corner","corner",.2,-.1,.73,"full"),
           ("q_corner_large_offset","corner",2.5,-.4,-1.27,"full"),
           ("q_repeat_A","repeat_A",.1,.15,.73,"full"),
           ("q_repeat_B_wrong_birth","repeat_B",.1,.15,.73,"full"),
           ("q_repeat_partial","repeat_A",.2,-.1,2.43,"partial"),
           ("q_shelves","shelves",.3,.2,-1.27,"full"),
           ("q_irregular","irregular",.2,.15,1.63,"full"),
           ("q_floor","floor",.2,.15,.73,"full")]
    lookup={e["place_id"]:e for e in manifest["keyframes"]};labels={};queries=[]
    for i,(name,place,dx,dy,yaw,visibility) in enumerate(cases):
        kind="repeat" if place.startswith("repeat") else place
        points=geometry(kind,500+i)
        if visibility=="partial": points=points[(points[:,0]<1.8)&(points[:,1]>-.5)]
        points+=np.random.default_rng(900+i).normal(0,.006,points.shape)
        T_K_Q=pose(dx,dy,yaw);pointsQ=transform(points,np.linalg.inv(T_K_Q))
        meta=input_metadata(pointsQ,output,name,200_000_000_000+i*100_000_000,i*.57)
        meta.update(query_id=name,visibility_model="analytic_mask_not_real_lidar_occlusion")
        save(output/f"{name}.json",meta);queries.append(f"{name}.json")
        T=np.array(lookup[place]["T_map_keyframe"])@T_K_Q
        labels[name]={"correct_place_id":place,"T_map_query":T.tolist(),"visibility":visibility,
                      "truth_is_scoring_only":True}
    save(output/"queries.json",queries);save(output/"labels.json",labels)
    manual={"expected_start_pose":{"x":12.4,"y":-.2,"yaw":None},"xy_search_radius":3.,
            "yaw_search_range":None,"candidate_region_ids":[],"expand_to_global_when_no_eligible":True}
    manual_inputs={}
    centers=[(.4,-.2),(2.,-.5),(12.4,-.2),(12.4,-.2),(12.4,-.2),(.4,12.4),(12.4,12.4),(24.4,12.4)]
    for case,center in zip(cases,centers):
        p=copy.deepcopy(manual);p["expected_start_pose"].update(x=center[0],y=center[1])
        manual_inputs[case[0]]=p
    save(output/"manual_prior.json",manual);save(output/"manual_inputs.json",manual_inputs)
    build_database(output/"manifest.json",output/"database",config)
    return queries,labels,manual_inputs


def errors(T,label):
    T=np.asarray(T);U=np.asarray(label["T_map_query"])
    xy=float(np.linalg.norm(T[:2,3]-U[:2,3]));z=abs(float(T[2,3]-U[2,3]))
    yaw=abs(wrap(yaw_of(T)-yaw_of(U)))
    return xy,z,yaw


def score(result,label):
    eligible=[h for h in result["hypotheses"] if h["local_eligible"]]
    correct=[h for h in eligible if h["place_id"]==label["correct_place_id"]]
    registered=[h for h in result["hypotheses"] if h["place_id"]==label["correct_place_id"]]
    captured=[h for h in correct if max(errors(h["fine_pose_map_query"],label))<=.1]
    aligned=[h for h in registered if max(errors(h["fine_pose_map_query"],label))<=.1]
    best=min(eligible,key=lambda h:h["gicp_score"]) if eligible else None
    raw=result["searches"][-1]["ranked_places"] if result["searches"] else []
    rank=next((h["rank"] for h in raw if h["place_id"]==label["correct_place_id"]),None)
    best_errors=errors(best["fine_pose_map_query"],label) if best else (None,None,None)
    right=min(correct,key=lambda h:sum(errors(h["fine_pose_map_query"],label))) if correct else None
    right_errors=errors(right["fine_pose_map_query"],label) if right else (None,None,None)
    raw_right=min(registered,key=lambda h:sum(errors(h["fine_pose_map_query"],label))) if registered else None
    raw_errors=errors(raw_right["fine_pose_map_query"],label) if raw_right else (None,None,None)
    T_K_M=np.linalg.inv(np.asarray(next(e for e in result["evaluation_database_poses"] if e["place_id"]==label["correct_place_id"])["T_map_keyframe"]))
    true_relative=T_K_M@np.asarray(label["T_map_query"])
    sc_yaw=[abs(wrap(h["coarse_yaw_rad"]-yaw_of(true_relative))) for h in result["hypotheses"] if h["place_id"]==label["correct_place_id"]]
    result.pop("evaluation_database_poses")
    best_correct=bool(best and best["place_id"]==label["correct_place_id"] and max(best_errors)<=.1)
    single=result["status"] in ["single_hypothesis","prior_conditioned_single"]
    return {"query_id":result["query_id"],"method":result["coarse_method"],"mode":result["mode"],
        "status":result["status"],"correct_place":label["correct_place_id"],"correct_rank":rank,
        "recall_at_1":bool(rank and rank<=1),"recall_at_3":bool(rank and rank<=3),"recall_at_5":bool(rank and rank<=5),
        "correct_basin_captured":bool(captured),"correct_sc_yaw_error_rad":min(sc_yaw) if sc_yaw else None,
        "correct_registered_basin":bool(aligned),"correct_raw_fine_xy_error_m":raw_errors[0],
        "correct_raw_fine_z_error_m":raw_errors[1],"correct_raw_fine_yaw_error_rad":raw_errors[2],
        "correct_raw_seed_method":raw_right["seed_method"] if raw_right else None,
        "correct_fine_xy_error_m":right_errors[0],"correct_fine_z_error_m":right_errors[1],"correct_fine_yaw_error_rad":right_errors[2],
        "naive_best_xy_error_m":best_errors[0],"naive_best_yaw_error_rad":best_errors[2],"naive_best_correct":best_correct,
        "shadow_false_single":bool(single and not best_correct),"false_active_accept":bool(result["active_accept"] and not best_correct),
        "candidate_count":result["candidate_count"],"eligible_count":result["eligible_count"],"basin_count":result["basin_count"],
        "selected_keyframes":sum(s["selected_keyframes"] for s in result["searches"]),
        "prior_relaxed":result["prior_relaxed"],"query_wall_ms":result["query_wall_ms"],
        "cache_used":result["query_latency_includes_cached_reuse"]}


def input_checks(inputs,config,work):
    q=json.loads((inputs/"q_corner.json").read_text());db=inputs/"database/database.json"
    mutations={"unknown_basis":{"normalization_status":"unknown"},"wrong_map":{"map_revision":"other"},
        "wrong_basis":{"basis_revision":"other"},"expired":{"receipt_timestamp_ns":q["timestamp_ns"]+1_000_000_000},
        "frame_leak":{"source_frame_id":"db_corner"},"cloud_leak":{"cloud_path":"db_corner.pcd"},
        "time_mismatch":{"transform_timestamp_ns":q["timestamp_ns"]+31_000_000}}
    outcomes={}
    for name,mutation in mutations.items():
        modified=dict(q,**mutation);path=inputs/f"negative_{name}.json";save(path,modified)
        result=Prototype(db,config,work/name).query(path)
        assert result["status"]=="REJECTED_INPUT" and not result["hypotheses"]
        outcomes[name]={"status":result["status"],"reason":result["reason"]}
    database=json.loads(db.read_text())
    for hash_key in ["normalized_sha256","descriptor_sha256"]:
        broken=copy.deepcopy(database);broken["keyframes"][0][hash_key]="0"*64
        path=db.parent/f"negative_{hash_key}.json";save(path,broken)
        try:
            Prototype(path,config,work/hash_key)
        except ValueError as error:
            outcomes[hash_key]={"status":"REJECTED_DATABASE","reason":str(error)}
        else: raise AssertionError("changed database artifact identity was not rejected")
    points=geometry("corner",800);normalized=[]
    for i,angle in enumerate([0,math.pi/2,math.pi]):
        meta=input_metadata(points,inputs,f"normalization_{i}",300_000_000_000,angle)
        normalized.append(normalize(read_pcd(inputs/meta["cloud_path"]),meta,config)[0])
    error=max(float(np.abs(p-points).max()) for p in normalized)
    yaw_scores=[compare(descriptor(normalized[0],config),descriptor(p,config),config)[0][0] for p in normalized]
    assert error<1e-7 and max(yaw_scores)<1e-8
    outcomes["gimbal_normalization"]={"max_XYZ_error_m":error,"descriptor_distances":yaw_scores,
        "meaning":"known synthetic transforms only, not hardware extrinsic validation"}
    return outcomes


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--methods",nargs="+",default=["sc_only","fpfh_ransac"])
    parser.add_argument("--modes",nargs="+",default=["global","manual","configured"])
    parser.add_argument("--reuse-results",action="store_true",help="Rescore existing outputs without rerunning registrations")
    args=parser.parse_args();root=Path(__file__).resolve().parents[2];out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load(Path(__file__).with_name("config.yaml").read_text())
    inputs=root/"build/l2_inputs";queries,labels,manual_inputs=fixture(inputs,config)
    rows=[];checks=input_checks(inputs,config,root/"build/l2_work/input_checks")
    db=inputs/"database/database.json";poses=json.loads(db.read_text())["keyframes"]
    start=time.perf_counter()
    for method in args.methods:
        for mode in args.modes:
            for query in queries:
                cfg=copy.deepcopy(config);cfg["coarse"]["method"]=method
                name=f"{method}_{mode}_{Path(query).stem}"
                # Fresh engine: quoted latency includes no cross-query caches.
                if args.reuse_results:
                    result=json.loads((out/f"{name}.json").read_text())
                else:
                    engine=Prototype(db,cfg,root/"build/l2_work"/name)
                    result=engine.query(inputs/query,mode,manual_inputs[Path(query).stem])
                result["evaluation_database_poses"]=poses
                row=score(result,labels[result["query_id"]]);rows.append(row)
                save(out/f"{name}.json",result)
                print(name,result["status"],"capture",row["correct_basin_captured"],"ms",round(row["query_wall_ms"]),flush=True)
    # Non-empty global fallback must be exercised even when the prior is empty.
    empty_prior=copy.deepcopy(manual_inputs["q_repeat_A"]);empty_prior["expected_start_pose"].update(x=100.,y=100.)
    result=Prototype(db,config,root/"build/l2_work/empty_prior").query(inputs/"q_repeat_A.json","manual",empty_prior)
    assert result["prior_relaxed"] and len(result["searches"])==2 and result["status"]=="ambiguous"
    save(out/"empty_prior_expansion.json",result)
    checks["empty_prior_expansion"]={"status":result["status"],"scope_history":[s["scope"] for s in result["searches"]]}
    assert all(not r["false_active_accept"] for r in rows)
    with (out/"results.csv").open("w") as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    summary={"data_kind":"analytic_synthetic_holdout","hardware_queries":0,"database_keyframes":len(poses),"queries":len(queries),
        "recipe_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),"config":config,
        "query_truth_not_read_by_prototype":True,"false_active_accept_zero_is_policy_not_real_accuracy":True,
        "input_checks":checks,"by_method_mode":{},"elapsed_sec":time.perf_counter()-start}
    for method in args.methods:
        for mode in args.modes:
            subset=[r for r in rows if r["method"]==method and r["mode"]==mode]
            summary["by_method_mode"][f"{method}/{mode}"]={
                "queries":len(subset),"recall_at_1":sum(r["recall_at_1"] for r in subset)/len(subset),
                "recall_at_3":sum(r["recall_at_3"] for r in subset)/len(subset),
                "recall_at_5":sum(r["recall_at_5"] for r in subset)/len(subset),
                "correct_basin_captured":sum(r["correct_basin_captured"] for r in subset),
                "correct_registered_basin":sum(r["correct_registered_basin"] for r in subset),
                "shadow_false_single":sum(r["shadow_false_single"] for r in subset),
                "status_counts":{s:sum(r["status"]==s for r in subset) for s in sorted({r["status"] for r in subset})},
                "median_query_ms":float(np.median([r["query_wall_ms"] for r in subset])),
                "max_query_ms":max(r["query_wall_ms"] for r in subset),
                "candidate_count_range":[min(r["candidate_count"] for r in subset),max(r["candidate_count"] for r in subset)]}
    save(out/"summary.json",summary);save(out/"fixture_manifest.json",json.loads((inputs/"manifest.json").read_text()))
    save(out/"scoring_labels.json",labels);save(out/"manual_inputs.json",manual_inputs)
    print(json.dumps(summary["by_method_mode"],indent=2),flush=True)


if __name__=="__main__": main()
