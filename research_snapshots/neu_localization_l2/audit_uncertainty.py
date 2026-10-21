#!/usr/bin/env python3
"""Reinterpret saved local fits, and audit floor H without new thresholds."""
from pathlib import Path
import argparse
import json
import hashlib
import numpy as np
import yaml
from prototype import uncertainty_view,cluster_hypotheses,Prototype,read_pcd,save


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--baseline-output",type=Path,default=Path("build/l2_full"))
    ap.add_argument("--output",type=Path,required=True);args=ap.parse_args()
    root=Path(__file__).resolve().parents[2];cfg=yaml.safe_load(Path(__file__).with_name("config.yaml").read_text())
    result={"stage":"Research","active_accept":False,"baseline_checkpoint":"2a914151",
        "new_thresholds":False,"source_recipe_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "hypotheses":["descriptor rejection must not hide multiple locally plausible registration basins",
                      "target-normal information can be full rank even with a near-planar source query"],"views":[]}
    files=sorted(args.baseline_output.glob("*.json"))
    for path in files:
        r=json.loads(path.read_text())
        if not all(k in r for k in ["hypotheses","searches","basins","status"]): continue
        groups=cluster_hypotheses([h for h in r["hypotheses"] if h["local_eligible"]],cfg)
        assert groups==r["basins"],"qualified groups changed from recorded baseline"
        view=uncertainty_view(r,cfg)
        result["views"].append({"record":path.name,"query_id":r["query_id"],"mode":r["mode"],
            "coarse_method":r["coarse_method"],"original_status":r["status"],"status":view["status"],
            "policy_status":view["policy_status"],"qualified_basin_count":r["basin_count"],
            "registration_basin_count":view["registration_basin_count"],
            "descriptor_rejected_good_local_fits":len(view["descriptor_rejected_local_fit_ids"]),"active_accept":r["active_accept"]})
    # Two direct queries check the CLI orchestration consumes the same grouping.
    checks={}
    for name in ["q_repeat_A","q_repeat_partial"]:
        r=Prototype(root/"build/l2_inputs/database/database.json",cfg,root/"build/l2_uncertainty"/name).query(root/"build/l2_inputs"/f"{name}.json")
        old=json.loads((args.baseline_output/f"sc_only_global_{name}.json").read_text());view=uncertainty_view(old,cfg)
        assert r["status"]==view["status"] and r["policy_status"]==old.get("policy_status",old["status"])
        assert len(r["hypotheses"])==len(old["hypotheses"]) and r["basins"]==old["basins"]
        delta=max(float(np.abs(np.asarray(a["fine_pose_map_query"])-np.asarray(b["fine_pose_map_query"])).max()) for a,b in zip(r["hypotheses"],old["hypotheses"]))
        assert delta<1e-6 and not r["active_accept"]
        checks[name]={"status":r["status"],"policy_status":r["policy_status"],"pose_delta":delta,
            "qualified_basins":r["basin_count"],"registration_basins":r["registration_basin_count"]}
        if name=="q_repeat_partial": save(root/"build/l2_uncertainty/partial_full_result.json",r)
    result["direct_query_checks"]=checks
    floor_work=root/"build/l2_uncertainty/q_floor"
    floor=Prototype(root/"build/l2_inputs/database/database.json",cfg,floor_work).query(root/"build/l2_inputs/q_floor.json")
    assert not any(h["fine_cache_hit"] for h in floor["hypotheses"])
    source=read_pcd(floor_work/"q_floor.pcd")
    source_eigen=np.linalg.eigvalsh(np.cov(source.T))
    result["floor_source"]={"normalized_unvoxelized_points":len(source),"point_covariance_eigenvalues_m2":source_eigen.tolist(),
        "min_max_point_covariance_ratio":float(source_eigen[0]/source_eigen[-1]),
        "not_a_pose_observability_gate":True,"matched_target_information":[]}
    for i,h in enumerate(floor["hypotheses"]):
        d=json.loads((floor_work/f"fine_{i}.json").read_text())
        H=np.asarray(d["T_information_average_JtJ"]);m=h["quality_metrics"]
        if h["place_id"]=="floor" or h["local_eligible"]:
            result["floor_source"]["matched_target_information"].append({"candidate_id":h["candidate_id"],"place_id":h["place_id"],
                "registration_source_points":d["filtered_source_points"],"overlap":m["T_passive_overlap"],
                "H_eigenvalues":np.linalg.eigvalsh(H).tolist(),"mean_target_normal_outer_product":H[:3,:3].tolist(),
                "target_normal_outer_product_eigenvalues":np.linalg.eigvalsh(H[:3,:3]).tolist(),
                "min_eigen":m["T_passive_min_eigenvalue"],"local_eligible":h["local_eligible"],
                "information_scope":"PCL target normals K=12 and fixed post-fit correspondences; not GICP optimizer Hessian"})
    changed=[r for r in result["views"] if r["original_status"]!=r["status"]]
    result["summary"]={"saved_records_reinterpreted":len(result["views"]),"status_changes":len(changed),
        "LOST_with_multiple_local_fits":sum(r["original_status"]=="LOST" and r["registration_basin_count"]>1 for r in result["views"]),
        "qualified_group_parity":True,"all_hypotheses_and_quality_unchanged":True,"active_accept_changes":0}
    save(args.output,result);print(json.dumps(result["summary"],ensure_ascii=False))


if __name__=="__main__": main()
