#!/usr/bin/env python3
"""Original, ROS-free Scan Context style retrieval and multi-basin prototype.

No publishers, active pose, best_pose or motion authority. Input poses normalize
frames and describe database geometry; query truth labels are never read here.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import subprocess
import sys
import time
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"neu_localization_l1"))
from run_baseline import read_pcd, write_pcd, PARAM_KEYS, COLUMNS, BOOLS, finite


def save(path, obj):
    Path(path).write_text(json.dumps(finite(obj),ensure_ascii=False,indent=2)+"\n")


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def pose(x=0.,y=0.,yaw=0.,z=0.):
    c,s = math.cos(yaw),math.sin(yaw)
    T=np.eye(4);T[:2,:2]=[[c,-s],[s,c]];T[:3,3]=[x,y,z]
    return T


def yaw_of(T):
    return math.atan2(T[1,0],T[0,0])


def rigid(T,tolerance=1e-6):
    T=np.asarray(T,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
        raise ValueError("invalid homogeneous transform")
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=tolerance,rtol=0) or abs(np.linalg.det(T[:3,:3])-1)>tolerance:
        raise ValueError("transform is not rigid")
    return T


def transform(points,T):
    return points@T[:3,:3].T+T[:3,3]


def normalize(points, meta, config):
    """Input frame -> canonical odom -> gravity-aligned base at query epoch."""
    status=meta.get("normalization_status")
    if status=="synthetic_exact":
        if not config["input"]["allow_synthetic"]: raise ValueError("synthetic input disabled")
    elif status=="verified_recording":
        if not meta.get("calibration_evidence") or not all(meta.get(k) is True for k in
                ["point_time_verified","deskew_verified","home_verified","clock_alignment_verified","gravity_alignment_verified"]):
            raise ValueError("hardware input evidence incomplete")
    else:
        raise ValueError("unknown numerical frame normalization")
    stamp=int(meta["timestamp_ns"])
    if stamp<=0 or not meta.get("source_frame") or not meta.get("basis_revision"):
        raise ValueError("missing source identity/time/basis")
    if abs(int(meta["transform_timestamp_ns"])-stamp)>30_000_000:
        raise ValueError("transform and query time mismatch")
    if not np.isfinite(points).all() or points.ndim!=2 or points.shape[1]!=3:
        raise ValueError("cloud must have finite XYZ")
    T_O_I=rigid(meta["T_odom_input"]);T_O_B=rigid(meta["T_odom_base"])
    T_O_Q=pose(*T_O_B[:2,3],yaw=yaw_of(T_O_B),z=T_O_B[2,3])
    T_Q_I=np.linalg.inv(T_O_Q)@T_O_I
    return transform(points,T_Q_I),T_Q_I


def descriptor(points, config):
    cfg=config["descriptor"];nr,ns=cfg["rings"],cfg["sectors"]
    radius=np.linalg.norm(points[:,:2],axis=1)
    keep=(radius>1e-6)&(radius<cfg["radius_m"])
    p=points[keep];r=radius[keep]
    D=np.zeros((nr,ns))
    if len(p):
        rings=np.minimum((r/cfg["radius_m"]*nr).astype(int),nr-1)
        sectors=np.floor(np.mod(np.arctan2(p[:,1],p[:,0]),2*math.pi)/(2*math.pi)*ns).astype(int)%ns
        heights=np.maximum(p[:,2]+cfg["height_offset_m"],0.)
        np.maximum.at(D,(rings,sectors),heights)
    return D


def compare(target,query,config, allowed_yaw=None):
    """Circular column cosine distance; +shift rotates query into keyframe."""
    ns=target.shape[1];scores=[]
    a=np.linalg.norm(target,axis=0)
    for shift in range(ns):
        yaw=wrap(shift*2*math.pi/ns)
        if allowed_yaw is not None and abs(wrap(yaw-allowed_yaw[0]))>allowed_yaw[1]:
            continue
        rotated=np.roll(query,shift,axis=1);b=np.linalg.norm(rotated,axis=0)
        keep=(a>0)&(b>0)
        score=1-float(np.mean(np.sum(target[:,keep]*rotated[:,keep],axis=0)/(a[keep]*b[keep]))) if keep.any() else 1.
        scores.append((max(0.,score),yaw,shift,int(keep.sum())))
    scores.sort(key=lambda item:(item[0],item[2]))
    peaks=[]
    for item in scores:
        if peaks and item[0]>peaks[0][0]+config["descriptor"]["yaw_tie_margin"]: break
        if any(abs(wrap(item[1]-p[1]))<config["descriptor"]["yaw_peak_separation_rad"] for p in peaks): continue
        peaks.append(item)
        if len(peaks)>=config["descriptor"]["max_yaw_hypotheses"]: break
    return peaks


def build_database(manifest_path, output, config):
    manifest_path=Path(manifest_path).resolve();m=json.loads(manifest_path.read_text())
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    entries=[];identities=set()
    for entry in m["keyframes"]:
        e=copy.deepcopy(entry)
        if e["source_frame_id"] in identities: raise ValueError("duplicate database source frame")
        identities.add(e["source_frame_id"])
        if e["map_revision"]!=m["map_revision"] or e["basis_revision"]!=m["basis_revision"]:
            raise ValueError("database entry map/basis revision mismatch")
        points,T=normalize(read_pcd(manifest_path.parent/e["cloud_path"]),e,config)
        rigid(e["T_map_keyframe"])
        p=output/f"{e['keyframe_id']}.pcd";write_pcd(p,points)
        D=descriptor(points,config)
        np.savez_compressed(output/f"{e['keyframe_id']}.npz",descriptor=D,ring_key=D.mean(axis=1))
        e.update(normalized_cloud_path=p.name,descriptor_path=f"{e['keyframe_id']}.npz",
            input_sha256=hashlib.sha256((manifest_path.parent/e["cloud_path"]).read_bytes()).hexdigest(),
            normalized_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            descriptor_sha256=hashlib.sha256((output/f"{e['keyframe_id']}.npz").read_bytes()).hexdigest(),T_query_input=T.tolist())
        entries.append(e)
    db={"schema_version":1,"database_revision":m["database_revision"],"map_revision":m["map_revision"],
        "map_bundle":m["map_bundle"],"basis_revision":m["basis_revision"],
        "descriptor_config":config["descriptor"],"data_kind":m["data_kind"],"keyframes":entries}
    save(output/"database.json",db)
    return output/"database.json"


def cluster_hypotheses(hypotheses,config):
    """Descriptive grouping only, using the same fixed XY/yaw policy."""
    basins=[]
    for h in sorted(hypotheses,key=lambda h:h["gicp_score"]):
        T=np.array(h["fine_pose_map_query"]);group=None
        for basin in basins:
            U=np.array(basin["representative_pose"])
            if np.linalg.norm(T[:2,3]-U[:2,3])<=config["quality"]["basin_xy_separation_m"] and abs(wrap(yaw_of(T)-yaw_of(U)))<=config["quality"]["basin_yaw_separation_rad"]:
                group=basin;break
        if group is None:
            group={"basin_id":len(basins),"representative_pose":T.tolist(),"candidate_ids":[]};basins.append(group)
        group["candidate_ids"].append(h["candidate_id"])
    return basins


def uncertainty_view(output,config):
    """Keep good local fits visible even when descriptor policy rejects them.

    Does not change hypotheses, gates or active_accept. policy_status preserves
    the original qualified-basin decision; overall status preserves ambiguity.
    """
    valid=[]
    for h in output["hypotheses"]:
        m=h["quality_metrics"]
        quality=m["algorithm_accept"] and m["pose_gate_accept"]
        if config["quality"]["use_historical_T_as_research_qualifier"]: quality=quality and m["T_counterfactual_gate"]
        if quality: valid.append(h)
    basins=cluster_hypotheses(valid,config)
    policy_status=output.get("policy_status",output["status"])
    truncated=any(s["tie_group_truncated"] for s in output["searches"])
    status="ambiguous" if len(basins)>1 else "incomplete_search" if valid and truncated else policy_status
    by_id={h["candidate_id"]:h for h in valid};gaps=[]
    for i,a in enumerate(basins):
        for b in basins[i+1:]:
            ah,bh=by_id[a["candidate_ids"][0]],by_id[b["candidate_ids"][0]]
            A=np.asarray(a["representative_pose"]);B=np.asarray(b["representative_pose"])
            gaps.append({"basin_ids":[a["basin_id"],b["basin_id"]],
                "representative_candidate_ids":[ah["candidate_id"],bh["candidate_id"]],
                "fitness_gap":bh["gicp_score"]-ah["gicp_score"],"descriptor_gap":bh["coarse_score"]-ah["coarse_score"],
                "xy_separation_m":float(np.linalg.norm(A[:2,3]-B[:2,3])),"yaw_separation_rad":abs(wrap(yaw_of(A)-yaw_of(B)))})
    return {"status":status,"policy_status":policy_status,"registration_basins":basins,
        "registration_basin_count":len(basins),"registration_ambiguous":len(basins)>1,
        "registration_basin_pair_gaps":gaps,
        "descriptor_rejected_local_fit_ids":[h["candidate_id"] for h in valid if not h["local_eligible"]],
        "ambiguity_scope":"searched candidates only; excludes unsearched/prior-excluded places"}


class Prototype:
    def __init__(self,database,config,workdir):
        self.path=Path(database).resolve();self.db=json.loads(self.path.read_text())
        self.config=copy.deepcopy(config);self.work=Path(workdir).resolve();self.work.mkdir(parents=True,exist_ok=True)
        self.root=Path(__file__).resolve().parents[2]
        if config["coarse"]["method"] not in config["coarse"]["alternatives"]:
            raise ValueError("unknown coarse method")
        if self.db["descriptor_config"]!=config["descriptor"]: raise ValueError("descriptor config mismatch")
        for e in self.db["keyframes"]:
            for path_key,hash_key in [("normalized_cloud_path","normalized_sha256"),("descriptor_path","descriptor_sha256")]:
                actual=hashlib.sha256((self.path.parent/e[path_key]).read_bytes()).hexdigest()
                if actual!=e[hash_key]: raise ValueError(f"database artifact hash mismatch: {e['keyframe_id']}")
        self.descriptors={e["keyframe_id"]:np.load(self.path.parent/e["descriptor_path"])["descriptor"] for e in self.db["keyframes"]}
        cfg=yaml.safe_load((self.root/"src/rm_gicp_relocalization/config/gicp_3d.yaml").read_text())
        self.gicp=cfg["gicp_relocalization"]["ros__parameters"]
        self.fine_cache={};self.coarse_cache={}

    def retrieve(self,D,prior):
        cfg=self.config["retrieval"];ranked=[]
        for e in self.db["keyframes"]:
            T=rigid(e["T_map_keyframe"])
            if prior:
                expected=prior["expected_start_pose"]
                if math.hypot(T[0,3]-expected["x"],T[1,3]-expected["y"])>prior["xy_search_radius"]: continue
                if prior.get("candidate_region_ids") and e["place_id"] not in prior["candidate_region_ids"]: continue
            allowed=None
            if prior and prior["expected_start_pose"].get("yaw") is not None and prior.get("yaw_search_range") is not None:
                allowed=(wrap(prior["expected_start_pose"]["yaw"]-yaw_of(T)),prior["yaw_search_range"])
            peaks=compare(self.descriptors[e["keyframe_id"]],D,self.config,allowed)
            if not peaks: continue
            ranked.append({"entry":e,"score":peaks[0][0],"yaw_peaks":peaks,
                "ring_key_distance":float(np.linalg.norm(self.descriptors[e["keyframe_id"]].mean(axis=1)-D.mean(axis=1)))})
        ranked.sort(key=lambda r:(r["score"],r["entry"]["keyframe_id"]))
        for i,item in enumerate(ranked): item["rank"]=i+1
        requested=cfg["top_k"];count=min(requested,len(ranked))
        if count:
            cutoff=ranked[count-1]["score"]+cfg["descriptor_tie_margin"]
            while count<len(ranked) and ranked[count]["score"]<=cutoff: count+=1
        cap=min(count,cfg["max_keyframes"])
        return ranked,ranked[:cap],count>cap

    def coarse(self,entry,source):
        key=(entry["keyframe_id"],hashlib.sha256(source.read_bytes()).hexdigest())
        if key in self.coarse_cache: return self.coarse_cache[key],True
        cfg=self.config["coarse"]
        argv=[str(self.root/"build/l2_coarse/coarse_probe"),str(self.path.parent/entry["normalized_cloud_path"]),str(source)]
        argv += [str(cfg[k]) for k in ["leaf_m","normal_radius_m","feature_radius_m","max_iterations",
            "max_correspondence_distance_m","min_inlier_fraction","rng_seed"]]
        start=time.perf_counter()
        try:
            run=subprocess.run(argv,check=True,capture_output=True,text=True,timeout=30)
            values=[float(v) for v in run.stdout.strip().splitlines()[-1].split(',')]
            result={"converged":bool(values[0]),"inliers":int(values[1]),"source_points":int(values[2]),
                "target_points":int(values[3]),"compute_ms":values[4],"T_keyframe_query":np.array(values[5:]).reshape(4,4).tolist()}
        except (subprocess.TimeoutExpired,subprocess.CalledProcessError) as error:
            result={"converged":False,"reason":type(error).__name__,"T_keyframe_query":None}
        result["wall_ms"]=(time.perf_counter()-start)*1000
        self.coarse_cache[key]=result
        return result,False

    def fine(self,entry,source,seed,age):
        key=(entry["keyframe_id"],hashlib.sha256(source.read_bytes()).hexdigest(),seed.tobytes(),age)
        if key in self.fine_cache: return self.fine_cache[key],True
        tag=f"fine_{len(self.fine_cache)}";seed_path=self.work/f"{tag}.txt"
        np.savetxt(seed_path,seed,fmt="%.17g")
        diag=self.work/f"{tag}.json"
        argv=[str(self.root/"build/l1_probe/registration_probe"),str(self.path.parent/entry["normalized_cloud_path"]),str(source),
            str(seed[0,3]),str(seed[1,3]),str(yaw_of(seed)),str(age),"1"]
        argv += [str(self.gicp[k]) for k in PARAM_KEYS]+[str(diag),str(seed_path)]
        start=time.perf_counter()
        run=subprocess.run(argv,check=True,capture_output=True,text=True,timeout=30)
        values=run.stdout.strip().splitlines()[-1].split(',');row={}
        if len(values)!=len(COLUMNS): raise RuntimeError("GICP schema mismatch")
        for k,v in zip(COLUMNS,values): row[k]=v if k=="reason" else bool(int(v)) if k in BOOLS else float(v)
        T=np.array([[row[f"T{i}{j}"] for j in range(4)] for i in range(4)])
        result={"T_keyframe_query":T.tolist(),"metrics":row,"diagnostics":json.loads(diag.read_text()),
            "wall_ms":(time.perf_counter()-start)*1000}
        self.fine_cache[key]=result
        return result,False

    def query(self,query_path,mode="global",manual_prior=None):
        start=time.perf_counter();query_path=Path(query_path).resolve();q=json.loads(query_path.read_text())
        output={"schema_version":2,"query_id":q.get("query_id"),"source_timestamp_ns":q.get("timestamp_ns"),
            "source_frame":q.get("source_frame"),"source_frame_id":q.get("source_frame_id"),"map_revision":self.db["map_revision"],
            "database_revision":self.db["database_revision"],"database_manifest_sha256":hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "basis_revision":self.db["basis_revision"],
            "mode":mode,"coarse_method":self.config["coarse"]["method"],"active_accept":False,
            "global_uniqueness":"NOT_ESTABLISHED","hypotheses":[],"searches":[],"basins":[],
            "registration_basins":[],"registration_basin_count":0,"registration_ambiguous":False}
        try:
            if q["map_revision"]!=self.db["map_revision"] or q["basis_revision"]!=self.db["basis_revision"]:
                raise ValueError("query/database map or numerical basis mismatch")
            if q["source_frame_id"] in {e["source_frame_id"] for e in self.db["keyframes"]}:
                raise ValueError("query frame was used for database construction")
            raw=read_pcd(query_path.parent/q["cloud_path"])
            if hashlib.sha256((query_path.parent/q["cloud_path"]).read_bytes()).hexdigest() in {e["input_sha256"] for e in self.db["keyframes"]}:
                raise ValueError("query cloud identical to a database input")
            points,T_Q_I=normalize(raw,q,self.config)
            T_O_B=rigid(q["T_odom_base"])
            T_O_Q=pose(*T_O_B[:2,3],yaw=yaw_of(T_O_B),z=T_O_B[2,3])
            T_Q_B=np.linalg.inv(T_O_Q)@T_O_B
            age=(int(q["receipt_timestamp_ns"])-int(q["timestamp_ns"]))/1e9
            output.update(source_sha256=hashlib.sha256((query_path.parent/q["cloud_path"]).read_bytes()).hexdigest(),
                normalization_status=q["normalization_status"],T_query_input=T_Q_I.tolist(),T_query_base=T_Q_B.tolist(),
                fine_pose_frame="gravity_aligned_base_origin_at_query_time",source_age_sec=age,
                pose_gate_scope="local query-frame diagnostic; public map/base validation NOT RUN",
                source_age_scope="recorded receipt minus source timestamp; offline solve delay excluded")
            if age>self.config["input"]["max_source_age_sec"] or age<-.1: raise ValueError("query age/future invalid")
            D=descriptor(points,self.config)
            source=self.work/f"{q['query_id']}.pcd";write_pcd(source,points)
            if mode=="global": prior=None
            elif mode=="configured": prior=copy.deepcopy(self.config["prior"])
            elif mode=="manual":
                if manual_prior is None: raise ValueError("manual mode requires explicit prior")
                prior=copy.deepcopy(manual_prior)
            else: raise ValueError("invalid mode")
            if prior:
                if prior["xy_search_radius"]<0: raise ValueError("invalid prior radius")
                if not all(np.isfinite(prior["expected_start_pose"][k]) for k in ["x","y"]): raise ValueError("invalid prior pose")
                angle=prior["expected_start_pose"].get("yaw");span=prior.get("yaw_search_range")
                if angle is not None and not np.isfinite(angle): raise ValueError("invalid prior yaw")
                if span is not None and (not np.isfinite(span) or span<0): raise ValueError("invalid prior yaw range")
            output["prior"]=prior;attempts=[prior]
            if prior and prior.get("expand_to_global_when_no_eligible",True): attempts.append(None)
            for attempt,search_prior in enumerate(attempts):
                ranked,selected,truncated=self.retrieve(D,search_prior)
                info={"scope":"prior" if search_prior else "global","ranked_places":[{
                    "place_id":r["entry"]["place_id"],"keyframe_id":r["entry"]["keyframe_id"],
                    "descriptor_score":r["score"],"rank":r["rank"],"coarse_yaw":r["yaw_peaks"][0][1]} for r in ranked],
                    "requested_k":self.config["retrieval"]["top_k"],"selected_keyframes":len(selected),
                    "tie_group_truncated":truncated}
                output["searches"].append(info)
                for item in selected:
                    e=item["entry"]
                    seeds=[(pose(yaw=peak[1]),peak,"sc") for peak in item["yaw_peaks"]]
                    if search_prior:
                        expected=search_prior["expected_start_pose"]
                        local_center=np.linalg.inv(rigid(e["T_map_keyframe"]))@np.array([expected["x"],expected["y"],0.,1.])
                        seeds += [(pose(local_center[0],local_center[1],peak[1]),peak,"prior_sc") for peak in item["yaw_peaks"]]
                    coarse_result=None;coarse_cached=False
                    if self.config["coarse"]["method"]=="fpfh_ransac":
                        coarse_result,coarse_cached=self.coarse(e,source)
                        if coarse_result["converged"]:
                            # PCL returns float32. Match the probe's SE(3)
                            # numerical tolerance, without altering the pose.
                            try:
                                seed=rigid(coarse_result["T_keyframe_query"],tolerance=1e-5)
                                seeds.append((seed,item["yaw_peaks"][0],"fpfh_ransac"))
                            except ValueError:
                                coarse_result=dict(coarse_result,converged=False,reason="invalid_coarse_SE3")
                    for seed,peak,seed_method in seeds:
                        fine,cached=self.fine(e,source,seed,age)
                        metrics=fine["metrics"]
                        local=metrics["algorithm_accept"] and metrics["pose_gate_accept"]
                        if self.config["quality"]["use_historical_T_as_research_qualifier"]:
                            local=local and metrics["T_counterfactual_gate"]
                        eligible=local and item["score"]<=self.config["retrieval"]["max_descriptor_distance"]
                        T_map_Q=rigid(e["T_map_keyframe"])@np.array(fine["T_keyframe_query"])
                        output["hypotheses"].append({"candidate_id":f"{q['query_id']}:{attempt}:{e['keyframe_id']}:{seed_method}:{peak[2]}",
                            "place_id":e["place_id"],"keyframe_id":e["keyframe_id"],"descriptor_rank":item["rank"],
                            "coarse_score":item["score"],"coarse_yaw_rad":peak[1],"yaw_peak_score":peak[0],
                            "ring_key_distance":item["ring_key_distance"],"seed_method":seed_method,
                            "coarse_pose_keyframe_query":seed.tolist(),"coarse_registration":coarse_result,
                            "fine_pose_map_query":T_map_Q.tolist(),"gicp_score":metrics["fitness"],"quality_metrics":metrics,
                            "correspondences":{k:fine["diagnostics"][k] for k in ["filtered_source_points","fitness_inliers","T_normal_valid_inliers","unique_target_points"]},
                            "registration_quality_valid":bool(local),"descriptor_qualified":bool(item["score"]<=self.config["retrieval"]["max_descriptor_distance"]),
                            "local_eligible":bool(eligible),"search_scope":info["scope"],"active_accept":False,
                            "fine_cache_hit":cached,"coarse_cache_hit":coarse_cached,
                            "fine_wall_ms":fine["wall_ms"],"source_query_id":q["query_id"],"source_timestamp_ns":q["timestamp_ns"],
                            "map_revision":self.db["map_revision"],"database_revision":self.db["database_revision"]})
                if any(h["local_eligible"] for h in output["hypotheses"]): break
            eligible=sorted([h for h in output["hypotheses"] if h["local_eligible"]],key=lambda h:h["gicp_score"])
            output["basins"]=cluster_hypotheses(eligible,self.config)
            truncated=any(s["tie_group_truncated"] for s in output["searches"])
            output["status"]="ambiguous" if len(output["basins"])>1 else "incomplete_search" if eligible and truncated else "prior_conditioned_single" if eligible and prior and len(output["searches"])==1 else "single_hypothesis" if eligible else "LOST"
            output["prior_relaxed"]=bool(prior and len(output["searches"])>1)
            output["candidate_count"]=len(output["hypotheses"])
            output["eligible_count"]=len(eligible)
            output["basin_count"]=len(output["basins"])
            if len(eligible)>1:
                a,b=eligible[:2];A=np.array(a["fine_pose_map_query"]);B=np.array(b["fine_pose_map_query"])
                output["first_second_gap"]={"candidate_ids":[a["candidate_id"],b["candidate_id"]],
                    "fitness_gap":b["gicp_score"]-a["gicp_score"],"descriptor_gap":b["coarse_score"]-a["coarse_score"],
                    "xy_separation_m":float(np.linalg.norm(A[:2,3]-B[:2,3])),"yaw_separation_rad":abs(wrap(yaw_of(A)-yaw_of(B)))}
            else: output["first_second_gap"]=None
            by_id={h["candidate_id"]:h for h in eligible};output["basin_pair_gaps"]=[]
            for i,a in enumerate(output["basins"]):
                for b in output["basins"][i+1:]:
                    ah,bh=by_id[a["candidate_ids"][0]],by_id[b["candidate_ids"][0]]
                    A=np.array(a["representative_pose"]);B=np.array(b["representative_pose"])
                    output["basin_pair_gaps"].append({"basin_ids":[a["basin_id"],b["basin_id"]],
                        "representative_candidate_ids":[ah["candidate_id"],bh["candidate_id"]],
                        "fitness_gap":bh["gicp_score"]-ah["gicp_score"],
                        "descriptor_gap":bh["coarse_score"]-ah["coarse_score"],
                        "xy_separation_m":float(np.linalg.norm(A[:2,3]-B[:2,3])),
                        "yaw_separation_rad":abs(wrap(yaw_of(A)-yaw_of(B)))})
            output.update(uncertainty_view(output,self.config))
        except (ValueError,KeyError) as error:
            output.update(status="REJECTED_INPUT",policy_status="REJECTED_INPUT",reason=str(error),candidate_count=0,eligible_count=0,basin_count=0,prior_relaxed=False)
        output["query_wall_ms"]=(time.perf_counter()-start)*1000
        output["query_latency_includes_cached_reuse"]=any(h.get("fine_cache_hit") or h.get("coarse_cache_hit") for h in output["hypotheses"])
        return finite(output)


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--config",type=Path,default=Path(__file__).with_name("config.yaml"))
    sub=ap.add_subparsers(dest="action",required=True)
    build=sub.add_parser("build-db");build.add_argument("--manifest",type=Path,required=True);build.add_argument("--output",type=Path,required=True)
    query=sub.add_parser("query");query.add_argument("--database",type=Path,required=True);query.add_argument("--query",type=Path,required=True)
    query.add_argument("--mode",choices=["global","manual","configured"],default="global")
    query.add_argument("--manual-prior",type=Path);query.add_argument("--work",type=Path,required=True);query.add_argument("--output",type=Path,required=True)
    args=ap.parse_args();config=yaml.safe_load(args.config.read_text())
    if args.action=="build-db": print(build_database(args.manifest,args.output,config))
    else:
        prior=json.loads(args.manual_prior.read_text()) if args.manual_prior else None
        engine=Prototype(args.database,config,args.work);result=engine.query(args.query,args.mode,prior)
        save(args.output,result);print(result["status"],result["candidate_count"])


if __name__=="__main__": main()
