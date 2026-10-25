"""Lightweight summaries and manifest; full traces remain outside the repository."""
import argparse
import csv
import gzip
import hashlib
import json
import pathlib
import statistics
from world import stats

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--data",type=pathlib.Path,required=True);ap.add_argument("--output",type=pathlib.Path,required=True);a=ap.parse_args()
    out=a.output;out.mkdir(parents=True,exist_ok=False)
    rows=json.loads((a.data/"pilot_01/results.json").read_text())
    summary={"verdict":None,"scope":"Phase4A ideal input; no holdout, no real STVL comparison, no hardware claim","trial_count":len(rows),"scenes":{},"system":{},"paired":{},"failed_trials":[],"unmeasured":["causal tracking/uncertainty calibration","local deadlock witness","unnecessary stopping labels","whole-harness peak RSS","10-class holdout"]}
    metrics=["min_clearance_m","wait_s","path_length_m","stop_count","command_sign_flips","trajectory_churn","replan_hz","min_hold_ttc_margin_s","cpu_cores_at_realtime","consumer_rss_peak_mib"]
    for scene in ("static","crossing","head_on","multiple"):
        summary["scenes"][scene]={}
        for arm in ("simple","search","native"):
            r=[x for x in rows if x["scene"]==scene and x["arm"]==arm]
            summary["scenes"][scene][arm]={"n":len(r),"safe_completion":sum(x["safe_completion"] for x in r),"completion":sum(x["completion"] for x in r),"collision":sum(x["collision"] for x in r),"arrival_s":stats([x["arrival_s"] for x in r if x["arrival_s"] is not None]),**{k:stats([x[k] for x in r if x[k] is not None]) for k in metrics}}
        summary["paired"][scene]={}
        for arm in ("simple","native"):
            deltas=[];new_failures=[]
            for seed in (13,29,47):
                h=next(x for x in rows if x["scene"]==scene and x["seed"]==seed and x["arm"]=="search")
                b=next(x for x in rows if x["scene"]==scene and x["seed"]==seed and x["arm"]==arm)
                if h["safe_completion"] and b["safe_completion"]:deltas.append((b["arrival_s"]-h["arrival_s"])/b["arrival_s"])
                if b["safe_completion"] and not h["safe_completion"]:new_failures.append(seed)
            summary["paired"][scene][arm]={"safe_time_improvement_fraction":deltas,"median":statistics.median(deltas) if deltas else None,"new_search_failures":new_failures}
    for arm in ("simple","search","native"):
        r=[x for x in rows if x["arm"]==arm];lat=[];updates=[]
        for x in r:
            trace=[json.loads(line) for line in gzip.open(a.data/"pilot_01"/(x["id"]+".jsonl.gz"),"rt")]
            lat.extend(z["response"]["roundtrip_ms"] for z in trace)
            updates.extend(trace[i+1]["t"]-trace[i]["t"] for i in range(len(trace)-1))
        summary["system"][arm]={"latency_ms":stats(lat),"update_interval_s":stats(updates),"deadline_misses":sum(x["deadline_misses"] for x in r),"no_verified_action":sum(x["no_verified_action"] for x in r),"consumer_cpu_cores_realtime":stats([x["cpu_cores_at_realtime"] for x in r]),"consumer_cpu_cores_offline_wall":stats([x["consumer_cpu_s"]/x["offline_wall_s"] for x in r]),"consumer_rss_peak_mib":max(x["consumer_rss_peak_mib"] for x in r),"acceleration_max_mps2":max(x["max_acceleration_mps2"] for x in r),"jerk_max_mps3":max(x["jerk_mps3"]["max"] for x in r)}
    gains={a:[] for a in ("simple","native")}
    for sc in ("crossing","head_on","multiple"):
        for arm in gains:
            pair=summary["paired"][sc][arm]
            h=summary["scenes"][sc]["search"];b=summary["scenes"][sc][arm]
            safety=h["safe_completion"]-b["safe_completion"]>=2
            efficiency=pair["median"] is not None and pair["median"]>=.1 and sum(v>0 for v in pair["safe_time_improvement_fraction"])>=2
            if not pair["new_search_failures"] and (safety or efficiency):gains[arm].append(sc)
    regression=any(summary["paired"][s][a]["new_search_failures"] for s in summary["paired"] for a in gains)
    collisions=sum(x["collision"] for x in rows if x["arm"]=="search")
    summary["gate"]={"dynamic_gain_classes":gains,"search_collisions":collisions,"new_safe_completion_regression":regression}
    summary["verdict"]="Eligible_for_holdout" if not regression and not collisions and all(len(v)>=2 for v in gains.values()) else "Stop"
    summary["failed_trials"]=[{k:x[k] for k in ["id","collision","completion","elapsed_s","min_clearance_m","worst","no_verified_action","reasons"]} for x in rows if not x["safe_completion"]]
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    # One row per trial keeps distributions/failed observations rather than only an average.
    for r in rows:
        for field in ("jerk_mps3","acceleration_mps2","latency_ms"):
            for measure in ("mean","p50","p95","p99","max"):r[field+"_"+measure]=r[field][measure]
    keys=["id","scene","seed","arm","safe_completion","completion","collision","arrival_s","elapsed_s",*metrics,"deadline_misses","no_verified_action","native_safe_raw_unsafe_aggregate_cycles"]+[f+"_"+m for f in ("jerk_mps3","acceleration_mps2","latency_ms") for m in ("mean","p50","p95","p99","max")]
    with (out/"trials.csv").open("w") as f:
        writer=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore");writer.writeheader();writer.writerows(rows)
    diagnosis=json.loads((a.data/"failure_audit_01/diagnosis.json").read_text())
    confirmation=json.loads((a.data/"confirmation_head_on_13/head_on_13_search.json").read_text())
    diagnosis["confirmation"]={k:confirmation[k] for k in ["collision","completion","elapsed_s","min_clearance_m","no_verified_action","deadline_misses"]}
    (out/"failure_diagnosis.json").write_text(json.dumps(diagnosis,indent=2)+"\n")
    (out/"failure_fixture.json").write_bytes((a.data/"failure_audit_01/fixture.json").read_bytes())
    identity=json.loads((a.data/"pilot_source_manifest.json").read_text())
    manifest={"dataset_id":"rm2027-state-time-pilot-20261025","root":str(a.data.resolve()),"filename_convention":"<cohort>/<scene>_<seed>_<arm>.json[ l.gz ]; non-trial checks use explicit filenames","origin":"generated by research/dynamic_state_time on this host, no real sensor data","purpose":"36 paired pilot episodes, failed baseline sanity, witnesses, one unchanged failure confirmation and fixed-input diagnostic","generated_or_real":"generated","required_experiment":"Phase4A; pilot_01 + confirmation_head_on_13 + failure_audit_01","generator_identity":identity,"files":[]}
    for p in sorted(a.data.rglob("*")):
        if not p.is_file() or "build_cache" in p.parts:continue
        raw=p.read_bytes();manifest["files"].append({"path":str(p.relative_to(a.data)),"size":len(raw),"sha256":hashlib.sha256(raw).hexdigest()})
    manifest["total_bytes"]=sum(x["size"] for x in manifest["files"])
    (out/"dataset_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(json.dumps({"verdict":summary["verdict"],"paired":summary["paired"],"raw_bytes":manifest["total_bytes"],"manifest_files":len(manifest["files"]),"system":summary["system"]},indent=2))
if __name__=="__main__":main()
