"""Lightweight closeout of the two fixed-fixture runs; no experiment execution."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--data",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    keys=("arm","start_epoch_s","duration_s","cycles","collision","completion","safe_completion",
          "arrival_s","elapsed_s","min_clearance_m","wait_s","stop_count","unnecessary_stop_s","deadlock",
          "command_sign_flips","trajectory_churn","sequence_target_rms_mps","latency_ms","deadline_misses",
          "no_verified_action","consumer_cpu_s","consumer_rss_peak_mib","cpu_cores_at_realtime","replan_hz",
          "max_speed_mps","max_acceleration_mps2")
    summary={"verdict":"Stop","scope":"per-beam-node emergency-to-CV-refuge certification fails unchanged runtime gate",
             "baseline_commit":"21b9e56076b23f17f1140b9ed68bff5edce9b5b6","pilot_36_run":False,"runs":[]}
    csv_rows=[]
    for run in ("fixture_01","fixture_confirmation"):
        gate=json.loads((a.data/run/"gate.json").read_text())
        fixed=json.loads((a.data/run/"fixed_input.json").read_text())
        trials=[{k:t[k] for k in keys} for t in gate["trials"]]
        raw=[json.loads(line) for line in gzip.open(a.data/run/"fixture_recourse.jsonl.gz","rt")]
        worst=max(raw,key=lambda x:x["response"]["roundtrip_ms"])
        summary["runs"].append({"run":run,"fixture_pass":gate["fixture_pass"],"fixed_input":fixed,
            "parking_audit":gate.get("parking_audit"),"trials":trials,
            "worst_latency":{"epoch_s":worst["t"],**{k:worst["response"][k] for k in
               ("compute_ms","roundtrip_ms","expanded","continuation_checks","continuation_rejections")}}})
        for t in trials:
            row={"run":run,**{k:v for k,v in t.items() if not isinstance(v,dict)}}
            for metric in ("latency_ms","sequence_target_rms_mps"):
                row.update({metric+"_"+k:v for k,v in t[metric].items()})
            csv_rows.append(row)
    (a.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    with (a.output/"trials.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(csv_rows[0]),lineterminator="\n");writer.writeheader();writer.writerows(csv_rows)
    files=[]
    for run in ("fixture_01","fixture_confirmation"):
        for p in sorted((a.data/run).iterdir()):
            if p.is_file():files.append({"path":str(p.relative_to(a.data)),"size":p.stat().st_size,"sha256":hashlib.sha256(p.read_bytes()).hexdigest()})
    manifest={"dataset_id":"rm2027-terminal-recourse-fixture-20261025","root":str(a.data.resolve()),
      "filename_convention":"<fixture_01|fixture_confirmation>/fixture_<search|recourse>.{json,jsonl.gz,log}; fixed_input.json and gate.json",
      "origin":"local independent C++/Python harness; g++ 13.3.0 -O3 -DNDEBUG C++17, i9-13900HX, one logical CPU",
      "purpose":"decisive frozen fixture and one unchanged algorithm confirmation; no 36-episode study",
      "generated_or_real":"generated ideal CV","required_experiment":"run_recourse.sh; baseline fixture is already versioned, not re-archived",
      "files":files,"total_bytes":sum(x["size"] for x in files),"excluded":"build_cache and historical data"}
    (a.output/"dataset_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("summary",len(csv_rows),"trials; external bytes",manifest["total_bytes"])
if __name__=="__main__":main()
