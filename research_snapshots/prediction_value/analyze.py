"""Read every pre-registered trial; paired analysis and all-scene figures."""
import gzip
import json
import math
import pathlib
import statistics
from run import P,percentile

ROOT=pathlib.Path(__file__).resolve().parent
def read_records(path):
    with gzip.open(path,"rt") as f:return [json.loads(line) for line in f]

def main():
    out=ROOT/"evidence"/"p2"
    results=json.loads((out/"results.json").read_text())
    assert len(results)==90
    keys={(r["scene"],r["seed"],r["group"]) for r in results}
    assert len(keys)==90 and keys=={(s,n,g) for s in P["scenes"] for n in P["formal_seeds"] for g in P["groups"]}
    summary=[];paired=[];all_latency={g:[] for g in P["groups"]};diagnostics=[]
    for scene in P["scenes"]:
        for group in P["groups"]:
            rows=[r for r in results if r["scene"]==scene and r["group"]==group]
            body=[r["body_min_clearance_m"] for r in rows if r["body_min_clearance_m"] is not None]
            arrivals=[r["arrival_s"] for r in rows if r["success"]]
            lat=[];forward_flips=0;max_retreat=0.;acc_violations=0
            for row in rows:
                records=read_records(out/f"{scene}_{row['seed']}_{group}.jsonl.gz")
                lat.extend(r["response"]["roundtrip_ms"] for r in records)
                velocities=[r["state_after"][3]*math.cos(r["state_after"][2])-r["state_after"][4]*math.sin(r["state_after"][2]) for r in records]
                signs=[1 if v>.02 else -1 for v in velocities if abs(v)>.02]
                flips=sum(a!=b for a,b in zip(signs,signs[1:]));forward_flips+=flips
                peak=0.;retreat=0.
                for r in records:
                    peak=max(peak,r["state_after"][0]);retreat=max(retreat,peak-r["state_after"][0])
                max_retreat=max(max_retreat,retreat)
                diagnostic={"scene":scene,"seed":row["seed"],"group":group,"forward_sign_flips":flips,"max_retreat_m":retreat,
                    "score_consumed_nonzero_cycles":sum(r["response"].get("dynamic_cost_max",0)>0 for r in records),
                    "all_candidates_dynamic_unsafe_cycles":sum(r["response"].get("safe_candidate_count")==0 for r in records),
                    "safe_candidates_unsafe_aggregate_cycles":row["safe_candidate_but_unsafe_aggregate_cycles"],
                    "optimistic_aggregate_cycles":row["optimistic_aggregate_cycles"]}
                diagnostics.append(diagnostic)
            all_latency[group].extend(lat)
            summary.append({"scene":scene,"group":group,"n":len(rows),"success":sum(r["success"] for r in rows),"safe_success":sum(r["safe_success"] for r in rows),
                "collisions":sum(r["collision"] for r in rows),"body_clearance_worst":min(body) if body else None,"body_clearance_median":statistics.median(body) if body else None,
                "arrival_median_s":statistics.median(arrivals) if arrivals else None,"arrival_success_n":len(arrivals),
                "wait_median_s":statistics.median(r["wait_s"] for r in rows),"path_median_m":statistics.median(r["path_length_m"] for r in rows),
                "stop_start_median":statistics.median(r["stop_start_count"] for r in rows),"max_normalized_acceleration":max(r["max_normalized_acceleration"] for r in rows),
                "command_jump_p95_median":statistics.median(r["command_jump_p95"] for r in rows),"forward_sign_flips_total":forward_flips,"max_retreat_m":max_retreat,
                "p50_ms":percentile(lat,.5),"p95_ms":percentile(lat,.95),"p99_ms":percentile(lat,.99),"cycles":sum(r["cycles"] for r in rows),
                "deadline_misses":sum(r["deadline_misses"] for r in rows),"native_failures":sum(r["native_failures"] for r in rows),
                "consumed_cycles":sum(r["prediction_consumed_cycles"] for r in rows),"unavailable_cycles":sum(r["prediction_unavailable_cycles"] for r in rows)})
        for baseline in ("A_proxy","Aprime_zoh","CV","C_KF"):
            for seed in P["formal_seeds"]:
                b=next(r for r in results if (r["scene"],r["seed"],r["group"])==(scene,seed,"B_oracle"))
                a=next(r for r in results if (r["scene"],r["seed"],r["group"])==(scene,seed,baseline))
                paired.append({"scene":scene,"seed":seed,"baseline":baseline,"B_outcome":b["outcome"],"baseline_outcome":a["outcome"],
                    "B_safe_success":b["safe_success"],"baseline_safe_success":a["safe_success"],
                    "time_change_fraction":b["arrival_s"]/a["arrival_s"]-1 if b["success"] and a["success"] else None,
                    "clearance_change_m":b["body_min_clearance_m"]-a["body_min_clearance_m"] if b["body_min_clearance_m"] is not None else None})
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    (out/"paired.json").write_text(json.dumps(paired,indent=2)+"\n")
    (out/"consumption_diagnostics.json").write_text(json.dumps(diagnostics,indent=2)+"\n")
    lat_summary={g:{"p50_ms":percentile(v,.5),"p95_ms":percentile(v,.95),"p99_ms":percentile(v,.99),"max_ms":max(v),"n":len(v)} for g,v in all_latency.items()}
    (out/"latency.json").write_text(json.dumps(lat_summary,indent=2)+"\n")
    gates={"overall_decision":"Modify: native proxy evidence; full STVL and physical hardware not rerun", "safety_signal":{},"efficiency_signal_vs_CV":[]}
    for comparator in ("A_proxy","Aprime_zoh","CV","C_KF"):
        passing=[]
        for scene in P["scenes"][1:]:
            b=next(v for v in summary if v["scene"]==scene and v["group"]=="B_oracle")
            a=next(v for v in summary if v["scene"]==scene and v["group"]==comparator)
            if b["safe_success"]==3 and a["safe_success"]<=1:passing.append(scene)
        gates["safety_signal"][comparator]=passing
    for scene in P["scenes"][1:]:
        comparisons=[v["time_change_fraction"] for v in paired if v["scene"]==scene and v["baseline"]=="CV"]
        if all(v is not None for v in comparisons) and statistics.median(comparisons)<=-.1 and sum(v<0 for v in comparisons)>=2:gates["efficiency_signal_vs_CV"].append(scene)
    (out/"frozen_gates.json").write_text(json.dumps(gates,indent=2)+"\n")
    lines=["| 场景 | 组 | 到达/安全成功/碰撞 | 到达中位(s) | 最坏/中位body净空(m) | WAIT中位(s) | p95(ms) |", "| --- | --- | --- | --- | --- | --- | --- |"]
    def f(v):return "—" if v is None else f"{v:.3f}"
    for r in summary:lines.append(f"| {r['scene']} | {r['group']} | {r['success']}/3 / {r['safe_success']}/3 / {r['collisions']}/3 | {f(r['arrival_median_s'])} | {f(r['body_clearance_worst'])} / {f(r['body_clearance_median'])} | {f(r['wait_median_s'])} | {f(r['p95_ms'])} |")
    (out/"table.md").write_text("\n".join(lines)+"\n")
    plot(out)

def plot(out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors={"A_proxy":"#747474","Aprime_zoh":"#c78000","CV":"#8c65b6","B_oracle":"#168356","C_KF":"#1967b4"}
    for kind in ("xy","progress"):
        fig,axes=plt.subplots(2,3,figsize=(13,7),constrained_layout=True)
        for scene,ax in zip(P["scenes"],axes.flat):
            for group in P["groups"]:
                rows=read_records(out/f"{scene}_29_{group}.jsonl.gz")
                t=[r["t"] for r in rows];x=[r["state_after"][0] for r in rows];y=[r["state_after"][1] for r in rows]
                ax.plot(x if kind=="xy" else t,y if kind=="xy" else x,color=colors[group],label=group,lw=1.5)
                if kind=="xy":ax.scatter(x[-1],y[-1],s=12,color=colors[group])
            ax.set_title(scene+" / seed 29");ax.grid(alpha=.2)
            ax.set_xlabel("x (m)" if kind=="xy" else "time (s)");ax.set_ylabel("y (m)" if kind=="xy" else "forward x (m)")
            if kind=="xy":
                ax.set_xlim(-.2,4.3);ax.set_ylim(-1.3,1.3)
                if scene in ("narrow","wait_release"):
                    ax.axhline(.7,color="black",lw=1);ax.axhline(-.7,color="black",lw=1)
            else:ax.set_ylim(-.2,4.3)
        axes.flat[0].legend(fontsize=8)
        fig.suptitle("Native MPPI prediction intervention / proxy closed loop (all groups, fixed seed)")
        fig.savefig(out/f"{kind}_all_scenes.png",dpi=150);plt.close(fig)

if __name__=="__main__":main()
