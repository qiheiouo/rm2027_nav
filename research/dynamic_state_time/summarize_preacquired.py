"""Audit executable reserve continuity and retain only compact experiment evidence."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
from reserve_run import at, FIXTURE
from world import clearance, actors_at

RUNS = ("normal_01", "normal_confirmation", "late_01", "late_confirmation")


def audit(rows):
    unchanged = 0; postponed = 0; queued = 0; actual_backup_steps = 0; busy_backup_steps = 0
    last = None; pending = []
    for row in rows:
        backup = row["active_backup"]
        assert backup is not None and row["validity"]=="valid", row["t"]
        assert backup["execution_cursor"]==sum(e[0]<=row["t"]+1e-8 for e in backup["controls"])
        for entry in pending:
            if entry[0]>=row["t"]-1e-8:
                assert entry in row["immutable_queue"], (row["t"],"immutable command removed")
                queued += 1
        pending = row["immutable_queue"]
        if last is not None:
            if not row["accepted"]:
                assert backup["id"]==last["id"] and backup["controls"]==last["controls"]
                assert backup["switch_time"]==last["switch_time"]
                assert backup["execution_cursor"]>=last["execution_cursor"]
                unchanged += 1
            if backup["switch_time"]>last["switch_time"]+1e-8:
                assert row["accepted"] and row["candidate_available"] and row["reason"] in ("splice_old","new_backup")
                postponed += 1
        for ex, owner in zip(row["executed"],row["executed_owners"]):
            # Commands due at the left edge affect the following .01 s plant step.
            expected = at(row["immutable_queue"],ex[0]-.005,row["target"])
            assert max(abs(a-b) for a,b in zip(ex[1:3],expected))<1e-10
            if owner and owner["backup"]:
                actual_backup_steps += 1
                if row["proposed"] is None or row["response_ms"]>50:busy_backup_steps += 1
        last = backup
    return dict(preserved_rejection_or_busy_cycles=unchanged,
                certificate_checked_postponements=postponed,immutable_queue_entries_checked=queued,
                actual_backup_command_steps=actual_backup_steps,
                backup_steps_during_late_or_busy_cycles=busy_backup_steps,
                unconditional_postponements=0,queue_retractions=0,invalid_executions=0,
                scope="audit of recorded deterministic simulation, not OS/hardware timing")


def event(row,label):
    b=row["active_backup"]
    return dict(label=label,t=row["t"],state=row["state"],
                proposal=row["proposed"]["command"] if row["proposed"] else None,
                accepted=row["accepted"],committed=row["committed_action"],reason=row["reason"],
                backup_id=b["id"],lineage=b["lineage"],acquired_epoch=b["acquired_epoch"],
                absolute_backup_controls=b["backup_controls"],switch_time=b["switch_time"],
                cursor=b["execution_cursor"],safe_rest=b["safe_rest"],validity=row["validity"],
                executed_command=row["executed"][-1][1:3],executed_owner=row["executed_owners"][-1])


def plot(data,out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(9,5.5),sharex="col",layout="constrained")
    colors={"B":"#9a416e","C0":"#b87418","C1":"#087e8b"}
    for col,(run,condition) in enumerate((("normal_01","normal"),("late_01","late"))):
        axes[0,col].set_title(condition+" latency")
        for arm in ("B","C0","C1"):
            rows=[json.loads(x) for x in gzip.open(data/run/f"{arm}_{condition}.jsonl.gz","rt")]
            samples=[s for r in rows for s in r["executed"]]
            times=[s[0] for s in samples];ys=[s[4] for s in samples]
            clear=[clearance(s[3:],actors_at(FIXTURE["scene"],s[0]),FIXTURE["wall"]) for s in samples]
            axes[0,col].plot(times,ys,color=colors[arm],label=arm,lw=1.4)
            axes[1,col].plot(times,clear,color=colors[arm],lw=1.4)
            if arm!="C1":axes[1,col].plot(times[-1],clear[-1],"x",color=colors[arm])
        for ax in axes[:,col]:
            ax.axvspan(3.55,6.30,color="#999999",alpha=.12)
            ax.axvline(2.50,color="#999999",ls=":",lw=.7)
            ax.grid(alpha=.2)
        axes[1,col].axhline(.05,color="black",ls="--",lw=.7)
        axes[1,col].set_xlabel("absolute episode time (s)")
        axes[0,col].legend(loc="best",fontsize=8)
    axes[0,0].set_ylabel("robot y (m)");axes[1,0].set_ylabel("physical clearance (m)")
    fig.suptitle("Frozen head-on episode; shading marks historical analysis window only",fontsize=10)
    fig.savefig(out/"episode_comparison.png",dpi=125)
    plt.close(fig)


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--data",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True);ap.add_argument("--plot",action="store_true")
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[];audits={};timelines={};files=[]
    for run in RUNS:
        directory=args.data/run
        results.extend(dict(run=run,**r) for r in json.loads((directory/"results.json").read_text()))
        condition="normal" if run.startswith("normal") else "late"
        rows=[json.loads(x) for x in gzip.open(directory/f"C1_{condition}.jsonl.gz","rt")]
        audits[run]=audit(rows)
        life=json.loads((directory/f"C1_{condition}_lifecycle.json").read_text())
        if run.endswith("01"):
            selected=[event(rows[0],"start")]
            for t in (2.5,3.55,6.3):selected.append(event(min(rows,key=lambda r:abs(r["t"]-t)),f"historical_epoch_{t}"))
            for label,predicate in (("first_rejection",lambda r:not r["accepted"]),
                                    ("first_late",lambda r:r["reason"]=="planner_deadline_rejected"),
                                    ("first_busy",lambda r:r["proposed"] is None)):
                row=next((r for r in rows if predicate(r)),None)
                if row:selected.append(event(row,label))
            timelines[condition]=dict(representative_events=selected,
                                     execution_phase_transitions=[e for e in life if e["state"] in ("EXECUTING_BACKUP","SAFE_REST","INVALID","UNAVAILABLE")],
                                     note="REVALIDATED/REPLACED per-cycle records remain in external raw lifecycle files")
        for path in sorted(directory.iterdir()):
            files.append(dict(path=str(path.relative_to(args.data)),size=path.stat().st_size,
                              sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    c1s=[r for r in results if r["arm"]=="C1"]
    passed=all(r["initialized"] and r["held_cycles"]>1 and r["completion"] and not r["collision"]
               and r["minimum_clearance_m"]>=.05 and not r["invalid"] and r["timeout_brakes"]==0
               and r["intrinsic_deadline_misses"]==0 and r["total_cycle_ms"]["p99"]<=40 and r["total_cycle_ms"]["max"]<=50 for r in c1s)
    initial=json.loads((args.data/"normal_01/initial_gate.json").read_text())
    initial.pop("certificate")
    summary=dict(verdict="Go-to-36" if passed else "Modify/Stop: inspect failure",experiment_36="NOT_RUN",
                 scope="one frozen head_on seed13 episode, two unchanged repeats per latency condition; ideal CV only",
                 initial_gate=initial,persistence_audits=audits,results=results)
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    (args.output/"timeline.json").write_text(json.dumps(timelines,indent=2)+"\n")
    columns=["run","arm","condition","collision","completion","minimum_clearance_m","arrival_s","wait_s",
             "held_cycles","backup_dispatches","late_backup_dispatches","timeout_brakes","intrinsic_deadline_misses","response_deadline_misses"]
    with (args.output/"trials.csv").open("w") as f:
        w=csv.DictWriter(f,fieldnames=columns+["total_p99_ms","total_max_ms"],lineterminator="\n");w.writeheader()
        for r in results:w.writerow({k:r[k] for k in columns}|dict(total_p99_ms=r["total_cycle_ms"]["p99"],total_max_ms=r["total_cycle_ms"]["max"]))
    manifest=dict(dataset_id="rm2027-preacquired-backup-headon-20261026",root=str(args.data.resolve()),
                  filename_convention="<normal_01|normal_confirmation|late_01|late_confirmation>/<B|C0|C1>_<normal|late>.{json,jsonl.gz}; *_lifecycle.json; *_planner.log; *_reserve.log; initial_gate.json; results.json",
                  origin="frozen independent C++/Python harness, GCC -O3 -DNDEBUG C++17, single logical CPU",
                  purpose="pre-acquired reserve lifecycle and timing attribution; one full head-on episode",
                  generated_or_real="generated ideal CV; no future truth supplied to either consumer",
                  required_experiment="run_preacquired.sh normal first; late requires --normal-proof; summarize_preacquired.py",
                  files=files,total_bytes=sum(f["size"] for f in files),excluded="build_cache and preliminary initial_gate duplicate; no historical archives")
    (args.output/"dataset_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    if args.plot:plot(args.data,args.output)
    print(summary["verdict"]);print(json.dumps(audits,indent=2))


if __name__=="__main__":main()
