"""Full-episode lifecycle gate; never dispatches the 36-episode experiment."""
import argparse
import json
import math
import os
from pathlib import Path
from run import Consumer
from reserve_run import FIXTURE, question, at, trial
from world import observed, advance, clearance, actors_at


def initial_gate(build, out):
    c = Consumer(build/"reserve_consumer", "reserve", out/"initial_gate.log")
    try:
        _, actors = observed(FIXTURE["scene"], 0.)
        q = question([0.,0.,0.,0.], [0.,0.], [], 0., actors, 0, 0)
        r = c.call(q | {"op":"init"}); cert = r["certificate"]
        if cert is None:
            result = dict(available=False, verdict="Stop", reason="full_episode_initialization_unavailable")
        else:
            vq = q | {"op":"validate","backup":cert}
            assert c.call(vq)["reason"]=="valid"
            tests = [("source_epoch",-.2,"stale_observation"),
                     ("state",[.01,0.,0.,0.],"state_deviation"),
                     ("queue",[[.1,.5,0.]],"immutable_queue_mismatch"),
                     ("dynamics",[.01,.5,.7,.355,.330],"dynamics_changed"),
                     ("source_version",-1,"prediction_version_regressed"),
                     ("queue_version",-1,"queue_version_regressed")]
            for key,value,reason in tests:
                assert c.call(vq | {key:value})["reason"]==reason,reason
            state=q["state"].copy(); target=q["target"].copy(); minimum=math.inf
            for k,expected in enumerate(cert["reference_states"][1:]):
                target=at(cert["controls"], k*.01+.005, target); state=advance(state,target)
                assert max(abs(a-b) for a,b in zip(state,expected))<1e-10
                minimum=min(minimum,clearance(state,actors_at(FIXTURE["scene"],(k+1)*.01),FIXTURE["wall"]))
            result=dict(available=True, initialization_ms=r["roundtrip_ms"], invalidation_checks=len(tests),
                        independent_rollout_min_clearance_m=minimum,certificate=cert,
                        note="The replay checks the certificate, not navigation task completion")
        (out/"initial_gate.json").write_text(json.dumps(result,indent=2)+"\n")
        print("initial acquisition",result["available"],flush=True)
        return result["available"]
    finally:c.close()


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--build",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--condition",choices=["normal","late"],default="normal")
    ap.add_argument("--normal-proof",type=Path);ap.add_argument("--initial-only",action="store_true")
    args=ap.parse_args()
    if args.condition=="late":
        if args.normal_proof is None:raise SystemExit("late requires --normal-proof results.json")
        c1=next(r for r in json.loads(args.normal_proof.read_text()) if r["arm"]=="C1")
        assert c1["initialized"] and c1["held_cycles"]>1 and not c1["collision"] and not c1["invalid"]
        assert c1["total_cycle_ms"]["p99"]<=40 and c1["total_cycle_ms"]["max"]<=50
        assert c1["intrinsic_deadline_misses"]==0
    args.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    if not initial_gate(args.build,args.output) or args.initial_only:return
    results=[trial(args.build,args.output,arm,args.condition,full_episode=True) for arm in ("B","C0","C1")]
    (args.output/"results.json").write_text(json.dumps(results,indent=2)+"\n")


if __name__=="__main__":main()
