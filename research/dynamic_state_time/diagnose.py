"""One frozen failure audit, not a new controller or parameter search."""
import argparse
import gzip
import json
import math
import pathlib
from run import Consumer
from world import advance,actors_at,cap,clearance,scene,static_grid

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--build",type=pathlib.Path,required=True);ap.add_argument("--data",type=pathlib.Path,required=True);ap.add_argument("--output",type=pathlib.Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    cfg=scene("head_on",13);grid,_=static_grid(cfg)
    rows=[json.loads(s) for s in gzip.open(a.data/"head_on_13_search.jsonl.gz","rt")]
    # Replay every actually executed interval, including the last collision-truncated interval.
    err=0.
    for i,x in enumerate(rows):
        end=rows[i+1]["t"] if i+1<len(rows) else json.loads((a.data/"head_on_13_search.json").read_text())["elapsed_s"]
        state=x["state"].copy();u=x["target_before"].copy()
        events=sorted(x["queue_before"]+[[.1,*x["command"]]])
        for k in range(round((end-x["t"])/.01)):
            for due,ux,uy in events:
                if due<=(k+.5)*.01+1e-9:u=[ux,uy]
            state=advance(state,u)
        err=max(err,max(abs(p-q) for p,q in zip(state,x["after"])))
    i=next(i for i,x in enumerate(rows) if x["response"].get("viable")==0)
    x=rows[i];prior=rows[i-1]
    q={"mode":"search","state":x["state"],"target":x["target_before"],"queue":x["queue_before"],"actors":x["actors"],"wall":2.,"grid":grid,
       "previous":[[prior["t"]+e[0]-x["t"],e[1],e[2]] for e in prior["response"]["schedule"]]}
    c=Consumer(a.build/"search_consumer","search",a.output/"frozen.log")
    try:response=c.call(q)
    finally:c.close()
    # Post-hoc existence witness, NOT a tuned candidate: ten legal primitives.
    # A safe member omitted by the bounded beam refutes 'no physical solution'.
    state=x["state"].copy();u=x["target_before"].copy();tau=0.;minimum=math.inf;controls=[];samples=[]
    for k in range(10):
        for due,ux,uy in x["queue_before"]:
            if due<=(k+.5)*.01+1e-9:u=[ux,uy]
        state=advance(state,u);tau+=.01
        minimum=min(minimum,clearance(state,actors_at(cfg,x["t"]+tau)))
    for ax,ay in [(-1,1)]*4+[(0,1)]+[(0,0)]*5:
        dv=cap([ax*.8*.2,ay*.8*.2],.8*.2)
        u=cap([state[2]+dv[0],state[3]+dv[1]],.5);controls.append([tau,*u])
        for _ in range(20):
            state=advance(state,u);tau+=.01
            d=clearance(state,actors_at(cfg,x["t"]+tau));minimum=min(minimum,d)
            samples.append([tau,*state,d])
    tail_steps=math.ceil(math.hypot(*state[2:])/.8/.01)+30
    for _ in range(tail_steps):
        state=advance(state,[0,0]);tau+=.01
        d=clearance(state,actors_at(cfg,x["t"]+tau));minimum=min(minimum,d)
        samples.append([tau,*state,d])
    result={"executed_interval_replay_max_state_error":err,"first_no_new_candidate_epoch":x["t"],"frozen_response":response,
      "posthoc_witness_controls":controls,"witness_duration_s":tau,"witness_min_physical_clearance_m":minimum,
      "witness_scope":"same primitive library/queued prefix/full 2s+brake tail; post-hoc feasibility only, no candidate change or claimed navigation win",
      "first_unverified_epoch":next(r["t"] for r in rows if not r["response"]["verified"]),"collision_epoch":json.loads((a.data/"head_on_13_search.json").read_text())["elapsed_s"]}
    # Source-time failure fixture is small and sufficient to re-run both checks.
    fixture={k:v for k,v in q.items() if k!="grid"};fixture["scene"]=cfg
    (a.output/"fixture.json").write_text(json.dumps(fixture,indent=2)+"\n")
    (a.output/"diagnosis.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    assert err<1e-10
if __name__=="__main__":main()
