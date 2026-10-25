"""Only the saved decisive fixture. No scene tuning, grid sweep or pilot dispatch."""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
from run import Consumer, trial
from world import advance, clearance, static_grid

ROOT=Path(__file__).resolve().parent

def simulate(q, schedule):
    """Execute the committed prefix and returned whole schedule with the common plant."""
    s=q["state"].copy();u=q["target"].copy();minimum=math.inf
    events=sorted(q["queue"]+schedule)
    # Exact CV input extrapolation, no scene truth; tail includes a full stop and .3s.
    duration=max(e[0] for e in schedule)+.5/.8+.3
    for k in range(math.ceil(duration/.01)):
        t=(k+.5)*.01
        for due,vx,vy in events:
            if due<=t+1e-9:u=[vx,vy]
        s=advance(s,u)
        actors=[[x+vx*(k+1)*.01,y+vy*(k+1)*.01,vx,vy,r] for x,y,vx,vy,r in q["actors"]]
        minimum=min(minimum,clearance(s,actors,q["wall"]))
    return s,math.ceil(duration/.01)*.01,minimum

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--build",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    fixture=json.loads((ROOT/"results/failure_fixture.json").read_text())
    diagnosis=json.loads((ROOT/"results/failure_diagnosis.json").read_text())
    q={k:fixture[k] for k in ("mode","state","target","queue","actors","wall","previous")}
    q["grid"],_=static_grid(fixture["scene"])
    consumers=[Consumer(a.build/b,"search",a.output/f"{b}.log") for b in ("search_baseline","search_consumer")]
    old,c=consumers
    try:
        baseline=old.call(q);response=c.call(q)
        for key in ("command","verified","reason","prefix_safe","viable","expanded","schedule"):
            assert baseline[key]==response[key],(key,baseline[key],response[key])
        assert response["reason"]=="reused" and response["viable"]==0
        # Analytic refuge checks catch late collision, divergence, tangency and a corner miss.
        probes=[([2.,0.,-.45,0.,.3],False),([2.,0.,.45,0.,.3],True),
                ([2.,.8,-.45,0.,.3],True),([2.,.63,-.45,0.,.3],False),
                ([.57,.545,0.,0.,.3],False),([.60,.575,0.,0.,.3],True)]
        for actor,expected in probes:
            r=c.call({"recourse_probe":True,"state":[0,0,0,0],"actors":[actor],"wall":2.})
            assert r["stationary_refuge"]==expected,(actor,r)
        def probe(s,t):
            return c.call({"recourse_probe":True,"state":s,"epoch":t,"actors":q["actors"],"wall":q["wall"]})
        brake_state,brake_time,brake_clearance=simulate(q,[[.1,0,0]])
        brake_probe=probe(brake_state,brake_time)
        witness=diagnosis["posthoc_witness_controls"]+[[2.1,0,0]]
        witness_state,witness_time,witness_clearance=simulate(q,witness)
        witness_probe=probe(witness_state,witness_time)
        q["mode"]="recourse";candidate=c.call(q)
        selected_state,selected_time,selected_clearance=simulate(q,candidate["schedule"])
        selected_probe=probe(selected_state,selected_time)
        result={"baseline_response":baseline,"unchanged_B_behavior":True,"analytic_refuge_checks":len(probes),
          "early_brake":{"state":brake_state,"epoch_relative_s":brake_time,"min_clearance_m":brake_clearance,"probe":brake_probe},
          "old_witness":{"min_clearance_m":witness_clearance,"state":witness_state,"probe":witness_probe},
          "candidate_response":candidate,"candidate_schedule":{"min_clearance_m":selected_clearance,"state":selected_state,"probe":selected_probe}}
        (a.output/"fixed_input.json").write_text(json.dumps(result,indent=2)+"\n")
        print("fixed input",json.dumps({"old":baseline["reason"],"C":candidate["reason"],"C_ms":candidate["roundtrip_ms"],"C_min_clearance":selected_clearance,"early_brake_backup":brake_probe["backup_exists"]}),flush=True)
    finally:
        for consumer in consumers:consumer.close()
    initial={k:fixture[k] for k in ("state","target","queue","previous")}
    initial["epoch"]=diagnosis["first_no_new_candidate_epoch"]
    results=[trial(fixture["scene"],arm,a.build,a.output,label="fixture_"+arm,initial=initial) for arm in ("search","recourse")]
    # Distinguish an unsafe hold policy from lack of a library continuation at
    # the actual early/late stopped states; these are observations, not retuning.
    rows=[json.loads(line) for line in gzip.open(a.output/"fixture_search.jsonl.gz","rt")]
    parked=next(row for row in rows if math.hypot(*row["state"][2:])<1e-9)
    late=next(row for row in rows if not row["response"]["verified"])
    parking_audit={}
    c=Consumer(a.build/"search_consumer","recourse",a.output/"parking_audit.log")
    try:
        for name,row in (("original_early_stop",parked),("original_first_unverified",late)):
            parking_audit[name]={"epoch_s":row["t"],"state":row["state"],"actors":row["actors"],
                "probe":c.call({"recourse_probe":True,"state":row["state"],"actors":row["actors"],"wall":q["wall"]})}
    finally:c.close()
    cresult=results[-1]
    (a.output/"gate.json").write_text(json.dumps({"fixture_pass":bool(
        not brake_probe["stationary_refuge"] and witness_probe["stationary_refuge"]
        and candidate["verified"] and selected_clearance>=.05 and selected_probe["stationary_refuge"]
        and cresult["safe_completion"] and cresult["deadline_misses"]==0
        and cresult["latency_ms"]["p99"]<=40 and cresult["latency_ms"]["max"]<=50
        and candidate["roundtrip_ms"]<=50),"parking_audit":parking_audit,"trials":results},indent=2)+"\n")
if __name__=="__main__":main()
