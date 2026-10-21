"""Time-advancing proxy closed loop with native MPPI. No hardware outputs."""
import argparse
import csv
import hashlib
import json
import math
import pathlib
import random
import statistics
import subprocess
import time
from providers import Sources
from scenes import parameters,truth,clearance,wall_clearance

ROOT=pathlib.Path(__file__).resolve().parent
P=json.loads((ROOT/"protocol.json").read_text())

def percentile(values,q):
    if not values: return None
    values=sorted(values);index=(len(values)-1)*q
    lo=int(index);hi=min(lo+1,len(values)-1)
    return values[lo]+(index-lo)*(values[hi]-values[lo])

class Native:
    def __init__(self,binary,out):
        self.log=open(out,"w")
        self.p=subprocess.Popen([binary,"--ros-args","--params-file",str(ROOT/"baseline.yaml")],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
    def call(self,row):
        start=time.perf_counter()
        self.p.stdin.write(json.dumps(row,allow_nan=False)+"\n");self.p.stdin.flush()
        line=self.p.stdout.readline()
        if not line: raise RuntimeError("native consumer exited")
        result=json.loads(line);result["roundtrip_ms"]=(time.perf_counter()-start)*1000
        return result
    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=20);self.log.close()

def advance(state,target,dt):
    old=state[3:].copy()
    for k in range(3):
        lo,hi=P["velocity_limits"][k]
        goal=max(lo,min(hi,target[k]))
        delta=max(-P["acceleration_limits"][k]*dt,min(P["acceleration_limits"][k]*dt,goal-old[k]))
        state[3+k]=old[k]+delta
    vx,vy,wz=[(old[k]+state[3+k])*.5 for k in range(3)]
    yaw=state[2]+wz*dt*.5
    state[0]+=dt*(vx*math.cos(yaw)-vy*math.sin(yaw))
    state[1]+=dt*(vx*math.sin(yaw)+vy*math.cos(yaw))
    state[2]+=wz*dt

def trial(native,scene,seed,mode,directory):
    config=parameters(scene,seed)
    native.call({"reset":seed})
    state=[0.,0.,0.,0.,0.,0.];sources=Sources();rng=random.Random(seed)
    target=[0.,0.,0.];queue=[];path_length=0.;wait=0.;stopstarts=0;was_moving=False
    min_body=math.inf;min_padded=math.inf;min_wall=math.inf
    timeouts=failures=consumed=unavailable=0;optimistic=0;aggregate_unsafe=0;safe_then_unsafe=0
    errors=[];times=[];native_times=[];command_jump=[];accel=[];records=[];prediction_errors=[]
    occupied_cycles=0;outcome="timeout";done=False;last_command=[0.,0.,0.]
    tick=0;t=0.
    while t<P["timeout_s"]-1e-8:
        sample_time=t-P["observation_delay"]
        true_observation=truth(config,sample_time)
        # Draw at every clock tick, including invisible ticks, so paired groups
        # receive identical noise at the same source epoch even after paths diverge.
        noise=[rng.gauss(0.,P["observation_noise_sigma_m"]) for _ in range(2)]
        current=None
        if true_observation is not None and math.hypot(true_observation[0]-state[0],true_observation[1]-state[1])<=P["observation_range_m"]:
            current=[v+noise[k] for k,v in enumerate(true_observation)]
            occupied_cycles+=1
        sources.update(current,sample_time)
        # B receives only the permitted oracle prediction envelope. Same current
        # occupancy and observation availability as every other group.
        pred=None
        if mode=="B_oracle" and current is not None:
            pred=[truth(config,t+(k+1)*P["prediction_dt"]) for k in range(30)]
        elif mode!="A_proxy":
            pred=sources.prediction(mode,t,current)
        if pred is None and mode!="A_proxy" and current is not None:unavailable+=1
        if pred is not None:
            consumed+=1
            prediction_errors.append([math.dist(pred[k],truth(config,t+(k+1)*.1)) for k in (0,9,29)])
        request={"state":state,"wall":config["wall"],"radius":config["radius"],"current":current,"prediction":pred}
        response=native.call(request)
        ms=response["roundtrip_ms"];times.append(ms)
        native_times.append(response.get("compute_ms",ms))
        fallback=""
        if "error" in response:
            failures+=1;errors.append({"tick":tick,"reason":response["error"]});fallback="exception"
        if ms>P["compute_budget_ms"]:timeouts+=1;fallback="deadline"
        command=[0.,0.,0.] if fallback else response["command"]
        command_jump.append(math.dist(command,last_command));last_command=command
        # Actual computation delay is included in execution; output remains old
        # until the new target arrives. No B-only pause or ideal robot state.
        queue.append((t+ms*.001+P["output_delay"],command))
        rollout=response.get("rollout",[])
        true_agg=min((clearance(r,truth(config,t+(k+1)*.1),config["radius"],.03) for k,r in enumerate(rollout)),default=None) if scene!="empty" else None
        predicted=response.get("aggregate_predicted_clearance")
        if true_agg is not None and true_agg<=.02:
            aggregate_unsafe+=1
            if response.get("safe_candidate_count",0) or 0:safe_then_unsafe+=1
            if predicted is not None and predicted>.02:optimistic+=1
        record={"t":t,"state":state.copy(),"current":current,"prediction":pred,
            "response":response,"fallback":fallback,"aggregate_truth_clearance":true_agg}
        # Truth annotations above are evaluator only and never sent to native.
        for sub in range(10):
            now=t+(sub+.5)*.01
            while queue and queue[0][0]<=now:
                _,target=queue.pop(0)
            old=state.copy();advance(state,target,.01)
            accel.append(max(abs(state[3+k]-old[3+k])/.01/P["acceleration_limits"][k] for k in range(3)))
            path_length+=math.dist(old[:2],state[:2])
            if math.hypot(state[3],state[4])<.02:wait+=.01
            point=truth(config,t+(sub+1)*.01)
            body=clearance(state,point,config["radius"])
            padded=clearance(state,point,config["radius"],.03)
            wc=wall_clearance(state,config["wall"])
            if body is not None:min_body=min(min_body,body);min_padded=min(min_padded,padded)
            min_wall=min(min_wall,wc)
            if (body is not None and body<=-P["collision_tolerance_m"]) or wc<=-P["collision_tolerance_m"]:
                outcome="collision";done=True;break
            if math.dist(state[:2],config["goal"])<=P["goal_xy_tolerance_m"] and abs(math.atan2(math.sin(state[2]),math.cos(state[2])))<=P["goal_yaw_tolerance_rad"]:
                outcome="success";done=True;break
        moving=math.hypot(state[3],state[4])>=.02
        if moving and not was_moving:stopstarts+=1
        was_moving=moving;record["state_after"]=state.copy();records.append(record)
        t+=P["control_dt"];tick+=1
        if done:break
    result={"scene":scene,"seed":seed,"group":mode,"outcome":outcome,"collision":outcome=="collision",
       "success":outcome=="success","elapsed_s":round(t,6),"arrival_s":round(t,6) if outcome=="success" else None,
       "body_min_clearance_m":None if min_body==math.inf else min_body,"padded_min_clearance_m":None if min_padded==math.inf else min_padded,
       "wall_min_clearance_m":min_wall,"safe_success":outcome=="success" and min_body>=.05-.003 and min_wall>=-.003,
       "path_length_m":path_length,"wait_s":wait,"stop_start_count":stopstarts,
       "native_failures":failures,"deadline_misses":timeouts,"cycles":tick,"prediction_consumed_cycles":consumed,
       "prediction_unavailable_cycles":unavailable,"current_occupied_cycles":occupied_cycles,
       "p50_ms":percentile(times,.5),"p95_ms":percentile(times,.95),"p99_ms":percentile(times,.99),
       "native_p95_ms":percentile(native_times,.95),"optimistic_aggregate_cycles":optimistic,
       "aggregate_truth_unsafe_cycles":aggregate_unsafe,"safe_candidate_but_unsafe_aggregate_cycles":safe_then_unsafe,
       "command_jump_p95":percentile(command_jump,.95),"max_normalized_acceleration":max(accel,default=0),
       "prediction_error_01_1_3s_m":[statistics.mean(e[k] for e in prediction_errors) for k in range(3)] if prediction_errors else None,
       "errors":errors,"parameters":config}
    path=directory/f"{scene}_{seed}_{mode}.json"
    path.write_text(json.dumps(result,indent=2)+"\n")
    import gzip
    with gzip.open(directory/f"{scene}_{seed}_{mode}.jsonl.gz","wt") as f:
        for record in records:f.write(json.dumps(record,separators=(",",":"))+"\n")
    return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--binary",required=True);parser.add_argument("--phase",choices=("p1","p2"),required=True);parser.add_argument("--output",required=True);args=parser.parse_args()
    out=pathlib.Path(args.output)/args.phase
    out.mkdir(parents=True,exist_ok=False)  # Never overwrite an earlier cohort.
    native=Native(args.binary,out/"native.log")
    seeds=[P["p1_seed"]] if args.phase=="p1" else P["formal_seeds"]
    scenes=P["p1_scenes"] if args.phase=="p1" else P["scenes"]
    results=[]
    try:
        for scene in scenes:
            for seed in seeds:
                for mode in P["groups"]:
                    result=trial(native,scene,seed,mode,out);results.append(result)
                    print(scene,seed,mode,result["outcome"],result["arrival_s"],result["body_min_clearance_m"],"p95",round(result["p95_ms"],2),flush=True)
    finally:native.close()
    (out/"results.json").write_text(json.dumps(results,indent=2)+"\n")
    with (out/"trials.csv").open("w") as f:
        keys=[k for k in results[0] if k not in ("errors","parameters")]
        writer=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore");writer.writeheader();writer.writerows(results)
    (out/"protocol_sha256.txt").write_text(hashlib.sha256((ROOT/"protocol.json").read_bytes()).hexdigest()+"\n")

if __name__=="__main__":main()
