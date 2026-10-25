"""Research-only delayed closed loop, native consumer subprocess, external raw evidence."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import os
import pathlib
import resource
import select
import subprocess
import time
from world import advance,actors_at,cap,clearance,follow,observed,scene,static_grid,stats,witness

ROOT=pathlib.Path(__file__).resolve().parent
P=json.loads((ROOT/"protocol.json").read_text())

class Consumer:
    def __init__(self,binary,arm,out):
        self.log=open(out,"w")
        command=[str(binary)]
        if arm=="native":command += ["--ros-args","--params-file",str(ROOT/"native.yaml")]
        self.p=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
        self.arm=arm
        if arm=="native":self.call({"reset":13})
        else:self.call({"probe":True,"state":[0,0,0,0],"actor":[3,0,0,0,.3],"command":[0,0]})
    def call(self,q):
        start=time.perf_counter()
        self.p.stdin.write(json.dumps(q,separators=(",",":"),allow_nan=False)+"\n");self.p.stdin.flush()
        if not select.select([self.p.stdout],[],[],15)[0]:raise RuntimeError("consumer did not respond in 15s")
        line=self.p.stdout.readline()
        if not line:raise RuntimeError("consumer exited")
        r=json.loads(line);r["roundtrip_ms"]=(time.perf_counter()-start)*1000
        return r
    def usage(self):
        fields=pathlib.Path(f"/proc/{self.p.pid}/stat").read_text().split()
        ticks=os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        cpu=(float(fields[13])+float(fields[14]))/ticks
        mem=pathlib.Path(f"/proc/{self.p.pid}/status").read_text().splitlines()
        rss=int(next(x for x in mem if x.startswith("VmHWM:")).split()[1])/1024
        return cpu,rss
    def close(self):
        self.p.stdin.close()
        try:self.p.wait(timeout=10)
        except subprocess.TimeoutExpired:self.p.terminate();self.p.wait(timeout=5)
        self.log.close()

def hold_ttc(s,target,actors,wall,queue):
    state=s.copy();u=target.copy()
    for k in range(300):
        tau=k*.01
        for due,v in queue:
            if due<=tau+.005:u=v
        state=advance(state,u)
        future=[[x+vx*(tau+.01),y+vy*(tau+.01),vx,vy,r] for x,y,vx,vy,r in actors]
        if clearance(state,future,wall)<=0:return tau+.01
    return None

def project_trace(s,target,queue,schedule):
    state=s.copy();u=target.copy();positions=[]
    events=sorted(queue+[(x[0],x[1:3]) for x in schedule])
    for k in range(60):
        tau=k*.01
        for due,v in events:
            if due<=tau+.005:u=v
        state=advance(state,u)
        positions.append(state[:2])
    return positions

def trial(cfg,arm,build,out,label=None,initial=None):
    name=label or f'{cfg["name"]}_{cfg["seed"]}_{arm}'
    grid,path=static_grid(cfg)
    consumer=None if arm=="witness" else Consumer(build/("native_consumer" if arm=="native" else "search_consumer"),arm,out/f'{name}.log')
    if arm=="native":consumer.call({"reset":cfg["seed"]})
    cpu0,rss=(consumer.usage() if consumer else (0,0));wall0=time.perf_counter();own_cpu0=time.process_time()
    state=[0.,0.,0.,0.];target=[0.,0.];queue=[];previous=[];previous_trace=None
    t=0.;arrival=None;min_clear=math.inf;worst=None;collision=False;near=False
    if initial is not None:
        t=initial["epoch"];state=initial["state"].copy();target=initial["target"].copy()
        queue=[(t+e[0],e[1:3]) for e in initial["queue"]]
        previous=[(t+e[0],e[1:3]) for e in initial["previous"]]
    start_epoch=t
    path_length=0.;wait=0.;stop_run=0.;stop_count=0;flips=0;last_sign=[0,0];last_acc=[0.,0.]
    churn=0;deadline=0;no_action=0;errors=[];reasons={};latencies=[];jerks=[];accelerations=[];intervals=[];finite_ttc=[]
    min_ttc_margin=None;max_expanded=0;native_unsafe=0;cycles=0;maxspeed=0.;maxacc=0.;speed_history=[]
    last_schedule=None;sequence_rms=[]
    try:
      with gzip.open(out/f'{name}.jsonl.gz',"wt") as trace:
        while t<40 if arrival is None else t<arrival+3:
            current,actors=observed(cfg,t)
            relative=[(due-t,u) for due,u in queue]
            q={"mode":arm,"state":state,"target":target,"queue":[[dt,*u] for dt,u in relative],"actors":actors,"wall":cfg["wall"],"grid":grid,
               "previous":[[due-t,*u] for due,u in previous]}
            start=time.perf_counter()
            if arm=="native":
                q={"state":[state[0],state[1],0.,state[2],state[3],0.],"actors":actors,"current":current,"wall":cfg["wall"],"path":path}
                response=consumer.call(q)
            elif arm=="witness":response={"command":witness(cfg,path,t,state),"reason":"independent_witness","verified":True,"roundtrip_ms":0}
            else:response=consumer.call(q)
            elapsed=response["roundtrip_ms"]/1000
            latencies.append(elapsed*1000);cycles+=1
            raw_command=response.get("command")
            if not isinstance(raw_command,list) or len(raw_command)<2 or any(v is None or not math.isfinite(v) for v in raw_command):
                response["error"]=response.get("error","nonfinite_or_missing_command")
            reason=response.get("reason","native");reasons[reason]=reasons.get(reason,0)+1
            bad="error" in response or elapsed>.05
            if "error" in response:errors.append({"t":t,"error":response["error"]})
            if elapsed>.05:deadline+=1
            if not response.get("verified",True):no_action+=1
            command=cap(response.get("command",[0.,0.])[:2],.5) if not bad else [0.,0.]
            delay=.1 if not bad else elapsed+.05
            schedule=response.get("schedule",[[delay,*command]]) if not bad else [[delay,0.,0.]]
            previous=[(t+e[0],e[1:3]) for e in schedule]
            if last_schedule is not None:
                # Compare targets at equal absolute future epochs, not shifted indices.
                old_t,old_schedule=last_schedule
                def at(events,stamp):
                    applicable=[e[1:3] for e in events if e[0]<=stamp+1e-9]
                    return applicable[-1] if applicable else target
                errors_sq=[math.dist(at(schedule,.1+.2*k),at(old_schedule,t-old_t+.1+.2*k))**2 for k in range(10)]
                sequence_rms.append(math.sqrt(sum(errors_sq)/10))
            last_schedule=(t,schedule)
            predicted=project_trace(state,target,relative,schedule)
            if previous_trace is not None:
                old_epoch,old=previous_trace
                shift=round((t-old_epoch)/.01)
                common=min(50,len(old)-shift)
                if common>0 and max(math.dist(predicted[k],old[k+shift]) for k in range(common))>.15:churn+=1
            previous_trace=(t,predicted)
            # Counterfactual TTC includes queued commitments, then holds the current target.
            ttc=hold_ttc(state,target,actors,cfg["wall"],relative)
            if ttc is not None:
                finite_ttc.append(ttc);margin=ttc-delay
                min_ttc_margin=margin if min_ttc_margin is None else min(min_ttc_margin,margin)
            if response.get("safe_candidate_count",0) and response.get("aggregate_predicted_clearance",1) is not None and response.get("aggregate_predicted_clearance",1)<=.02:native_unsafe+=1
            max_expanded=max(max_expanded,response.get("expanded",0))
            queue.append((t+delay,command));queue.sort(key=lambda x:x[0])
            for j in range(2):
                sg=0 if abs(command[j])<.03 else (1 if command[j]>0 else -1)
                if sg and last_sign[j] and sg!=last_sign[j]:flips+=1
                if sg:last_sign[j]=sg
            interval=max(.05,math.ceil(elapsed/.01-1e-9)*.01)
            intervals.append(interval)
            record={"t":t,"state":state.copy(),"target_before":target.copy(),"queue_before":[[dt,*u] for dt,u in relative],"actors":actors,"command":command,"response":response,"hold_ttc_s":ttc}
            for k in range(round(interval/.01)):
                now=t+.01
                while queue and queue[0][0]<=t+.005+1e-9:_,target=queue.pop(0)
                old=state;state=advance(state,target);t=now
                a=[(state[2+j]-old[2+j])/.01 for j in range(2)]
                an=math.hypot(*a);accelerations.append(an);jerks.append(math.dist(a,last_acc)/.01);last_acc=a
                maxacc=max(maxacc,an);maxspeed=max(maxspeed,math.hypot(*state[2:]));path_length+=math.dist(old[:2],state[:2])
                speed=math.hypot(*state[2:]);speed_history.append(speed)
                if speed<.03:
                    wait+=.01;stop_run+=.01
                    if abs(stop_run-.3)<.005:stop_count+=1
                else:stop_run=0.
                clear=clearance(state,actors_at(cfg,t),cfg["wall"])
                if clear<min_clear:min_clear=clear;worst={"t":t,"state":state.copy(),"clearance_m":clear}
                near|=clear<.05
                if clear<0:collision=True
                if arrival is None and math.dist(state[:2],[4.,0.])<=.15:arrival=t
                if collision:break
            record["after"]=state.copy();record["minimum_clearance_so_far_m"]=min_clear
            trace.write(json.dumps(record,separators=(",",":"),allow_nan=False)+"\n")
            if collision:break
    finally:
        cpu1,rss=(consumer.usage() if consumer else (0,0))
        if consumer:consumer.close()
    completed=arrival is not None and t>=arrival+3-1e-6 and math.dist(state[:2],[4.,0.])<=.15
    result={"id":name,"scene":cfg["name"],"seed":cfg["seed"],"arm":arm,"config":cfg,"cycles":cycles,
      "collision":collision,"near_collision":near,"completion":completed,"safe_completion":completed and not collision and min_clear>=.05,
      "arrival_s":arrival if completed else None,"elapsed_s":t,"min_clearance_m":min_clear,"worst":worst,"path_length_m":path_length,
      "wait_s":wait,"stop_count":stop_count,"unnecessary_stop_s":None,"deadlock":None,
      "start_epoch_s":start_epoch,"duration_s":t-start_epoch,"sequence_target_rms_mps":stats(sequence_rms),
      "unknown_labels":"No cycle-specific counterfactual progress witness computed; global feasible witness does not label local deadlock or unnecessary stopping.",
      "command_sign_flips":flips,"trajectory_churn":churn,"replan_hz":cycles/(t-start_epoch),"acceleration_mps2":stats(accelerations),"jerk_mps3":stats(jerks),
      "max_speed_mps":maxspeed,"max_acceleration_mps2":maxacc,"latency_ms":stats(latencies),"update_interval_s":stats(intervals),
      "consumer_cpu_s":cpu1-cpu0,"cpu_cores_at_realtime":(cpu1-cpu0)/(t-start_epoch),"consumer_rss_peak_mib":rss,
      "harness_cpu_s":time.process_time()-own_cpu0,"offline_wall_s":time.perf_counter()-wall0,
      "hold_ttc_s_finite":stats(finite_ttc),"ttc_censored_cycles":cycles-len(finite_ttc),"min_hold_ttc_margin_s":min_ttc_margin,
      "deadline_misses":deadline,"no_verified_action":no_action,"reasons":reasons,"max_expanded":max_expanded,"errors":errors,
      "native_safe_raw_unsafe_aggregate_cycles":native_unsafe,
      "prediction":{"source_age_s":.1,"position_error_by_horizon_m":[0,0,0,0,0],"velocity_error_mps":0,"stale_rate":0,"invalid_rate":0,"continuity":1,"coverage":1,"scope":"exact generated CV state, not measured tracking performance"}}
    (out/f'{name}.json').write_text(json.dumps(result,indent=2)+"\n")
    print(name,"safe",result["safe_completion"],"collision",collision,"arrival",result["arrival_s"],"clear",round(min_clear,4),"p99_ms",round(result["latency_ms"]["p99"],2),flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument("--build",type=pathlib.Path,required=True);p.add_argument("--output",type=pathlib.Path,required=True)
    p.add_argument("--mode",choices=["witness","sanity","pilot","repeat"],required=True);p.add_argument("--scene",default="static");p.add_argument("--arm",default="search");p.add_argument("--seed",type=int,default=13)
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    affinity=os.sched_getaffinity(0);os.sched_setaffinity(0,{min(affinity)})
    results=[]
    if args.mode=="repeat":jobs=[(args.scene,args.seed,args.arm)]
    elif args.mode=="sanity":jobs=[("static",13,a) for a in P["arms"]]
    else:jobs=[(s,seed,a) for s in P["scenes"] for seed in P["seeds"] for a in (["witness"] if args.mode=="witness" else P["arms"])]
    for s,seed,a in jobs:results.append(trial(scene(s,seed),a,args.build,out))
    (out/"results.json").write_text(json.dumps(results,indent=2)+"\n")
    (out/"protocol.sha256").write_text(hashlib.sha256((ROOT/"protocol.json").read_bytes()).hexdigest()+"\n")

if __name__=="__main__":main()
