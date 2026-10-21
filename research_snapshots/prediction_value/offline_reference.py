"""D: finite offline physical witnesses. Not a controller or optimal bound."""
import json
import math
import pathlib
from run import advance,P
from scenes import parameters,truth,clearance,wall_clearance

def witness(config,wait,lateral):
    state=[0.,0.,0.,0.,0.,0.];target=[0.,0.,0.];queue=[];min_gap=math.inf;min_wall=math.inf;length=0.
    waypoints=[(1.,lateral),(3.,lateral),(4.,0.)] if lateral else [(4.,0.)]
    waypoint=0;records=[]
    for tick in range(400):
        t=tick*.1
        if t<wait:command=[0.,0.,0.]
        else:
            point=waypoints[waypoint]
            if math.dist(state[:2],point)<.15 and waypoint<len(waypoints)-1:
                waypoint+=1;point=waypoints[waypoint]
            distance=math.dist(state[:2],point)
            magnitude=min(.4,distance)
            command=[magnitude*(point[k]-state[k])/max(distance,1e-9) for k in (0,1)]+[0.]
        queue.append((t+.05,command))
        records.append([t,*state[:3]])
        for sub in range(10):
            now=t+(sub+.5)*.01
            while queue and queue[0][0]<=now:_,target=queue.pop(0)
            old=state.copy();advance(state,target,.01);length+=math.dist(old[:2],state[:2])
            gap=clearance(state,truth(config,t+(sub+1)*.01),config["radius"])
            if gap is not None:min_gap=min(min_gap,gap)
            min_wall=min(min_wall,wall_clearance(state,config["wall"]))
            if min_gap<.047 or min_wall<-.003:
                return {"success":False,"elapsed_s":t+.1,"body_min_clearance_m":min_gap,"wall_min_clearance_m":min_wall},records
            if math.dist(state[:2],(4.,0.))<=.15:
                return {"success":True,"arrival_s":t+.1,"body_min_clearance_m":None if min_gap==math.inf else min_gap,
                        "wall_min_clearance_m":min_wall,"path_length_m":length},records
    return {"success":False,"reason":"timeout","body_min_clearance_m":None if min_gap==math.inf else min_gap},records

def main():
    out=pathlib.Path(__file__).resolve().parent/"evidence"/"D"
    out.mkdir(exist_ok=False)
    all_results=[]
    for scene in P["scenes"]:
        for seed in P["formal_seeds"]:
            config=parameters(scene,seed);candidates=[];best=None;best_records=None
            # Frozen finite set, evaluated entirely offline using truth labels.
            for wait in range(0,19,2):
                for lateral in (0.,-.9,.9):
                    result,records=witness(config,wait,lateral)
                    result.update(wait_s=wait,lateral_m=lateral);candidates.append(result)
                    if result["success"] and (best is None or result["arrival_s"]<best["arrival_s"]):best=result;best_records=records
            row={"scene":scene,"seed":seed,"best_found":best,"finite_schedules":30,"feasible_count":sum(r["success"] for r in candidates),"candidates":candidates}
            all_results.append(row)
            if best_records:(out/f"{scene}_{seed}_witness.json").write_text(json.dumps(best_records)+"\n")
    (out/"results.json").write_text(json.dumps(all_results,indent=2)+"\n")
    print([(r["scene"],r["seed"],r["feasible_count"],(r["best_found"] or {}).get("arrival_s")) for r in all_results])

if __name__=="__main__":main()
