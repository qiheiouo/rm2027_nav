"""Direct checks of geometry, integrator, queued execution and fixture witnesses."""
import argparse
import json
import math
import pathlib
import random
from run import Consumer,project_trace
from world import advance,clearance,scene,static_grid

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--build",type=pathlib.Path,required=True);ap.add_argument("--output",type=pathlib.Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    c=Consumer(a.build/"search_consumer","search",a.output/"checks.log")
    rng=random.Random(20261025);distance_error=0.;state_error=0.
    try:
        for _ in range(100):
            s=[rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-.4,.4),rng.uniform(-.4,.4)]
            ob=[rng.uniform(-2,2),rng.uniform(-2,2),0.,0.,.3];u=[rng.uniform(-1,1),rng.uniform(-1,1)]
            r=c.call({"probe":True,"state":s,"actor":ob,"command":u})
            # Ignore workspace wall for isolated box/disc check.
            distance_error=max(distance_error,abs(r["distance"]-clearance(s,[ob],100,padding=.03)))
            state_error=max(state_error,max(abs(x-y) for x,y in zip(r["next"],advance(s,u))))
        assert distance_error<1e-12 and state_error<1e-12
        st=[0.,0.,0.,0.];grid,path=static_grid(scene("crossing",13))
        q={"mode":"search","state":st,"target":[0,0],"queue":[],"actors":[],"wall":2.,"grid":grid,"previous":[]}
        r=c.call(q);assert r["verified"] and r["command"][0]>0,r
        tr=project_trace(st,[0,0],[],r["schedule"])
        assert all(math.hypot(*p)<1e-12 for p in tr[:10])
        assert tr[11][0]>0
        # Known incoming disc will hit the immutable prefix. New command must not be certified.
        q["actors"]=[[.67,0,-.5,0,.3]]
        r=c.call(q);assert not r["prefix_safe"] and not r["verified"],r
        # Brake-only constant input gives analytic stopping distance v²/(2a).
        s=[0.,0.,.4,0.]
        for _ in range(50):s=advance(s,[0.,0.])
        assert abs(s[0]-.1)<1e-10 and abs(s[2])<1e-10
        result={"geometry_and_model_samples":100,"max_distance_error":distance_error,"max_state_error":state_error,"commit_delay_check":True,"unsafe_prefix_rejected":True,"analytic_brake_distance":s[0]}
        (a.output/"checks.json").write_text(json.dumps(result,indent=2)+"\n");print(result)
    finally:c.close()
if __name__=="__main__":main()
