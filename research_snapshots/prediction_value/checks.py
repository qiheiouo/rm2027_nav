"""Minimal validity checks for attribution; not a repository regression suite."""
import argparse
import json
import pathlib
from run import Native
from providers import Sources
from scenes import clearance

def main():
    p=argparse.ArgumentParser();p.add_argument("--binary",required=True);p.add_argument("--output",required=True);a=p.parse_args()
    out=pathlib.Path(a.output)
    n=Native(a.binary,out/"checks_native.log")
    checks={}
    try:
        ready=n.call({"reset":13});m=ready["metadata"]
        assert abs(m["model_dt"]-.1)<1e-6 and m["time_steps"]==30 and m["batch_size"]==300 and m["shift"]
        checks["native_parameter_readback"]=m
        request={"state":[1.,0.,0.,.35,0.,0.],"wall":3.5,"radius":.36,"current":[2.,-1.],"prediction":None}
        n.call({"reset":13});first=n.call(request)
        n.call({"reset":13});second=n.call(request)
        for key in ("command","rollout","dynamic_cost_min","dynamic_cost_max"):
            assert first[key]==second[key],(key,first[key],second[key])
        checks["paired_native_noise_and_reset_exact"]=True
        current_probe=dict(request);current_probe["current"]=[1.8,0.]
        n.call({"reset":13});occupied=n.call(current_probe)
        n.call({"reset":13});no_current=dict(request);no_current["current"]=None
        free=n.call(no_current)
        assert "error" in occupied or occupied["command"]!=free["command"]
        checks["current_occupancy_constrains_native"]={"occupied":occupied.get("command",occupied.get("error")),"free":free["command"]}
        n.call({"reset":13})
        request["prediction"]=[[2.,-1.+(k+1)*.1] for k in range(30)]
        future=n.call(request)
        assert future["dynamic_cost_max"]>0
        assert future["command"]!=first["command"]
        checks["future_input_changes_native_command"]={"base":first["command"],"future":future["command"],"cost_max":future["dynamic_cost_max"]}
        n.call({"reset":13});request["prediction"]=[[2.,-1.]]
        rejected=n.call(request);assert "prediction grid" in rejected["error"]
        checks["malformed_grid_rejected"]=True
        source=Sources()
        for i in range(20):source.update((2.,-.6+.03*i),i*.1)
        pred=source.prediction("C_KF",2.,(2.,0.))
        assert pred is not None and pred[-1][1]>pred[0][1]
        assert source.prediction("C_KF",3.,(2.,0.)) is None
        checks["existing_KF_motion_and_stale_rejection"]=True
        assert abs(clearance([0,0,0],(1.,0.),.36)-.315)<1e-12
        assert clearance([0,0,0],(.2,0.),.36)<0
        checks["independent_signed_geometry"]=True
    finally:n.close()
    (out/"checks.json").write_text(json.dumps(checks,indent=2)+"\n")
    print(json.dumps(checks,indent=2))

if __name__=="__main__":main()
