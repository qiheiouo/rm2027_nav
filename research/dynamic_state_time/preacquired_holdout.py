"""120 frozen scene/input pairs, complete B/C0/C1; no tuning or integration."""
import argparse
import gzip
import json
import os
from pathlib import Path
import platform
import subprocess
from reserve_run import trial, FIXTURE
from preacquired_run import initial_gate
from holdout_input import CausalInput, check_inputs

ROOT=Path(__file__).resolve().parent


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--build',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--sanity',action='store_true');ap.add_argument('--confirm',nargs=2,metavar=('CASE_ID','ARM'))
    args=ap.parse_args();protocol=json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    assert len(protocol['cases'])==120
    args.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    environment=dict(platform=platform.platform(),python=platform.python_version(),affinity=sorted(os.sched_getaffinity(0)),
                     compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
                     git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    (args.output/'environment.json').write_text(json.dumps(environment,indent=2)+'\n')
    if args.sanity:
        checks=check_inputs(protocol['cases'])
        assert initial_gate(args.build,args.output)
        case=dict(id='sanity_original_head_on_13',scene=FIXTURE['scene'],motion_events=[],
                  input=dict(kind='exact',latency_s=.1,start_s=2.,duration_s=0.))
        r=trial(args.build,args.output,'C1','normal',full_episode=True,cfg=case['scene'],observation_source=CausalInput(case))
        archived=json.loads((ROOT/'results/preacquired/summary.json').read_text())['results']
        old=next(x for x in archived if x['run']=='normal_01' and x['arm']=='C1')
        for key in ('collision','completion','arrival_s','minimum_clearance_m','backup_dispatches','held_cycles'):
            assert r[key]==old[key],(key,r[key],old[key])
        checks['original_fixture_outcome_parity']=True
        (args.output/'sanity.json').write_text(json.dumps(checks,indent=2)+'\n')
        return
    cases=protocol['cases'];jobs=[(c,a) for c in cases for a in protocol['arms']]
    if args.confirm:
        jobs=[(c,a) for c,a in jobs if (c['id'],a)==tuple(args.confirm)];assert len(jobs)==1
    results=[]
    for i,(case,arm) in enumerate(jobs):
        out=args.output/f"{case['id']}_{arm}";out.mkdir()
        print(f"{i+1}/{len(jobs)} {case['id']} {case['mode']} {arm}",flush=True)
        r=trial(args.build,out,arm,'normal',full_episode=True,cfg=case['scene'],observation_source=CausalInput(case))
        r.update(id=out.name,case_id=case['id'],category=case['category'],seed=case['seed'],input_mode=case['mode'],disturbance=case['input'])
        r['terminated_on_invalidation']=r.get('terminated_on_invalidation',False)
        r['collision_observation_censored']=bool(r['terminated_on_invalidation'] or r.get('status')=='initialization_unavailable')
        results.append(r)
        (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
