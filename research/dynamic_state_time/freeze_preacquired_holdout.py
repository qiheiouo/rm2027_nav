"""Generate the complete holdout before any closed-loop run; never tunes from results."""
import argparse
import json
import math
from pathlib import Path
import random

CATEGORIES=['static','crossing','head_on','stop_start','sudden_turn','occlusion_reappearance','id_switch','prediction_error','stale_observation','multiple']


def generate():
    cases=[];rng=random.Random(20261026)
    cv={'static','crossing','head_on','multiple'}
    for ci,category in enumerate(CATEGORIES):
        paired={}
        for j in range(12):
            seed=10000+ci*100+j
            key=j//2 if category in cv else j
            if key not in paired:paired[key]=[round(rng.uniform(-.6,.6),4),round(rng.uniform(-.12,.12),4)]
            phase,lateral=paired[key]
            base=category if category in cv else ('multiple' if category=='id_switch' else 'head_on' if category=='stale_observation' else 'crossing')
            static=[[2.,lateral,0.,0.,.3]] if base=='static' else []
            dynamic={'static':[], 'crossing':[[2.,-2.5+.6*phase,0.,.6,.3]],
                     'head_on':[[5.-.45*phase,lateral*.25,-.45,0.,.3]],
                     'multiple':[[1.5,-2.2+.6*phase,0.,.6,.3],[2.7,2.8-.6*phase,0.,-.6,.3]]}[base]
            cfg=dict(name=base,seed=seed,phase=phase,wall=2.,static=static,dynamic=dynamic,goal=[4.,0.])
            mode='ideal_cv' if category in cv and j%2==0 else 'causal_imperfect'
            d=dict(kind='exact',latency_s=.1,start_s=2.,duration_s=0.,error_m=0.,error_mps=0.)
            events=[]
            if mode=='causal_imperfect' and category in cv:
                if category=='static':d.update(kind='position_bias',error_m=[.005,.01,.02][key%3])
                elif category=='head_on':d.update(kind='velocity_bias',error_mps=[.01,.03,.06][key%3])
                else:d.update(kind='latency',latency_s=[.05,.10,.15][key%3])
            if category=='stop_start':
                duration=[.2,.5,1.][j%3]
                events=[dict(actor=0,t=2.5,velocity=[0.,0.]),dict(actor=0,t=2.5+duration,velocity=[0.,.6])]
                d.update(kind='exact_piecewise',duration_s=duration)
            if category=='sudden_turn':
                angle=[-90,90,180][j%3]*math.pi/180
                events=[dict(actor=0,t=2.5,velocity=[-.6*math.sin(angle),.6*math.cos(angle)])]
                d.update(kind='exact_piecewise',turn_degrees=[-90,90,180][j%3])
            if category=='occlusion_reappearance':d.update(kind='occlusion',duration_s=[.15,.30,.60,.90][j%4])
            if category=='id_switch':d.update(kind='id_relabel' if j<6 else 'id_misassociate',duration_s=[.15,.30,.60][j%3])
            if category=='prediction_error':d.update(kind='velocity_bias',error_mps=[.01,.03,.06,.12][j%4],error_y_mps=-[.01,.03,.06,.12][j%4])
            if category=='stale_observation':d.update(kind='stale',duration_s=[.10,.20,.40,.80][j%4])
            cases.append(dict(id=f'{category}_{seed}',category=category,seed=seed,paired_initial_key=key,mode=mode,
                              scene=cfg,input=d,motion_events=events))
    return dict(stage='Research; frozen ten-class holdout',base_commit='7e04111f7fe8d9b0851bddfc0b8ef0eb68b684ea',
        generator_seed=20261026,case_count=120,arms=['B','C0','C1'],rollout_count=360,
        count_definition='120 paired scene/input episodes, each B/C0/C1 = 360 attempted rollouts. No extra ideal replay that leaks non-CV future.',
        registration='Earlier protocol named ten categories only. This file freezes the previously unspecified initial conditions and finite disturbances before any holdout run.',
        mode_counts={'ideal_cv':24,'causal_imperfect':96},
        pairing='Four CV categories have six paired initial states, each ideal and imperfect input. Six intrinsically nonideal categories have 12 causal episodes each.',
        preserved='Frozen search/checker/library/threshold/plant/lifecycle. Input and evaluator hooks only. Same initial robot state, .1s command delay, .05s control, .01s plant, limits, completion and cost for all arms.',
        disturbances='Fixed exogenous schedules, never conditioned on robot outcome; no noise re-sampling during a run. Source metadata passed honestly; no obstacle identities added to frozen consumer interface.',
        termination='Preserve frozen harness: C1 invalidation exits after current control slot; initialization unavailable does not roll out. Such episodes are incomplete and collision-censored, NEVER zero-collision successes. Do not invent fallback/recovery or extrapolate beyond observed prefix.',
        validity_audit='Audit fresh admissions and actual command ownership separately from stored certified=true. A stale certificate is not considered valid even if frozen code produces one. Distinguish immutable previously queued actions from newly dispatched invalid backup.',
        go={'collision':0,'minimum_clearance_m':.05,'completion':'not lower than B','benefit':'safe completion +10 percentage points OR paired traversal >=10% gain in two categories; persists under imperfect input', 'new_systematic_deadlock':False,'p99_ms':40,'max_ms':50,'miss_rate':.001,'rss_mib':256},
        modify='Stable ideal evidence but one bounded observation/uncertainty/stale interface issue prevents nonideal acceptance; or a clear margin boundary.',
        stop='Benefit confined to toy head-on, widespread nonideal collisions, frequent unavailable/termination and lower completion, or tuning/library/future truth/realtime failure needed.',
        review='Keep all prespecified results including invalidation-censored attempts. Inspect first C1 failure and make one unchanged confirmation; no tuning, no expansion/integration.',
        raw_policy='Raw JSONL/lifecycle/complete logs/build external; compact summaries/manifest/plots/source/config/report in Git.',cases=cases)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    with args.output.open('x') as f:json.dump(generate(),f,indent=2);f.write('\n')
