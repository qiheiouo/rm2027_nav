"""Offline classification only. Causal features and screening never receive truth.

prepare/calibrate/evaluate are separate commands: evaluation labels are not loaded
by calibration. Existing closed-loop source, parameters and policies are untouched.
"""
import argparse
import copy
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from forensic_certificate_failures import (schedule_states, cv_array, clear_array,
    ray, projection, same_queued_prefix, write_csv)
from world import stats

ROOT=Path(__file__).resolve().parent
PROTOCOL=ROOT/'uncertainty_certificate_protocol.json'


def causal_observation(row):
    m=row['observation']
    return dict(t=row['t'],source=m['source_epoch'],receive=m['receive_time'],
                ids=m['track_ids'],states=m['source_states'])


class History:
    def __init__(self,config):
        self.cfg=config;self.tracks={};self.ids=None;self.last_change=None;self.last_jump=None
    def observe(self,o):
        t=o['t'];source=o['source'];assert source<=o['receive']+1e-9<=t+1e-9
        ids=o['ids'];assert len(ids)==len(o['states'])
        if self.ids is not None and set(ids)!=self.ids:self.last_change=t
        self.ids=set(ids);tracks=[]
        for id_,state in zip(ids,o['states']):
            previous=self.tracks.get(id_)
            residuals=list(previous['residuals']) if previous else []
            accelerations=list(previous['accelerations']) if previous else []
            if previous and source>previous['source']+1e-8:
                dt=source-previous['source'];old=previous['state']
                fd=[(state[j]-old[j])/dt for j in (0,1)]
                residual=math.dist(state[2:4],fd)
                delta=math.dist(state[2:4],old[2:4]);accel=delta/dt
                if delta>self.cfg['quality_velocity_jump_mps']:self.last_jump=t
                if residual<self.cfg['numeric_zero']:residual=0.
                if accel<self.cfg['numeric_zero']:accel=0.
                residuals=(residuals+[residual])[-self.cfg['history_window']:]
                accelerations=(accelerations+[accel])[-self.cfg['history_window']:]
            if not previous or source>=previous['source']-1e-8:
                self.tracks[id_]=dict(source=source,state=list(state),residuals=residuals,accelerations=accelerations)
            tracks.append(dict(id=id_,velocity=list(state[2:4]),residuals=residuals,accelerations=accelerations,
                residual_max=max(residuals,default=0.),acceleration_max=max(accelerations,default=0.),cold=not residuals))
        return dict(t=t,source=source,receive=o['receive'],age=t-source,tracks=tracks,obstacle_count=len(ids),
                    set_change_age=None if self.last_change is None else t-self.last_change,
                    velocity_jump_age=None if self.last_jump is None else t-self.last_jump,
                    cold=any(x['cold'] for x in tracks))


def robust_ray(state,actor,base_time,e):
    """Exact positive-clearance minimum of distance(box,CV ray)-e*(base+tau).
    Used only for a nominally safe ray. Unbounded growth is never horizon-clipped.
    """
    ordinary=ray(state,actor)
    if e==0:return ordinary
    vx,vy=actor[2:4];speed=math.hypot(vx,vy)
    if e>speed+1e-12:return -math.inf
    px,py=actor[0]-state[0],actor[1]-state[1]
    def value(t):
        qx,qy=abs(px+vx*t)-.325,abs(py+vy*t)-.3
        return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-actor[4]-e*(base_time+t)
    if ordinary<.05:return min(ordinary-e*base_time,value(0))
    if abs(e-speed)<=1e-12:
        if speed==0:return ordinary
        limit=(px*vx+py*vy)/speed-(.325*abs(vx)+.3*abs(vy))/speed-actor[4]-e*base_time
        return min(value(0),limit)
    def derivative(t):
        x,y=px+vx*t,py+vy*t;qx,qy=max(abs(x)-.325,0),max(abs(y)-.3,0)
        n=math.hypot(qx,qy)
        return (qx*math.copysign(1,x)*vx+qy*math.copysign(1,y)*vy)/n-e if n>0 else -e
    if derivative(0)>=0:return value(0)
    hi=1.
    while derivative(hi)<0 and hi<1e8:hi*=2
    if derivative(hi)<0:return -math.inf
    lo=0.
    for _ in range(60):
        mid=(lo+hi)/2
        if derivative(mid)<0:lo=mid
        else:hi=mid
    return min(value(0),value((lo+hi)/2))


def geometry(cert,target):
    ts,ss=schedule_states(cert['start_state'],target,cert['controls'],cert['epoch'],cert['safe_rest']['epoch'])
    assert math.dist(ss[-1],cert['safe_rest']['state'])<1e-7
    ages=ts-cert['epoch'];actors=cert['prediction']['actors'];wall=cert['assumptions']['wall']
    distances=[];rest=[]
    for a in actors:
        arr=cv_array([a],ts,cert['prediction']['epoch'])
        distances.append(clear_array(ss,arr,1e9))
        rest.append([a[0]+a[2]*ages[-1],a[1]+a[3]*ages[-1],*a[2:]])
    nominal=min([float((wall-np.abs(ss[:,1])-.3).min()),*[float(v.min()) for v in distances],*[ray(ss[-1],a) for a in rest]])
    return dict(ts=ts,states=ss,h=ages,distances=distances,rest=rest,wall_min=float((wall-np.abs(ss[:,1])-.3).min()),nominal=nominal)


def screen(g,features,method,param):
    if param==0:return g['nominal'],'disabled'
    if method in ('velocity_residual','acceleration_tube','track_quality') and features['cold']:
        return -math.inf,'cold_history'
    if method=='track_quality':
        bad=any(x is not None and x<param-1e-9 for x in (features['set_change_age'],features['velocity_jump_age']))
        return (-math.inf,'recent_track_change') if bad else (g['nominal'],'quality_clear')
    value=g['wall_min'];age=features['age'];h=g['h']
    assert len(features['tracks'])==len(g['rest'])
    for tr,dist,actor in zip(features['tracks'],g['distances'],g['rest']):
        if method=='velocity_residual':
            e=param*tr['residual_max'];rho=e*(age+h)
            tail=robust_ray(g['states'][-1],actor,age+h[-1],e)
        elif method=='acceleration_tube':
            a=param*tr['acceleration_max']
            if a>0:return -math.inf,'unbounded_acceleration_rest'
            rho=np.zeros(len(h));tail=ray(g['states'][-1],actor)
        elif method=='age_inflation':
            rho=param*math.hypot(*tr['velocity'])*age
            tail=ray(g['states'][-1],actor)-rho
        else:raise ValueError(method)
        value=min(value,float(np.min(dist-rho)),tail)
    return value,'tube_check'


def read_samples(path):
    with gzip.open(path,'rt') as f:return [json.loads(x) for x in f]


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def metrics(samples,method,param):
    key=str(param);safe=[s for s in samples if s['safe']];unsafe=[s for s in samples if not s['safe']]
    accepted=lambda s:s['scores'][method][key]['accepted']
    retained=sum(accepted(s) for s in safe);false_safe=sum(accepted(s) for s in unsafe)
    ideal=[s for s in samples if s['mode']=='ideal_cv']
    per=defaultdict(list)
    for s in samples:per[s['case_id']].append(s)
    safe_macro=[];unsafe_macro=[]
    for v in per.values():
        a=[s for s in v if s['safe']];b=[s for s in v if not s['safe']]
        if a:safe_macro.append(sum(accepted(s) for s in a)/len(a))
        if b:unsafe_macro.append(sum(not accepted(s) for s in b)/len(b))
    return dict(samples=len(samples),episodes=len(per),safe=len(safe),unsafe=len(unsafe),true_safe_retained=retained,false_safe=false_safe,
        unsafe_rejection=1-false_safe/len(unsafe) if unsafe else None,false_safe_rate=false_safe/len(unsafe) if unsafe else None,
        safe_retention=retained/len(safe) if safe else None,ideal_retention=sum(accepted(s) for s in ideal)/len(ideal) if ideal else None,
        unsafe_among_accepted=false_safe/(false_safe+retained) if false_safe+retained else None,
        safe_retention_episode_macro=stats(safe_macro),unsafe_rejection_episode_macro=stats(unsafe_macro),
        screen_ms=stats([s['scores'][method][key]['cost_ms'] for s in samples]))


def prepare(args,cfg):
    ids=cfg[args.split];out=args.data/(args.split+'_samples.jsonl.gz')
    assert not out.exists(), 'Do not overwrite a sealed split'
    args.data.mkdir(parents=True,exist_ok=True)
    if args.split=='evaluation':
        selected=json.loads((args.output/'selected_calibration.json').read_text())
        assert selected['protocol_sha256']==digest(PROTOCOL)
        params={m:[v['parameter']] for m,v in selected['per_method'].items()}
    else:params={m:v['parameters'] for m,v in cfg['methods'].items()}
    protocol=json.loads((args.raw/'holdout_120/protocol.json').read_text())
    assert protocol==json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    cases={c['id']:c for c in protocol['cases']};count=0;feature_cost=[]
    with gzip.open(out,'wt') as sink:
        for cid in ids:
            print(args.split,cid,flush=True)
            directory=args.raw/'holdout_120'/f'{cid}_C1'
            with gzip.open(directory/'C1_normal.jsonl.gz','rt') as f:rows=[json.loads(x) for x in f]
            life=json.loads((directory/'C1_normal_lifecycle.json').read_text())
            initial=next(x['certificate'] for x in life if 'certificate' in x)
            history=History(cfg);seen=set();previous=None
            for i,row in enumerate(rows):
                begin=time.perf_counter_ns();features=history.observe(causal_observation(row));feature_cost.append((time.perf_counter_ns()-begin)/1e6)
                candidates=([initial] if i==0 else [])+[b for b in (row['active_backup'],row['retained_invalid_backup']) if b]
                for cert in candidates:
                    if cert['id'] in seen:continue
                    seen.add(cert['id']);assert abs(cert['epoch']-row['t'])<1e-8
                    target=[0.,0.] if cert['id']==0 else row['target']
                    g=geometry(cert,target);scores={}
                    for method,values in params.items():
                        scores[method]={}
                        for param in values:
                            begin=time.perf_counter_ns();score,reason=screen(g,features,method,param);elapsed=(time.perf_counter_ns()-begin)/1e6
                            scores[method][str(param)]=dict(accepted=bool(score>=.05),clearance_m=float(score) if math.isfinite(score) else None,reason=reason,cost_ms=elapsed)
                    # Label path begins here. Neither label nor case enters History/screen.
                    label,_,_=projection(cases[cid],cert['start_state'],target,cert['controls'],cert['epoch'],cert['safe_rest']['epoch'],cert['prediction']['actors'],cert['epoch'])
                    assert abs(label['predicted_min_m']-g['nominal'])<1e-7
                    safe=bool(label['truth_schedule_min_m']>=.05)
                    prior={}
                    if not safe and previous is not None:
                        old,_,_=projection(cases[cid],row['state'],row['target'],previous['controls'],row['t'],max(row['t']+.94,previous['safe_rest']['epoch']),row['input_actors'],row['t'])
                        prior=dict(prefix_compatible=same_queued_prefix(row,previous),truth_min_m=float(old['truth_schedule_min_m']),was_authorized=row['validity']=='valid')
                    sample=dict(id=f'{cid}:{cert["id"]}',case_id=cid,category=cases[cid]['category'],mode=cases[cid]['mode'],split=args.split,
                        cert_id=cert['id'],t=cert['epoch'],horizon_s=float(g['h'][-1]),predicted_clearance_m=float(g['nominal']),truth_clearance_m=float(label['truth_schedule_min_m']),
                        safe=safe,features=features,scores=scores,prior_saved_backup=prior)
                    sink.write(json.dumps(sample,separators=(',',':'),allow_nan=False)+'\n');count+=1;previous=cert
    meta=dict(split=args.split,episodes=len(ids),certificates=count,protocol_sha256=digest(PROTOCOL),sample_sha256=digest(out),feature_update_ms=stats(feature_cost))
    (args.data/(args.split+'_extraction.json')).write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2))


def calibrate(args,cfg):
    assert not (args.output/'selected_calibration.json').exists(),'Calibration is a one-time selection'
    samples=read_samples(args.data/'calibration_samples.jsonl.gz')
    assert {s['case_id'] for s in samples}==set(cfg['calibration'])
    evaluations=[];best={}
    for method,info in cfg['methods'].items():
        valid=[]
        for param in info['parameters']:
            m=metrics(samples,method,param)
            success_cats={cat:metrics([s for s in samples if s['category']==cat],method,param)['safe_retention'] for cat in ('crossing','multiple')}
            ok=m['safe_retention']>=.95 and m['ideal_retention']>=.95 and all(v>=.90 for v in success_cats.values())
            entry=dict(method=method,parameter=param,feasible=ok,**m,successful_categories=success_cats)
            evaluations.append(entry)
            if ok:valid.append(entry)
        assert valid,'Disabled nominal comparator must be feasible'
        best[method]=max(valid,key=lambda x:(x['unsafe_rejection'],-x['parameter']))
    winner=max(best,key=lambda k:best[k]['unsafe_rejection'])
    selected=dict(protocol_sha256=digest(PROTOCOL),calibration_sha256=digest(args.data/'calibration_samples.jsonl.gz'),
        selected_before_evaluation_extraction=True,per_method={m:dict(parameter=v['parameter'],calibration_metrics=v) for m,v in best.items()},winner=winner)
    (args.output/'selected_calibration.json').write_text(json.dumps(selected,indent=2)+'\n')
    write_csv(args.output/'calibration_grid.csv',evaluations)
    print(json.dumps(dict(winner=winner,parameters={k:v['parameter'] for k,v in best.items()},rejections={k:v['unsafe_rejection'] for k,v in best.items()}),indent=2))


def evaluate(args,cfg):
    selected=json.loads((args.output/'selected_calibration.json').read_text());assert selected['protocol_sha256']==digest(PROTOCOL)
    samples=read_samples(args.data/'evaluation_samples.jsonl.gz');assert {s['case_id'] for s in samples}==set(cfg['evaluation'])
    results=[];categories=[];episode=[]
    for method,info in selected['per_method'].items():
        param=info['parameter'];results.append(dict(method=method,parameter=param,**metrics(samples,method,param)))
        for category in dict.fromkeys(s['category'] for s in samples):
            categories.append(dict(method=method,parameter=param,category=category,**metrics([s for s in samples if s['category']==category],method,param)))
        for cid in cfg['evaluation']:
            episode.append(dict(method=method,case_id=cid,**metrics([s for s in samples if s['case_id']==cid],method,param)))
    # Previously known 29 false-safe failure cases: descriptive paired audit.
    old=list(csv.DictReader((ROOT/'results/certificate_failure_forensics/failure_mechanisms.csv').open()))
    old29=[r for r in old if int(r['optimistic_certificates'])>0]
    calibration=read_samples(args.data/'calibration_samples.jsonl.gz');all_samples=calibration+samples
    coverage=[]
    for r in old29:
        cid=r['case_id'];ss=[s for s in all_samples if s['case_id']==cid]
        assert any(not s['safe'] for s in ss)
        for method,info in selected['per_method'].items():
            param=info['parameter'];bad=[s for s in ss if not s['safe']]
            hits=[s for s in bad if not s['scores'][method][str(param)]['accepted']]
            first=min(hits,key=lambda s:s['t']) if hits else None
            event=float(r['analysis_event_s']);prior=first['prior_saved_backup'] if first else {}
            escape=bool(first and prior.get('prefix_compatible') and prior.get('truth_min_m',-1)>=.05)
            coverage.append(dict(case_id=cid,category=r['category'],split=ss[0]['split'],method=method,unsafe_certificates=len(bad),rejected_unsafe=len(hits),
                first_correct_rejection_s=first['t'] if first else None,physical_event_s=event,
                lead_to_event_s=event-first['t'] if first else None,last_logged_safe_evidence_s=r['last_saved_remaining_executable_truth_safe_s'],
                prior_saved_escape_physically_feasible=escape,prior_authorized=prior.get('was_authorized'),
                early_and_known_escape=bool(first and event-first['t']>=.1 and escape)))
    timings={}
    for method in selected['per_method']:
        ev=[x for x in coverage if x['split']=='evaluation' and x['method']==method]
        timings[method]=dict(known_false_safe_failure_episodes=len(ev),detected=sum(x['rejected_unsafe']>0 for x in ev),
            early_with_saved_escape=sum(x['early_and_known_escape'] for x in ev),
            lead_s=stats([x['lead_to_event_s'] for x in ev if x['lead_to_event_s'] is not None]))
    winner=selected['winner'];wm=next(x for x in results if x['method']==winner)
    cats=[x for x in categories if x['method']==winner and x['unsafe']]
    timing=timings[winner];early=timing['early_with_saved_escape']/timing['known_false_safe_failure_episodes']
    success_cats=[x for x in categories if x['method']==winner and x['category'] in ('crossing','multiple')]
    go=wm['unsafe_rejection']>=.90 and wm['safe_retention']>=.95 and wm['ideal_retention']>=.95 and all(x['unsafe_rejection']>=.75 for x in cats) and all(x['safe_retention']>=.90 for x in success_cats) and early>=.8 and wm['screen_ms']['p99']<=10
    gate_verdict='Go-to-prototype' if go else 'Modify' if wm['unsafe_rejection']>=.5 and wm['safe_retention']>=.9 else 'Stop'
    verdict=args.decision or gate_verdict
    summary=dict(protocol_sha256=digest(PROTOCOL),historical_exposure=cfg['historical_exposure'],selected_method=winner,verdict=verdict,
        numerical_gate_verdict=gate_verdict,adjudication_reason=args.decision_reason,
        scope='Offline retrospective classification only; original closed-loop implementation remains Stop; no automatic prototype',results=results,categories=categories,
        timing=timings,groups=dict(calibration_episodes=40,evaluation_episodes=80,calibration_certificates=len(calibration),evaluation_certificates=len(samples)),
        field_notes=dict(false_safe_rate='accepted unsafe / all unsafe',safe_retention='accepted safe / all safe',
            timing='First rejection of unsafe saved certificate only; prior plan truth audit is evaluation-only; global last possible escape Unknown',
            complexity='Screening over already available certificate reference states; excludes trajectory reconstruction, truth labels, IO. Not a measured online 20Hz loop.'),
        feature_update_ms=json.loads((args.data/'evaluation_extraction.json').read_text())['feature_update_ms'])
    write_csv(args.output/'evaluation_methods.csv',results);write_csv(args.output/'evaluation_categories.csv',categories)
    write_csv(args.output/'evaluation_episodes.csv',episode);write_csv(args.output/'known29_coverage.csv',coverage)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    # Small representative samples; selected only after evaluation is sealed.
    representatives=[]
    for cid in cfg['evaluation']:
        ss=[s for s in samples if s['case_id']==cid]
        representatives.append(min(ss,key=lambda s:s['truth_clearance_m']))
    (args.output/'representative_samples.json').write_text(json.dumps(representatives,separators=(',',':'))+'\n')
    files=[dict(path=f.name,size_bytes=f.stat().st_size,sha256=digest(f)) for f in sorted(args.data.iterdir()) if f.is_file()]
    manifest=dict(dataset_id='rm2027-offline-uncertainty-certificates-20261027',root=str(args.data),origin=str(args.raw),
        generated_or_real='derived from frozen synthetic C1 traces; no new closed-loop experiment',purpose='episode-separated causal uncertainty classification',
        filename_convention='<calibration|evaluation>_samples.jsonl.gz and extraction.json',required_experiment='offline_uncertainty_certificate.py: prepare calibration -> calibrate -> prepare evaluation -> evaluate',
        protocol_sha256=digest(PROTOCOL),total_bytes=sum(x['size_bytes'] for x in files),files=files)
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    authorization_audit(args,cfg,selected)
    plot_results(args.output,results)
    print(json.dumps(dict(selected=winner,verdict=verdict,results=[{k:r[k] for k in ('method','parameter','safe','unsafe','false_safe_rate','safe_retention','ideal_retention')} for r in results],timing=timings),indent=2))


def authorization_audit(args,cfg,selected):
    directory=args.raw/'holdout_120/head_on_10201_C1'
    with gzip.open(directory/'C1_normal.jsonl.gz','rt') as f:rows=[json.loads(x) for x in f]
    history=History(cfg);records=[]
    for row in rows:
        features=history.observe(causal_observation(row))
        if row['t'] not in (5.05,5.10,5.15):continue
        old=row['active_backup'] or row['retained_invalid_backup']
        # Diagnostic re-evaluation of a SAVED suffix, not a new certificate issuance.
        b=copy.deepcopy(old);b['epoch']=row['t'];b['start_state']=row['state']
        b['prediction']['epoch']=row['t'];b['prediction']['actors']=row['input_actors']
        g=geometry(b,row['target'])
        label,_,_=projection(
            next(c for c in json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())['cases'] if c['id']=='head_on_10201'),
            row['state'],row['target'],b['controls'],row['t'],b['safe_rest']['epoch'],row['input_actors'],row['t'])
        for method,info in selected['per_method'].items():
            score,reason=screen(g,features,method,info['parameter'])
            records.append(dict(t=row['t'],method=method,parameter=info['parameter'],
                original_authorized=bool(row['active_backup']),original_reason=row['validity'],
                immutable_prefix_compatible=same_queued_prefix(row,old),nominal_min_m=g['nominal'],truth_min_m=label['truth_schedule_min_m'],
                uncertainty_min_m=score if math.isfinite(score) else None,uncertainty_accept=bool(score>=.05),reason=reason))
    write_csv(args.output/'authorization_diagnostic.csv',records)


def plot_results(output,results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels=['Velocity residual','Acceleration tube','Age inflation','Track quality']
    fig,ax=plt.subplots(figsize=(8,4.5),layout='constrained');x=np.arange(4);w=.25
    ax.bar(x-w,[100*r['unsafe_rejection'] for r in results],w,label='Unsafe certificate rejection')
    ax.bar(x,[100*r['safe_retention'] for r in results],w,label='True-safe retention')
    ax.bar(x+w,[100*r['ideal_retention'] for r in results],w,label='Ideal-CV retention')
    ax.set(xticks=x,xticklabels=labels,ylabel='Percent',ylim=(0,110),title='80-episode evaluation; parameters selected on 40 calibration episodes')
    ax.legend(fontsize=8,loc='lower left');ax.grid(axis='y',alpha=.15)
    fig.savefig(output/'screening_tradeoff.png',dpi=150);plt.close(fig)


def self_test(cfg):
    h=History(cfg);features=[]
    for i in range(8):
        t=i*.05
        features.append(h.observe(dict(t=t+.1,source=t,receive=t+.1,ids=[0],states=[[4-.45*t,0,-.44,0,.3]])))
    assert abs(features[-1]['tracks'][0]['residual_max']-.01)<1e-10
    frozen=copy.deepcopy(features)
    h.observe(dict(t=9,source=8.9,receive=9,ids=[999],states=[[999,2,7,6,.3]]))
    assert features==frozen
    original=dict(t=.1,observation=dict(source_epoch=0.,receive_time=.1,track_ids=[0],source_states=[[1,1,.2,0,.3]],evaluation_actor_indices=[888],kind='future_oracle'))
    different=copy.deepcopy(original);different['observation']['evaluation_actor_indices']=[999];different['observation']['kind']='other';different['truth']=[999]
    assert causal_observation(original)==causal_observation(different)
    rng=np.random.default_rng(714)
    for _ in range(100):
        actor=[float(rng.uniform(-2,2)),float(rng.uniform(.9,2)),float(rng.uniform(-.8,.8)),float(rng.uniform(.05,.5)),.3]
        speed=math.hypot(*actor[2:4]);e=speed*.2
        if ray([0,0,0,0],actor)<.05:continue
        analytic=robust_ray([0,0,0,0],actor,.3,e)
        ts=np.linspace(0,200,40001);a=cv_array([actor],ts,0)
        sampled=float(np.min(clear_array(np.zeros((len(ts),4)),a,1e9)-e*(.3+ts)))
        assert analytic<=sampled+1e-7 and sampled-analytic<.002,(analytic,sampled)
    assert robust_ray([0,0,0,0],[2,1,0,0,.3],.1,.01)==-math.inf
    print('PASS: causal prefix/metadata isolation, velocity-residual identity, 100 seeded ray comparisons, stationary expanding tube')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('command',choices=['test','prepare','calibrate','evaluate'])
    ap.add_argument('--split',choices=['calibration','evaluation']);ap.add_argument('--raw',type=Path);ap.add_argument('--data',type=Path);ap.add_argument('--output',type=Path)
    ap.add_argument('--decision',choices=['Go-to-prototype','Modify','Stop']);ap.add_argument('--decision-reason',default='')
    args=ap.parse_args();cfg=json.loads(PROTOCOL.read_text())
    assert not set(cfg['calibration'])&set(cfg['evaluation'])
    if args.command=='test':self_test(cfg);return
    assert args.data.resolve().is_relative_to(ROOT.parents[2]) is False
    args.output.mkdir(parents=True,exist_ok=True);os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    {'prepare':prepare,'calibrate':calibrate,'evaluate':evaluate}[args.command](args,cfg)


if __name__=='__main__':main()
