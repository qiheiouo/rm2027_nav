"""Read-only, post-hoc forensics of stored C1 traces. No planner/checker calls.

Truth is evaluator-only. Saved command schedules are projected open-loop to
measure their conditional certificates; no episode is rerun or policy changed.
Detailed per-cycle derivatives stay outside Git. Small tables are exported.
"""
import argparse
import csv
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from world import advance, clearance, stats
from holdout_input import truth

ROOT = Path(__file__).resolve().parent
HORIZONS = (0., .25, .5, 1., 2., 3.)


def write_csv(path, rows):
    with path.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v,separators=(',',':')) if isinstance(v,(dict,list)) else v for k,v in row.items()})


def schedule_states(state, target, events, start, end):
    """Same 10 ms plant, replaying ONLY a recorded open-loop schedule."""
    state = list(state); target = list(target); events = sorted(events, key=lambda e:e[0])
    n = max(0, round((end-start)/.01)); values = [state.copy()]; j = 0
    for k in range(n):
        mid = start+(k+.5)*.01
        while j<len(events) and events[j][0]<=mid+1e-8:
            target=list(events[j][1:3]);j+=1
        state=advance(state,target);values.append(state)
    return start+np.arange(n+1)*.01, np.asarray(values)


def actor_array(case, ts):
    out=[]
    for index,a in enumerate(case['scene']['static']+case['scene']['dynamic']):
        base=np.asarray(a,float);arr=np.tile(base,(len(ts),1));arr[:,:2]+=ts[:,None]*base[2:4]
        prev=base[2:4].copy()
        for ev in case['motion_events']:
            if ev['actor']!=index:continue
            new=np.asarray(ev['velocity']);dt=np.maximum(0,ts-ev['t'])
            arr[:,:2]+=dt[:,None]*(new-prev)
            arr[ts>=ev['t']-1e-10,2:4]=new;prev=new
        out.append(arr)
    return out


def cv_array(actors,ts,epoch):
    out=[]
    for a in actors:
        arr=np.tile(np.asarray(a,float),(len(ts),1));arr[:,:2]+=(ts-epoch)[:,None]*arr[:,2:4];out.append(arr)
    return out


def clear_array(states, actors, wall, inflated=False):
    hx,hy=(.355,.33) if inflated else (.325,.3)
    val=wall-np.abs(states[:,1])-hy
    for a in actors:
        q=np.abs(a[:,:2]-states[:,:2])-np.array([hx,hy])
        v=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)-a[:,4]
        val=np.minimum(val,v)
    return val


def ray(state,a,hx=.325,hy=.3,duration=math.inf):
    """Analytic stationary rectangle / CV disk ray, same geometry as checker.
    Negative signed minima inside the rectangle need not be exact; positive ones are.
    """
    p=[a[0]-state[0],a[1]-state[1]];v=a[2:4];h=[hx,hy]
    cuts=[0.]
    for j in (0,1):
        if abs(v[j])>1e-14:
            for sign in (-1,1):
                t=(sign*h[j]-p[j])/v[j]
                if 0<t<duration:cuts.append(t)
    if math.isfinite(duration):cuts.append(duration)
    cuts=sorted(set(cuts));best=math.inf
    def value(t):
        q=[abs(p[j]+v[j]*t)-h[j] for j in (0,1)]
        return math.hypot(max(q[0],0),max(q[1],0))+min(max(q),0)-a[4]
    for k,lo in enumerate(cuts):
        hi=cuts[k+1] if k+1<len(cuts) else duration
        best=min(best,value(lo))
        mid=(lo+hi)/2 if math.isfinite(hi) else lo+1
        numer=denom=0.
        for j in (0,1):
            q=p[j]+v[j]*mid
            if abs(q)>h[j]:
                b=p[j]-math.copysign(h[j],q);numer+=v[j]*b;denom+=v[j]**2
        if denom>0:best=min(best,value(max(lo,min(hi,-numer/denom))))
    return best


def rest_clear(state,actors,wall,inflated=False):
    hx,hy=(.355,.33) if inflated else (.325,.3)
    return min([wall-abs(state[1])-hy,*[ray(state,a,hx,hy) for a in actors]])


def truth_rest(case,state,epoch):
    # Future truth only for a COUNTERFACTUAL execution of the recorded certificate.
    cuts=[epoch]+sorted(set(e['t'] for e in case['motion_events'] if e['t']>epoch+1e-9))
    best=case['scene']['wall']-abs(state[1])-.3
    for k,t in enumerate(cuts):
        duration=cuts[k+1]-t if k+1<len(cuts) else math.inf
        for a in truth(case,t):best=min(best,ray(state,a,duration=duration))
    return best


def projection(case,state,target,events,start,end,actors,actor_epoch):
    ts,ss=schedule_states(state,target,events,start,end);wall=case['scene']['wall']
    pred=cv_array(actors,ts,actor_epoch);actual=actor_array(case,ts)
    cp=clear_array(ss,pred,wall);ct=clear_array(ss,actual,wall);ci=clear_array(ss,pred,wall,True)
    lastactors=[[a[j]+a[j+2]*(ts[-1]-actor_epoch) for j in (0,1)]+list(a[2:]) for a in actors]
    rest=rest_clear(ss[-1],lastactors,wall);rest_i=rest_clear(ss[-1],lastactors,wall,True)
    actual_rest=truth_rest(case,ss[-1],float(ts[-1]))
    return dict(predicted_min_m=min(float(cp.min()),rest),predicted_rollout_min_m=float(cp.min()),
                predicted_inflated_min_m=min(float(ci.min()),rest_i),safe_rest_physical_m=rest,
                safe_rest_inflated_m=rest_i,truth_schedule_min_m=min(float(ct.min()),actual_rest),
                truth_rollout_min_m=float(ct.min()),truth_rest_m=actual_rest,
                terminal_state=ss[-1].tolist(),terminal_clearance_m=float(cp[-1]),
                remaining_lateral_m=float(ss[-1,1]-ss[0,1]),
                absolute_lateral_travel_m=float(np.abs(np.diff(ss[:,1])).sum()),
                progress_m=math.dist(state[:2],case['scene']['goal'])-math.dist(ss[-1,:2],case['scene']['goal'])),ts,ss


def decline_windows(rows,key,duration=.5,drop=.02):
    n=round(duration/.05);found=[]
    for i in range(n,len(rows)):
        w=rows[i-n:i+1];v=[r.get(key) for r in w]
        if any(x is None for x in v) or abs(w[-1]['t']-w[0]['t']-duration)>1e-6:continue
        if v[0]-v[-1]>=drop and sum(b<=a+1e-6 for a,b in zip(v,v[1:]))>=.8*n:
            found.append(w[0]['t'])
    return found


def last_time(rows,test):
    return next((r['t'] for r in reversed(rows) if test(r)),None)


def first_time(rows,test):
    return next((r['t'] for r in rows if test(r)),None)


def same_queued_prefix(row, certificate):
    # Earliest NEW command can execute at now+.10, never immediately.
    def control(events,t,u):
        for e in events:
            if e[0]<=t+1e-8:u=e[1:3]
        return u
    return all(math.dist(control(certificate['controls'],row['t']+(k+.5)*.01,row['target']),
                         control(row['immutable_queue'],row['t']+(k+.5)*.01,row['target']))<1e-7 for k in range(10))


def classify(case, evidence):
    """Evidence interpretation only, not a controller branch or a learned label."""
    category=case['category']
    if category=='head_on':
        primary='E: model-change-driven reserve authorization loss'
        secondary=['wrong CV','repeated accepted stop / delayed backup execution','backup unavailable']
        if evidence['optimistic_present_error']:secondary.append('optimistic certificate under biased CV')
        confidence='High on observed sequence; isolated causal share Unknown'
    else:
        primary='A: prediction / observation coverage mismatch'
        specific={'stop_start':'stop/start future change plus observation delay',
                  'occlusion_reappearance':'occluded actor removed from certificate world',
                  'id_switch':'velocity misassociation (not pure ID relabel)',
                  'prediction_error':'biased CV / optimistic certificate'}
        secondary=[specific[category]]
        if evidence['first_unavailable_s'] is not None:secondary.append('later backup unavailable')
        confidence='High on stored-schedule optimism; isolated causal share Unknown'
    return primary,secondary,confidence


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True)
    ap.add_argument('--derived',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();assert args.derived.resolve().is_relative_to(ROOT.parents[2]) is False
    args.derived.mkdir(parents=True,exist_ok=True);args.output.mkdir(parents=True,exist_ok=True)
    protocol=json.loads((args.data/'holdout_120/protocol.json').read_text())
    assert protocol==json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    cases={c['id']:c for c in protocol['cases']}
    raw=json.loads((args.data/'holdout_120/results.json').read_text())
    c1=[r for r in raw if r['arm']=='C1'];fail=[r for r in c1 if r['collision'] or r['minimum_clearance_m']<.05]
    assert len(fail)==31 and sum(r['collision'] for r in fail)==28
    failures={r['case_id'] for r in fail}
    controls={'head_on_10200','static_10000','crossing_10100','multiple_10900',
              'stop_start_10304','sudden_turn_10400','stale_observation_10801'}
    all_metrics=[];episode_rows=[];control_rows=[];timeline={};audits=Counter();mismatch=defaultdict(list)
    for raw_r in c1:
        cid=raw_r['case_id'];case=cases[cid];directory=args.data/'holdout_120'/raw_r['id']
        source=directory/'C1_normal.jsonl.gz'
        with gzip.open(source,'rt') as f:rows=[json.loads(line) for line in f]
        # Input-error cohort includes ALL 120 C1 cases, not only failures.
        for r in rows:
            t=r['t'];meta=r['observation'];actual=truth(case,t)
            phase='pre_event' if t<case['input']['start_s'] else 'post_event'
            if case['motion_events']:phase='pre_event' if t<case['motion_events'][0]['t'] else 'post_event'
            if meta['age_s']>.15+1e-9:phase='stale'
            missing=len(actual)-len(r['input_actors'])
            mismatch[(case['category'],phase,'missing_actor_count')].append(missing)
            for pred,index in zip(r['input_actors'],meta['evaluation_actor_indices']):
                for h in HORIZONS:
                    a=truth(case,t+h)[index]
                    err=math.hypot(pred[0]+pred[2]*h-a[0],pred[1]+pred[3]*h-a[1])
                    mismatch[(case['category'],phase,f'horizon_{h:g}_m')].append(err)
                mismatch[(case['category'],phase,'velocity_error_mps')].append(math.dist(pred[2:4],actual[index][2:4]))
        if cid not in failures|controls:continue
        print('Forensics',cid,flush=True)
        lifecycle=json.loads((directory/'C1_normal_lifecycle.json').read_text())
        initial=next(e['certificate'] for e in lifecycle if 'certificate' in e)
        certs={initial['id']:initial};targets={initial['id']:[0.,0.]};cache={}
        for r in rows:
            for b in (r['active_backup'],r['retained_invalid_backup']):
                if b and b['id'] not in certs:
                    assert abs(b['epoch']-r['t'])<1e-8
                    certs[b['id']]=b;targets[b['id']]=r['target']
        def cert_metric(b):
            if b['id'] not in cache:
                pr,ts,ss=projection(case,b['start_state'],targets[b['id']],b['controls'],b['epoch'],b['safe_rest']['epoch'],b['prediction']['actors'],b['prediction']['epoch'])
                error=math.dist(pr['terminal_state'],b['safe_rest']['state'])
                rest_error=abs(pr['safe_rest_inflated_m']-b['safe_rest']['cv_ray_clearance_m'])
                assert error<1e-7,(cid,b['id'],error)
                assert rest_error<1e-7,(cid,b['id'],rest_error)
                audits['certificates_reconstructed']+=1
                cache[b['id']]=(pr,ts,ss)
            return cache[b['id']]
        previous=initial;derived=[];dominance=[]
        for r in rows:
            t=r['t'];b=r['active_backup'];stored=b or r['retained_invalid_backup'];obs=r['observation']
            actual=truth(case,t);pred=r['input_actors'];miss=len(actual)-len(pred)
            pe=max([math.dist(a[:2],actual[i][:2]) for a,i in zip(pred,obs['evaluation_actor_indices'])],default=0.)
            ve=max([math.dist(a[2:4],actual[i][2:4]) for a,i in zip(pred,obs['evaluation_actor_indices'])],default=0.)
            pr=ss=None
            if stored:pr,ts,ss=cert_metric(stored)
            physical=[clearance(e[3:],truth(case,e[0]),case['scene']['wall']) for e in r['executed']]
            last=r['state'];err=0
            for e in r['executed']:
                err=max(err,math.dist(advance(last,e[1:3]),e[3:]));last=e[3:]
            assert err<1e-10;audits['execution_steps_checked']+=len(r['executed'])
            command=r['proposed']['command'] if r['proposed'] else None
            action='none' if command is None else 'stop' if math.hypot(*command)<1e-7 else 'lateral' if abs(command[1])>abs(command[0]) else 'forward' if command[0]>0 else 'reverse'
            point=dict(case_id=cid,t=t,source_epoch=obs['source_epoch'],receive_time=obs['receive_time'],age_s=obs['age_s'],
                certificate_id=b['id'] if b else None,retained_invalid_id=stored['id'] if not b and stored else None,
                certificate_source_version=stored['prediction']['version'] if stored else None,
                valid=bool(b),precommit_validity=r['validity'],replacement_reason=r['reason'],
                certificate_age_s=t-stored['epoch'] if stored else None,
                predicted_min_m=pr['predicted_min_m'] if b else None,
                predicted_rest_m=pr['safe_rest_physical_m'] if b else None,
                recorded_inflated_rest_m=stored['safe_rest']['cv_ray_clearance_m'] if stored else None,
                truth_schedule_min_m=pr['truth_schedule_min_m'] if b else None,
                predicted_terminal_clearance_m=pr['terminal_clearance_m'] if b else None,
                remaining_duration_s=max(0,stored['safe_rest']['epoch']-t) if b else None,
                remaining_lateral_m=stored['safe_rest']['state'][1]-r['state'][1] if b else None,
                backup_type=('lateral' if any(abs(e[2])>1e-7 for e in stored['backup_controls']) else 'braking_safe_rest') if stored else None,
                backup_switch_s=stored['switch_time'] if b else None,
                backup_cursor=stored['execution_cursor'] if stored else None,
                actual_backup_execution_s=sum(bool(o and o['backup']) for o in r['executed_owners'])*.01,
                physical_clearance_m=min(physical),physical_at_start_m=clearance(r['state'],actual,case['scene']['wall']),
                observed_position_error_m=pe,observed_velocity_error_mps=ve,missing_actors=miss,
                accepted=r['accepted'],proposal_action=action,proposal_command=command,
                candidate_available=r['candidate_available'],ttc_margin_s=r['hold_ttc_margin_s'],
                state=r['state'],truth_actors=actual,source_states=obs['source_states'],input_actors=pred,
                command=r['target'],executed_commands=[e[1:3] for e in r['executed']],queue=r['immutable_queue'],
                executed_terminal_state=r['executed'][-1][3:],dynamics_error=err,
                proposal_predicted_clearance_m=None,proposal_progress_m=None,
                old_current_model_min_m=None,new_current_model_min_m=None,old_truth_min_m=None,new_truth_min_m=None,
                old_better_by_m=None,immediate_progress_gain_m=None,old_still_authorized=r['validity']=='valid')
            ht,hs=schedule_states(r['state'],r['target'],r['immutable_queue'],t,t+3.)
            hc=clear_array(hs,actor_array(case,ht),case['scene']['wall'])
            hits=np.flatnonzero(hc[1:]<=0)
            point['truth_hold_ttc_3s']=float(ht[hits[0]+1]-t) if len(hits) else None
            if stored:
                rem,_,_=projection(case,r['state'],r['target'],stored['controls'],t,
                    max(t+.94,stored['safe_rest']['epoch']),pred,t)
                point['retained_remaining_current_cv_min_m']=rem['predicted_min_m']
                point['retained_remaining_truth_min_m']=rem['truth_schedule_min_m']
                point['retained_immutable_prefix_compatible']=same_queued_prefix(r,stored)
            else:
                point['retained_remaining_current_cv_min_m']=None
                point['retained_remaining_truth_min_m']=None
                point['retained_immutable_prefix_compatible']=False
            if r['proposed']:
                ev=[e for e in r['immutable_queue'] if e[0]<t+.1-1e-8]+[[t+e[0],*e[1:3]] for e in r['proposed']['schedule']]
                end=max([t+2.1,*[e[0] for e in ev]])+.94
                pp,_,_=projection(case,r['state'],r['target'],ev,t,end,pred,t)
                point['proposal_predicted_clearance_m']=pp['predicted_min_m'];point['proposal_progress_m']=pp['progress_m']
            if b and r['accepted'] and previous and b['id']!=previous['id']:
                # Alternatives from SAME measured state BEFORE the new command is committed.
                end=max(t+.94,previous['safe_rest']['epoch'])
                op,_,os=projection(case,r['state'],r['target'],previous['controls'],t,end,pred,t)
                np_,_,ns=projection(case,r['state'],r['target'],b['controls'],t,b['safe_rest']['epoch'],pred,t)
                _,short_old=schedule_states(r['state'],r['target'],previous['controls'],t,t+.15)
                _,short_new=schedule_states(r['state'],r['target'],b['controls'],t,t+.15)
                gain=math.dist(short_old[-1,:2],case['scene']['goal'])-math.dist(short_new[-1,:2],case['scene']['goal'])
                point.update(old_current_model_min_m=op['predicted_min_m'],new_current_model_min_m=np_['predicted_min_m'],
                    old_truth_min_m=op['truth_schedule_min_m'],new_truth_min_m=np_['truth_schedule_min_m'],
                    old_better_by_m=op['predicted_min_m']-np_['predicted_min_m'],immediate_progress_gain_m=gain)
                point['terminal_progress_gain_m']=np_['progress_m']-op['progress_m']
                if op['predicted_min_m']>=.05 and op['predicted_min_m']-np_['predicted_min_m']>=.02 and gain<=.01:
                    dominance.append(dict(t=t,old_authorized=r['validity']=='valid',old_pred_m=op['predicted_min_m'],new_pred_m=np_['predicted_min_m'],
                        old_truth_m=op['truth_schedule_min_m'],new_truth_m=np_['truth_schedule_min_m'],progress_gain_m=gain,
                        terminal_progress_gain_m=np_['progress_m']-op['progress_m'],action=action))
            derived.append(point)
            if stored:previous=stored
        # Certificate-level truth audit: logged validity is conditional on exact CV.
        issued=[]
        for bid,b in certs.items():
            pr,_,_=cert_metric(b);rr=next((r for r in rows if abs(r['t']-b['epoch'])<1e-8),rows[0])
            actual=truth(case,b['epoch']);actors=b['prediction']['actors'];indices=rr['observation']['evaluation_actor_indices']
            present_error=len(actors)!=len(actual) or any(math.dist(a[:4],actual[i][:4])>1e-7 for a,i in zip(actors,indices))
            issued.append(dict(id=bid,t=b['epoch'],predicted_min_m=pr['predicted_min_m'],truth_schedule_min_m=pr['truth_schedule_min_m'],
                               mismatch_at_issue=present_error,future_motion_event=any(e['t']>b['epoch']+1e-8 for e in case['motion_events'])))
        danger=[i for i in issued if i['predicted_min_m']>=.05 and i['truth_schedule_min_m']<.05]
        end=raw_r['end_s'];violation=next((d['t'] for d in derived if d['physical_clearance_m']<.05),end)
        pre=[d for d in derived if d['t']<=violation]
        end_scope=derived if raw_r['collision'] else pre
        last_valid=last_time(end_scope,lambda d:d['valid']);last_generated=last_time(end_scope,lambda d:d['candidate_available'])
        last_truth_safe=max([i['t'] for i in issued if i['t']<=violation and i['truth_schedule_min_m']>=.05],default=None)
        erosion={f'{duration:g}s_{drop:g}m':decline_windows(pre,'predicted_min_m',duration,drop) for duration in (.25,.5,1.) for drop in (.005,.02,.05)}
        predicted_onset=first_time(pre,lambda d:d['valid'] and d['truth_schedule_min_m']<.05 and d['predicted_min_m']>=.05)
        er=dict(case_id=cid,category=case['category'],outcome='collision' if raw_r['collision'] else 'low_clearance_completion' if cid in failures else 'safe_completion_control',
            primary_mechanism='UNADJUDICATED',secondary_mechanisms=[],confidence='pending',
            min_clearance_m=raw_r['minimum_clearance_m'],end_s=end,first_physical_violation_cycle_s=violation,
            analysis_event_s=end if raw_r['collision'] else violation,
            initial_backup_exists=raw_r['initialized'],last_valid_before_event_s=last_valid,
            last_constructed_before_event_s=last_generated,last_logged_truth_safe_certificate_s=last_truth_safe,
            last_valid_to_collision_s=end-last_valid if raw_r['collision'] and last_valid is not None else None,
            last_constructed_to_collision_s=end-last_generated if raw_r['collision'] and last_generated is not None else None,
            last_logged_truth_safe_to_collision_s=end-last_truth_safe if raw_r['collision'] and last_truth_safe is not None else None,
            first_current_input_mismatch_s=first_time(pre,lambda d:d['missing_actors']>0 or d['observed_position_error_m']>1e-7 or d['observed_velocity_error_mps']>1e-7),
            first_optimistic_certificate_s=predicted_onset,first_unavailable_s=first_time(pre,lambda d:not d['valid']),
            certificates=len(issued),optimistic_certificates=len(danger),optimistic_present_error=sum(d['mismatch_at_issue'] for d in danger),
            optimistic_future_change_only=sum(not d['mismatch_at_issue'] and d['future_motion_event'] for d in danger),
            optimistic_other=sum(not d['mismatch_at_issue'] and not d['future_motion_event'] for d in danger),
            predicted_certificate_min_m=min(i['predicted_min_m'] for i in issued),
            accepted_stop_cycles_before_violation=sum(d['accepted'] and d['proposal_action']=='stop' for d in pre),
            worsening_half_second_windows=len(erosion['0.5s_0.02m']),first_worsening_window_s=next(iter(erosion['0.5s_0.02m']),None),
            sustained_decline_lead_cycles=round(((end if raw_r['collision'] else violation)-erosion['0.5s_0.02m'][0])/.05) if erosion['0.5s_0.02m'] else None,
            replacement_safety_loss_small_progress=len(dominance),loss_when_old_authorized=sum(d['old_authorized'] for d in dominance),
            stop_replacement_safety_loss=sum(d['action']=='stop' for d in dominance),
            terminal_small_progress_safety_loss=sum(d['terminal_progress_gain_m']<=.01 for d in dominance),
            unauthorized_but_remaining_cv_safe_cycles=sum(not d['valid'] and d['retained_remaining_current_cv_min_m'] is not None and d['retained_remaining_current_cv_min_m']>=.05 for d in pre),
            unauthorized_but_remaining_truth_safe_cycles=sum(not d['valid'] and d['retained_remaining_truth_min_m'] is not None and d['retained_remaining_truth_min_m']>=.05 for d in pre),
            unauthorized_but_executable_truth_safe_cycles=sum(not d['valid'] and d['retained_immutable_prefix_compatible'] and d['retained_remaining_truth_min_m'] is not None and d['retained_remaining_truth_min_m']>=.05 for d in pre),
            last_saved_remaining_executable_truth_safe_s=last_time(pre,lambda d:d['retained_immutable_prefix_compatible'] and d['retained_remaining_truth_min_m'] is not None and d['retained_remaining_truth_min_m']>=.05),
            backup_executed_s=sum(d['actual_backup_execution_s'] for d in derived),
            dynamics_error_max=max(d['dynamics_error'] for d in derived),
            erosion_sensitivity={k:dict(count=len(v),first=next(iter(v),None)) for k,v in erosion.items()},
            key_replacements=dominance)
        if cid in failures:
            er['primary_mechanism'],er['secondary_mechanisms'],er['confidence']=classify(case,er)
            episode_rows.append(er)
        else:
            er['primary_mechanism']='successful diagnostic control';er['confidence']='Observed safe completion'
            control_rows.append(er)
        markers={0.,violation,last_valid,last_generated,last_truth_safe,predicted_onset,er['first_current_input_mismatch_s'],er['first_unavailable_s'],end-.05}
        markers.update(e['t']+offset for e in case['motion_events'] for offset in (0,.1,.15))
        if cid=='head_on_10201':markers.update((2.05,2.1,2.5,3.,3.5,3.55,4.,4.5,5.,5.05,5.1,6.3,6.5))
        timeline[cid]=[d for i,d in enumerate(derived) if i in {min(range(len(derived)),key=lambda j:abs(derived[j]['t']-t)) for t in markers if t is not None}]
        all_metrics.extend(derived)
        (args.derived/f'{cid}_certificates.json').write_text(json.dumps(issued,separators=(',',':'))+'\n')
        with (args.derived/f'{cid}_cycles.jsonl').open('w') as f:
            for d in derived:f.write(json.dumps(d,separators=(',',':'))+'\n')
    write_csv(args.output/'failure_mechanisms.csv',episode_rows)
    write_csv(args.output/'successful_controls.csv',control_rows)
    errors=[dict(category=k[0],phase=k[1],metric=k[2],**stats(v)) for k,v in mismatch.items()]
    write_csv(args.output/'model_error_by_phase.csv',errors)
    (args.output/'representative_timelines.json').write_text(json.dumps({k:v for k,v in timeline.items() if k in ('head_on_10201','stop_start_10300','occlusion_reappearance_10502','prediction_error_10703','id_switch_10608','sudden_turn_10400','stale_observation_10801')},separators=(',',':'))+'\n')
    summary=dict(analysis='Post-hoc forensics; no episode rerun; no planner/checker invocation',archive_commit='0633dc5a105cad751e602f6cbc19cd7f17b39090',
        adjudication='Go-to-next-hypothesis for a bounded uncertainty/certificate-coverage study only; current navigation implementation remains Stop',
        attribution_limit='Primary labels are mechanistic interpretations, not isolated causal effects; no intervention or new closed-loop test performed',
        failed_episodes=len(episode_rows),collision=28,low_clearance=3,controls=sorted(controls),audits=dict(audits),
        cycles=len(all_metrics),episodes_with_optimistic_certificates=sum(e['optimistic_certificates']>0 for e in episode_rows),
        episodes_with_present_error_optimism=sum(e['optimistic_present_error']>0 for e in episode_rows),
        episodes_with_future_change_optimism=sum(e['optimistic_future_change_only']>0 for e in episode_rows),
        episodes_with_reserve_decline=sum(e['worsening_half_second_windows']>0 for e in episode_rows),
        episodes_with_safety_loss_small_progress=sum(e['replacement_safety_loss_small_progress']>0 for e in episode_rows),
        episodes_with_authorized_old_safety_loss=sum(e['loss_when_old_authorized']>0 for e in episode_rows),
        successful_controls_with_decline=sum(e['worsening_half_second_windows']>0 for e in control_rows),
        successful_controls_with_small_progress_safety_loss=sum(e['replacement_safety_loss_small_progress']>0 for e in control_rows),
        mechanism_counts=dict(Counter(e['primary_mechanism'] for e in episode_rows)),
        episodes_unauthorized_saved_plan_still_executable_truth_safe=int(sum(e['unauthorized_but_executable_truth_safe_cycles']>0 for e in episode_rows)),
        method_notes=dict(predicted_clearance='Reconstructed physical geometry .325/.300; stored certificate safe-rest uses inflated .355/.330. Never compare inflated raw number directly to 0.05.',
            truth_schedule='Counterfactual execution of RECORDED controls then stationary rest; future truth evaluation only, no decision input. Not actual execution, not proof alternative would complete task.',
            last_escape='Only last logged certificate or saved suffix whose recorded schedule passes truth audit. Suffix must preserve the first .10s immutable queue. No exhaustive library probe or viability claim.',
            dominance='Same measured start; old remaining schedule vs new controls under current CV. Loss>=.02m and first .15s goal-progress gain<=.01m. Diagnostic post-hoc thresholds; no policy.',
            erosion='11 consecutive 50ms points, valid certificate min drops>=.02m in .5s and >=80% steps nonincreasing. Sensitivity .25/.5/1s and .005/.02/.05m.',
            missing='No sudden-turn or stale failures exist. Use successful controls; missing actors are recorded separately, not zero prediction error.',
            proposed='Planner did not log clearance/progress scores. Reconstructed its saved schedule plus brake/rest; this is diagnostic, not its internal score.' ))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    files=[]
    for f in sorted(args.derived.iterdir()):
        if f.is_file():files.append(dict(path=f.name,size_bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
    manifest=dict(dataset_id='rm2027-certificate-failure-forensics-20261027',root=str(args.derived),origin=str(args.data),
        source_manifest=str(ROOT/'results/certificate_holdout_rerun/dataset_manifest.json'),
        purpose='per-control-cycle reconstruction of 31 failures plus seven successful controls; evaluator-only counterfactual saved schedules',
        generated_or_real='derived synthetic evidence; not new experiment',filename_convention='<case>_cycles.jsonl and <case>_certificates.json',
        required_experiment='none; run forensic_certificate_failures.py on archived data',files=files,total_bytes=sum(f['size_bytes'] for f in files))
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    plot_head_on(args.derived,args.output)
    print(json.dumps(summary,indent=2))


def plot_head_on(derived,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[json.loads(x) for x in (derived/'head_on_10201_cycles.jsonl').open()]
    t=[r['t'] for r in rows]
    def series(k):return [np.nan if r.get(k) is None else r[k] for r in rows]
    fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True,layout='constrained')
    axes[0].plot(t,series('predicted_min_m'),label='Certificate: predicted physical clearance')
    axes[0].plot(t,series('truth_schedule_min_m'),label='Saved schedule: offline truth clearance',ls='--')
    axes[0].axhline(.05,color='red',lw=1,label='0.05 m gate')
    axes[0].set(ylabel='Clearance (m)',ylim=(0,.15));axes[0].legend(fontsize=8,loc='upper left')
    axes[1].plot(t,series('physical_clearance_m'),label='Actually executed: physical clearance')
    axes[1].axhline(.05,color='red',lw=1);axes[1].set(ylabel='Clearance (m)');axes[1].legend(fontsize=8)
    axes[2].plot(t,series('ttc_margin_s'),label='Online CV hold TTC margin (3 s cap)')
    axes[2].plot(t,series('truth_hold_ttc_3s'),ls='--',label='Offline truth hold TTC (3 s cap)')
    axes[2].set(xlabel='Episode time (s)',ylabel='Time (s)');axes[2].legend(fontsize=8)
    for ax in axes:
        ax.axvline(5.10,color='black',ls=':',lw=1)
        ax.axvspan(5.10,6.51,color='red',alpha=.07)
        ax.grid(alpha=.2);ax.set_xlim(0,6.55)
    axes[0].set_title('head_on_10201: certificate authority lost at 5.10 s; collision at 6.51 s')
    fig.savefig(output/'head_on_reserve.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
