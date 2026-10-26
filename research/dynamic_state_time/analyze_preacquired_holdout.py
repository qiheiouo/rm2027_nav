"""Offline evidence analysis; does not feed evaluator truth back into consumers."""
import argparse
from collections import Counter,defaultdict
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
from reserve_run import at
from world import stats
from holdout_input import truth

ROOT=Path(__file__).resolve().parent
ARMS=('B','C0','C1')


def event(row,label):
    b=row['active_backup']
    return dict(label=label,t=row['t'],state=row['state'],target=row['target'],immutable_queue=row['immutable_queue'],
        proposal=row['proposed']['command'] if row['proposed'] else None,accepted=row['accepted'],reason=row['reason'],
        validity=row['validity'],source_epoch=row['observation']['source_epoch'],source_age_s=row['observation']['age_s'],
        backup={k:b[k] for k in ('id','epoch','lineage','acquired_epoch','switch_time','execution_cursor','backup_controls','safe_rest')} if b else None,
        input_actors=row['input_actors'],track_ids=row['observation']['track_ids'],
        executed=row['executed'][-1],executed_owner=row['executed_owners'][-1],total_cycle_ms=row['total_cycle_ms'])


def analyze_one(raw,case,directory):
    trace=directory/f"{raw['arm']}_normal.jsonl.gz"
    rows=[json.loads(x) for x in gzip.open(trace,'rt')] if trace.exists() else []
    c1=raw['arm']=='C1';n=len(rows);old=None;pending=[]
    metrics=Counter();invalidation=Counter(e['reason'] for e in raw['invalid']);rejections=Counter();replacement=Counter()
    lat=[];age=[];pos=[];vel=[];horizon={str(h):[] for h in (0.,.25,.5,1.,2.,3.)};missing=[]
    causal_errors=0;control_variation=0.;last_command=[0.,0.];switches=0;actual_backup=0;moving_backup=0
    for row in rows:
        t=row['t'];meta=row['observation'];b=row['active_backup'];stale=meta['age_s']>.15+1e-9
        age.append(meta['age_s']);lat.append(row['total_cycle_ms']);metrics['stale_input_cycles']+=int(stale)
        causal_errors+=int(meta['source_epoch']>t+1e-9)
        if row['proposed'] is not None and not row['accepted']:rejections[row['reason']]+=1
        for e in row['lifecycle']:
            if e['state']=='REPLACED':replacement[e['reason']]+=1
        # A newly generated certificate does not waive its own source-age assumption.
        admitted_valid=bool(b and row['candidate_available'] and not stale and row['reason'] in ('splice_old','new_backup'))
        active_valid=bool(b and not stale and (admitted_valid if row['accepted'] else row['validity']=='valid'))
        if c1:
            metrics['valid_active_cycles']+=int(active_valid)
            metrics['accepted_without_valid_backup']+=int(row['accepted'] and not admitted_valid)
            metrics['stale_active_certificate_claims']+=int(b is not None and stale)
            metrics['new_stale_backup_dispatches']+=int(stale and row['backup_dispatch'])
            metrics['rejection_old_retained']+=int(row['proposed'] is not None and not row['accepted'] and row['validity']=='valid' and b is not None)
            if row['validity'] not in ('valid','not_held'):
                metrics['invalidations_followed_by_new_certificate']+=int(admitted_valid)
                metrics['invalidations_without_fresh_new_certificate']+=int(not admitted_valid)
            if b:
                metrics['cursor_errors']+=int(b['execution_cursor']!=sum(e[0]<=t+1e-8 for e in b['controls']))
            if old is not None and b is not None and row['validity']=='valid' and not row['accepted']:
                metrics['retention_errors']+=int(b['id']!=old['id'] or b['controls']!=old['controls'] or b['switch_time']!=old['switch_time'])
        for e in pending:
            if e[0]>=t-1e-8:metrics['queue_retractions']+=int(e not in row['immutable_queue'])
        pending=row['immutable_queue'];old=b
        for ex,owner in zip(row['executed'],row['executed_owners']):
            expected=at(row['immutable_queue'],ex[0]-.005,row['target'])
            metrics['execution_queue_mismatch']+=int(math.dist(ex[1:3],expected)>1e-10)
            change=math.dist(ex[1:3],last_command);control_variation+=change
            switches+=int(change>1e-7);last_command=ex[1:3]
            if owner and owner['backup']:
                actual_backup+=1;moving_backup+=int(math.hypot(*ex[1:3])>1e-7)
                metrics['stale_backup_execution_steps']+=int(stale)
                metrics['backup_queue_steps_after_invalidation']+=int(row['validity'] not in ('valid','not_held'))
        # Evaluation only. Actor-index identity mapping is never passed to either C++ consumer.
        expected_actors=truth(case,t)
        missing.append(len(expected_actors)-len(row['input_actors']))
        for prediction,index in zip(row['input_actors'],meta['evaluation_actor_indices']):
            actual=expected_actors[index];pos.append(math.dist(prediction[:2],actual[:2]));vel.append(math.dist(prediction[2:4],actual[2:4]))
            for h in horizon:
                future=truth(case,t+float(h))[index]
                projected=[prediction[k]+prediction[k+2]*float(h) for k in (0,1)]
                horizon[h].append(math.dist(projected,future[:2]))
    init=raw.get('initialized');censored=raw['collision_observation_censored']
    clear=raw['minimum_clearance_m'];safe=bool(raw['completion'] and raw['collision'] is False and clear>=.05 and not censored)
    d=case['input'];r=dict(id=raw['id'],case_id=case['id'],category=case['category'],seed=case['seed'],arm=raw['arm'],input_mode=case['mode'],
        input_kind=d['kind'],latency_setting_s=d['latency_s'],velocity_error_setting_mps=math.hypot(d.get('error_mps',0),d.get('error_y_mps',0)),
        position_error_setting_m=d.get('error_m',0),duration_setting_s=d.get('duration_s',0),
        collision=raw['collision'],collision_censored=censored,completion=raw['completion'],safe_completion=safe,
        timeout=not raw['completion'] and raw.get('end_s',0)>=40 and not raw['collision'] and not censored,
        min_clearance_m=clear,observed_safety_violation=bool(raw['collision'] or clear is not None and clear<.05),
        arrival_s=raw['arrival_s'],end_s=raw.get('end_s',0),wait_s=raw.get('wait_s'),
        init_success=init,initialization_ms=raw.get('initialization_ms'),control_cycles=n,
        valid_backup_ratio=metrics['valid_active_cycles']/n if n and c1 else None,
        revalidated_ratio=raw.get('held_cycles',0)/n if n and c1 else None,
        invalidation_reasons=dict(invalidation),proposal_rejections=sum(rejections.values()),rejection_reasons=dict(rejections),
        replacements=sum(replacement.values()),replacement_reasons=dict(replacement),
        backup_executed_s=actual_backup*.01,moving_backup_s=moving_backup*.01,
        accepted_without_valid_backup=metrics['accepted_without_valid_backup'],stale_backup_execution_steps=metrics['stale_backup_execution_steps'],
        stale_active_claims=metrics['stale_active_certificate_claims'],new_stale_backup_dispatches=metrics['new_stale_backup_dispatches'],
        backup_queue_steps_after_invalidation=metrics['backup_queue_steps_after_invalidation'],
        rejection_old_retained=metrics['rejection_old_retained'],queue_retractions=metrics['queue_retractions'],
        invalidations_followed_by_new_certificate=metrics['invalidations_followed_by_new_certificate'],
        invalidations_without_fresh_new_certificate=metrics['invalidations_without_fresh_new_certificate'],
        execution_queue_mismatch=metrics['execution_queue_mismatch'],retention_errors=metrics['retention_errors'],cursor_errors=metrics['cursor_errors'],
        timeout_brakes=raw['timeout_brakes'],action_switches=switches,command_variation_mps=control_variation,
        ttc_margin_min_s=raw.get('hold_ttc_margin_s',{}).get('min'),sequence_rms_mean_mps=raw.get('sequence_rms_mps',{}).get('mean'),
        cycle_mean_ms=stats(lat)['mean'],cycle_p50_ms=stats(lat)['p50'],cycle_p95_ms=stats(lat)['p95'],cycle_p99_ms=stats(lat)['p99'],cycle_max_ms=stats(lat)['max'],
        deadline_misses=sum(v>50 for v in lat),deadline_miss_rate=sum(v>50 for v in lat)/n if n else None,
        consumer_cpu_s=raw.get('consumer_cpu_s'),consumer_rss_peak_sum_mib=raw.get('consumer_rss_peak_sum_mib'),harness_cpu_s=raw.get('harness_cpu_s'),
        age_mean_s=stats(age)['mean'],age_max_s=stats(age)['max'],position_error_p99_m=stats(pos)['p99'],position_error_max_m=stats(pos)['max'],
        velocity_error_max_mps=stats(vel)['max'],stale_input_rate=metrics['stale_input_cycles']/n if n else None,
        missing_actor_cycle_rate=sum(v>0 for v in missing)/n if n else None,causality_errors=causal_errors,
        worst=raw.get('worst'),lifecycle_counts=raw.get('lifecycle_counts',{}))
    r['failure_reasons']=[]
    if raw['collision']:r['failure_reasons'].append('collision')
    if clear is not None and clear<.05:r['failure_reasons'].append('clearance_below_0.05')
    if not raw['completion']:r['failure_reasons'].append('task_incomplete')
    if censored:r['failure_reasons'].append('invalidation_censored' if n else 'init_unavailable')
    if r['accepted_without_valid_backup'] or r['stale_active_claims'] or r['stale_backup_execution_steps']:r['failure_reasons'].append('stale_certificate_contract')
    if lat and (max(lat)>50 or stats(lat)['p99']>40 or r['deadline_miss_rate']>.001):r['failure_reasons'].append('realtime')
    if r['consumer_rss_peak_sum_mib'] is not None and r['consumer_rss_peak_sum_mib']>256:r['failure_reasons'].append('memory')
    selected=[]
    if rows:
        selected=[event(rows[-1],'last_record'),event(max(rows,key=lambda x:x['total_cycle_ms']),'worst_latency')]
        if raw.get('worst'):selected.append(event(min(rows,key=lambda x:abs(x['t']-raw['worst']['t'])),'worst_clearance'))
        failure=next((x for x in rows if x['validity'] not in ('valid','not_held')),None)
        if failure:selected.append(event(failure,'first_invalidation'))
    return r,dict(latency=lat,age=age,position_error=pos,velocity_error=vel,horizon=horizon),selected


def group(results,samples,category,mode,arm):
    selected=[r for r in results if r['arm']==arm and (category=='all' or r['category']==category) and (mode=='all' or r['input_mode']==mode)]
    if not selected:return None
    ids=[r['id'] for r in selected];lat=[v for id_ in ids for v in samples[id_]['latency']]
    def dist(k, filt=lambda r:True):return stats([r[k] for r in selected if r.get(k) is not None and filt(r)])
    total=len(selected);complete=sum(r['completion'] for r in selected);safe=sum(r['safe_completion'] for r in selected)
    reasons=Counter()
    for r in selected:reasons.update(r['invalidation_reasons'])
    return dict(episodes=total,completion=complete,safe_completion=safe,safe_completion_rate=safe/total,
        observed_collisions=sum(bool(r['collision']) for r in selected),collision_censored=sum(r['collision_censored'] for r in selected),
        observed_safety_violations=sum(r['observed_safety_violation'] for r in selected),observed_safety_violation_rate=sum(r['observed_safety_violation'] for r in selected)/total,
        timeouts=sum(r['timeout'] for r in selected),min_clearance_observed_prefix_m=dist('min_clearance_m'),
        min_clearance_completed_m=dist('min_clearance_m',lambda r:r['completion']),traversal_completed_s=dist('arrival_s'),wait_s=dist('wait_s'),
        ttc_margin_min_s=dist('ttc_margin_min_s'),cycle_ms=stats(lat),deadline_misses=sum(t>50 for t in lat),deadline_miss_rate=sum(t>50 for t in lat)/len(lat) if lat else None,
        cpu_s=dist('consumer_cpu_s'),rss_peak_sum_mib=dist('consumer_rss_peak_sum_mib'),init_success=sum(bool(r['init_success']) for r in selected),
        init_ms=dist('initialization_ms'),backup_availability=dist('valid_backup_ratio'),invalidation_reasons=dict(reasons),
        replacements=sum(r['replacements'] for r in selected),backup_executed_s=sum(r['backup_executed_s'] for r in selected),
        invalidations_followed_by_new_certificate=sum(r['invalidations_followed_by_new_certificate'] for r in selected),
        invalidations_without_fresh_new_certificate=sum(r['invalidations_without_fresh_new_certificate'] for r in selected),
        moving_backup_s=sum(r['moving_backup_s'] for r in selected),rejection_old_retained=sum(r['rejection_old_retained'] for r in selected),
        accepted_without_valid_backup=sum(r['accepted_without_valid_backup'] for r in selected),stale_active_claims=sum(r['stale_active_claims'] for r in selected),
        stale_backup_execution_steps=sum(r['stale_backup_execution_steps'] for r in selected),new_stale_backup_dispatches=sum(r['new_stale_backup_dispatches'] for r in selected),
        age_s=stats([v for id_ in ids for v in samples[id_]['age']]),
        position_error_m=stats([v for id_ in ids for v in samples[id_]['position_error']]),
        velocity_error_mps=stats([v for id_ in ids for v in samples[id_]['velocity_error']]),
        horizon_error_m={h:stats([v for id_ in ids for v in samples[id_]['horizon'][h]]) for h in ('0.0','0.25','0.5','1.0','2.0','3.0')})


def write_csv(path,rows):
    fields=list(rows[0])
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader()
        for row in rows:w.writerow({k:json.dumps(v,separators=(',',':')) if isinstance(v,(dict,list)) else v for k,v in row.items()})


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--plot',action='store_true');args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    primary=args.data/'holdout_120';protocol=json.loads((primary/'protocol.json').read_text())
    assert protocol==json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    raw=json.loads((primary/'results.json').read_text());assert len(raw)==360
    cases={c['id']:c for c in protocol['cases']}
    assert {(r['case_id'],r['arm']) for r in raw}=={(c,a) for c in cases for a in ARMS}
    results=[];samples={};events={}
    for r in raw:
        result,sample,selected=analyze_one(r,cases[r['case_id']],primary/r['id'])
        results.append(result);samples[r['id']]=sample;events[r['id']]=selected
    categories=list(dict.fromkeys(c['category'] for c in protocol['cases']))
    groups={}
    for c in ['all',*categories]:
        for m in ('all','ideal_cv','causal_imperfect'):
            for a in ARMS:
                g=group(results,samples,c,m,a)
                if g:groups[f'{c}/{m}/{a}']=g
    pairs=[]
    for case_id,case in cases.items():
        b=next(r for r in results if r['case_id']==case_id and r['arm']=='B');c=next(r for r in results if r['case_id']==case_id and r['arm']=='C1')
        pairs.append(dict(case_id=case_id,category=case['category'],input_mode=case['mode'],B_safe=b['safe_completion'],C1_safe=c['safe_completion'],
            C1_censored=c['collision_censored'],traversal_gain_fraction=1-c['arrival_s']/b['arrival_s'] if c['completion'] and b['completion'] else None))
    confirms=[]
    for directory in sorted(args.data.glob('confirm_*')):
        for raw_r in json.loads((directory/'results.json').read_text()):
            r,_,e=analyze_one(raw_r,cases[raw_r['case_id']],directory/raw_r['id']);confirms.append(r);events[directory.name]=e
    failed=[r for r in results if r['failure_reasons']]
    # Full trial table once in CSV. JSON stores group distributions, no duplicated raw trace.
    write_csv(args.output/'trials.csv',results);write_csv(args.output/'paired.csv',pairs)
    write_csv(args.output/'failures.csv',[{k:r[k] for k in ('id','category','input_mode','collision','collision_censored','completion','min_clearance_m','end_s','invalidation_reasons','accepted_without_valid_backup','stale_backup_execution_steps','failure_reasons','worst')} for r in failed])
    robustness={}
    for category in categories:
        for a in ARMS:
            grouped=defaultdict(list)
            for r in results:
                if r['category']==category and r['arm']==a:
                    key=(r['input_kind'],r['latency_setting_s'],r['velocity_error_setting_mps'],r['position_error_setting_m'],r['duration_setting_s'])
                    grouped[key].append(r)
            robustness[f'{category}/{a}']=[dict(kind=k[0],latency_s=k[1],velocity_error_mps=k[2],position_error_m=k[3],duration_s=k[4],n=len(v),complete=sum(r['completion'] for r in v),safe=sum(r['safe_completion'] for r in v),observed_collision=sum(bool(r['collision']) for r in v),censored=sum(r['collision_censored'] for r in v),minimum_observed_clearance_m=min(r['min_clearance_m'] for r in v if r['min_clearance_m'] is not None)) for k,v in grouped.items()]
    c1=groups['all/all/C1'];b=groups['all/all/B'];im=groups['all/causal_imperfect/C1'];ib=groups['all/causal_imperfect/B']
    time_gains={cat:stats([p['traversal_gain_fraction'] for p in pairs if p['category']==cat and p['traversal_gain_fraction'] is not None]) for cat in categories}
    gate=dict(no_collision_or_censoring=c1['observed_collisions']==0 and c1['collision_censored']==0,
        observed_prefix_clearance=c1['min_clearance_observed_prefix_m']['min']>=.05,
        full_episode_safety_proven=c1['observed_collisions']==0 and c1['collision_censored']==0 and c1['min_clearance_observed_prefix_m']['min']>=.05,
        completion=c1['completion']>=b['completion'],
        safety_gain_percentage_points=100*(c1['safe_completion_rate']-b['safe_completion_rate']),
        imperfect_gain_percentage_points=100*(im['safe_completion_rate']-ib['safe_completion_rate']),
        realtime=c1['cycle_ms']['p99']<=40 and c1['cycle_ms']['max']<=50 and c1['deadline_miss_rate']<=.001,
        memory=c1['rss_peak_sum_mib']['max']<=256,
        valid_commit=c1['accepted_without_valid_backup']==0 and c1['stale_active_claims']==0,
        observed_stale_backup_execution_zero=c1['stale_backup_execution_steps']==0)
    summary=dict(stage='Research; 120 paired cases / 360 attempted B,C0,C1 rollouts',base_commit=protocol['base_commit'],
        decision='Stop' if not gate['valid_commit'] and not gate['completion'] else 'Review gate evidence',
        decision_scope='Frozen implementation is not eligible for tracker/STVL integration; censored nonideal rollouts do not prove persistent backup intrinsically unsafe or deadlocked.',
        interpretation='Collision counts are observed-prefix counts; censored invalidations are failures, never proof of no collision. Pure relabel is not a real tracker.',
        environment=json.loads((primary/'environment.json').read_text()),gate=gate,groups=groups,robustness=robustness,
        total_failed_attempts=len(failed),confirmations=confirms,paired_traversal_gain_fraction=time_gains,
        worst={a:{'clearance':min((r for r in results if r['arm']==a and r['min_clearance_m'] is not None),key=lambda r:r['min_clearance_m']),
                  'latency':max((r for r in results if r['arm']==a and r['cycle_max_ms'] is not None),key=lambda r:r['cycle_max_ms'])} for a in ARMS},
        audit_totals={a:{k:sum(r[k] for r in results if r['arm']==a) for k in ('causality_errors','queue_retractions','execution_queue_mismatch','retention_errors','cursor_errors','timeout_brakes','backup_queue_steps_after_invalidation')} for a in ARMS},
        next_stage='NOT_STARTED; verdict in report; no integration')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=1)+'\n')
    chosen={}
    for cat in categories:
        for arm in ('B','C1'):
            members=[r for r in results if r['category']==cat and r['arm']==arm]
            for candidate in (min(members,key=lambda r:r['min_clearance_m'] if r['min_clearance_m'] is not None else -999),next((r for r in members if r['failure_reasons']),members[0])):
                chosen[candidate['id']]=events[candidate['id']]
    for k in events:
        if k.startswith('confirm_'):chosen[k]=events[k]
    (args.output/'failure_timeline.json').write_text(json.dumps(chosen,separators=(',',':'))+'\n')
    files=[]
    for directory in [primary,args.data/'sanity',*sorted(args.data.glob('confirm_*'))]:
        for p in sorted(directory.rglob('*')):
            if p.is_file():files.append(dict(path=str(p.relative_to(args.data)),size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    manifest=dict(dataset_id='rm2027-preacquired-backup-holdout-20261026',root=str(args.data.resolve()),
        filename_convention='<holdout_120|confirm_*>/<category>_<seed>_<B|C0|C1>/<arm>_normal.{json,jsonl.gz}; *_lifecycle.json; *_planner.log; *_reserve.log; protocol/environment/results.json; sanity/*',
        origin='Frozen synthetic causal observation adapter + original plant/search/reserve; GCC C++17 -O3 -DNDEBUG; single logical CPU',
        purpose='120 paired fixed holdout episodes, all failures/censored attempts/latency tails; original fixture parity and first failure confirmation',
        generated_or_real='generated; no real tracking or sensors; future truth evaluator only',required_experiment='freeze_preacquired_holdout.py then run_preacquired_holdout.sh sanity --sanity; holdout_120; unchanged first failure confirmation; analyze_preacquired_holdout.py',
        total_bytes=sum(p['size'] for p in files),files=files,excluded='build_cache, old raw archives; no full logs or raw in Git')
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=1)+'\n')
    if args.plot:plot(results,groups,categories,args.output)
    print(json.dumps(dict(gate=gate,failed=len(failed),raw_bytes=manifest['total_bytes']),indent=2))


def plot(results,groups,categories,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    colors={'B':'#9a416e','C0':'#b87418','C1':'#087e8b'}
    for arm,dx in zip(ARMS,(-.24,0,.24)):
        gs=[groups[f'{c}/all/{arm}'] for c in categories];x=[i+dx for i in range(10)]
        axes[0,0].bar(x,[g['safe_completion'] for g in gs],width=.23,color=colors[arm],label=arm)
        axes[0,1].bar(x,[g['collision_censored'] for g in gs],width=.23,color=colors[arm])
        axes[1,0].scatter(x,[g['min_clearance_observed_prefix_m']['min'] for g in gs],color=colors[arm],s=20)
        axes[1,1].scatter(x,[g['cycle_ms']['p99'] for g in gs],color=colors[arm],s=20)
    titles=['Safe completion / 12 (censored = failure)','Invalidation-censored attempts / 12','Worst observed-prefix clearance (m)','Pooled full-cycle p99 (ms)']
    for ax,title in zip(axes.flat,titles):
        ax.set_title(title,fontsize=10);ax.set_xticks(range(10),categories,rotation=50,ha='right',fontsize=7);ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);axes[0,0].set_ylim(0,13);axes[0,1].set_ylim(0,13)
    axes[1,0].axhline(.05,color='black',ls='--',lw=.8);axes[1,1].axhline(40,color='black',ls='--',lw=.8)
    fig.suptitle('120 paired holdout cases; invalidation is NOT a zero-collision success')
    fig.savefig(out/'holdout_comparison.png',dpi=120);plt.close(fig)


if __name__=='__main__':main()
