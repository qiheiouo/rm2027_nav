"""Paired offline analysis of the unchanged 120 cases; never a decision input."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
from analyze_preacquired_holdout import analyze_one, group, write_csv, event, ARMS
from world import stats

ROOT=Path(__file__).resolve().parent


def additional(raw,directory):
    path=directory/f"{raw['arm']}_normal.jsonl.gz"
    rows=[json.loads(x) for x in gzip.open(path,'rt')] if path.exists() else []
    c1=raw['arm']=='C1';counts=Counter();invalid_episodes=set();unavailable_run=0;max_unavailable=0
    for r in rows:
        stale=r['observation']['age_s']>.15+1e-9
        tr=r.get('certificate_transaction');active=bool(r['active_backup'])
        invalid=r['validity'] not in ('valid','not_held')
        if invalid:invalid_episodes.add(r['validity'])
        counts['stale_rejections']+=int(stale and r['proposed'] is not None and not r['accepted'])
        counts['stale_certificates_issued']+=int(stale and r['candidate_available'])
        counts['fresh_candidate_construction_failure']+=int(r['proposed'] is not None and not r['candidate_available'] and r['reason'] in ('unavailable','unsafe_prefix'))
        counts['candidate_generated']+=int(r['candidate_available'])
        counts['candidate_not_committed']+=int(r['candidate_available'] and not r['accepted'])
        if c1:
            authorized=bool(tr and tr['authorized'])
            counts['authorization_trace_mismatch']+=int(active!=authorized)
            counts['accepted_without_transaction_authority']+=int(r['accepted'] and (not authorized or stale))
            counts['unavailable_cycles']+=int(not authorized)
            unavailable_run=unavailable_run+1 if not authorized else 0
            max_unavailable=max(max_unavailable,unavailable_run)
            counts['same_cycle_replacement_after_invalidation']+=int(invalid and r['accepted'] and authorized)
            counts['new_dispatch_without_authority']+=int(not authorized and any(abs(e[0]-(r['t']+.1))<1e-8 for e in r['immutable_queue']))
            for ex,owner in zip(r['executed'],r['executed_owners']):
                if owner and owner['backup']:
                    counts['backup_steps_with_current_authority']+=int(authorized)
                    counts['backup_steps_without_current_authority']+=int(not authorized)
    last=rows[-1] if rows else None
    end=raw.get('end_s',0);complete=raw['completion'];collision=raw['collision']
    censored=raw['collision_observation_censored'];clear=raw.get('minimum_clearance_m')
    stopped_timeout=False
    if end>=40 and not complete and not collision and rows:
        tail=[r for r in rows if r['t']>=end-5.]
        # ex = [t, target_x, target_y, x, y, vx, vy]; use velocity, not position.
        stopped_timeout=bool(tail and all((ex[5]**2+ex[6]**2)**.5<.03 for r in tail for ex in r['executed']))
    if collision:outcome='collision';termination='collision_checker'
    elif censored:outcome='backup_unavailable';termination=raw.get('status','invalidation_censored')
    elif complete:
        outcome='safe_completion' if clear>=.05 else 'completion_clearance_violation'
        termination='goal_and_3s_hold'
    elif end>=40:
        if c1 and last and last['active_backup'] is None:outcome='timeout_backup_unavailable'
        elif stopped_timeout:outcome='stopped_timeout'
        else:outcome='timeout'
        termination='40s_task_limit'
    else:outcome='other_task_failure';termination='post_arrival_goal_not_held'
    return dict(outcome=outcome,termination_reason=termination,
                final_certificate_validity=last['validity'] if last else 'unavailable',
                invalidation_episode_reasons=sorted(invalid_episodes),
                stale_rejections=counts['stale_rejections'],stale_certificates_issued=counts['stale_certificates_issued'],
                candidate_generated=counts['candidate_generated'],candidate_not_committed=counts['candidate_not_committed'],
                replacement_construction_failures=counts['fresh_candidate_construction_failure'],
                unavailable_s=counts['unavailable_cycles']*.05,max_unavailable_run_s=max_unavailable*.05,
                same_cycle_replacement_after_invalidation=counts['same_cycle_replacement_after_invalidation'],
                backup_execution_authorized_s=counts['backup_steps_with_current_authority']*.01,
                backup_execution_precommitted_without_current_authority_s=counts['backup_steps_without_current_authority']*.01,
                authorization_trace_mismatch=counts['authorization_trace_mismatch'],
                accepted_without_transaction_authority=counts['accepted_without_transaction_authority'],
                new_dispatch_without_authority=counts['new_dispatch_without_authority'],
                stopped_timeout_proxy=stopped_timeout,
                consumer_cpu_per_simulated_second=raw.get('consumer_cpu_s',0)/end if end else None),rows


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True)
    ap.add_argument('--old-data',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--decision',choices=('Go','Modify','Stop'))
    ap.add_argument('--decision-reason',default='')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    primary=args.data/'holdout_120';oldroot=args.old_data/'holdout_120'
    protocol=json.loads((primary/'protocol.json').read_text())
    assert protocol==json.loads((oldroot/'protocol.json').read_text())==json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    cases={c['id']:c for c in protocol['cases']};raw=json.loads((primary/'results.json').read_text())
    old_raw=json.loads((oldroot/'results.json').read_text());old_by={(r['case_id'],r['arm']):r for r in old_raw}
    assert len(raw)==360 and {(r['case_id'],r['arm']) for r in raw}=={(c,a) for c in cases for a in ARMS}
    results=[];samples={};selected={};end_rows={}
    for r in raw:
        d,s,ev=analyze_one(r,cases[r['case_id']],primary/r['id'])
        extra,rows=additional(r,primary/r['id']);d.update(extra)
        # Preserve the observed execution counter but distinguish irrevocable past
        # queue execution from NEW invalid dispatch. Do not call the former valid.
        if d['stale_backup_execution_steps'] and not (d['accepted_without_valid_backup'] or d['stale_active_claims']):
            d['failure_reasons']=[x for x in d['failure_reasons'] if x!='stale_certificate_contract']
            d['failure_reasons'].append('precommitted_backup_executed_during_stale_input')
        if d['stale_certificates_issued'] or d['new_dispatch_without_authority'] or d['accepted_without_transaction_authority']:
            d['failure_reasons'].append('certificate_transaction_contract')
        results.append(d);samples[d['id']]=s
        if d['failure_reasons'] and d['arm']=='C1':
            selected[d['id']]=ev
            unavailable=next((i for i,r in enumerate(rows) if not r['certificate_transaction']['authorized']),None)
            if unavailable is not None:
                if unavailable: selected[d['id']].append(event(rows[unavailable-1],'before_first_unavailable'))
                selected[d['id']].append(event(rows[unavailable],'first_unavailable'))
        if rows:end_rows[d['id']]=event(rows[-1],'terminal_record')
    categories=list(dict.fromkeys(c['category'] for c in protocol['cases']))
    groups={}
    for cat in ['all',*categories]:
        for mode in ('all','ideal_cv','causal_imperfect'):
            for arm in ARMS:
                g=group(results,samples,cat,mode,arm)
                if not g:continue
                members=[r for r in results if r['arm']==arm and (cat=='all' or r['category']==cat) and (mode=='all' or r['input_mode']==mode)]
                g['outcomes']=dict(Counter(r['outcome'] for r in members))
                for k in ('stale_rejections','stale_certificates_issued','replacement_construction_failures',
                          'same_cycle_replacement_after_invalidation','unavailable_s','candidate_generated','candidate_not_committed',
                          'backup_execution_authorized_s','backup_execution_precommitted_without_current_authority_s',
                          'authorization_trace_mismatch','accepted_without_transaction_authority','new_dispatch_without_authority'):
                    g[k]=sum(r[k] for r in members)
                g['invalidation_episodes']={key:sum(key in r['invalidation_episode_reasons'] for r in members) for key in g['invalidation_reasons']}
                g['max_unavailable_run_s']=stats([r['max_unavailable_run_s'] for r in members])
                g['consumer_cpu_per_simulated_second']=stats([r['consumer_cpu_per_simulated_second'] for r in members if r['consumer_cpu_per_simulated_second'] is not None])
                g['harness_cpu_s']=stats([r['harness_cpu_s'] for r in members if r['harness_cpu_s'] is not None])
                g['completed_margin_above_005_m']=stats([r['min_clearance_m']-.05 for r in members if r['completion']])
                groups[f'{cat}/{mode}/{arm}']=g
    by={(r['case_id'],r['arm']):r for r in results};pairs=[];truncation=Counter();by_reason={}
    for cid,case in cases.items():
        old=old_by[cid,'C1'];new=by[cid,'C1'];censored=old['collision_observation_censored']
        old_outcome='truncated' if censored else ('collision' if old['collision'] else 'safe_completion' if old['completion'] and old['minimum_clearance_m']>=.05 else 'other_failure')
        reason=old['invalid'][0]['reason'] if old['invalid'] else 'goal_and_3s_hold' if old['completion'] else 'collision_checker'
        pairs.append(dict(case_id=cid,category=case['category'],input_mode=case['mode'],old_outcome=old_outcome,new_outcome=new['outcome'],
            old_termination_reason=reason,new_termination_reason=new['termination_reason'],new_final_certificate_validity=new['final_certificate_validity'],
            old_min_clearance_m=old['minimum_clearance_m'],new_min_clearance_m=new['min_clearance_m'],
            old_completion=old['completion'],new_completion=new['completion'],old_traversal_s=old['traversal_s'],new_traversal_s=new['arrival_s'],
            old_end_s=old['end_s'],new_end_s=new['end_s'],certificate_replacement_count=new['replacements'],replacement_construction_failures=new['replacement_construction_failures'],
            stale_rejection_count=new['stale_rejections'],backup_execution_s=new['backup_executed_s'],
            backup_execution_authorized_s=new['backup_execution_authorized_s'],unavailable_s=new['unavailable_s'],
            new_accepted_without_valid_backup=new['accepted_without_valid_backup'],new_stale_certificates_issued=new['stale_certificates_issued']))
        if censored:
            truncation[new['outcome']]+=1
            by_reason.setdefault(reason,Counter())[new['outcome']]+=1
    assert sum(truncation.values())==75
    c1=groups['all/all/C1'];b=groups['all/all/B'];im=groups['all/causal_imperfect/C1'];ib=groups['all/causal_imperfect/B']
    gains={cat:stats([1-r['arrival_s']/by[r['case_id'],'B']['arrival_s'] for r in results if r['arm']=='C1' and r['category']==cat and r['completion'] and by[r['case_id'],'B']['completion']]) for cat in categories}
    # Retain the original quantitative benefit threshold and paired observations.
    gate=dict(no_collision=c1['observed_collisions']==0,no_censoring=c1['collision_censored']==0,
        completed_clearance=c1['min_clearance_completed_m']['min']>=.05 if c1['completion'] else False,
        all_observed_clearance=c1['min_clearance_observed_prefix_m']['min']>=.05,
        completion_not_lower_than_B=c1['completion']>=b['completion'],
        safe_completion_gain_pp=100*(c1['safe_completion_rate']-b['safe_completion_rate']),
        imperfect_safe_completion_gain_pp=100*(im['safe_completion_rate']-ib['safe_completion_rate']),
        categories_with_mean_paired_time_gain_ge_10pct=[k for k,v in gains.items() if v['n'] and v['mean']>=.1],
        realtime=c1['cycle_ms']['p99']<=40 and c1['cycle_ms']['max']<=50 and c1['deadline_miss_rate']<=.001,
        memory=c1['rss_peak_sum_mib']['max']<=256,
        valid_commit=c1['accepted_without_valid_backup']==0 and c1['accepted_without_transaction_authority']==0,
        no_stale_issuance=c1['stale_certificates_issued']==0,
        no_invalid_dispatch=c1['new_dispatch_without_authority']==0,
        no_new_stale_claims=c1['stale_active_claims']==0)
    baseline_changes=[]
    for r in results:
        if r['arm'] not in ('B','C0'):continue
        old=old_by[r['case_id'],r['arm']]
        differences={k:[old[k],r[n]] for k,n in (('collision','collision'),('completion','completion'),('minimum_clearance_m','min_clearance_m'),('arrival_s','arrival_s')) if old[k]!=r[n]}
        if differences:baseline_changes.append(dict(id=r['id'],differences=differences))
    confirms=[]
    for directory in sorted(args.data.glob('confirm_*')):
        for r in json.loads((directory/'results.json').read_text()):
            d,_,ev=analyze_one(r,cases[r['case_id']],directory/r['id']);extra,_=additional(r,directory/r['id']);d.update(extra)
            confirms.append(d);selected[directory.name]=ev
    worst={a:{'clearance':min((r for r in results if r['arm']==a and r['min_clearance_m'] is not None),key=lambda r:r['min_clearance_m']),
              'latency':max((r for r in results if r['arm']==a and r['cycle_max_ms'] is not None),key=lambda r:r['cycle_max_ms'])} for a in ARMS}
    for a in ARMS:
        for r in worst[a].values():
            selected.setdefault(r['id'],[]).append(end_rows[r['id']])
    failed=[r for r in results if r['failure_reasons']]
    summary=dict(stage='Research paired before/after: same 120 cases, all 360 rollouts rerun',
        lifecycle_commit=json.loads((primary/'environment.json').read_text())['git_head'],
        old_holdout_commit='f1203c3949f8d48237a9ea0125c895f7f5a8ef2c',reuse='none',
        decision=args.decision or 'Unadjudicated',decision_reason=args.decision_reason,
        decision_scope='Frozen lifecycle implementation on the original declared 120-case scope; no automatic integration',gate=gate,groups=groups,
        old_truncations_destinations=dict(truncation),old_truncation_reason_destinations={k:dict(v) for k,v in by_reason.items()},
        paired_time_gains=gains,baseline_changes=baseline_changes,confirmations=confirms,worst=worst,
        failed_attempts=len(failed),environment=json.loads((primary/'environment.json').read_text()),
        audit_totals={a:{k:sum(r[k] for r in results if r['arm']==a) for k in ('causality_errors','queue_retractions','execution_queue_mismatch','retention_errors','cursor_errors','timeout_brakes','authorization_trace_mismatch','accepted_without_transaction_authority','new_dispatch_without_authority')} for a in ARMS},
        metric_notes='No future truth used for control. TTC is existing hold-command CV margin, not oracle collision time. Stopped-timeout is a final-5s velocity proxy, not a proven inevitable deadlock. Backup execution without current authority may be irrevocably prequeued; separately retained, never counted as currently certified.',
        process_resources=(args.data/'process_resources.txt').read_text())
    write_csv(args.output/'trials.csv',results);write_csv(args.output/'paired_before_after.csv',pairs)
    write_csv(args.output/'old_truncated_destinations.csv',[r for r in pairs if r['old_outcome']=='truncated'])
    write_csv(args.output/'failures.csv',[{k:r[k] for k in ('id','category','input_mode','outcome','termination_reason','collision','completion','min_clearance_m','end_s','failure_reasons','invalidation_reasons','stale_rejections','stale_certificates_issued','unavailable_s','worst')} for r in failed])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=1)+'\n')
    (args.output/'failure_timeline.json').write_text(json.dumps(selected,separators=(',',':'))+'\n')
    files=[]
    for p in sorted(args.data.rglob('*')):
        if p.is_file() and 'build_cache' not in p.parts:
            files.append(dict(path=str(p.relative_to(args.data)),size_bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    manifest=dict(dataset_id='rm2027-certificate-lifecycle-paired-holdout-rerun-20261026',root=str(args.data),
        filename_convention='holdout_120/<case>_<B|C0|C1>/<arm>_normal.{json,jsonl.gz}; *_lifecycle.json; *_planner.log; *_reserve.log; confirm_*/*; sanity/*',
        origin='frozen 120 scenarios from f1203c3, lifecycle e819e10; all arms newly executed',purpose='pair all 120 C1 outcomes and reveal old 75 truncations; retain every failure',
        generated_or_real='generated causal synthetic observations, not real sensors',required_experiment='run_preacquired_holdout.sh sanity --sanity; preacquired_holdout.py holdout_120; unchanged first-C1-failure confirmation',
        old_dataset_reference=str(args.old_data),total_bytes=sum(f['size_bytes'] for f in files),files=files,
        excluded='build_cache binaries; no old raw archive copied')
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=1)+'\n')
    print(json.dumps(dict(gate=gate,truncation=dict(truncation),raw_bytes=manifest['total_bytes']),indent=2))


if __name__=='__main__':main()
