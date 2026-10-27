"""Offline lease diagnostics. No planner, certificate issuance or executor.

Candidate clocks consume only current certificate/observation/mode history.
Truth is used after clock calculation for evaluation, never for expiry/renewal.
"""
import argparse
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from forensic_certificate_failures import (schedule_states, actor_array, clear_array,
    ray, projection, same_queued_prefix, write_csv)
from holdout_input import truth
from offline_information_mode import observed
from world import stats

ROOT=Path(__file__).resolve().parent
CONFIG=ROOT/'safety_lease_protocol.json'


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read_gz(p):
    with gzip.open(p,'rt') as f:
        return [json.loads(x) for x in f]


def control(events,t,initial):
    u=list(initial)
    for e in events:
        if e[0]<=t+1e-8:u=list(e[1:3])
    return u


def dispatch_deadline(b,initial,cfg):
    """Latest NEW commit for unchanged absolute B, not model validity expiry."""
    main=control(b['controls'],b['switch_time']-cfg['plant_dt_s']/2,initial)
    event=next((e for e in b['backup_controls'] if math.dist(e[1:3],main)>cfg['numerical_tolerance']),None)
    return max(b['epoch'],event[0]-cfg['command_delay_s']) if event else None


def candidate_clocks(b,initial,o,mode,past_span,cfg):
    now=b['epoch']
    stable=mode=='STABLE_TRACK'
    return dict(horizon=now+cfg['nominal_horizon_s'],
                dispatch_deadline=dispatch_deadline(b,initial,cfg),
                information_quality=max(now,o['source']+cfg['freshness_s']) if stable else now,
                past_consistency=now+min(past_span,cfg['nominal_horizon_s']) if stable else now)


def first_rest_risk(case,state,start):
    """Evaluation ONLY: first 5cm violation of a fixed resting robot.

    In each future motion interval the ray minimum is monotone in interval
    length, so bisection locates first entry; future changes stay evaluator-only.
    """
    cuts=[start]+sorted(set(e['t'] for e in case['motion_events'] if e['t']>start+1e-9))
    best=math.inf
    for k,t in enumerate(cuts):
        duration=cuts[k+1]-t if k+1<len(cuts) else math.inf
        for a in truth(case,t):
            if ray(state,a,duration=duration)>=.05:continue
            hi=duration
            if not math.isfinite(hi):
                hi=1.
                while ray(state,a,duration=hi)>=.05:
                    hi*=2
                    assert hi<1e8
            lo=0.
            for _ in range(55):
                mid=(lo+hi)/2
                if ray(state,a,duration=mid)<.05:hi=mid
                else:lo=mid
            best=min(best,t+hi)
        if math.isfinite(best):break
    return best


def first_risk(case,b,initial):
    ts,ss=schedule_states(b['start_state'],initial,b['controls'],b['epoch'],b['safe_rest']['epoch'])
    assert math.dist(ss[-1],b['safe_rest']['state'])<1e-7
    cs=clear_array(ss,actor_array(case,ts),case['scene']['wall'])
    bad=np.flatnonzero(cs<.05)
    if len(bad):return float(ts[bad[0]])
    result=first_rest_risk(case,ss[-1],float(ts[-1]))
    assert math.isfinite(result),'unsafe label must have a physical risk time'
    return result


def metrics(rows,method):
    safe=[r for r in rows if r['safe']];unsafe=[r for r in rows if not r['safe']]
    expiry=lambda r:r['expiry'][method]
    early=lambda r:expiry(r) is not None and expiry(r)<r['first_physical_risk_s']-1e-8
    premature=lambda r:expiry(r) is not None and expiry(r)<r['motion_end_s']-1e-8
    after=[];provenance=[]
    for r in rows:
        e=expiry(r)
        after.append(sum(max(0.,b-max(a,e)) for a,b in r['ownership']) if e is not None else 0.)
        provenance.append(sum(max(0.,b-max(a,e)) for a,b in r['execution']) if e is not None else 0.)
    return dict(certificates=len(rows),safe=len(safe),unsafe=len(unsafe),
                no_finite_expiry=sum(expiry(r) is None for r in rows),
                unsafe_expiry_before_risk=sum(early(r) for r in unsafe),
                unsafe_early_rate=sum(early(r) for r in unsafe)/len(unsafe) if unsafe else None,
                unsafe_abstain_at_issue=sum(expiry(r) is not None and expiry(r)<=r['issue_s']+1e-8 for r in unsafe),
                unsafe_early_via_immediate_abstention=sum(early(r) and expiry(r)<=r['issue_s']+1e-8 for r in unsafe),
                unsafe_early_via_positive_lifetime=sum(early(r) and expiry(r)>r['issue_s']+1e-8 for r in unsafe),
                safe_premature_expiry=sum(premature(r) for r in safe),
                safe_premature_rate=sum(premature(r) for r in safe)/len(safe) if safe else None,
                safe_finite_motion_coverage=sum(expiry(r) is not None and not premature(r) for r in safe)/len(safe) if safe else None,
                unsafe_expiry_lead_s=stats([r['first_physical_risk_s']-expiry(r) for r in unsafe if expiry(r) is not None]),
                false_safe_tag_duration_s=stats([max(0.,expiry(r)-r['issue_s']) for r in unsafe if expiry(r) is not None]),
                risk_while_unexpired_s=stats([max(0.,expiry(r)-r['first_physical_risk_s']) for r in unsafe if expiry(r) is not None]),
                expires_before_irrevocable_window_end=sum(expiry(r) is not None and expiry(r)<r['issue_s']+.15-1e-8 for r in rows),
                original_owned_past_expiry_certificates=sum(v>1e-8 for v in after),
                original_owned_past_expiry_s=sum(after),
                command_origin_past_expiry_s=sum(provenance))


def run(args,cfg):
    args.output.mkdir(parents=True,exist_ok=True)
    args.derived.mkdir(parents=True,exist_ok=True)
    assert not (args.derived/'certificates.jsonl.gz').exists(),'Do not overwrite sealed reconstruction'
    original_protocol=json.loads((args.raw/'holdout_120/protocol.json').read_text())
    assert original_protocol==json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    mode_manifest=json.loads((ROOT/'results/information_mode/dataset_manifest.json').read_text())
    modes=defaultdict(dict)
    for m in read_gz(Path(mode_manifest['root'])/'modes.jsonl.gz'):modes[m['case_id']][m['t']]=m
    label_manifest=json.loads((ROOT/'results/uncertainty_certificate/dataset_manifest.json').read_text())
    labels=read_gz(Path(label_manifest['root'])/'calibration_samples.jsonl.gz')+read_gz(Path(label_manifest['root'])/'evaluation_samples.jsonl.gz')
    labels={(s['case_id'],s['cert_id']):s for s in labels}
    outcomes={r['case_id']:r for r in json.loads((args.raw/'holdout_120/results.json').read_text()) if r['arm']=='C1'}
    forensic={r['case_id']:r for r in csv.DictReader((ROOT/'results/certificate_failure_forensics/failure_mechanisms.csv').open())}
    all_rows=[];episode=[];renewals=[];known=[];head=[];costs=[]
    for case in original_protocol['cases']:
        cid=case['id'];directory=args.raw/'holdout_120'/f'{cid}_C1'
        raw=read_gz(directory/'C1_normal.jsonl.gz')
        life=json.loads((directory/'C1_normal_lifecycle.json').read_text())
        initial=next(e['certificate'] for e in life if 'certificate' in e)
        certs={initial['id']:initial};initials={initial['id']:[0.,0.]};at_issue={};ownership=defaultdict(list);execution=defaultdict(list);backup_execution=defaultdict(list)
        last_unstable_source=raw[0]['observation']['source_epoch']
        for r in raw:
            m=modes[cid][r['t']];o=observed(r)
            if m['mode']!='STABLE_TRACK':last_unstable_source=o['source']
            at_issue[r['t']]=(o,m['mode'],max(0.,o['source']-last_unstable_source))
            if r['accepted']:
                b=r['active_backup'];assert abs(b['epoch']-r['t'])<1e-8
                certs[b['id']]=b;initials[b['id']]=r['target']
            if r['active_backup']:
                ownership[r['active_backup']['id']].append((r['t'],min(r['t']+.05,outcomes[cid]['end_s'])))
            for step,owner in zip(r['executed'],r['executed_owners']):
                if owner is not None:
                    interval=(step[0]-.01,step[0]);execution[owner['id']].append(interval)
                    if owner['backup']:backup_execution[owner['id']].append(interval)
        records=[];byid={}
        for id_,b in certs.items():
            s=labels[(cid,id_)];o,mode,span=at_issue[b['epoch']]
            begin=time.perf_counter_ns();expiry=candidate_clocks(b,initials[id_],o,mode,span,cfg);costs.append((time.perf_counter_ns()-begin)/1e6)
            # Truth path starts AFTER every candidate clock was fixed.
            risk=first_risk(case,b,initials[id_]) if not s['safe'] else None
            record=dict(case_id=cid,category=case['category'],cert_id=id_,issue_s=b['epoch'],source_s=o['source'],age_s=o['t']-o['source'],mode=mode,
                        lineage=b['lineage'],acquired_epoch=b['acquired_epoch'],motion_end_s=b['safe_rest']['epoch'],
                        duration_s=b['safe_rest']['epoch']-b['epoch'],command_delay_s=.1,controls=b['controls'],backup_controls=b['backup_controls'],
                        safe=s['safe'],predicted_clearance_m=s['predicted_clearance_m'],truth_clearance_m=s['truth_clearance_m'],
                        first_physical_risk_s=risk,expiry=expiry,ownership=ownership[id_],execution=execution[id_],backup_execution=backup_execution[id_],
                        original_completion=outcomes[cid]['completion'],original_collision=outcomes[cid]['collision'],
                        actual_collision_s=outcomes[cid]['end_s'] if outcomes[cid]['collision'] else None,
                        last_owned_until_s=max((b for _,b in ownership[id_]),default=None),
                        last_origin_command_execution_s=max((b for _,b in execution[id_]),default=None))
            records.append(record);byid[id_]=record
        previous=byid[initial['id']]
        for r in raw:
            if not r['accepted']:continue
            new=byid[r['active_backup']['id']]
            for method in cfg['candidates']:
                old_exp,new_exp=previous['expiry'][method],new['expiry'][method]
                renewals.append(dict(case_id=cid,category=case['category'],method=method,t=r['t'],
                                     mode_allows=new['mode']=='STABLE_TRACK',old_expiry=old_exp,new_expiry=new_exp,
                                     extends=old_exp is not None and new_exp is not None and new_exp>old_exp+1e-8,
                                     same_lineage=new['lineage']==previous['lineage'],
                                     expired_before_attempt=old_exp is not None and r['t']>old_exp+1e-8))
            previous=new
        for method in cfg['candidates']:
            values=metrics(records,method)
            episode.append(dict(case_id=cid,category=case['category'],method=method,**values))
        if cid in forensic:
            f=forensic[cid];event=float(f['analysis_event_s']);last_escape=float(f['last_saved_remaining_executable_truth_safe_s']) if f['last_saved_remaining_executable_truth_safe_s'] else None
            unsafe=sorted((r for r in records if not r['safe']),key=lambda r:r['issue_s'])
            if unsafe:
                first=unsafe[0]
                for method in cfg['candidates']:
                    e=first['expiry'][method]
                    known.append(dict(case_id=cid,category=case['category'],method=method,first_unsafe_issue_s=first['issue_s'],expiry=e,
                                      original_event_s=event,lead_to_original_event_s=None if e is None else event-e,
                                      last_logged_physical_escape_s=last_escape,margin_to_logged_escape_s=None if e is None or last_escape is None else last_escape-e,
                                      authoritative_chain_interruption='Unknown: no counterfactual execution or renewal proof'))
            if case['category']=='head_on':
                stamp=float(f['last_logged_truth_safe_certificate_s'])
                b=next(b for b in certs.values() if b['epoch']==stamp)
                rec=byid[b['id']]
                for r in raw:
                    if stamp-1e-8<=r['t']<=stamp+.15+1e-8:
                        audit,_,_=projection(case,r['state'],r['target'],b['controls'],r['t'],max(r['t']+.94,b['safe_rest']['epoch']),r['input_actors'],r['t'])
                        head.append(dict(case_id=cid,t=r['t'],selected_certificate_issue_s=stamp,absolute_dispatch_deadline=rec['expiry']['dispatch_deadline'],
                                         deadline_margin_s=None if rec['expiry']['dispatch_deadline'] is None else rec['expiry']['dispatch_deadline']-r['t'],
                                         original_authorized=bool(r['active_backup'] and r['active_backup']['id']==b['id']),
                                         original_reason=r['validity'],prefix_compatible=same_queued_prefix(r,b),
                                         remaining_current_cv_m=audit['predicted_min_m'],remaining_truth_m=audit['truth_schedule_min_m']))
        all_rows.extend(records)
        print(cid,len(records),flush=True)
    assert len(all_rows)==len(labels)==26540
    results=[];categories=[];renew_summary=[]
    for method in cfg['candidates']:
        results.append(dict(method=method,**metrics(all_rows,method)))
        for cat in dict.fromkeys(c['category'] for c in original_protocol['cases']):
            categories.append(dict(method=method,category=cat,**metrics([r for r in all_rows if r['category']==cat],method)))
        rr=[r for r in renewals if r['method']==method]
        renew_summary.append(dict(method=method,attempts=len(rr),mode_permitted=sum(r['mode_allows'] for r in rr),
                                  mode_denied=sum(not r['mode_allows'] for r in rr),expiry_extended=sum(r['extends'] for r in rr),
                                  same_lineage_extended=sum(r['extends'] and r['same_lineage'] for r in rr),
                                  attempts_after_old_expiry=sum(r['expired_before_attempt'] for r in rr),
                                  extension_s=stats([r['new_expiry']-r['old_expiry'] for r in rr if r['extends']])))
    summary=dict(protocol_sha256=digest(CONFIG),scope=cfg['exposure'],episodes=120,certificates=len(all_rows),
                 prototype=False,
                 results=results,renewals=renew_summary,candidate_compute_ms=stats(costs),
                 additional_waiting_s=None,prevented_collisions=None,global_latest_escape='Unknown')
    write_csv(args.output/'methods.csv',results);write_csv(args.output/'categories.csv',categories)
    write_csv(args.output/'episodes.csv',episode);write_csv(args.output/'known29.csv',known);write_csv(args.output/'head_on_deadlines.csv',head)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with gzip.open(args.derived/'certificates.jsonl.gz','wt') as f:
        for r in all_rows:f.write(json.dumps(r,separators=(',',':'))+'\n')
    with gzip.open(args.derived/'renewal_attempts.jsonl.gz','wt') as f:
        for r in renewals:f.write(json.dumps(r,separators=(',',':'))+'\n')
    manifest=dict(dataset_id='rm2027-safety-lease-offline-20261027',root=str(args.derived),origin=str(args.raw),
                  source_manifest='../certificate_holdout_rerun/dataset_manifest.json',mode_manifest='../information_mode/dataset_manifest.json',
                  label_manifest='../uncertainty_certificate/dataset_manifest.json',generated_or_real='derived from unchanged synthetic C1 logs; no new rollout',
                  purpose='time-bounded lease and absolute commitment deadline diagnostics',filename_convention='certificates.jsonl.gz; renewal_attempts.jsonl.gz',
                  required_experiment='offline_safety_lease.py test -> analyze',protocol_sha256=digest(CONFIG),
                  files=[dict(path=p.name,size_bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(args.derived.iterdir()) if p.is_file()])
    manifest['total_bytes']=sum(x['size_bytes'] for x in manifest['files'])
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    finalize(args,cfg)
    print((args.output/'summary.json').read_text())


def finalize(args,cfg):
    """Aggregate sealed classifications against the pre-written gate; no tuning."""
    rows=read_gz(args.derived/'certificates.jsonl.gz')
    attempts=read_gz(args.derived/'renewal_attempts.jsonl.gz')
    summary=json.loads((args.output/'summary.json').read_text())
    summary['results']=[dict(method=m,**metrics(rows,m)) for m in cfg['candidates']]
    category_rows=[dict(method=m,category=cat,**metrics([r for r in rows if r['category']==cat],m))
                   for m in cfg['candidates'] for cat in dict.fromkeys(r['category'] for r in rows)]
    write_csv(args.output/'methods.csv',summary['results'])
    write_csv(args.output/'categories.csv',category_rows)
    episode_rows=[dict(case_id=cid,category=next(r['category'] for r in rows if r['case_id']==cid),method=m,
                       **metrics([r for r in rows if r['case_id']==cid],m))
                  for cid in dict.fromkeys(r['case_id'] for r in rows) for m in cfg['candidates']]
    write_csv(args.output/'episodes.csv',episode_rows)
    with (args.output/'categories.csv').open() as f:cats=list(csv.DictReader(f))
    with (args.output/'head_on_deadlines.csv').open() as f:head=list(csv.DictReader(f))
    controls=[];gates=[];lineages=[]
    for method in cfg['candidates']:
        for cat in ('crossing','multiple'):
            ss=[r for r in rows if r['category']==cat and r['safe'] and r['mode']=='STABLE_TRACK']
            premature=lambda r:r['expiry'][method] is not None and r['expiry'][method]<r['motion_end_s']-1e-8
            bycase=defaultdict(list)
            for r in ss:bycase[r['case_id']].append(r)
            controls.append(dict(method=method,category=cat,safe_stable_certificates=len(ss),
                                 premature_expiries=sum(premature(r) for r in ss),
                                 premature_rate=sum(premature(r) for r in ss)/len(ss),
                                 episodes_with_premature_expiry=sum(any(premature(r) for r in vv) for vv in bycase.values()),
                                 episodes=len(bycase),actual_forced_backup_rate='Unknown: no lease-policy rollout'))
        m=next(r for r in summary['results'] if r['method']==method)
        required=[r for r in cats if r['method']==method and r['category'] in ('stop_start','occlusion_reappearance','prediction_error')]
        critical=next(h for h in head if h['case_id']=='head_on_10201' and h['t']==h['selected_certificate_issue_s'])
        cert=next(r for r in rows if r['case_id']=='head_on_10201' and abs(r['issue_s']-float(critical['selected_certificate_issue_s']))<1e-8)
        clock=cert['expiry'][method];ell=cert['expiry']['dispatch_deadline']
        gate=cfg['go_gate']
        checks=dict(unsafe_early=m['unsafe_early_rate']>=gate['unsafe_expiry_before_physical_risk_min'],
                    each_required_category=all(float(r['unsafe_early_rate'])>=gate['each_stop_occlusion_prediction_min'] for r in required),
                    safe_motion_coverage=m['safe_finite_motion_coverage']>=gate['safe_finite_coverage_through_saved_motion_min'],
                    stable_interference=all(c['premature_rate']<=gate['stable_crossing_multiple_premature_expiry_max'] for c in controls if c['method']==method),
                    head_on_commit_window=clock is not None and ell is not None and clock<=ell+1e-8,
                    incremental_cost=summary['candidate_compute_ms']['p99']<=gate['incremental_p99_ms_max'])
        gates.append(dict(method=method,checks=checks,numerical_pass=all(checks.values()),
                          physically_supported_validity=False,
                          validity_limitation='Queue deadline constrains dispatch, not future-model validity' if method=='dispatch_deadline' else 'No evidence maps this time proxy to future behavior/coverage validity'))
    for cid,lineage in sorted(set((r['case_id'],r['lineage']) for r in rows)):
        ss=[r for r in rows if r['case_id']==cid and r['lineage']==lineage]
        vv=[r['expiry']['dispatch_deadline'] for r in ss if r['expiry']['dispatch_deadline'] is not None]
        lineages.append(dict(case_id=cid,lineage=lineage,certificates=len(ss),
                             first_issue_s=min(r['issue_s'] for r in ss),last_issue_s=max(r['issue_s'] for r in ss),
                             absolute_deadline_drift_s=max(vv)-min(vv) if vv else None))
    # Numeric failure is sufficient to Stop; the separate validity defect is also retained.
    numeric=any(g['numerical_pass'] for g in gates)
    summary['verdict']='Modify' if numeric else 'Stop'
    summary['recommend_freeze_line']=not numeric
    summary['decision_reason']=('Numerical gates pass but physical validity/renewal evidence remains unsupported.' if numeric else
        'Every candidate fails the frozen gate. Dispatch timing is useful metadata, not validity expiry; short clocks truncate safe backups and long clocks miss risk. Freeze current line.')
    summary['gates']=gates
    summary['renewal_permission_is_not_validity_proof']=True
    summary['same_lineage_deadline_drift_s']=stats([r['absolute_deadline_drift_s'] for r in lineages if r['absolute_deadline_drift_s'] is not None])
    rates=[]
    for cid in set(r['case_id'] for r in rows):
        count=sum(a['case_id']==cid and a['method']=='horizon' for a in attempts)
        duration=max(b for r in rows if r['case_id']==cid for _,b in r['ownership'])
        rates.append(count/duration)
    summary['original_replacement_rate_hz']=stats(rates)
    write_csv(args.output/'stable_controls.csv',controls)
    write_csv(args.output/'lineage_deadline_drift.csv',lineages)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


def self_test(cfg):
    b=dict(epoch=0.,switch_time=.15,controls=[[.1,.5,0],[.15,0,.5],[1.55,0,0]],backup_controls=[[.15,0,.5],[1.55,0,0]])
    assert abs(dispatch_deadline(b,[0,0],cfg)-.05)<1e-9
    o=dict(source=-.1)
    a=candidate_clocks(b,[0,0],o,'STABLE_TRACK',.4,cfg)
    assert abs(a['information_quality']-.05)<1e-9 and a['horizon']==2 and a['past_consistency']==.4
    assert candidate_clocks(b,[0,0],o,'STALE',.4,cfg)['information_quality']==0
    rest=dict(epoch=0.,switch_time=.15,controls=[[.1,0,0],[.15,0,0]],backup_controls=[[.15,0,0]])
    assert dispatch_deadline(rest,[0,0],cfg) is None
    # This evaluator-only change alters the risk label, not a candidate clock.
    case=dict(scene=dict(static=[],dynamic=[[2,0,0,0,.3]],wall=2.),motion_events=[dict(actor=0,t=1.,velocity=[-1.,0.])])
    assert abs(first_rest_risk(case,[0,0,0,0],0.)-2.325)<1e-7
    case['motion_events'][0]['t']=10.
    assert abs(first_rest_risk(case,[0,0,0,0],0.)-11.325)<1e-7
    assert a==candidate_clocks(b,[0,0],o,'STABLE_TRACK',.4,cfg)
    # At ell the .1s prefix ends at the absolute event; one control tick later is too late.
    ell=dispatch_deadline(b,[0,0],cfg)
    assert ell+.1<=.15+1e-9 and ell+.05+.1>.15+1e-9
    print('PASS: absolute dispatch deadline vs validity clock; no-action deadline Unknown; mode/freshness clock; future truth changes labels only; rest-risk ray; immutable-delay boundary')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('command',choices=['test','analyze','summarize'])
    ap.add_argument('--raw',type=Path);ap.add_argument('--derived',type=Path);ap.add_argument('--output',type=Path)
    args=ap.parse_args();cfg=json.loads(CONFIG.read_text())
    if args.command=='test':self_test(cfg);return
    assert not args.derived.resolve().is_relative_to(ROOT.parents[1])
    {'analyze':run,'summarize':finalize}[args.command](args,cfg)


if __name__=='__main__':main()
