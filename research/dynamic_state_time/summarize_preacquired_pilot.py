"""Read-only pilot analysis; raw execution records remain outside the repository."""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
from summarize_preacquired import audit
from world import stats

ROOT = Path(__file__).resolve().parent
ARMS = ('B', 'C0', 'C1')
SCENES = ('static', 'crossing', 'head_on', 'multiple')


def event(row, label):
    b = row['active_backup']
    return dict(label=label, t=row['t'], state=row['state'],
                proposal=row['proposed']['command'] if row['proposed'] else None,
                accepted=row['accepted'], committed=row['committed_action'], reason=row['reason'],
                backup={k:b[k] for k in ('id','lineage','acquired_epoch','switch_time','execution_cursor','backup_controls','safe_rest')} if b else None,
                validity=row['validity'], executed=row['executed'][-1][1:3],
                executed_owner=row['executed_owners'][-1],total_cycle_ms=row['total_cycle_ms'])


def analyze(r, directory):
    trace = directory/f"{r['arm']}_normal.jsonl.gz"
    rows = [json.loads(x) for x in gzip.open(trace, 'rt')] if trace.exists() else []
    # Retain scalar/distribution results, not per-cycle lifecycle payloads.
    result = {k:v for k,v in r.items() if k not in ('config','lifecycle','first_rejections')}
    result['safe_completion'] = bool(r['completion'] and r['collision'] is False and r['minimum_clearance_m'] >= .05)
    result['timeout'] = not r['completion'] and not r['collision'] and r.get('end_s',0) >= 40
    result['proposal_rejection_count'] = sum(x['proposed'] is not None and not x['accepted'] for x in rows)
    result['rejection_reasons'] = dict(Counter(x['reason'] for x in rows if not x['accepted']))
    result['backup_availability_ratio'] = sum(x['active_backup'] is not None and x['validity']=='valid' for x in rows)/len(rows) if rows and r['arm']=='C1' else None
    result['accepted_without_valid_backup'] = sum(x['accepted'] and (not x['active_backup'] or not x['candidate_available'] or x['reason'] not in ('splice_old','new_backup')) for x in rows) if r['arm']=='C1' else None
    result['old_backup_retained_on_rejection'] = sum(x['proposed'] is not None and not x['accepted'] and x['active_backup'] is not None and x['validity']=='valid' for x in rows)
    result['replacement_reasons'] = dict(Counter(e['reason'] for x in rows for e in x['lifecycle'] if e['state']=='REPLACED'))
    pairs = [(ex, owner) for x in rows for ex, owner in zip(x['executed'],x['executed_owners'])]
    owned = [(ex, owner) for ex,owner in pairs if owner and owner['backup']]
    result['actual_backup_steps'] = len(owned)
    result['actual_moving_backup_steps'] = sum(math.hypot(*ex[1:3]) > 1e-7 for ex,_ in owned)
    result['actual_backup_s'] = len(owned)*.01
    result['actual_moving_backup_s'] = result['actual_moving_backup_steps']*.01
    previous = [0.,0.]; changes = []
    for ex, _ in pairs:
        change = math.dist(ex[1:3], previous)
        if change > 1e-7: changes.append(change)
        previous = ex[1:3]
    result['actual_action_switches'] = len(changes)
    result['command_total_variation_mps'] = sum(changes)
    result['command_change_mps'] = stats(changes)
    lat = [x['total_cycle_ms'] for x in rows]
    result['total_cycle_ms'] = stats(lat)
    result['full_cycle_deadline_misses'] = sum(t>50 for t in lat)
    result['full_cycle_miss_rate'] = result['full_cycle_deadline_misses']/len(lat) if lat else None
    result['consumer_cpu_cores_at_realtime'] = r.get('consumer_cpu_s',0)/r['end_s'] if r.get('end_s') else None
    if r['arm']=='C1' and rows:
        try: result['persistence_audit'] = dict(passed=True, **audit(rows))
        except AssertionError as error: result['persistence_audit'] = dict(passed=False, error=str(error))
    selected = []
    if rows:
        selected += [event(rows[0], 'start'), event(max(rows,key=lambda x:x['total_cycle_ms']), 'worst_latency')]
        if r.get('worst'): selected.append(event(min(rows,key=lambda x:abs(x['t']-r['worst']['t'])), 'worst_clearance'))
        for label,predicate in (
            ('first_rejected_proposal',lambda x:x['proposed'] is not None and not x['accepted']),
            ('first_actual_backup',lambda x:any(o and o['backup'] for o in x['executed_owners'])),
            ('first_actual_moving_backup',lambda x:any(o and o['backup'] and math.hypot(*ex[1:3])>1e-7 for ex,o in zip(x['executed'],x['executed_owners'])))):
            row=next((x for x in rows if predicate(x)),None)
            if row: selected.append(event(row,label))
        if result['timeout']:
            tail=[ex for ex,_ in pairs if ex[0]>=r['end_s']-10]
            result['last_10s_displacement_m']=math.dist(tail[0][3:5],tail[-1][3:5])
            result['last_10s_goal_distance_change_m']=math.dist(tail[-1][3:5],[4,0])-math.dist(tail[0][3:5],[4,0])
    failures=[]
    if r['collision']:failures.append('collision')
    if r.get('minimum_clearance_m') is not None and r['minimum_clearance_m']<.05:failures.append('clearance_below_0.05')
    if not r['completion']:failures.append('task_incomplete')
    if r['arm']=='C1' and (not r.get('initialized') or r['invalid']):failures.append('reserve_unavailable_or_invalid')
    if lat and (stats(lat)['p99']>40 or max(lat)>50 or result['full_cycle_miss_rate']>.001):failures.append('realtime_gate')
    if r['arm']=='C1' and (result['accepted_without_valid_backup'] or not result.get('persistence_audit',{}).get('passed',False)):failures.append('persistence_contract')
    result['failed_checks']=failures
    return result, rows, selected


def aggregate(results, samples):
    out={}
    for scene in (*SCENES,'all'):
        for arm in ARMS:
            group=[r for r in results if r['arm']==arm and (scene=='all' or r['scene']==scene)]
            ids={r['id'] for r in group}
            latency=[x['total_cycle_ms'] for id_, rows in samples.items() if id_ in ids for x in rows]
            out[f'{scene}/{arm}']=dict(
                episodes=len(group),collisions=sum(bool(r['collision']) for r in group),
                completions=sum(r['completion'] for r in group),safe_completions=sum(r['safe_completion'] for r in group),
                timeouts=sum(r['timeout'] for r in group),
                clearance_m=stats([r['minimum_clearance_m'] for r in group if r['minimum_clearance_m'] is not None]),
                traversal_s=stats([r['traversal_s'] for r in group if r['traversal_s'] is not None]),
                wait_s=stats([r['wait_s'] for r in group if 'wait_s' in r]),
                total_cycle_ms=stats(latency),full_cycle_deadline_misses=sum(t>50 for t in latency),
                full_cycle_miss_rate=sum(t>50 for t in latency)/len(latency) if latency else None,
                consumer_cpu_s=stats([r['consumer_cpu_s'] for r in group if 'consumer_cpu_s' in r]),
                consumer_cpu_cores_at_realtime=stats([r['consumer_cpu_cores_at_realtime'] for r in group if r['consumer_cpu_cores_at_realtime'] is not None]),
                consumer_rss_peak_sum_mib=stats([r['consumer_rss_peak_sum_mib'] for r in group if 'consumer_rss_peak_sum_mib' in r]),
                initialization_ms=stats([r['initialization_ms'] for r in group if r['initialization_ms'] is not None]),
                actual_backup_s=sum(r['actual_backup_s'] for r in group),
                actual_moving_backup_s=sum(r['actual_moving_backup_s'] for r in group),
                rejections=sum(r['proposal_rejection_count'] for r in group),
                initialized=sum(bool(r.get('initialized')) for r in group),
                accepted_without_valid_backup=sum(r['accepted_without_valid_backup'] or 0 for r in group))
    return out


def plot(results, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(2,2,figsize=(11,6),layout='constrained')
    keys=[f'{s}_{n}' for s in SCENES for n in (13,29,47)]
    colors=['#9a416e','#b87418','#087e8b']
    for arm,color,shift in zip(ARMS,colors,(-.23,0,.23)):
        rs=[next(r for r in results if r['id']==f'{k}_{arm}') for k in keys]
        x=[i+shift for i in range(12)]
        axes[0,0].scatter(x,[r['minimum_clearance_m'] for r in rs],color=color,label=arm,s=20)
        axes[0,1].scatter(x,[r['traversal_s'] if r['completion'] else r.get('end_s',0) for r in rs],color=color,s=20)
        for xx,r in zip(x,rs):
            if not r['completion']:axes[0,1].scatter([xx],[r.get('end_s',0)],marker='x',s=65,color=color)
        axes[1,0].bar(x,[r.get('wait_s',0) for r in rs],width=.22,color=color)
        axes[1,1].scatter(x,[r['total_cycle_ms']['p99'] for r in rs],color=color,s=20)
    axes[0,0].axhline(.05,color='black',ls='--',lw=.8)
    axes[1,1].axhline(40,color='black',ls='--',lw=.8)
    titles=['Minimum physical clearance (m)','Arrival time; x = failed episode end (s)','Waiting time including post-goal (s)','Full cycle p99 (ms)']
    labels=[k.replace('head_on','head').replace('crossing','cross').replace('multiple','multi') for k in keys]
    for ax,title in zip(axes.flat,titles):
        ax.set_title(title,fontsize=10);ax.set_xticks(range(12),labels,rotation=55,ha='right',fontsize=7);ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Frozen 36-episode pilot; normal latency; ideal CV input',fontsize=12)
    fig.savefig(output/'pilot_comparison.png',dpi=125);plt.close(fig)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--plot',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    primary=args.data/'pilot_36'
    raw=json.loads((primary/'results.json').read_text());assert len(raw)==36
    protocol=json.loads((primary/'protocol.json').read_text())
    assert protocol==json.loads((ROOT/'preacquired_pilot_protocol.json').read_text())
    expected={(s,n,a) for s in SCENES for n in (13,29,47) for a in ARMS}
    assert {(r['scene'],r['seed'],r['arm']) for r in raw}==expected
    results=[];samples={};timelines={}
    for r in raw:
        result,rows,selected=analyze(r,primary/r['id'])
        results.append(result);samples[r['id']]=rows;timelines[r['id']]=selected
    groups=aggregate(results,samples)
    archived=list(csv.DictReader((ROOT/'results/trials.csv').open()))
    baseline_parity={}
    for r in results:
        if r['arm']!='B':continue
        old=next(x for x in archived if x['scene']==r['scene'] and int(x['seed'])==r['seed'] and x['arm']=='search')
        baseline_parity[r['id']]=(old['collision']==str(r['collision']) and old['completion']==str(r['completion'])
            and abs(float(old['min_clearance_m'])-r['minimum_clearance_m'])<1e-9
            and (not r['completion'] or abs(float(old['arrival_s'])-r['traversal_s'])<1e-9))
    matched_execution={}
    for s in SCENES:
        for n in (13,29,47):
            execution=[[ex for row in samples[f'{s}_{n}_{a}'] for ex in row['executed']] for a in ARMS]
            matched_execution[f'{s}_{n}']=execution[0]==execution[1]==execution[2]
    paired=[]
    for s in SCENES:
        for n in (13,29,47):
            c=next(r for r in results if (r['scene'],r['seed'],r['arm'])==(s,n,'C1'))
            for a in ('B','C0'):
                b=next(r for r in results if (r['scene'],r['seed'],r['arm'])==(s,n,a))
                paired.append(dict(scene=s,seed=n,baseline=a,c1_safe=c['safe_completion'],baseline_safe=b['safe_completion'],
                    traversal_delta_s=c['traversal_s']-b['traversal_s'] if c['completion'] and b['completion'] else None,
                    traversal_delta_fraction=c['traversal_s']/b['traversal_s']-1 if c['completion'] and b['completion'] else None,
                    wait_delta_s=c.get('wait_s',0)-b.get('wait_s',0),
                    wait_comparison_censored=not(c['completion'] and b['completion'])))
    c1s=[r for r in results if r['arm']=='C1'];g=groups['all/C1']
    gates=dict(safety=all(r['collision'] is False and r['minimum_clearance_m'] is not None and r['minimum_clearance_m']>=.05 for r in c1s),
        completion=g['completions']>=max(groups['all/B']['completions'],groups['all/C0']['completions']),
        no_new_timeout=not any(r['timeout'] for r in c1s),
        persistence=all(r.get('initialized') and r['backup_availability_ratio']==1 and not r['invalid'] and r['accepted_without_valid_backup']==0 and r.get('persistence_audit',{}).get('passed') for r in c1s),
        realtime=g['total_cycle_ms']['p99']<=40 and g['total_cycle_ms']['max']<=50 and g['full_cycle_miss_rate']<=.001
            and all(r['total_cycle_ms']['p99'] is not None and r['total_cycle_ms']['p99']<=40 and r['total_cycle_ms']['max']<=50 for r in c1s))
    benefits=[(s,n) for s in SCENES for n in (13,29,47) if all(any(p['scene']==s and p['seed']==n and p['baseline']==a and p['c1_safe'] and not p['baseline_safe'] for p in paired) for a in ('B','C0'))]
    gates['beyond_original_fixture']=len(benefits)>1 and any(k!=('head_on',13) for k in benefits)
    warnings=[p for p in paired if p['scene']!='head_on' and p['traversal_delta_fraction'] is not None and p['traversal_delta_fraction']>.1]
    confirms=[]
    for directory in sorted(args.data.glob('confirm_*')):
        for r in json.loads((directory/'results.json').read_text()):
            result,_,selected=analyze(r,directory/r['id']);confirms.append(dict(run=directory.name,**result));timelines[directory.name]=selected
    summary=dict(scope='Research; 36 frozen episodes, 12 deterministic configs; ideal CV; normal latency; no real tracker/STVL/Nav2',
        base_commit=protocol['base_commit'],holdout='NOT_RUN',
        gate_checks=gates,mechanical_gate_verdict='Go-to-holdout' if all(gates.values()) else 'Gate failed; see report for Modify/Stop',
        successful_safety_pairs=benefits,conservatism_warnings=warnings,
        frozen_B_outcome_parity=baseline_parity,exact_executed_plant_trace_equal_B_C0_C1=matched_execution,
        environment=json.loads((primary/'environment.json').read_text()),groups=groups,paired_comparisons=paired,
        failures=[dict(id=r['id'],checks=r['failed_checks'],worst=r.get('worst')) for r in results if r['failed_checks']],
        results=results,confirmations=confirms)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (args.output/'timeline.json').write_text(json.dumps(timelines,indent=2)+'\n')
    columns=['id','scene','seed','arm','collision','completion','safe_completion','timeout','minimum_clearance_m','traversal_s','wait_s','proposal_rejection_count','replacement_count','actual_backup_s','actual_moving_backup_s','backup_availability_ratio','accepted_without_valid_backup','actual_action_switches','command_total_variation_mps','full_cycle_deadline_misses','consumer_cpu_s','consumer_rss_peak_sum_mib']
    with (args.output/'trials.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=columns+['cycle_p50_ms','cycle_p95_ms','cycle_p99_ms','cycle_max_ms'],lineterminator='\n');w.writeheader()
        for r in results:w.writerow({k:r.get(k) for k in columns}|{f'cycle_{k}_ms':r['total_cycle_ms'][k] for k in ('p50','p95','p99','max')})
    files=[]
    for directory in [primary,*sorted(args.data.glob('confirm_*'))]:
        for p in sorted(directory.rglob('*')):
            if p.is_file():files.append(dict(path=str(p.relative_to(args.data)),size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    manifest=dict(dataset_id='rm2027-preacquired-backup-pilot-20261026',root=str(args.data.resolve()),
        filename_convention='<pilot_36|confirm_*>/<scene>_<seed>_<arm>/<arm>_normal.{json,jsonl.gz}; *_lifecycle.json; *_planner.log; *_reserve.log; protocol.json; environment.json; results.json',
        origin='Frozen synthetic world.scene + C++ search/certificate services; sequential single logical CPU',
        purpose='All 36 safety/task/timing and persistence failures/worst evidence',generated_or_real='generated exact CV, no real sensor data',
        required_experiment='run_preacquired_pilot.sh pilot_36; optional single unchanged confirmation; summarize_preacquired_pilot.py',
        total_bytes=sum(p['size'] for p in files),files=files,excluded='build_cache; no raw or historical archives in Git')
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if args.plot:plot(results,args.output)
    print(json.dumps(dict(gates=gates,benefits=benefits,failures=summary['failures'],warnings=warnings),indent=2))


if __name__=='__main__':main()
