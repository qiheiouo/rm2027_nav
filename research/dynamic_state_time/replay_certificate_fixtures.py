"""Contract snapshots + one 50 ms queue step, never a full navigation episode."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import platform
import time
from run import Consumer
from reserve_run import at
from world import advance, static_grid, stats
from certificate_lifecycle import CertificateLifecycle
from test_certificate_lifecycle import contracts

ROOT=Path(__file__).resolve().parent


def restore(f):
    b=copy.deepcopy(f['old_certificate'])
    b['initial_target']=f['old_initial_target']
    state=b['start_state'].copy();target=b['initial_target'];samples=[state]
    for k in range(round((b['safe_rest']['epoch']-b['epoch'])/.01)):
        target=at(b['controls'],b['epoch']+k*.01+.005,target)
        state=advance(state,target);samples.append(state)
    assert math.dist(state,b['safe_rest']['state'])<1e-10
    b['reference_states']=samples
    b['prediction'].update(receive_time=f['prior_request']['receive_time'],
                           obstacle_set_version=0,frame_epoch=0,clock_epoch=0)
    # The archive bound the generation queue, before its admitted append.
    # Validate the known post-commit queue against controls before rebinding it.
    for e in f['old_committed_queue']:
        assert math.dist(at(b['controls'],e[0],b['initial_target']),e[1:])<1e-10
    b['immutable_queue']=f['old_committed_queue']
    b['immutable_queue_version']=f['old_committed_queue_version']
    return b


def manager(checker,f,cert):
    m=CertificateLifecycle(checker);m.certificate=copy.deepcopy(cert)
    m.latest=copy.deepcopy(f['prior_request']);m.authorized=True;m.state='OLD_VALID'
    return m


def replay(checker,legacy,planner,fixtures,repeats):
    rows=[];measurements=[];violations=0;stale_issued=0
    for f in fixtures:
        q=f['request'];cert=restore(f)
        prior=f['prior_request'] | {'queue':f['old_committed_queue'],'queue_version':f['old_committed_queue_version']}
        assert checker.call(prior | {'op':'validate','backup':cert})['reason']=='valid'
        grid,_=static_grid(f['grid_scene'])
        pq=dict(mode='search',state=q['state'],target=q['target'],actors=q['actors'],wall=q['wall'],
                queue=[[e[0]-q['epoch'],*e[1:]] for e in q['queue']],previous=f['previous'],grid=grid)
        samples=[];last=None
        for i in range(repeats):
            # Same immutable planner; inputs are saved measured state/CV/queue only.
            proposal=planner.call(pq)
            assert math.dist(proposal['command'],f['proposal'])<1e-12
            start=time.perf_counter()
            lv=legacy.call(q | {'op':'validate','backup':cert})
            lr=legacy.call(q | {'op':'admit','command':proposal['command'],
                                'backup':cert if lv['reason']=='valid' else None})
            old_ms=(time.perf_counter()-start)*1000
            assert lv['reason']==f['original_reason'] and lr['available']
            m=manager(checker,f,cert)
            start=time.perf_counter()
            vr=m.begin(q);prepared=m.prepare(proposal['command']);result=m.finish(prepared['certificate'])
            tx_ms=(time.perf_counter()-start)*1000
            assert vr['reason']==f['original_reason']
            stale=f['original_reason']=='stale_observation'
            stale_issued+=int(stale and prepared['certificate'] is not None)
            post=q | {'queue':result['queue'],'queue_version':result['queue_version']}
            final_valid=checker.call(post | {'op':'validate','backup':result['certificate']})['reason']=='valid'
            violations+=int(result['accepted'] and not final_valid)
            assert result['accepted']==(not stale)
            assert result['authorized']==(not stale)
            assert not stale or result['queue']==q['queue']
            next_valid=None
            if not stale:
                # One actual immutable-queue step, then revalidate the held reserve.
                # A reused observation ages to .15 s; no new future measurement is invented.
                state=q['state'].copy();target=q['target'];queue=copy.deepcopy(result['queue'])
                for k in range(5):
                    stamp=q['epoch']+k*.01
                    while queue and queue[0][0]<=stamp+.005+1e-9:
                        target=queue.pop(0)[1:]
                    state=advance(state,target)
                nq=q | {'epoch':q['epoch']+.05,'state':state,'target':target,'queue':queue,
                        'queue_version':result['queue_version'],
                        'actors':[[a[0]+.05*a[2],a[1]+.05*a[3],*a[2:]] for a in q['actors']]}
                next_valid=m.begin(nq)['reason']
                assert next_valid=='valid'
                held=m.finish(None)
                assert held['authorized'] and not held['accepted']
                assert held['certificate']['controls']==result['certificate']['controls']
            sample=dict(fixture=f['id'],repeat=i,legacy_ms=old_ms,transaction_ms=tx_ms,
                        added_ms=tx_ms-old_ms,planner_ms=proposal['roundtrip_ms'],
                        total_ms=proposal['roundtrip_ms']+tx_ms)
            samples.append(sample);measurements.append(sample)
            last=dict(id=f['id'],reason=vr['reason'],original_accepted=True,original_terminated=True,
                      accepted=result['accepted'],authorized=result['authorized'],state=result['state'],
                      source_epoch=q['source_epoch'],receive_time=q['receive_time'],age_s=result['age_s'],
                      certificate_source_version=result['certificate_source_version'],
                      next_cycle_validity=next_valid,queued_commands_preserved=all(e in result['queue'] for e in q['queue']),
                      certificate_issued=prepared['certificate'] is not None)
        last['latency_ms']={k:stats([s[k] for s in samples]) for k in ('transaction_ms','added_ms','total_ms')}
        rows.append(last)
    assert violations==0 and stale_issued==0
    return dict(fixtures=rows,repeats=repeats,transactions=len(measurements),
                accepted_without_valid_backup=violations,stale_certificates_issued=stale_issued,
                stale_admissions=sum(r['accepted'] for r in rows if r['reason']=='stale_observation'),
                latency_ms={k:stats([s[k] for s in measurements]) for k in ('legacy_ms','transaction_ms','added_ms','planner_ms','total_ms')},
                latency_by_input={kind:{k:stats([s[k] for s in measurements if s['fixture'].startswith('stale_')==stale])
                                      for k in ('legacy_ms','transaction_ms','added_ms','total_ms')}
                                  for kind,stale in (('fresh_replacement',False),('stale_rejection',True))},
                deadline_misses=sum(s['total_ms']>50 for s in measurements),
                worst=max(measurements,key=lambda s:s['total_ms']),
                scope='saved snapshots and one 50 ms execution step only; no completion/safety or full holdout claim'),measurements


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--legacy-build',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--repeats',type=int,default=20);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    c=Consumer(args.build/'reserve_consumer','reserve',args.output/'current.log')
    old=Consumer(args.legacy_build/'reserve_consumer','reserve',args.output/'legacy.log')
    planner=Consumer(args.legacy_build/'search_baseline','search',args.output/'planner.log')
    try:
        tested=contracts(c)  # mandatory gate before any saved failure replay
        fixtures=json.loads((ROOT/'results/certificate_lifecycle/failure_fixtures.json').read_text())
        result,measurements=replay(c,old,planner,fixtures,args.repeats)
        result['contracts']=tested
        result['environment']=dict(platform=platform.platform(),python=platform.python_version(),affinity=sorted(os.sched_getaffinity(0)))
        result['consumer_usage']={name:dict(cpu_s=tool.usage()[0],peak_rss_mib=tool.usage()[1]) for name,tool in [('current',c),('legacy',old),('planner',planner)]}
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.output/'measurements.json').write_text(json.dumps(measurements,separators=(',',':'))+'\n')
        print(json.dumps({k:result[k] for k in ('transactions','accepted_without_valid_backup','stale_admissions','latency_ms','deadline_misses')},indent=2))
    finally:
        for tool in (c,old,planner):tool.close()
