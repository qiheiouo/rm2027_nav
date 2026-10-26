"""Three control ticks per contract input; verifies the actual executor wiring.

This is not a complete episode or an algorithm performance experiment.
"""
import argparse
import gzip
import json
from pathlib import Path
from reserve_run import trial, FIXTURE
from world import observed, actors_at


class Input:
    def __init__(self, kind): self.kind=kind
    def truth(self,t): return actors_at(FIXTURE['scene'],t)
    def sample(self,t):
        current,actors=observed(FIXTURE['scene'],t)
        source=t-.1;receive=t;version=round((source+10)*1e6);sets=0
        if self.kind=='prediction' and t>=.05:
            actors[0][0]+=.005
        if self.kind=='set' and t>=.05:
            actors=[];sets=1
        if self.kind=='stale_recovery' and t<.1:
            source=-.15;receive=0.;version=round((source+10)*1e6)
        return current,actors,dict(source_epoch=source,source_version=version,receive_time=receive,
                                   obstacle_set_version=sets,age_s=t-source)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);summary=[]
    for kind in ('prediction','set','stale_recovery'):
        out=args.output/kind;out.mkdir()
        result=trial(args.build,out,'C1','normal',full_episode=True,observation_source=Input(kind),max_control_steps=3)
        rows=[json.loads(x) for x in gzip.open(out/'C1_normal.jsonl.gz','rt')]
        assert len(rows)==3 and result['contract_prefix_only']
        assert not result['terminated_on_invalidation'] and rows[2]['accepted']
        if kind=='stale_recovery':
            assert not rows[1]['accepted'] and rows[1]['active_backup'] is None
            assert rows[1]['validity']=='stale_observation'
        else:
            assert rows[1]['accepted']
            assert rows[1]['validity']==('prediction_changed' if kind=='prediction' else 'obstacle_set_changed')
        illegal=sum(r['accepted'] and not r['certificate_transaction']['authorized'] for r in rows)
        assert illegal==0
        summary.append(dict(case=kind,control_ticks=len(rows),accepted_without_valid_backup=illegal,
                            admission=[r['accepted'] for r in rows],validity=[r['validity'] for r in rows],
                            ended_by_test_bound=True,completion_claim=False,total_cycle_ms=result['total_cycle_ms']))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
