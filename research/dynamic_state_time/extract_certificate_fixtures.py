"""Crop controller-only inputs at saved failures, never copy raw trajectories."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def extract(raw, output):
    cases={c['id']:c for c in json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())['cases']}
    selected=['static_10001','stop_start_10300','occlusion_reappearance_10500']
    selected += [f'stale_observation_{n}' for n in (10801,10802,10803,10805,10806,10807,10809,10810,10811)]
    fixtures=[];manifest=[]
    for name in selected:
        path=raw/(name+'_C1')/'C1_normal.jsonl.gz'
        rows=[json.loads(line) for line in gzip.open(path,'rt')]
        before,row=rows[-2:];assert before['accepted'] and row['accepted']
        old=before['active_backup'];new=row['active_backup']
        assert old['epoch']==before['t'] and new['epoch']==row['t']
        def request(r,set_version):
            b=r['active_backup'];o=r['observation']
            return dict(state=r['state'],target=r['target'],queue=b['immutable_queue'],
                        epoch=r['t'],actors=r['input_actors'],wall=cases[name]['scene']['wall'],
                        queue_version=b['immutable_queue_version'],source_epoch=o['source_epoch'],
                        source_version=o['source_version'],receive_time=r['t'],
                        obstacle_set_version=set_version,frame_epoch=0,clock_epoch=0,
                        dynamics=[.01,.5,.8,.355,.330])
        q=request(row,int(row['validity']=='obstacle_set_changed'))
        if name.startswith('stale_'):q['receive_time']=2.  # frozen update was received at t=2
        prior=request(before,0)
        if name.startswith('stale_'):prior['receive_time']=2.
        scene=cases[name]['scene']
        fixtures.append(dict(id=name,original_reason=row['validity'],old_certificate=old,
            old_initial_target=before['target'],old_committed_queue=before['immutable_queue'],
            old_committed_queue_version=before['queue_version'],prior_request=prior,request=q,
            proposal=row['proposed']['command'],
            previous=[[before['t']+e[0]-row['t'],*e[1:]] for e in before['proposed']['schedule']],
            grid_scene={k:scene[k] for k in ('wall','static','goal')},
            original_accepted=row['accepted'],original_terminated=True))
        manifest.append(dict(dataset_id=name+'_C1',path=str(path),size_bytes=path.stat().st_size,
                             sha256=hashlib.sha256(path.read_bytes()).hexdigest(),origin='f1203c3 holdout synthetic controller trace',
                             generated=True,purpose='two snapshots around first invalidation',required_experiment='certificate contract replay'))
    output.mkdir(parents=True,exist_ok=False)
    (output/'failure_fixtures.json').write_text(json.dumps(fixtures,separators=(',',':'))+'\n')
    (output/'dataset_manifest.json').write_text(json.dumps(dict(
        dataset_id='certificate_lifecycle_20261026',archive_commit='f1203c3949f8d48237a9ea0125c895f7f5a8ef2c',
        convention='<external holdout root>/<case>_C1/C1_normal.jsonl.gz',
        sources=manifest,fixture=dict(path='failure_fixtures.json',generated=True,
        size_bytes=(output/'failure_fixtures.json').stat().st_size,
        sha256=hashlib.sha256((output/'failure_fixtures.json').read_bytes()).hexdigest(),
        purpose='12 minimal before/failure controller snapshots, no full trajectory; reference states regenerated from frozen plant'),
        metadata_note='receive_time/set-version/frame/clock were absent in archive: receive=t except frozen stale received at 2 s; set version 0 before, 1 after observed removal; domains 0. No actor value or command is changed.'),indent=2)+'\n')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--raw',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();extract(args.raw,args.output)
