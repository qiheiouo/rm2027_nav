#!/usr/bin/env python3
"""Read-only verification by default; --write-summary updates only new evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).parent
OLD='6c5e8227a78d806bfddfe9c4e65d9742156b5c14'
BASE=ROOT/'docs/research/full_base_simulation_validation/evidence'
NEW=BASE/'contracts_20261008'
GROUPS=['timing','heading_reorder','frame_epoch','heading_epoch','startup_sensor',
        'startup_reference','startup_parent','startup_unconfirmed','startup_home_unconfirmed','startup_attitude_missing']

def main():
    p=argparse.ArgumentParser();p.add_argument('--write-summary',action='store_true');p.add_argument('--evidence-dir',type=Path,default=NEW);args=p.parse_args()
    target=args.evidence_dir.resolve()
    checks=[]
    def check(name,ok):checks.append(dict(name=name,passed=bool(ok)))
    hashes=json.loads((BASE/'evidence_sha256.json').read_text());historical_evidence=0
    for path,expected in hashes.items():
        if path.startswith('docs/'):
            data=(ROOT/path).read_bytes();historical_evidence+=1
        else:
            data=subprocess.check_output(['git','show',OLD+':'+path],cwd=ROOT)
        check('frozen historical hash: '+path,hashlib.sha256(data).hexdigest()==expected)
    historical=json.loads(subprocess.check_output([sys.executable,str(HERE/'verify_evidence.py')],cwd=ROOT))
    check('historical 19 physics / 4 analytical / 217 gates retained',historical['verdict']=='Simulation Go' and historical['checks']==217 and historical['samples']==14454)
    total_rows=0;total_contract_checks=0
    comparisons=[]
    for name in GROUPS:
        d=json.loads((target/(name+'.json')).read_text());total_rows+=len(d['rows'])
        check(name+': all expected before/after outputs and statuses',d['passed'] and all(all(r['checks'].values()) for r in d['rows']))
        total_contract_checks+=sum(len(r['checks']) for r in d['rows'])
        check(name+': same declared raw/heading routes; no oracle input',all(
            {'/shadow/input/raw','/shadow/input/heading'}<=set(t for t,_ in g['subscriptions']) and
            not any(t.startswith('/simulation/') or t in ['/joint_states','/tf','/tf_static'] for t,_ in g['subscriptions'])
            for g in d['graph'].values()))
        check(name+': no shadow TF payload or command publisher',not d['tf_payloads'] and all(
            not any(t in ['/cmd_vel','/simulation/gimbal_yaw_cmd'] or ('full' in k and t in ['/tf','/tf_static'])
                    for t,_ in g['publishers']) for k,g in d['graph'].items()))
        check(name+': actual planar publish_tf remains false',all(d['graph'][v+'_planar']['parameters']['publish_tf'] is False for v in ['old','new']))
        for row in d['rows']:
            comparisons.append(dict(group=name,label=row['label'],results=row['results']))
    for label in ['hidden20ms_fresh_metadata','periodic20ms_alias']:
        row=next(r for r in comparisons if r['label']==label)
        check(label+': unobservable delay still admitted and visibly wrong',all(
            row['results'][v+'_full']['status']['status']=='SOLVED' and
            abs(row['results'][v+'_full']['orientation_error_deg']-np.degrees(.008))<1e-7 for v in ['old','new']))
    for name,status in [('continuum','AMBIGUOUS'),('near_singular','ILL_CONDITIONED'),('two_roots','AMBIGUOUS'),('time_rewind','INVALID_TIME')]:
        d=json.loads((target/'boundaries'/(name+'.json')).read_text())
        check(name+': current wrapper retains analytical rejection',d['gate'] and d['sequence'][2]['status']['status']==status and not d['sequence'][2]['full_output'])
    physical=json.loads((target/'physics/flat_B.json').read_text())
    trace=json.loads(gzip.decompress((target/'physics/flat_B.trace.json.gz').read_bytes()))
    check('flat_B: original authority/oracle fences',physical['no_oracle_subscriptions'] and physical['no_shadow_tf_or_commands'])
    motion=[r for r in trace if r['phase'] not in ['startup','settle','stop']]
    check('flat_B: normal asynchronous motion remains fully SOLVED',all(r['status']['status']=='SOLVED' and 'full' in r for r in motion))
    metrics={}
    for kind in ['planar','full']:
        errors=np.array([[np.linalg.norm(np.array(r[kind])[:3,3]-np.array(r['truth'])[:3,3]),
                          np.degrees(Rotation.from_matrix(np.array(r['truth'])[:3,:3].T@np.array(r[kind])[:3,:3]).magnitude())]
                         for r in motion if kind in r])
        metrics[kind]=dict(position_max_m=float(errors[:,0].max()),orientation_max_deg=float(errors[:,1].max()))
        check('flat_B: '+kind+' score recomputed from current trace',np.allclose(errors.max(axis=0),
            [max(v[kind]['position_m']['max'] for k,v in physical['phases'].items() if k not in ['startup','settle','stop']),
             max(v[kind]['orientation_deg']['max'] for k,v in physical['phases'].items() if k not in ['startup','settle','stop'])],atol=1e-10,rtol=0))
    check('flat_B: existing ideal-input numerical screen',metrics['full']['position_max_m']<.005 and metrics['full']['orientation_max_deg']<.15)
    refs=json.loads((BASE/'source_identity.json').read_text())
    for ref,sha in refs.items():
        if ref.startswith(('experiment/','origin/')) or ref=='main':
            check('protected ref unchanged: '+ref,subprocess.check_output(['git','rev-parse',ref],cwd=ROOT,text=True).strip()==sha)
    unit=subprocess.run([sys.executable,str(HERE/'test_input_contract.py')],capture_output=True,text=True)
    check('11 deterministic receipt/sequence/reference checks',unit.returncode==0)
    check('mathematical adapter remains byte-identical to 5733f126',hashlib.sha256((HERE/'scripts/adapter.py').read_bytes()).hexdigest()==refs['algorithm_source_sha256'])
    result=dict(verdict='Conditional Simulation Go' if all(c['passed'] for c in checks) else 'Modify',
        recommendation='freeze shadow research pending new-car measurements',old_commit=OLD,
        historical_source_evidence_hashes=len(hashes),historical_evidence_files=historical_evidence,historical_regression=historical,
        live_contract_groups=len(GROUPS),live_contract_rows=total_rows,live_contract_expectations=total_contract_checks,
        deterministic_tests=11,current_analytical_ROS_boundaries=4,current_physics_cases=['flat_B'],current_physics_motion_samples=len(motion),
        physics_metrics=metrics,checks=checks,comparisons=comparisons,
        limits=['unchanged hidden-delay/periodic-alias observability','no physical clock/home/attitude/extrinsic certification',
                'unannounced epoch reuse with consistent stamps and identities remains unobservable','no Deployment Go'])
    if args.write_summary:(target/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    frozen=target/'evidence_sha256.json'
    frozen_failures=[]
    if frozen.exists() and not args.write_summary:
        for path,expected in json.loads(frozen.read_text()).items():
            if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=expected:frozen_failures.append(path)
    print(json.dumps({k:result[k] for k in ['verdict','live_contract_groups','live_contract_rows','live_contract_expectations','current_physics_motion_samples']}|
                     dict(checks=len(checks),failed=[c['name'] for c in checks if not c['passed']],frozen_hash_mismatches=frozen_failures),indent=2))
    return 0 if all(c['passed'] for c in checks) and not frozen_failures else 1
if __name__=='__main__':sys.exit(main())
