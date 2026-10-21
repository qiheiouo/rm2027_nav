#!/usr/bin/env python3
"""Check measured evidence, reject silent output on invalid or ambiguous samples."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).parent
sys.path.insert(0,str(HERE/'scripts'))
from adapter import inv
EVIDENCE=ROOT/'docs/research/full_base_simulation_validation/evidence'
NOMINAL=['flat_A','flat_B','up11_A','up11_B','up15_A','up15_B','down11_A','down11_B','down15_A','down15_B','bank1_A','bank2_B','tilted_pitch11']
FAULTS=['missing_startup','time_faults','source_skew','frame_fault','heading_reset','raw_epoch']
def main():
    checks=[];tables=[];domain_samples=0
    for name in NOMINAL+FAULTS:
        path=EVIDENCE/(name+'.json');d=json.loads(path.read_text())
        trace=json.loads(gzip.decompress((EVIDENCE/(name+'.trace.json.gz')).read_bytes()))
        checks.append((name+': isolated input/authority',d['no_oracle_subscriptions'] and d['no_shadow_tf_or_commands']))
        checks.append((name+': rejected sample has no output',all('full' not in r for r in trace if r['status']['status'] not in ['SOLVED','NO_STATUS'])))
        checks.append((name+': no unknown status on motion',all(r['status']['status']!='NO_STATUS' for r in trace if r['phase'] not in ['startup','settle','stop'])))
        checks.append((name+': solved source time and unique root',all(
            abs(r['status']['diagnostics']['heading_sensor_skew_ns'])<=1_000_000 and len(r['status']['diagnostics']['candidates'])==1
            for r in trace if r['status']['status']=='SOLVED')))
        for phase,v in d['phases'].items():
            if phase in ['startup','settle','stop']:continue
            rows=[r for r in trace if r['phase']==phase];domain_samples+=len(rows)
            table=dict(case=name,phase=phase,statuses=v['statuses'],samples=v['input_samples'])
            for k in ['planar','full']:
                table[k]=v[k]
                # Independently recompute maxima from recorded outputs and scoring truth.
                if v[k]['output_samples']:
                    e=[[np.linalg.norm(np.array(r[k])[:3,3]-np.array(r['truth'])[:3,3]),
                        np.degrees(Rotation.from_matrix(np.array(r['truth'])[:3,:3].T@np.array(r[k])[:3,:3]).magnitude())]
                       for r in rows if k in r]
                    checks.append((name+': '+phase+' '+k+' score recomputation',
                        bool(np.allclose(np.max(e,axis=0),[v[k]['position_m']['max'],v[k]['orientation_deg']['max']],atol=1e-10,rtol=0))))
            tables.append(table)
            if name in NOMINAL:
                checks.append((name+': '+phase+' solved coverage',v['statuses']=={'SOLVED':v['input_samples']} and v['full']['output_samples']==v['input_samples']))
                # Numerical screening for ideal observation + declared approximate fixture startup.
                checks.append((name+': '+phase+' geometric screening',v['full']['position_m']['max']<.005 and v['full']['orientation_deg']['max']<.15))
    for name,status in [('continuum','AMBIGUOUS'),('near_singular','ILL_CONDITIONED'),('two_roots','AMBIGUOUS'),('time_rewind','INVALID_TIME')]:
        d=json.loads((EVIDENCE/(name+'.json')).read_text());r=d['sequence'][2]
        checks.append((name+': ROS state and withheld pose',d['gate'] and r['status']['status']==status and not r['full_output']))
    for name in ['heading_reset','raw_epoch']:
        d=json.loads((EVIDENCE/(name+'.json')).read_text());v=d['phases']['identity_returns']
        checks.append((name+': epoch return does not unlatch',v['full']['output_samples']==0 and set(v['statuses'])=={'INVALID_TIME'}))
    d=json.loads((EVIDENCE/'missing_startup.json').read_text());v=d['phases']['missing_startup']
    checks.append(('missing startup: no truth supplementation',v['full']['output_samples']==0 and set(v['statuses'])=={'INVALID_CALIBRATION'}))
    readback=json.loads((EVIDENCE/'frame_fault.json').read_text())['graph']['planar_actual_parameters']
    checks.append(('actual planar parameters retain shadow authority',readback['publish_tf'] is False and readback['output_odom_topic']=='/shadow/planar/odom' and readback['base_frame']=='shadow_planar_base'))
    s=json.loads((EVIDENCE/'timestamp_sensitivity.json').read_text())
    checks.append(('1ms policy boundary and hidden time error',all(r['strict_status']==('SOLVED' if r['delay_ms']<=1. else 'INVALID_TIME') and r['hidden_stamp_status']=='SOLVED' for r in s['rows'])))
    source='5733f126b71b52e7f4ddd2340b226c1589b4419b'
    upstream=subprocess.check_output(['git','show',source+':experiments/full_base_heading_adapter/adapter.py'],cwd=ROOT)
    checks.append(('minimal candidate copy is unchanged',upstream==(HERE/'scripts/adapter.py').read_bytes()))
    checks.append(('historical 6c5e8227 production adapter and baseline model unchanged',subprocess.run(['git','diff','--exit-code','d5673e3a','6c5e8227','--',
         'src/rm_localization_adapters','src/rm_new_car_simulation'],cwd=ROOT,stdout=subprocess.DEVNULL).returncode==0))
    result=dict(verdict='Simulation Go' if all(p for _,p in checks) else 'Modify',
        scope='shadow pose conversion with ideal declared simulated observations, fixture startup reference, valid same-time heading; no deployment Go',
        validation_branch='experiment/full-base-simulation-validation',simulation_base='d5673e3a1cfe6937e4126e1df894ee6d66e63e07',
        candidate_source=source,checks=[dict(name=n,passed=bool(p)) for n,p in checks],physics_cases=len(NOMINAL)+len(FAULTS),
        analytical_ROS_boundaries=4,scored_motion_fault_samples=domain_samples,rows=tables,
        screening_note='5mm / 0.15deg engineering screening of ideal-input research domain; not measured hardware thresholds or pre-registered statistical acceptance')
    # Historical evidence is immutable; this command only recomputes its checks.
    print(json.dumps({'verdict':result['verdict'],'checks':len(checks),'failed':[n for n,p in checks if not p],'samples':domain_samples},indent=2))
    return 0 if result['verdict']=='Simulation Go' else 1
if __name__=='__main__':sys.exit(main())
