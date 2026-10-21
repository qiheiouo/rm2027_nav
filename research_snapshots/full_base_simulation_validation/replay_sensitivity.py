#!/usr/bin/env python3
"""Re-score fixed observed-input trace perturbations; truth only evaluates results."""
import argparse
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from adapter import PoseSample,HeadingSample,Startup,solve
from common import calibration,CLOCK,REFERENCE
ROOT=Path(__file__).resolve().parents[2];EVIDENCE=ROOT/'docs/research/full_base_simulation_validation/evidence'

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=EVIDENCE/'contracts_20261008/timestamp_sensitivity.json');args=p.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    path=EVIDENCE/'flat_A.trace.json.gz';trace=json.loads(gzip.decompress(path.read_bytes()))
    evidence=json.loads((EVIDENCE/'flat_A.json').read_text());t0=evidence['startup_stamp_ns']
    start=next(s for s in trace if s['stamp_ns']==t0);current=next(s for s in reversed(trace) if s['phase']=='combined_wrap_multiturn' and 'heading' in s)
    cal=calibration(evidence['manifest'])
    def sample(row):
        h=row['heading'];return (PoseSample(np.array(row['raw']),row['stamp_ns'],CLOCK,row['raw_parent'],row['raw_parent'],row['raw_child']),
          HeadingSample(h['yaw'],h['stamp_ns'],CLOCK,h['epoch'],REFERENCE,h['valid'],h['online']))
    raw0,h0=sample(start);raw,h=sample(current)
    R0=Rotation.from_euler('xyz',[*evidence['declared_startup_roll_pitch_rad'],h0.yaw_rad]).as_matrix()
    startup=Startup(raw0,h0,R0,R0.T@[0.,0.,-1.],t0,t0,CLOCK,True,0.,cal.calibration_id,'shadow_full_odom')
    rows=[];truth=np.array(current['truth']);nominal=solve(raw,h,cal,startup)
    for ms in [0.,.5,1.,1.000001,2.,5.,10.,20.,60.]:
        ns=int(round(ms*1_000_000));value=h.yaw_rad-current['heading']['rate']*ms/1000
        strict=solve(raw,replace(h,yaw_rad=value,stamp_ns=h.stamp_ns-ns),cal,startup)
        hidden=solve(raw,replace(h,yaw_rad=value),cal,startup)
        row=dict(delay_ms=ms,strict_status=strict.status.value,hidden_stamp_status=hidden.status.value,
                 perturbation_model='constant measured azimuth rate over offset, not recorded asynchronous hardware')
        if hidden.T_odom_base is not None:
            T=hidden.T_odom_base;row.update(position_error_m=float(np.linalg.norm(T[:3,3]-truth[:3,3])),
                orientation_error_deg=float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@T[:3,:3]).magnitude())),
                incremental_position_response_m=float(np.linalg.norm(T[:3,3]-nominal.T_odom_base[:3,3])),
                incremental_orientation_response_deg=float(np.degrees(Rotation.from_matrix(nominal.T_odom_base[:3,:3].T@T[:3,:3]).magnitude())))
        rows.append(row)
    result=dict(domain='offline perturbation of actual physics/ROS observed-input trace',source_trace=path.name,
                source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),source_stamp_ns=current['stamp_ns'],
                measured_heading_rate_rad_s=current['heading']['rate'],rows=rows)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
