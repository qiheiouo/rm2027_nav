#!/usr/bin/env python3
"""Small common-input ROS before/after fixtures. No Gazebo, TF or commands.

Old nodes are built/read from 6c5e8227; current nodes use this worktree install.
These fixtures prove observable contracts, not physical timestamp truth.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.parameter_client import AsyncParameterClient
from rm_competition_interfaces.msg import ChassisHeadingState
from scipy.spatial.transform import Rotation
import yaml
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from adapter import extrinsic,inv
from common import calibration,set_pose,stamp,matrix
ROOT=Path(__file__).resolve().parents[2]
OLD='6c5e8227a78d806bfddfe9c4e65d9742156b5c14'
BUILD=ROOT/'build/contracts_20261008'
EVIDENCE=ROOT/'docs/research/full_base_simulation_validation/evidence/contracts_20261008'
BASELINE_MANIFEST=json.loads((ROOT/'docs/research/full_base_simulation_validation/evidence/flat_A.json').read_text())['manifest']
MANIFEST={k:BASELINE_MANIFEST[k] for k in ['lidar_selection','gimbal_center_m','sensor_offset_m']}

def prepare_old():
    BUILD.mkdir(parents=True,exist_ok=True)
    for name in ['candidate.py','adapter.py','common.py']:
        p=BUILD/'old_candidate'/name;p.parent.mkdir(exist_ok=True)
        p.write_bytes(subprocess.check_output(['git','show',OLD+':experiments/full_base_simulation_validation/scripts/'+name],cwd=ROOT))
    pkg='src/rm_localization_adapters'
    for name in subprocess.check_output(['git','ls-tree','-r','--name-only',OLD,'--',pkg],cwd=ROOT,text=True).splitlines():
        p=BUILD/'old_source'/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(subprocess.check_output(['git','show',OLD+':'+name],cwd=ROOT))
    with (BUILD/'old_build.log').open('w') as log:
        subprocess.run(['colcon','--log-base',str(BUILD/'old_log'),'build','--base-paths',str(BUILD/'old_source'/pkg),
                        '--build-base',str(BUILD/'old_build'),'--install-base',str(BUILD/'old_install'),
                        '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)

class Recorder(Node):
    def __init__(self):
        super().__init__('input_contract_recorder');self.status={};self.poses={};self.tf_payloads=[]
        for topic in ['/tf','/tf_static']:
            self.create_subscription(TFMessage,topic,lambda m:self.tf_payloads.extend([v.child_frame_id for v in m.transforms]),100)
        self.raw=self.create_publisher(Odometry,'/shadow/input/raw',100)
        self.heading=self.create_publisher(ChassisHeadingState,'/shadow/input/heading',100)
        for version in ['old','new']:
            self.create_subscription(String,f'/contract/{version}/full_status',lambda m,v=version:self.record(m,v),100)
            for kind in ['full','planar']:
                key=version+'_'+kind;self.poses[key]={}
                self.create_subscription(Odometry,f'/contract/{key}/odom',lambda m,k=key:self.poses[k].update({stamp(m.header.stamp):m}),100)
    def record(self,m,version):
        d=json.loads(m.data);self.status.setdefault(version,{})[d['stamp_ns']]=d
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.002)


def messages(i):
    t=1_000_000_000+i*20_000_000;h=i*.008
    cal=calibration(MANIFEST);T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',h).as_matrix()
    raw=Odometry();raw.header.stamp.sec,raw.header.stamp.nanosec=divmod(t,10**9)
    raw.header.frame_id='sensor_epoch_1';raw.child_frame_id='sim_lidar_link'
    set_pose(raw.pose.pose,inv(extrinsic(cal,0.))@T@extrinsic(cal,0. if i==0 else .3))
    heading=ChassisHeadingState();heading.header=copy.deepcopy(raw.header);heading.header.frame_id='sim_gravity_heading'
    heading.valid=True;heading.online=True;heading.yaw_rad=h;heading.yaw_rate_rad_s=.4
    heading.source_boot_id=1;heading.sample_sequence=i+1
    return raw,heading,T


def run_group(name,steps,startup=True):
    out=BUILD/name;out.mkdir(parents=True,exist_ok=True);(out/'manifest.json').write_text(json.dumps(MANIFEST))
    log=(out/'nodes.log').open('w');children=[]
    old_planar=BUILD/'old_install/rm_localization_adapters/lib/rm_localization_adapters/lio_adapter'
    new_planar=ROOT/'install/rm_localization_adapters/lib/rm_localization_adapters/lio_adapter'
    for version,exe in [('old',old_planar),('new',new_planar)]:
        params=dict(raw_odom_topic='/shadow/input/raw',output_odom_topic=f'/contract/{version}_planar/odom',
            expected_input_odom_frame='sensor_epoch_1',odom_frame='shadow_planar_odom',base_frame='shadow_planar_base',
            input_sensor_frame='sim_lidar_link',publish_tf=False,use_tf_sensor_to_base=False,
            pose_conversion_mode='chassis_heading_fusion',raw_odom_parent_frame_mode='sensor_initial',twist_mode='finite_difference',
            **{'heading_fusion.heading_topic':'/shadow/input/heading','heading_fusion.derived_gimbal_topic':f'/contract/{version}/gimbal',
               'heading_fusion.initial_alignment_confirmed':True,'twist_estimator.max_linear_speed':100.,'twist_estimator.max_angular_speed':100.})
        for j,k in enumerate('xyz'):
            params['heading_fusion.gimbal_center_in_base.'+k]=MANIFEST['gimbal_center_m'][j]
            params['heading_fusion.initial_base_to_sensor.'+k]=MANIFEST['gimbal_center_m'][j]+MANIFEST['sensor_offset_m'][j]
        path=out/(version+'_planar.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        children.append(subprocess.Popen([str(exe),'--ros-args','-r',f'__node:={version}_planar','--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        candidate=BUILD/'old_candidate/candidate.py' if version=='old' else ROOT/'install/rm_full_base_shadow_validation/lib/rm_full_base_shadow_validation/candidate.py'
        cmd=[sys.executable,str(candidate),'--ros-args','-r',f'__node:={version}_full','-p','manifest:='+str(out/'manifest.json'),
             '-r',f'/shadow/full_base/odom:=/contract/{version}_full/odom','-r',f'/shadow/full_base/status:=/contract/{version}/full_status']
        if startup and startup!='home_only':cmd+=['-p','startup_roll_pitch:=[0.0,0.0]']
        if version=='new' and startup in [True,'home_only']:cmd+=['-p','startup_home_confirmed:=true']
        children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    rclpy.init();node=Recorder();rows=[]
    try:
        end=time.monotonic()+15
        while (node.raw.get_subscription_count()!=4 or node.heading.get_subscription_count()!=4) and time.monotonic()<end:
            if any(c.poll() is not None for c in children):raise RuntimeError('node exited: '+str(out/'nodes.log'))
            node.spin(.03)
        if node.raw.get_subscription_count()!=4:raise RuntimeError('four common-input consumers required')
        node.spin(.2)
        graph={}
        for v in ['old','new']:
            for k in ['full','planar']:
                client=AsyncParameterClient(node,v+'_'+k)
                if not client.wait_for_services(timeout_sec=2.):raise RuntimeError('parameter service unavailable')
                names=['publish_tf','input_sensor_frame','expected_input_odom_frame'] if k=='planar' else ['manifest','startup_roll_pitch'] if v=='old' else ['manifest','startup_home_confirmed','expected_raw_parent','expected_heading_reference']
                future=client.get_parameters(names);deadline=time.monotonic()+2
                while not future.done() and time.monotonic()<deadline:node.spin(.01)
                if not future.done():raise RuntimeError('parameter readback timeout')
                values=future.result().values
                readback={n:(p.bool_value if p.type==1 else p.string_value if p.type==4 else list(p.double_array_value)) for n,p in zip(names,values)}
                graph[v+'_'+k]=dict(parameters=readback,subscriptions=node.get_subscriber_names_and_types_by_node(v+'_'+k,'/'),
                                   publishers=node.get_publisher_names_and_types_by_node(v+'_'+k,'/'))
        for i,(label,mode,expected) in enumerate(steps):
            raw,h,T=messages(i);events=[]
            if name.startswith('startup_'):
                # Every startup retry declares home; no post-start joint supplied.
                cal=calibration(MANIFEST)
                set_pose(raw.pose.pose,inv(extrinsic(cal,0.))@T@extrinsic(cal,0.))
            def send(pub,m):
                events.append(dict(topic='raw' if pub is node.raw else 'heading',wall_time=time.monotonic(),
                    stamp_ns=stamp(m.header.stamp),frame=m.header.frame_id,
                    child=m.child_frame_id if pub is node.raw else None,
                    sequence=m.sample_sequence if pub is node.heading else None,
                    boot=m.source_boot_id if pub is node.heading else None,
                    reset=m.reset_counter if pub is node.heading else None,
                    valid=m.valid if pub is node.heading else None,online=m.online if pub is node.heading else None,
                    yaw=m.yaw_rad if pub is node.heading else None))
                pub.publish(m)
            if mode=='wrong_sensor':raw.child_frame_id='wrong_sensor'
            if mode=='empty_sensor':raw.child_frame_id=''
            if mode=='base_as_sensor':raw.child_frame_id='shadow_planar_base'
            if mode=='wrong_parent':raw.header.frame_id='sensor_epoch_2'
            if mode=='wrong_reference':h.header.frame_id='wrong_heading_reference'
            if mode=='duplicate_fresh':h.sample_sequence=i  # Previous sequence, new stamp.
            if mode=='hidden':h.yaw_rad-=.008
            if mode=='raw_first':send(node.raw,raw);node.spin(.02);send(node.heading,h)
            elif mode=='heading_reordered':
                future=copy.deepcopy(h);future.header.stamp.nanosec+=20_000_000;future.sample_sequence+=1;future.yaw_rad+=.008
                send(node.raw,raw);send(node.heading,future);node.spin(.02);send(node.heading,h)
            elif mode=='cached_offline':
                send(node.heading,h);node.spin(.01)
                invalid=copy.deepcopy(h);invalid.header.stamp.nanosec+=5_000_000;invalid.online=False
                send(node.heading,invalid);node.spin(.01);send(node.raw,raw)
            elif mode=='cached_late':send(node.heading,h);node.spin(.09);send(node.raw,raw)
            elif mode=='epoch_unmatched':
                changed=copy.deepcopy(h);changed.reset_counter=1;changed.valid=False;changed.header.stamp.sec+=1
                send(node.heading,changed);node.spin(.01);send(node.heading,h);node.spin(.01);send(node.raw,raw)
            elif mode=='alias':
                # The preceding physical heading is stamped one period later.
                # Its genuine next sequence and source stamp appear consistent.
                h.yaw_rad-=.008;send(node.heading,h);node.spin(.02);send(node.raw,raw)
            else:send(node.heading,h);node.spin(.02);send(node.raw,raw)
            node.spin(.12)
            t=stamp(raw.header.stamp);row=dict(label=label,mode=mode,stamp_ns=t,events=events,raw=matrix(raw.pose.pose).tolist(),scoring_pose=T.tolist(),results={})
            for key,poses in node.poses.items():
                m=poses.get(t);r=dict(output=m is not None)
                if key.endswith('full'):r['status']=node.status.get(key.split('_')[0],{}).get(t)
                if m:
                    E=matrix(m.pose.pose);r.update(position_error_m=float(np.linalg.norm(E[:3,3]-T[:3,3])),
                        orientation_error_deg=float(np.degrees(Rotation.from_matrix(T[:3,:3].T@E[:3,:3]).magnitude())))
                row['results'][key]=r
            row['checks']={}
            for key,value in expected.items():
                r=row['results'][key]
                row['checks'][key]=(r['status']['status']==value and r['output']==(value=='SOLVED')) if isinstance(value,str) else r['output']==value
            rows.append(row)
        result=dict(group=name,domain='declared analytical common-input ROS contract fixtures; no physical/hardware claim',
                    old_commit=OLD,graph=graph,tf_payloads=node.tf_payloads,rows=rows,passed=all(all(r['checks'].values()) for r in rows))
        EVIDENCE.mkdir(parents=True,exist_ok=True);(EVIDENCE/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(group=name,passed=result['passed'],checks=sum(len(r['checks']) for r in rows),
                             failed=[r['label'] for r in rows if not all(r['checks'].values())])),flush=True)
        if not result['passed']:raise RuntimeError('contract comparison failed')
    finally:
        node.destroy_node();rclpy.try_shutdown()
        for c in children:
            if c.poll() is None:
                os.killpg(c.pid,signal.SIGINT)
                try:c.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=3)
        log.close();(EVIDENCE/(name+'.nodes.log')).write_bytes((out/'nodes.log').read_bytes())


def full(new='SOLVED',old='SOLVED',**others):return dict(new_full=new,old_full=old,**others)
def set_evidence(path):
    global EVIDENCE
    EVIDENCE=path.resolve();EVIDENCE.mkdir(parents=True,exist_ok=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepare-old',action='store_true');p.add_argument('--case');p.add_argument('--evidence-dir',type=Path,default=EVIDENCE);args=p.parse_args()
    set_evidence(args.evidence_dir)
    if os.environ.get('ROS_DOMAIN_ID','0')=='0':raise RuntimeError('set an unused nonzero ROS_DOMAIN_ID')
    if args.prepare_old:prepare_old()
    groups=[('timing',[
        ('home','normal',full()),('raw_before_heading20ms','raw_first',full()),('heading_before_raw20ms','normal',full()),
        ('offline_after_cached_valid','cached_offline',full('INVALID_INPUT')),('recover_offline','normal',full()),
        ('same_sequence_new_stamp','duplicate_fresh',full('INVALID_TIME')),('recover_replay','normal',full()),
        ('cached_exact_after90ms','cached_late',full('INVALID_TIME')),('recover_receipt_age','normal',full()),
        ('hidden20ms_fresh_metadata','hidden',full()),('periodic20ms_alias','alias',full())],True),
        ('heading_reorder',[('home','normal',full()),('legitimate_heading_reorder20ms','heading_reordered',full())],True),
        ('frame_epoch',[
        ('home','normal',full(new_planar=False,old_planar=False)),
        ('wrong_sensor','wrong_sensor',full('INVALID_INPUT','INVALID_INPUT',new_planar=False,old_planar=True)),
        ('empty_sensor','empty_sensor',full('INVALID_INPUT','INVALID_INPUT',new_planar=False,old_planar=True)),
        ('base_instead_of_sensor','base_as_sensor',full('INVALID_INPUT','INVALID_INPUT',new_planar=False,old_planar=False)),
        ('restore_sensor','normal',full(new_planar=True,old_planar=True)),
        ('raw_epoch_changes','wrong_parent',full('INVALID_TIME','INVALID_TIME',new_planar=False,old_planar=False)),
        ('old_raw_name_returns','normal',full('INVALID_TIME','INVALID_TIME',new_planar=False,old_planar=True))],True),
        ('heading_epoch',[('home','normal',full()),
        ('unmatched_invalid_reset','epoch_unmatched',full('INVALID_TIME')),('old_heading_epoch_returns','normal',full('INVALID_TIME'))],True),
        ('startup_sensor',[('bad_first_sensor','wrong_sensor',full('INVALID_INPUT','INVALID_INPUT')),
        ('valid_startup_after_rejection','normal',full('SOLVED','INVALID_INPUT'))],True),
        ('startup_reference',[('bad_first_reference','wrong_reference',full('INVALID_INPUT')),
        ('valid_startup_after_rejection','normal',full('SOLVED','INVALID_INPUT'))],True),
        ('startup_parent',[('bad_first_parent','wrong_parent',full('INVALID_INPUT')),
        ('valid_startup_after_rejection','normal',full('SOLVED','INVALID_TIME'))],True),
        ('startup_home_unconfirmed',[('finite_attitude_without_home','normal',full('INVALID_CALIBRATION'))],'attitude_only'),
        ('startup_attitude_missing',[('confirmed_home_without_attitude','normal',full('INVALID_CALIBRATION'))],'home_only'),
        ('startup_unconfirmed',[('missing_attitude_and_home','normal',full('INVALID_CALIBRATION'))],False)]
    for name,steps,explicit in groups:
        if args.case is None or args.case==name:run_group(name,steps,explicit)
if __name__=='__main__':main()
