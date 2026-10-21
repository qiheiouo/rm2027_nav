#!/usr/bin/env python3
"""ROS transport boundary fixtures, explicitly NOT physics trajectories."""
import argparse
import copy
from collections import Counter
from dataclasses import replace
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
from rm_competition_interfaces.msg import ChassisHeadingState
from scipy.spatial.transform import Rotation
import yaml
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from common import calibration,set_pose,stamp
from adapter import extrinsic,inv,solve
ROOT=Path(__file__).resolve().parents[2];EVIDENCE=ROOT/'docs/research/full_base_simulation_validation/evidence'
class Replay(Node):
    def __init__(self):
        super().__init__('declared_boundary_replay');self.status={};self.full={};self.planar={}
        self.raw=self.create_publisher(Odometry,'/shadow/input/raw',100)
        self.heading=self.create_publisher(ChassisHeadingState,'/shadow/input/heading',100)
        self.create_subscription(String,'/shadow/full_base/status',self.record,100)
        self.create_subscription(Odometry,'/shadow/full_base/odom',lambda m:self.full.update({stamp(m.header.stamp):m}),100)
        self.create_subscription(Odometry,'/shadow/planar/odom',lambda m:self.planar.update({stamp(m.header.stamp):m}),100)
    def record(self,m):
        d=json.loads(m.data);self.status[d['stamp_ns']]=d
    def send(self,t,T,h):
        raw=Odometry();raw.header.stamp.sec,raw.header.stamp.nanosec=divmod(t,1_000_000_000)
        raw.header.frame_id='sensor_epoch_1';raw.child_frame_id='sim_lidar_link';set_pose(raw.pose.pose,T)
        heading=ChassisHeadingState();heading.header=copy.deepcopy(raw.header);heading.header.frame_id='sim_gravity_heading'
        heading.yaw_rad=float(h);heading.valid=True;heading.online=True;heading.source_boot_id=1;heading.sample_sequence=t//20_000_000
        self.heading.publish(heading)
        for _ in range(5):rclpy.spin_once(self,timeout_sec=.01)
        self.raw.publish(raw)
        end=time.monotonic()+.15
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)

def boundary(name,axis_pitch,roll):
    out=ROOT/'build/shadow_boundaries'/name;out.mkdir(parents=True,exist_ok=True)
    F=np.eye(4);F[:3,:3]=Rotation.from_euler('y',axis_pitch).as_matrix();F[:3,3]=[-.00194,-.00002,.06649]
    K=np.eye(4);K[:3,3]=[.13970,-.09149,.14112]
    manifest=dict(lidar_selection='boundary_fixture',gimbal_center_m=F[:3,3].tolist(),sensor_offset_m=K[:3,3].tolist(),
                  synthetic_T_base_joint=F.tolist(),synthetic_T_joint_sensor_home=K.tolist())
    (out/'manifest.json').write_text(json.dumps(manifest))
    home=F@K
    params={'raw_odom_topic':'/shadow/input/raw','output_odom_topic':'/shadow/planar/odom',
            'expected_input_odom_frame':'sensor_epoch_1','odom_frame':'shadow_planar_odom','base_frame':'shadow_planar_base',
            'input_sensor_frame':'sim_lidar_link','publish_tf':False,'use_tf_sensor_to_base':False,
            'pose_conversion_mode':'chassis_heading_fusion','raw_odom_parent_frame_mode':'sensor_initial','twist_mode':'finite_difference',
            'heading_fusion.heading_topic':'/shadow/input/heading','heading_fusion.derived_gimbal_topic':'/shadow/planar/gimbal',
            'heading_fusion.initial_alignment_confirmed':True,'twist_estimator.max_linear_speed':100.,'twist_estimator.max_angular_speed':100.}
    for i,k in enumerate('xyz'):
        params['heading_fusion.gimbal_center_in_base.'+k]=float(F[i,3]);params['heading_fusion.initial_base_to_sensor.'+k]=float(home[i,3])
    for k,v in zip(['roll','pitch','yaw'],Rotation.from_matrix(home[:3,:3]).as_euler('xyz')):
        params['heading_fusion.initial_base_to_sensor.'+k]=float(v)
    (out/'planar.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    log=(out/'nodes.log').open('w');children=[]
    for cmd in [['ros2','run','rm_localization_adapters','lio_adapter','--ros-args','-r','__node:=planar_shadow','--params-file',str(out/'planar.yaml')],
                ['ros2','run','rm_full_base_shadow_validation','candidate.py','--ros-args','-p','manifest:='+str(out/'manifest.json'),'-p','startup_roll_pitch:=[0.0,0.0]','-p','startup_home_confirmed:=true']]:
        children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    rclpy.init();node=Replay()
    try:
        deadline=time.monotonic()+15
        while (node.raw.get_subscription_count()<2 or node.heading.get_subscription_count()<2) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if node.raw.get_subscription_count()!=2:raise RuntimeError('both adapters must subscribe')
        cal=calibration(manifest);sequence=[]
        for i,(label,rpy,q,h) in enumerate([('home',(0,0,0),0,0),('level',(0,0,.2),.3,.2),
                ('boundary',(roll,.1 if axis_pitch else 0,.4),.6,.4),('recover_level',(0,0,.5),.8,.5)]):
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();T[:3,3]=[0.,0.,0.]
            if label=='boundary':T[:3,3]=[.7,-.2,.15]
            raw=inv(home)@T@extrinsic(cal,q);t=1_000_000_000+i*20_000_000
            if name=='time_rewind' and label=='boundary':t=1_010_000_000
            node.send(t,raw,h);s=node.status.get(t);full=node.full.get(t);planar=node.planar.get(t)
            row=dict(label=label,status=s,full_output=full is not None,planar_output=planar is not None,
                     raw=raw.tolist(),heading=h,scoring_pose=T.tolist())
            for key,msg in [('full',full),('planar',planar)]:
                if msg:
                    E=__import__('common').matrix(msg.pose.pose);row[key+'_position_error_m']=float(np.linalg.norm(E[:3,3]-T[:3,3]))
                    row[key+'_orientation_error_deg']=float(np.degrees(Rotation.from_matrix(T[:3,:3].T@E[:3,:3]).magnitude()))
            sequence.append(row)
        expected={'continuum':'AMBIGUOUS','near_singular':'ILL_CONDITIONED','two_roots':'AMBIGUOUS','time_rewind':'INVALID_TIME'}[name]
        passed=sequence[2]['status']['status']==expected and not sequence[2]['full_output'] and sequence[3]['status']['status']==('AMBIGUOUS' if name=='two_roots' else 'INVALID_TIME' if name=='time_rewind' else 'SOLVED')
        result=dict(name=name,domain='declared analytical ROS transport fixture; outside normal CAD physics envelope',
                    axis_pitch_rad=axis_pitch,roll_rad=roll,expected=expected,gate=passed,sequence=sequence)
        (EVIDENCE/(name+'.json')).write_text(json.dumps(result,indent=2));print(json.dumps({'boundary':name,'gate':passed}),flush=True)
        if not passed:raise RuntimeError('boundary rejection/return gate failed')
    finally:
        node.destroy_node();rclpy.try_shutdown()
        for c in children:
            os.killpg(c.pid,signal.SIGINT)
            try:c.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=5)
        log.close();(EVIDENCE/(name+'.nodes.log')).write_bytes((out/'nodes.log').read_bytes())
def global_evidence(path):
    global EVIDENCE
    EVIDENCE=path.resolve();EVIDENCE.mkdir(parents=True,exist_ok=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case');parser.add_argument('--evidence-dir',type=Path,default=EVIDENCE/'contracts_20261008'/'boundaries');args=parser.parse_args()
    global_evidence(args.evidence_dir)
    for name,p,r in [('continuum',0.,np.pi/2),('near_singular',0.,np.radians(89)),('two_roots',np.pi/3,0.),('time_rewind',0.,0.)]:
        if args.case is None or args.case==name:boundary(name,p,r)
if __name__=='__main__':main()
