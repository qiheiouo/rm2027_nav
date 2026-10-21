#!/usr/bin/env python3
"""Physics runner and scoring-only recorder. Nothing here chooses candidate roots."""
import argparse
from collections import Counter
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64,String
from tf2_msgs.msg import TFMessage
from rm_competition_interfaces.msg import ChassisHeadingState,GimbalState
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from common import matrix,stamp
from adapter import inv,rz
ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=ROOT/'docs/research/full_base_simulation_validation/evidence'

def prepare(case,out):
    spec=importlib.util.spec_from_file_location('scene',ROOT/'src/rm_new_car_simulation/scripts/baseline_scene.py')
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    manifest=scene.generate(ROOT/'src/rm_new_car_simulation',out,case.get('field','flat'),case.get('lidar','A'),
                            spawn=case.get('spawn',[0,0,.002,0,0,0]),ground_bank_deg=case.get('bank',2.))
    tree=ET.parse(out/'world.sdf');robot=tree.find("./world/model[@name='new_car']")
    if robot is None:raise RuntimeError('missing baseline model')
    ET.SubElement(robot,'plugin',filename='librm_shadow_observations.so',name='rm_shadow::Observations')
    tree.write(out/'world.sdf')
    pairs=[('/model/new_car/world_odometry','/simulation/ground_truth/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry'),
           ('/model/new_car/sensor_world_odometry','/simulation/sensor_truth','nav_msgs/msg/Odometry','gz.msgs.Odometry'),
           ('/model/new_car/shadow_observations','/shadow/observation_packet','nav_msgs/msg/Odometry','gz.msgs.Odometry'),
           ('/model/new_car/joint_state','/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model'),
           ('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock')]
    bridge=[dict(gz_topic_name=g,ros_topic_name=r,ros_type_name=rt,gz_type_name=gt,direction='GZ_TO_ROS') for g,r,rt,gt in pairs]
    for g,r,rt,gt in [('/model/new_car/cmd_vel','/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist'),
                      ('/model/new_car/gimbal_velocity','/simulation/gimbal_velocity','std_msgs/msg/Float64','gz.msgs.Double')]:
        bridge.append(dict(gz_topic_name=g,ros_topic_name=r,ros_type_name=rt,gz_type_name=gt,direction='ROS_TO_GZ'))
    (out/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    manifest['baseline_generated_world_sha256']=manifest['sha256']['world.sdf']
    manifest['sha256']['world.sdf']=hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest()
    manifest['shadow_bridge_sha256']=hashlib.sha256((out/'bridge.yaml').read_bytes()).hexdigest()
    manifest['shadow_observation_model']='ideal_sensor_relative_pose_plus_base_azimuth; no startup attitude or joint export'
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest
class Scorer(Node):
    def __init__(self):
        super().__init__('shadow_scoring_only');self.data={k:{} for k in ['truth','sensor','raw','heading','planar','full','status','joint','gimbal']}
        self.tf=[];self.tf_gids={};self.phase={};self.current='startup';self.subs=[]
        for key,topic,typ in [('truth','/simulation/ground_truth/odom',Odometry),('sensor','/simulation/sensor_truth',Odometry),
            ('raw','/shadow/input/raw',Odometry),('heading','/shadow/input/heading',ChassisHeadingState),
            ('planar','/shadow/planar/odom',Odometry),('full','/shadow/full_base/odom',Odometry),
            ('joint','/joint_states',JointState),('gimbal','/shadow/planar/gimbal',GimbalState)]:
            self.subs.append(self.create_subscription(typ,topic,lambda m,k=key:self.record(k,m),500))
        self.subs.append(self.create_subscription(String,'/shadow/full_base/status',self.status,500))
        self.subs.append(self.create_subscription(TFMessage,'/tf',self.record_tf,500))
        self.subs.append(self.create_subscription(TFMessage,'/tf_static',self.record_tf,
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10);self.gimbal=self.create_publisher(Float64,'/simulation/gimbal_velocity',10)
        self.inject=self.create_publisher(String,'/shadow/injection',10)
    def record_tf(self,m,info):
        self.tf.extend(m.transforms)
        gid=bytes(info.get('publisher_gid',[])).hex() or 'unavailable_in_Jazzy_rclpy_metadata'
        self.tf_info_keys=sorted(info)
        for t in m.transforms:self.tf_gids.setdefault(f'{t.header.frame_id}->{t.child_frame_id}',Counter())[gid]+=1
    def record(self,k,m):
        t=stamp(m.header.stamp);self.data[k][t]=m
        if k=='raw':self.phase[t]=self.current
    def status(self,m):
        d=json.loads(m.data);self.data['status'][d['stamp_ns']]=d
    def run(self,duration,cmd=(0,0,0),gimbal=0.,phase='settle',fault=None):
        self.current=phase;self.inject.publish(String(data=json.dumps(fault or {})))
        start=time.monotonic();last=0
        while time.monotonic()-start<duration:
            if time.monotonic()-last>.025:
                m=Twist();m.linear.x,m.linear.y,m.angular.z=map(float,cmd);self.cmd.publish(m)
                self.gimbal.publish(Float64(data=float(gimbal)));last=time.monotonic()
            rclpy.spin_once(self,timeout_sec=.002)
    def graph(self):
        result={}
        client=AsyncParameterClient(self,'/planar_shadow')
        names=['publish_tf','output_odom_topic','base_frame','expected_input_odom_frame','heading_fusion.heading_topic']
        if not client.wait_for_services(timeout_sec=3.):raise RuntimeError('planar parameter readback unavailable')
        future=client.get_parameters(names)
        rclpy.spin_until_future_complete(self,future,timeout_sec=3.)
        if future.result() is None:raise RuntimeError('planar parameter readback timed out')
        result['planar_actual_parameters']={name:(v.bool_value if v.type==1 else v.string_value) for name,v in zip(names,future.result().values)}
        for name in ['full_base_shadow','planar_shadow','shadow_input_ingress']:
            result[name]=dict(subscriptions=self.get_subscriber_names_and_types_by_node(name,''),
                             publishers=self.get_publisher_names_and_types_by_node(name,''))
        for topic in ['/cmd_vel','/shadow/input/raw','/shadow/input/heading','/odometry/simulation','/shadow/planar/odom','/shadow/full_base/odom']:
            result[topic]={'publishers':[p.node_name for p in self.get_publishers_info_by_topic(topic)]}
        owners={bytes(p.endpoint_gid).hex():p.node_name for topic in ['/tf','/tf_static'] for p in self.get_publishers_info_by_topic(topic)}
        result['tf_receipt_metadata_fields']=getattr(self,'tf_info_keys',[])
        result['tf_message_sources_if_available']={e:{owners.get(g,g):n for g,n in counts.items()} for e,counts in self.tf_gids.items()}
        result['tf_edges']=dict(Counter(f'{t.header.frame_id}->{t.child_frame_id}' for t in self.tf))
        return result

def angle(R):return float(Rotation.from_matrix(R).magnitude())
def summarize(node,manifest,case):
    data=node.data;statuses=data['status'];startup_times=[v.get('diagnostics',{}).get('sources',{}).get('startup_raw_stamp_ns') for v in statuses.values()]
    t0=next((t for t in startup_times if t in data['truth']),None)
    if t0 is None:raise RuntimeError('no observed candidate startup source and matching scoring truth')
    W0=matrix(data['truth'][t0].pose.pose);h0=data['heading'][t0].yaw_rad
    planar_basis=np.eye(4);planar_basis[:3,:3]=rz(h0)
    rows=[];trace=[]
    for t,raw in sorted(data['raw'].items()):
        if t not in data['truth']:continue
        truth=matrix(data['truth'][t].pose.pose);truth[:3,3]-=W0[:3,3]
        phase=node.phase[t];status=statuses.get(t,{'status':'NO_STATUS'});row={'stamp_ns':t,'phase':phase,'status':status['status']}
        tr=dict(stamp_ns=t,phase=phase,raw=matrix(raw.pose.pose).tolist(),raw_parent=raw.header.frame_id,raw_child=raw.child_frame_id,
                truth=truth.tolist(),status=status)
        h=data['heading'].get(t)
        if h:tr['heading']=dict(yaw=h.yaw_rad,rate=h.yaw_rate_rad_s,stamp_ns=stamp(h.header.stamp),epoch=f'{h.source_boot_id}:{h.reset_counter}',valid=h.valid,online=h.online)
        joint=data['joint'].get(t);q=None
        if joint and 'gimbal_yaw_joint' in joint.name:q=joint.position[joint.name.index('gimbal_yaw_joint')];tr['scoring_joint']=q
        for k in ['planar','full']:
            if t not in data[k]:continue
            T=matrix(data[k][t].pose.pose)
            if k=='planar':T=planar_basis@T
            tr[k]=T.tolist()
            err=np.linalg.norm(T[:3,3]-truth[:3,3]);ori=angle(T[:3,:3].T@truth[:3,:3])
            rp=Rotation.from_matrix(T[:3,:3]).as_euler('xyz')[:2]-Rotation.from_matrix(truth[:3,:3]).as_euler('xyz')[:2]
            row[k]=dict(position_m=float(err),orientation_deg=float(np.degrees(ori)),roll_pitch_error_deg=np.degrees(np.abs(rp)).tolist())
            if q is not None:
                if k=='full':
                    candidates=status.get('diagnostics',{}).get('candidates',[])
                    inferred=candidates[0]['joint_delta_rad'] if len(candidates)==1 else None
                else:
                    gm=data['gimbal'].get(t);inferred=gm.relative_yaw_rad if gm and gm.valid else None
                if inferred is not None:
                    c=np.array(manifest['gimbal_center_m']);o=np.array(manifest['sensor_offset_m'])
                    lever=T[:3,:3]@(c+rz(inferred)@o)
                    actual=truth[:3,:3]@(c+rz(q)@o)
                    row[k]['lever_arm_error_m']=float(np.linalg.norm(lever-actual))
                    row[k]['joint_error_deg']=float(np.degrees(abs(np.arctan2(np.sin(inferred-q),np.cos(inferred-q)))))
        rows.append(row);trace.append(tr)
    phases={}
    for phase in sorted(set(r['phase'] for r in rows)):
        rr=[r for r in rows if r['phase']==phase];p={'input_samples':len(rr),'statuses':dict(Counter(r['status'] for r in rr))}
        for k in ['planar','full']:
            values=[r[k] for r in rr if k in r];p[k]={'output_samples':len(values)}
            for metric in ['position_m','orientation_deg','lever_arm_error_m','joint_error_deg']:
                v=[a[metric] for a in values if metric in a]
                if v:p[k][metric]={'max':max(v),'p95':float(np.percentile(v,95)),'rms':float(np.sqrt(np.mean(np.square(v))))}
            if values:p[k]['max_roll_pitch_error_deg']=np.max([v['roll_pitch_error_deg'] for v in values],axis=0).tolist()
            # Incremental pose error detects wraps and jumps without Euler differencing.
            transitions=[];ordered=[tr for tr in trace if tr['phase']==phase and k in tr]
            for a,b in zip(ordered,ordered[1:]):
                if b['stamp_ns']-a['stamp_ns']>40_000_000:continue
                da=inv(np.array(a[k]))@np.array(b[k]);dt=inv(np.array(a['truth']))@np.array(b['truth'])
                transitions.append([np.linalg.norm(da[:3,3]-dt[:3,3]),np.degrees(angle(da[:3,:3].T@dt[:3,:3]))])
            if transitions:p[k]['max_increment_error_m_deg']=np.max(transitions,axis=0).tolist()
        phases[phase]=p
    graph=node.graph()
    forbidden=['/simulation/ground_truth/odom','/simulation/sensor_truth','/joint_states']
    no_oracle=all(topic not in dict(graph[n]['subscriptions']) for n in ['full_base_shadow','planar_shadow','shadow_input_ingress'] for topic in forbidden)
    no_authority=all(topic not in dict(graph[n]['publishers']) for n in ['full_base_shadow','planar_shadow'] for topic in ['/tf','/tf_static','/cmd_vel','/odometry/simulation'])
    # Planar creates tf/tf_static publisher endpoints even with publish_tf=false. Verify messages/edges separately.
    no_authority=not any('shadow_' in t.child_frame_id for t in node.tf) and all('/cmd_vel' not in dict(graph[n]['publishers']) for n in ['full_base_shadow','planar_shadow'])
    result=dict(case=case,phases=phases,startup_stamp_ns=t0,declared_startup_roll_pitch_rad=case.get('rp',[0.,0.]),
                scoring_startup_actual_rpy_rad=Rotation.from_matrix(W0[:3,:3]).as_euler('xyz').tolist(),
                graph=graph,no_oracle_subscriptions=no_oracle,no_shadow_tf_or_commands=no_authority,
                manifest=manifest,simulation_only=True,real_LIO=False,hardware_verified=False)
    result['matched_scoring_samples']=len(rows)
    return result,trace
CASES=[
 dict(name='flat_A',phases=[('straight',3,(.25,0,0),.6),('lateral',3,(0,.25,0),.6),('base_yaw',4,(0,0,.4),0),
          ('stationary_gimbal',4,(0,0,0),.6),('combined_wrap_multiturn',15,(0,0,.4),1.2)]),
 dict(name='flat_B',lidar='B',phases=[('straight',3,(.25,0,0),.6),('lateral',3,(0,.25,0),.6),
          ('stationary_gimbal',4,(0,0,0),.6),('combined_wrap_multiturn',15,(0,0,.4),1.2)]),
 dict(name='up11_A',field='rules2026_course',spawn=[-8,0,.002,0,0,0],phases=[('up11',18,(.25,0,0),.6)]),
 dict(name='up15_A',field='rules2026_course',spawn=[-8,-4,.002,0,0,0],phases=[('up15',18,(.25,0,0),.6)]),
 dict(name='up15_B',lidar='B',field='rules2026_course',spawn=[-8,-4,.002,0,0,0],phases=[('up15',18,(.25,0,0),.6)]),
 dict(name='down11_A',field='rules2026_course',spawn=[-4,0,.303,0,0,0],phases=[('down11',18,(-.25,0,0),.6)]),
 dict(name='down15_B',lidar='B',field='rules2026_course',spawn=[-4.15,-4,.303,0,0,0],phases=[('down15',18,(-.25,0,0),.6)]),
 dict(name='bank1_A',field='banked_ground',bank=1.,spawn=[0,2,-2*np.tan(np.radians(1))+.002,-np.radians(1),0,0],rp=[-np.radians(1),0.],
      phases=[('tilted_stationary',3,(0,0,0),.6),('bank1',10,(.25,0,0),.6)]),
 dict(name='bank2_B',lidar='B',field='banked_ground',bank=2.,spawn=[0,2,-2*np.tan(np.radians(2))+.002,-np.radians(2),0,0],rp=[-np.radians(2),0.],
      phases=[('tilted_stationary',3,(0,0,0),.6),('bank2',10,(.25,0,0),.6)]),
 dict(name='missing_startup',field='banked_ground',bank=2.,spawn=[0,2,-2*np.tan(np.radians(2))+.002,-np.radians(2),0,0],missing_startup=True,
      phases=[('missing_startup',3,(0,0,0),.6)]),
 dict(name='time_faults',phases=[('nominal',2,(0,0,.4),.6),('latency20ms',3,(0,0,.4),.6,{'delay_ns':20_000_000}),
      ('latency60ms',3,(0,0,.4),.6,{'delay_ns':60_000_000}),('missing',3,(0,0,.4),.6,{'missing':True}),
      ('recover',2,(0,0,.4),.6),('invalid_heading',2,(0,0,.4),.6,{'invalid':True}),('recover2',2,(0,0,.4),.6),
      ('hidden_skew20ms',3,(0,0,.4),.6,{'delay_ns':20_000_000,'lie_stamp':True,'zero_rate':True})]),
 dict(name='heading_reset',phases=[('nominal',2,(0,0,.4),.6),('reset',2,(0,0,.4),.6,{'heading_epoch':1}),('identity_returns',2,(0,0,.4),.6)]),
 dict(name='raw_epoch',phases=[('nominal',2,(0,0,.4),.6),('epoch2',2,(0,0,.4),.6,{'raw_epoch':True}),('identity_returns',2,(0,0,.4),.6)]),
]
CASES += [
 dict(name='source_skew',phases=[('nominal',2,(0,0,.4),.6),
   ('offset05ms',2,(0,0,.4),.6,{'stamp_offset_ns':500_000}),
   ('offset1ms',2,(0,0,.4),.6,{'stamp_offset_ns':1_000_000}),
   ('offset2ms',2,(0,0,.4),.6,{'stamp_offset_ns':2_000_000}),
   ('offset5ms',2,(0,0,.4),.6,{'stamp_offset_ns':5_000_000}),
   ('offset10ms',2,(0,0,.4),.6,{'stamp_offset_ns':10_000_000}),
   ('offset20ms_alias',2,(0,0,.4),.6,{'stamp_offset_ns':20_000_000}),
   ('offline',2,(0,0,.4),.6,{'offline':True}),('recover',2,(0,0,.4),.6)]),
 dict(name='frame_fault',phases=[('nominal',2,(0,0,.4),.6),('wrong_sensor',2,(0,0,.4),.6,{'frame':True}),('recover',2,(0,0,.4),.6)]),
 dict(name='up11_B',lidar='B',field='rules2026_course',spawn=[-8,0,.002,0,0,0],phases=[('up11',18,(.25,0,0),.6)]),
 dict(name='down11_B',lidar='B',field='rules2026_course',spawn=[-4,0,.303,0,0,0],phases=[('down11',18,(-.25,0,0),.6)]),
 dict(name='down15_A',field='rules2026_course',spawn=[-4.15,-4,.303,0,0,0],phases=[('down15',18,(-.25,0,0),.6)]),
 dict(name='tilted_pitch11',field='rules2026_course',spawn=[-5.6,0,.232,-0.,-np.radians(11),0.],rp=[0.,-np.radians(11)],
      phases=[('tilted_pitch_stationary',4,(0,0,0),.6),('tilted_pitch_motion',6,(.1,0,0),.6)]),
]

def run_case(case):
    out=ROOT/'build/shadow_system'/case['name'];out.mkdir(parents=True,exist_ok=True);manifest=prepare(case,out/'scene')
    log=(out/'launch.log').open('w')
    child=subprocess.Popen(['ros2','launch','rm_full_base_shadow_validation','shadow.launch.py',f'scene:={out/"scene"}',
      'startup_roll_pitch:='+json.dumps(case.get('rp',[0.,0.])),f'missing_startup:={str(case.get("missing_startup",False)).lower()}'],
      stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Scorer()
    try:
        deadline=time.monotonic()+40
        while not node.data['status'] and time.monotonic()<deadline and child.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
        if not node.data['status']:raise RuntimeError('shadow nodes not ready: '+str(out/'launch.log'))
        node.run(.5,phase='settle')
        for p in case['phases']:node.run(p[1],p[2],p[3],p[0],p[4] if len(p)>4 else None)
        node.run(.3,phase='stop')
        drain_until=time.monotonic()+.15
        while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.002)
        result,trace=summarize(node,manifest,case)
        EVIDENCE.mkdir(parents=True,exist_ok=True)
        (EVIDENCE/(case['name']+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        with gzip.open(EVIDENCE/(case['name']+'.trace.json.gz'),'wt') as f:json.dump(trace,f,separators=(',',':'),allow_nan=False)
        print(json.dumps({'case':case['name'],'samples':len(trace),'fences':[result['no_oracle_subscriptions'],result['no_shadow_tf_or_commands']],
                          'phases':result['phases']}),flush=True)
    finally:
        node.destroy_node();rclpy.try_shutdown();os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close()
        (EVIDENCE/(case['name']+'.launch.log')).write_bytes((out/'launch.log').read_bytes())
def global_evidence(path):
    global EVIDENCE
    EVIDENCE=path.resolve();EVIDENCE.mkdir(parents=True,exist_ok=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--evidence-dir',type=Path,default=EVIDENCE/'contracts_20261008'/'physics');args=p.parse_args()
    global_evidence(args.evidence_dir)
    if os.environ.get('ROS_DOMAIN_ID','0')=='0' or not os.environ.get('GZ_PARTITION'):
        raise RuntimeError('Set an unused nonzero ROS_DOMAIN_ID and GZ_PARTITION before running')
    for case in CASES:
        if args.case is None or case['name'] in args.case:run_case(case)
if __name__=='__main__':main()
