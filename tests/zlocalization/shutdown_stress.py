#!/usr/bin/env python3
"""Baseline experiment only: never alter production behavior or hide exit codes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import select
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('runtime_policy', ROOT/'src/rm_zlocalization_candidate/launch/runtime_policy.py')
policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--executable',required=True)
    p.add_argument('--backend-share',required=True,help='upstream backend source/install share (config/ and launch/)')
    p.add_argument('--output',required=True)
    p.add_argument('--runs',type=int,default=30)
    p.add_argument('--prediction',action='store_true')
    p.add_argument('--peer-exit',action='store_true')
    p.add_argument('--scenario',choices=('idle','stream','heavy','loop','mixed'),default='mixed')
    args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('shutdown_stress_fixture')
    odom_pub=node.create_publisher(Odometry,'/zlocalization/fastlio_odom',qos_profile_sensor_data)
    cloud_pub=node.create_publisher(PointCloud2,'/zlocalization/cloud_registered',qos_profile_sensor_data)
    prediction_pub=node.create_publisher(Odometry,'/zlocalization/unused_prediction',qos_profile_sensor_data)
    states=[]
    points=np.asarray([(float(x),float(y),float(z)) for x in np.arange(-3,3.01,.2)
            for y in np.arange(-3,3.01,.2) for z in (-1,2)]+[(float(x),float(y),float(z))
            for x in (-3,3) for y in np.arange(-3,3.01,.2) for z in np.arange(-1,2.01,.2)],dtype=np.float32)
    results=[]
    for index in range(args.runs):
        scenario=('idle','stream','heavy','loop')[index%4] if args.scenario=='mixed' else args.scenario
        states.clear()
        state_topic='/stress/state_'+str(index)
        state_sub=node.create_subscription(String,state_topic,states.append,
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        params=policy.backend_parameters(Path(args.backend_share),
            ROOT/'src/rm_zlocalization_candidate/config/z_localization_candidate.yaml',
            mode='mapping',loop=scenario=='loop',visualization=scenario in ('heavy','loop'),world_cloud=scenario=='heavy')
        params['common']['maintenance_period']=.01
        params['trajectory']['keyframe']['max_interval']=.02
        params['common']['scan_context']['exclude_recent']=2
        config=out/f'{index:03d}.yaml'
        config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        log_path=out/f'{index:03d}.log'
        with log_path.open('w') as log:
            proc=subprocess.Popen([args.executable,'--ros-args','--params-file',str(config),
                '-r','__node:=shutdown_stress_backend','-r','/tf:=/stress/quarantine/tf',
                '-r','~/localization_state:='+state_topic],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+15
            while not states and proc.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
            ready=bool(states)
            sent=0
            def feed():
                nonlocal sent
                stamp=node.get_clock().now().to_msg()
                odom=Odometry();odom.header=Header(stamp=stamp,frame_id='fast_lio_odom')
                odom.child_frame_id='lio_imu_link';odom.pose.pose.orientation.w=1.
                odom.pose.pose.position.x=.001*(sent%20)
                cloud=point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='fast_lio_odom'),
                    np.tile(points,(5,1)) if scenario=='heavy' else points)
                odom_pub.publish(odom);cloud_pub.publish(cloud)
                if args.prediction:prediction_pub.publish(odom)
                sent+=1
            if ready and scenario!='idle':
                deadline=time.monotonic()+(.15+.03*(index%5))
                while time.monotonic()<deadline and proc.poll() is None:
                    feed();rclpy.spin_once(node,timeout_sec=.002)
            projected=any('tf_published=true' in m.data for m in states)
            deadline=time.monotonic()+5
            while ready and scenario!='idle' and not projected and proc.poll() is None and time.monotonic()<deadline:
                feed();rclpy.spin_once(node,timeout_sec=.005)
                projected=any('tf_published=true' in m.data for m in states)
            peer=None
            if args.peer_exit:
                code="""import rclpy,sys,signal
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('shutdown_peer');seen=[];stopping=False
def stop(signum,frame):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop)
def receive(m):
 if not seen:print('READY',flush=True)
 seen.append(1)
n.create_subscription(String,sys.argv[1],receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 while not stopping:rclpy.spin_once(n,timeout_sec=.01)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
"""
                peer=subprocess.Popen([sys.executable,'-c',code,state_topic],stdout=subprocess.PIPE,stderr=log,text=True)
                peer_ready=False;deadline=time.monotonic()+10
                while not peer_ready and peer.poll() is None and time.monotonic()<deadline:
                    if scenario!='idle':feed()
                    rclpy.spin_once(node,timeout_sec=.002)
                    if select.select([peer.stdout],[],[],0)[0]:
                        peer_ready=peer.stdout.readline().strip()=='READY'
                if not peer_ready:
                    peer.kill();peer.wait();proc.kill();proc.wait()
                    raise RuntimeError('peer fixture did not become ready')
                peer.send_signal(signal.SIGINT)
            signal_at=time.monotonic()
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            deadline=signal_at+10
            while proc.poll() is None and time.monotonic()<deadline:
                if scenario in ('heavy','loop'):feed()
                rclpy.spin_once(node,timeout_sec=.002)
            timed_out=proc.poll() is None
            if timed_out:proc.kill()
            rc=proc.wait(timeout=5)
            peer_rc=peer.wait(timeout=5) if peer else None
        node.destroy_subscription(state_sub)
        text=log_path.read_text()
        findings=[key for key in ('ERROR: AddressSanitizer','runtime error:','WARNING: ThreadSanitizer','FATAL: ThreadSanitizer','LeakSanitizer') if key in text]
        result=dict(index=index,scenario=scenario,prediction=args.prediction,peer_exit=peer_rc,ready=ready,sent=sent,returncode=rc,
            timeout=timed_out,projected=projected,shutdown_sec=time.monotonic()-signal_at,sanitizer_findings=findings)
        results.append(result)
        (out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        if not ready or (scenario!='idle' and not projected) or timed_out or rc!=0 or findings or (peer is not None and peer_rc!=0):
            print(text[-16000:],flush=True)
            node.destroy_node();rclpy.shutdown();return 1
    node.destroy_node();rclpy.shutdown();return 0


if __name__=='__main__':sys.exit(main())
