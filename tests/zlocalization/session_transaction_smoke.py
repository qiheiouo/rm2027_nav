#!/usr/bin/env python3
"""Real owner/bridge/translation with deterministic versioned observations and clock.
No estimator or hardware-performance claim. Injects stale work into TF waiting.
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
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from localization_interfaces.msg import LocalizationFrame
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rm_competition_interfaces.msg import LocalizationStatus
from rm_competition_interfaces.srv import SelectLocalizationSource
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/rm_zlocalization_candidate/scripts'))
from geometry import matrix,quaternion,body_pose


def rotation(yaw=0.,xyz=(0.,0.,0.)):
    return matrix(xyz,[0.,0.,np.sin(yaw/2),np.cos(yaw/2)])


def fill(pose,t):
    pose.position.x,pose.position.y,pose.position.z=map(float,t[:3,3])
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quaternion(t[:3,:3]))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('session_transaction_fixture')
    clock=node.create_publisher(Clock,'/clock',10)
    pub=node.create_publisher(LocalizationFrame,'/zlocalization/backend_session_frame',100)
    tf=TransformBroadcaster(node)
    status=[];odometry=[];transforms=[]
    node.create_subscription(LocalizationStatus,'/localization/status',status.append,100)
    node.create_subscription(Odometry,'/odometry/lio',odometry.append,100)
    node.create_subscription(TFMessage,'/tf',transforms.append,100)
    logs=[];processes=[]
    def start(package,executable,params):
        path=Path(get_package_prefix(package))/'lib'/package/executable
        log=(out/(executable+'.log')).open('w');logs.append(log)
        argv=[str(path),'--ros-args','-p','use_sim_time:=true']
        for key,value in params.items():argv+=['-p',key+':='+str(value)]
        processes.append(subprocess.Popen(argv,stdout=log,stderr=log))
    start('rm_zlocalization_candidate','candidate_adapter.py',dict(frontend='zlio',canonical_session_transactions='true'))
    start('rm_localization_adapters','lio_adapter',dict(canonical_session_transactions='true',
        expected_input_odom_frame='zlio_odom',raw_odom_parent_frame_mode='sensor_initial',
        sensor_initial_reference_mode='first_source_sample',twist_mode='finite_difference'))
    start('rm_relocalization_bridge','map_odom_from_global_pose',dict(canonical_session_transactions='true',max_global_pose_age_sec=.5))
    sim=10_000_000_000;seq=0;epoch=0;producer='fixture-backend-A';generation=1
    gauge=rotation(.3,(2.,-1.,.1));physical_x=0.;truth={};last=None
    covariance=np.diag([.01,.02,.03,.04,.05,.06])+np.outer(np.arange(6),np.arange(6))*.0001
    def pump(seconds=.03):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes),'owner exited'
            rclpy.spin_once(node,timeout_sec=.005)
    def tick(sample=True,valid=True,advance=50_000_000,mechanical=True):
        nonlocal sim,seq,last
        sim+=advance;c=Clock();c.clock=Time(nanoseconds=sim).to_msg();clock.publish(c)
        seq+=1;m=LocalizationFrame(producer_id=producer,epoch=epoch,sequence=seq,
            state_generation=generation,pose_revision=generation,has_odometry=sample,valid=valid)
        m.emitted_at=c.clock
        if sample:
            base=rotation(.02+.005*np.sin(sim/3e9),(physical_x+.005*np.sin(sim/1e9),0.,0.))
            mount=rotation(.3*np.sin(sim/1e9),(.25,-.1,.4))
            m.odometry.header.stamp=c.clock;m.odometry.header.frame_id='zlio_odom';m.odometry.child_frame_id='lio_imu_link'
            fill(m.odometry.pose.pose,gauge@base@mount)
            m.global_pose.header.stamp=c.clock;m.global_pose.header.frame_id='map'
            fill(m.global_pose.pose.pose,base@mount);m.global_pose.pose.covariance=covariance.reshape(-1).tolist()
            if mechanical:
                t=TransformStamped();t.header.stamp=c.clock;t.header.frame_id='base_link';t.child_frame_id='lio_imu_link'
                t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,mount[:3,3])
                t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,quaternion(mount[:3,:3]))
                tf.sendTransform(t)
            expected,cov=body_pose(base@mount,mount,covariance)
            truth[sim]=(expected,cov)
        pub.publish(m);last=copy.deepcopy(m);pump()
        return m
    def recover():
        start_epoch=status[-1].session_epoch if status else 0
        for _ in range(30):
            tick()
            if status and status[-1].valid:return status[-1]
        raise AssertionError('new session did not recover; see owner logs')
    def validate_latest():
        m=status[-1];assert m.valid
        stamp=m.pose.header.stamp.sec*10**9+m.pose.header.stamp.nanosec
        expected,cov=truth[stamp]
        assert abs(m.pose.pose.pose.position.x-expected[0,3])<1e-8
        assert np.max(np.abs(np.asarray(m.pose.pose.covariance).reshape(6,6)-cov))<1e-8
        def stamp_of(t):return t.header.stamp.sec*10**9+t.header.stamp.nanosec
        end=time.monotonic()+1.
        while time.monotonic()<end:
            edges=[t for packet in transforms for t in packet.transforms]
            canonical=[t for t in edges if t.header.frame_id=='odom' and t.child_frame_id=='base_link' and stamp_of(t)==stamp]
            correction=[t for t in edges if t.header.frame_id=='map' and t.child_frame_id=='odom' and stamp_of(t)>=stamp]
            if canonical and correction:break
            pump(.01)
        assert canonical and correction,'missing same-session public TF'
        def tf_matrix(t):
            p,q=t.transform.translation,t.transform.rotation
            return matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        assert np.max(np.abs(tf_matrix(correction[-1])@tf_matrix(canonical[-1])-expected))<1e-8

    result={}
    try:
        # Discovery first; repeated notices never count as measurements.
        for _ in range(15):tick(sample=False,valid=False)
        recover();validate_latest();old=copy.deepcopy(last);initial_epoch=status[-1].session_epoch
        stale=copy.deepcopy(last);stale.emitted_at.sec-=2;stale.epoch+=1000;stale.sequence+=100000
        held_epoch=status[-1].session_epoch
        pub.publish(stale);pump(.1)
        assert status[-1].valid and status[-1].session_epoch==held_epoch,'expired callback poisoned active session'
        result['expired_callback_cannot_advance_epoch']=True
        # Leave an old sample waiting for mechanical TF, then cross an epoch.
        tick(mechanical=False);epoch+=1;generation+=1;tick(sample=False,valid=False)
        gauge=rotation(-.7,(-3.,2.,.4));physical_x=2.
        marker=len(status);pub.publish(old);pump(.15)
        assert not any(x.valid for x in status[marker:]),'old epoch restored validity'
        recover();validate_latest();assert status[-1].session_epoch>initial_epoch
        result['reset_pending_tf_and_old_epoch_rejected']=True
        # Many barriers before any new observation; no restart, no physical stitching.
        previous=status[-1].session_epoch
        for _ in range(100):epoch+=1;generation+=1;tick(sample=False,valid=False,advance=1_000_000)
        recover();validate_latest();assert status[-1].session_epoch>=previous+100
        result['rapid_100_resets_recovered']=True
        # LOST then NORMAL notice + old source with NEW output sequence must stay invalid.
        held=copy.deepcopy(last);generation+=1;tick(sample=False,valid=False)
        generation+=1;tick(sample=False,valid=True)
        seq+=1;held.sequence=seq;held.state_generation=generation;held.pose_revision=generation
        marker=len(status);pub.publish(held);pump(.1)
        assert not any(x.valid for x in status[marker:])
        recover();validate_latest();result['loss_old_pose_not_resurrected']=True
        # Reordered callback from this epoch cannot overwrite current pose/covariance.
        marker=len(status);pub.publish(held);pump(.1);validate_latest()
        result['reordered_callback_rejected']=True
        # A new NORMAL revision without a new measurement revokes the old model.
        generation+=1;tick(sample=False,valid=True)
        assert not status[-1].valid,'revision-only notice retained old model pose'
        recover();validate_latest();result['revision_notice_requires_fresh_observation']=True
        # Paused ROS clock + continuing notices must expire using steady time.
        for _ in range(20):tick(sample=False,valid=True,advance=0)
        assert status and not status[-1].valid
        recover();result['pause_resume_fresh_source']=True
        # A new process requires explicit source selection; old UUID is retired.
        client=node.create_client(SelectLocalizationSource,'/zlocalization/canonical/select_source')
        assert client.wait_for_service(timeout_sec=2.)
        future=client.call_async(SelectLocalizationSource.Request(producer_id='fixture-backend-B',minimum_epoch=0))
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:pump(.01)
        assert future.done() and future.result().success
        producer='fixture-backend-B';seq=0;epoch=0;generation=1
        pub.publish(old);pump(.1);recover();validate_latest()
        result['new_producer_selected_old_retired']=True
        # Roll back the clock: catch-up alone is insufficient; require a new epoch too.
        high=sim;sim-=2_000_000_000;marker=len(status)
        tick(sample=False,valid=False,advance=0);tick();pump(.1)
        assert status and not status[-1].valid
        epoch+=1;generation+=1;tick(sample=False,valid=False)
        assert not status[-1].valid
        sim=high+100_000_000;recover();validate_latest()
        result['time_jump_new_epoch_and_highwater_recovery']=True
        assert len({x.producer_id for x in status})==1,'bridge process restarted'
        assert all(b.sequence>a.sequence for a,b in zip(status,status[1:]))
        assert len({x.session_id for x in status if x.session_id})==1,'canonical owner restarted'
        result.update(status_messages=len(status),canonical_odometry=len(odometry),scope='synthetic ROS transaction, no hardware claim')
        (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:process.wait(timeout=10)
        for log in logs:log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        assert all(p.returncode==0 for p in processes),[p.returncode for p in processes]


if __name__=='__main__':main()
