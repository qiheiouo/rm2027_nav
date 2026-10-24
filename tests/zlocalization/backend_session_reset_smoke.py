#!/usr/bin/env python3
"""Actual backend ResetFrontend RPC + prior recovery + canonical owners stay alive.
The deterministic frontend is a protocol fixture, not a ZLIO accuracy evaluation.
"""
import argparse
import copy
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from localization_interfaces.msg import FrontendFrame, FrontendSession, LocalizationFrame
from localization_interfaces.srv import ResetFrontend
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from rm_competition_interfaces.msg import LocalizationStatus
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml

from session_transaction_smoke import fill, rotation, matrix, body_pose


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--resets',type=int,default=10);p.add_argument('--bundle-tool',default='')
    args=p.parse_args();out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('backend_reset_protocol_fixture')
    frames=node.create_publisher(FrontendFrame,'/zlocalization/frontend/frame',100)
    sessions=node.create_publisher(FrontendSession,'/zlocalization/frontend/session',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster=TransformBroadcaster(node);status=[];backend=[]
    node.create_subscription(LocalizationStatus,'/localization/status',status.append,100)
    node.create_subscription(LocalizationFrame,'/zlocalization/backend_session_frame',backend.append,100)
    points=np.asarray([(x,y,z) for x in np.arange(-3,4.01,.22) for y in np.arange(-2,3.01,.22) for z in (-1.,2.)]+
        [(x,y,z) for x in (-3.,4.) for y in np.arange(-2,3.01,.22) for z in np.arange(-1,2.01,.22)],dtype=np.float32)
    # Break the rectangular room's exact 180-degree ambiguity. This is a
    # protocol fixture, not an estimator threshold/tuning change.
    interior=np.asarray([(x,y,z) for x in (.4,.8) for y in np.arange(-1.7,.61,.12) for z in np.arange(-.4,1.51,.12)]+
        [(x,1.2,z) for x in np.arange(-2.6,-.6,.12) for z in np.arange(-.4,1.51,.12)],dtype=np.float32)
    points=np.vstack([points,interior])
    pcd=out/'prior_fixture.pcd'
    with pcd.open('w') as f:
        f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(f,points)
    from ament_index_python.packages import get_package_prefix
    tool=(Path(args.bundle_tool) if args.bundle_tool else
          Path(get_package_prefix('backend'))/'lib/backend/build_pcd_map_bundle').resolve()
    with (out/'prior_build.log').open('w') as log:
        subprocess.run([str(tool),'--input',str(pcd),'--out-bundle',str(out/'prior'),
            '--x-min','0','--x-max','0','--y-min','0','--y-max','0','--sensor-z','0',
            '--map-voxel','.15','--scan-voxel','.15','--max-keyframes','1','--sc-augment-count','1',
            '--occlusion','false','--fov-down','-90','--fov-up','90'],check=True,stdout=log,stderr=log)
    config=dict(common={'scan_context':{'augment_count':1,'lidar_height':0.},'maintenance_period':.02},
        reset={'min_gap':0.,'response_timeout':.1},startup_hint={'enabled':True,'force_until_matched':True,'x':.25,'y':-.1,'z':.4,'yaw_deg':0.})
    config_path=out/'backend.yaml';config_path.write_text(yaml.safe_dump(config))
    instance=10;epoch=0;sequence=0;reset_generation=0;gauge=rotation(.2,(1.,-2.,.1));last=None;cache={};requests=[];attempts={};mounts={};transfer_errors=[];covariance_errors=[]
    def reset(request,response):
        nonlocal epoch,reset_generation,gauge
        key=(request.requester_instance_id,request.transaction_id,request.target_epoch)
        attempts[key]=attempts.get(key,0)+1
        if key in cache:return copy.deepcopy(cache[key])
        if attempts[key]==1:
            response.outcome=ResetFrontend.Response.IN_PROGRESS
            response.requester_instance_id=request.requester_instance_id;response.transaction_id=request.transaction_id
            response.target_epoch=request.target_epoch;response.frontend_instance_id=instance
            response.reset_generation=reset_generation
            return response
        source_epoch=epoch;reset_generation+=1
        next_gauge=rotation(-.15*reset_generation,(-.3*reset_generation,.2*reset_generation,.1))
        old_from_new=gauge@np.linalg.inv(next_gauge)
        gauge=next_gauge;epoch=request.target_epoch
        response.outcome=ResetFrontend.Response.COMPLETED
        response.requester_instance_id=request.requester_instance_id;response.transaction_id=request.transaction_id
        response.target_epoch=request.target_epoch;response.frontend_instance_id=instance;response.reset_generation=reset_generation
        response.has_boundary=bool(last is not None and last.instance_id==instance)
        response.source_epoch=source_epoch
        if last is not None:response.boundary_stamp=last.odometry.header.stamp
        from session_transaction_smoke import quaternion
        response.old_from_new.translation.x,response.old_from_new.translation.y,response.old_from_new.translation.z=map(float,old_from_new[:3,3])
        q=quaternion(old_from_new[:3,:3]);response.old_from_new.rotation.x,response.old_from_new.rotation.y,response.old_from_new.rotation.z,response.old_from_new.rotation.w=map(float,q)
        cache[key]=copy.deepcopy(response);requests.append(key)
        sessions.publish(FrontendSession(instance_id=instance,epoch=epoch))
        return response
    node.create_service(ResetFrontend,'/zlocalization/reset_internal',reset)
    log=(out/'launch.log').open('w')
    process=subprocess.Popen(['ros2','launch','rm_zlocalization_candidate','z_localization_candidate.launch.py',
        'enabled:=true','frontend:=zlio','start_zlio:=false','canonical_session_transactions:=true',
        'prior_map:='+str(out/'prior'),'config_file:='+str(config_path)],stdout=log,stderr=log)
    client=node.create_client(Trigger,'/zlocalization/upstream_reset_request')
    def pump(seconds=.04):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll() is None,'candidate launch exited'
            rclpy.spin_once(node,timeout_sec=.005)
    def drive():
        nonlocal sequence,last
        sequence+=1;stamp=node.get_clock().now().to_msg()
        # Real time, exact same-source mechanical geometry. The body is stationary;
        # each reset changes only the raw numerical gauge.
        mount=rotation(.08*np.sin(sequence*.03),(.25,-.1,.4))
        t=TransformStamped();t.header=Header(stamp=stamp,frame_id='base_link');t.child_frame_id='lio_imu_link'
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,mount[:3,3])
        from session_transaction_smoke import quaternion
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,quaternion(mount[:3,:3]))
        broadcaster.sendTransform(t)
        mounts[stamp.sec*10**9+stamp.nanosec]=mount
        odom=Odometry();odom.header=Header(stamp=stamp,frame_id='zlio_odom');odom.child_frame_id='lio_imu_link'
        fill(odom.pose.pose,gauge@mount)
        for i in range(6):odom.pose.covariance[7*i]=.01
        local=(points-mount[:3,3])@mount[:3,:3]
        cloud=point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='lio_imu_link'),local.astype(np.float32))
        message=FrontendFrame(instance_id=instance,epoch=epoch,sequence=sequence,begin=stamp,
            odometry=odom,cloud=cloud,matches=len(local),lidar_updated=True)
        sessions.publish(FrontendSession(instance_id=instance,epoch=epoch));frames.publish(message);last=message
        pump()
    def valid_after(previous_epoch=-1,timeout=12.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            drive()
            if status and status[-1].valid and status[-1].session_epoch>previous_epoch:return status[-1]
        raise AssertionError('no new canonical valid session; see launch/backend state logs')
    def validate_transfer(current):
        stamp=current.pose.header.stamp.sec*10**9+current.pose.header.stamp.nanosec
        matched=[f for f in backend if f.has_odometry and f.valid and
            f.odometry.header.stamp==current.pose.header.stamp]
        deadline=time.monotonic()+1.
        while not matched and time.monotonic()<deadline:
            pump(.01)
            matched=[f for f in backend if f.has_odometry and f.valid and f.odometry.header.stamp==current.pose.header.stamp]
        assert matched and stamp in mounts,'missing atomic source'
        source=matched[-1].global_pose.pose
        def pose_matrix(p):
            x,q=p.position,p.orientation
            return matrix([x.x,x.y,x.z],[q.x,q.y,q.z,q.w])
        expected,cov=body_pose(pose_matrix(source.pose),mounts[stamp],np.asarray(source.covariance).reshape(6,6))
        error=float(np.max(np.abs(pose_matrix(current.pose.pose.pose)-expected)))
        cov_error=float(np.max(np.abs(np.asarray(current.pose.pose.covariance).reshape(6,6)-cov)))
        assert error<1e-8 and cov_error<1e-8,(error,cov_error)
        transfer_errors.append(error);covariance_errors.append(cov_error)
    result={}
    try:
        pump(2.);assert client.wait_for_service(timeout_sec=2.)
        valid_after();validate_transfer(status[-1]);owner=status[-1].session_id;bridge=status[-1].producer_id
        initial_position=status[-1].pose.pose.pose.position
        assert np.linalg.norm([initial_position.x,initial_position.y,initial_position.z])<.06,'fixture initial prior alignment failed'
        completed=[]
        for index in range(args.resets):
            old=copy.deepcopy(last);old_epoch=status[-1].session_epoch;marker=len(status);count=len(requests)
            future=client.call_async(Trigger.Request());deadline=time.monotonic()+3.
            while not future.done() and time.monotonic()<deadline:drive()
            assert future.done() and future.result().success
            deadline=time.monotonic()+3.
            while len(requests)==count and time.monotonic()<deadline:drive()
            assert len(requests)>count
            # Directly inject an old input after RPC completion, including a huge sequence.
            old.sequence=10**9;frames.publish(old)
            current=valid_after(old_epoch)
            assert any(not m.valid for m in status[marker:]),'reset had no invalid boundary'
            assert current.session_id==owner and current.producer_id==bridge,'owners restarted'
            validate_transfer(current)
            p=current.pose.pose.pose.position
            assert np.linalg.norm([p.x,p.y,p.z])<.06,'old gauge leaked into map/base pose'
            completed.append(current.session_epoch)
        result['actual_backend_reset_recovered']=len(completed)
        result['canonical_epochs']=completed
        # Frontend process replacement uses a different ID/epoch domain, forces
        # backend confirmation, then fresh prior registration recovers global pose.
        old=copy.deepcopy(last);previous=status[-1].session_epoch
        instance=20;epoch=0;sequence=0;reset_generation=0;gauge=rotation(-.6,(-2.,1.,.2));last=None
        sessions.publish(FrontendSession(instance_id=instance,epoch=epoch));frames.publish(old)
        recovered=valid_after(previous,timeout=15.)
        assert recovered.session_id==owner and recovered.producer_id==bridge
        validate_transfer(recovered)
        assert any(m.epoch>args.resets for m in backend),'backend epoch did not confirm new frontend'
        result['frontend_process_replacement_recovered']=True
        # Old native frame/session cannot pull the coordinated frontend back.
        frames.publish(old);sessions.publish(FrontendSession(instance_id=10,epoch=old.epoch))
        for _ in range(5):drive()
        assert status[-1].valid and status[-1].session_id==owner
        assert all(n>=2 for n in attempts.values()),'IN_PROGRESS was not retried with same transaction'
        result.update(old_frontend_messages_rejected=True,reset_rpc_calls=sum(attempts.values()),
            in_progress_same_transaction_retry=True,
            max_atomic_pose_transfer_error=max(transfer_errors),max_covariance_transfer_error=max(covariance_errors),
            scope='actual backend + deterministic frontend RPC + synthetic prior; no hardware/map acceptance')
        (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:
        process.send_signal(signal.SIGINT);process.wait(timeout=10);log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        assert process.returncode==0
        assert 'process has died' not in (out/'launch.log').read_text(),'child failed'


if __name__=='__main__':main()
