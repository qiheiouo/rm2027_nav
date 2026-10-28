#!/usr/bin/env python3
"""Real LIO adapter + backend quality gate + correction authority; synthetic inputs."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rm_competition_interfaces.msg import LioSourceStatus, LocalizationStatus


def ns(t):
    return t.sec*10**9+t.nanosec


class Fixture(Node):
    def __init__(self):
        super().__init__('correction_injection')
        self.validity = self.health = self.backend = self.transform = None
        self.history = []; self.states = []
        self.offset = 20.; self.mode = 'normal'; self.emit_global = True; self.emit_lio = True
        self.x = 0.; self.yaw = 0.; self.move = False
        self.raw = self.create_publisher(Odometry, '/odometry/fast_lio_raw', 10)
        self.global_pose = self.create_publisher(PoseWithCovarianceStamped, '/localization/amcl_pose_raw', 10)
        self.replay = self.create_publisher(LocalizationStatus, '/localization/amcl_backend_valid/status', 10)
        for typ, topic, attr in [(LocalizationStatus,'/localization/status','validity'),
            (LioSourceStatus,'/localization/lio_source_status','health'),
            (LocalizationStatus,'/localization/amcl_backend_valid/status','backend'),
            (TransformStamped,'/localization/map_to_odom','transform')]:
            self.create_subscription(typ,topic,lambda m,a=attr:setattr(self,a,m),100)
        self.create_subscription(LocalizationStatus,'/localization/status',lambda m:self.history.append(m),100)
        self.create_subscription(String,'/localization/correction_state',lambda m:self.states.append(m.data),100)
        self.prepare = self.create_client(Trigger,'/localization/reset_map_to_odom')
        self.reset_backend = self.create_client(Trigger,'/localization/amcl_backend_valid/reset_session')
        self.create_timer(.05,self.publish)

    def publish(self):
        now=self.get_clock().now()
        if self.move:
            self.x+=.025; self.yaw+=.35
        raw=Odometry();raw.header.stamp=now.to_msg();raw.header.frame_id='odom';raw.child_frame_id='base_link'
        raw.pose.pose.position.x=self.x
        raw.pose.pose.orientation.z=math.sin(self.yaw/2);raw.pose.pose.orientation.w=math.cos(self.yaw/2)
        if self.move:
            raw.twist.twist.linear.x=.5*math.cos(self.yaw)
            raw.twist.twist.linear.y=-.5*math.sin(self.yaw)
            raw.twist.twist.angular.z=7.
        if self.emit_lio:self.raw.publish(raw)
        p=PoseWithCovarianceStamped();p.header=copy.deepcopy(raw.header);p.header.frame_id='map';p.pose=copy.deepcopy(raw.pose)
        p.pose.pose.position.x+=self.offset
        mode=self.mode
        if mode=='large':p.pose.pose.position.x+=5
        elif mode=='stale':p.header.stamp=(now-Duration(seconds=2)).to_msg()
        elif mode=='nan':p.pose.pose.position.x=float('nan')
        elif mode=='inf':p.pose.pose.position.x=float('inf')
        elif mode=='quality':p.pose.covariance[0]=99.
        elif mode=='malformed':p.pose.pose.orientation.w=p.pose.pose.orientation.z=0.
        if self.emit_global:self.global_pose.publish(p)

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)

    def until(self,predicate,timeout=5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.spin(.02)
            if predicate():return
        raise AssertionError(f'timeout {self.mode}; valid={self.validity and self.validity.valid}; states={self.states[-8:]}')

    def call(self,client):
        assert client.wait_for_service(timeout_sec=3)
        f=client.call_async(Trigger.Request());self.until(f.done)
        assert f.result().success,f.result().message

    def transaction(self,offset):
        old=ns(self.validity.started_at)
        self.call(self.prepare);self.spin(.08)
        assert not self.validity.valid
        self.offset=offset;self.mode='normal'
        # Old backend remains active but cannot authorize the prepared transaction.
        self.spin(.2);assert not self.validity.valid
        self.call(self.reset_backend)
        self.until(lambda:self.validity.valid)
        assert ns(self.validity.started_at)>old
        self.spin(.1)
        assert abs(self.transform.transform.translation.x-offset)<.02


def run(output):
    output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(50+os.getpid()%100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST';os.environ['ROS_LOG_DIR']=str(output/'ros_log')
    rclpy.init();n=Fixture();children=[];logs=[]
    result={'passed':False,'hardware_validated':False,'cases':{}}
    try:
        for package,exe,params in [
            ('rm_localization_adapters','lio_adapter',['input_health.enabled:=true','input_health.trust_raw_twist:=true']),
            ('rm_relocalization_bridge','global_pose_gate_node',[]),
            ('rm_relocalization_bridge','map_odom_from_global_pose',['source_health_topic:=/localization/lio_source_status',
                'upstream_valid_topic:=/localization/amcl_backend_valid','correction_gate.enabled:=true'])]:
            log=(output/f'{exe}.log').open('w');logs.append(log)
            command=[str(Path(get_package_prefix(package))/'lib'/package/exe),'--ros-args']
            for p in params:command+=['-p',p]
            children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        n.until(lambda:n.validity and n.validity.valid);n.spin(.2)
        assert abs(n.transform.transform.translation.x-20)<.01
        result['startup_large_20m']=True
        n.offset+=.1;n.spin(.2);assert abs(n.transform.transform.translation.x-n.offset)<.01
        result['cases']['C1']=True
        # Real backend quality/freshness rejection; never reaches trusted correction.
        for case,mode in [('C2','stale'),('C4_nan','nan'),('C4_inf','inf'),('G7','quality'),('G3','malformed')]:
            before=n.transform.transform.translation.x;n.mode=mode;n.spin(.18)
            assert not n.validity.valid and n.health.healthy
            assert n.transform.transform.translation.x==before
            n.mode='normal';n.until(lambda:n.validity.valid)
            result['cases'][case]=True
        n.spin(.1);before=n.transform.transform.translation.x
        n.mode='large';n.publish();n.mode='normal';n.spin(.04)
        assert n.validity.valid and n.transform.transform.translation.x==before
        n.spin(.15);result['cases']['C5']=True
        for _ in range(6):
            n.mode='large';n.publish();n.mode='normal';n.spin(.15)
            assert n.validity.valid and abs(n.transform.transform.translation.x-before)<.01
        result['cases']['C7']=True
        retired=copy.deepcopy(n.backend)
        n.mode='large';n.until(lambda:any(s.startswith('FAULT_LATCHED') for s in n.states))
        n.mode='normal';n.spin(.4)
        assert not n.validity.valid and n.health.healthy
        assert abs(n.transform.transform.translation.x-before)<.01
        result['cases']['C6']=result['cases']['C10']=True
        n.transaction(-15.);result['cases']['C8']=True
        # Replay delayed old-session valid AND invalid status with fresh envelope.
        epoch=ns(n.validity.started_at)
        for valid in (True,False):
            retired.header.stamp=n.get_clock().now().to_msg();retired.sequence+=100;retired.valid=valid
            retired.valid_until=(n.get_clock().now()+Duration(seconds=.8)).to_msg()
            n.replay.publish(retired);n.spin(.12)
            assert n.validity.valid and ns(n.validity.started_at)==epoch
            assert abs(n.transform.transform.translation.x+15)<.01
        result['cases']['C3']=result['cases']['C9']=True
        replay=copy.deepcopy(n.backend)
        replay.sequence=1;replay.header.stamp=n.get_clock().now().to_msg()
        n.replay.publish(replay);n.spin(.12)
        assert n.validity.valid and abs(n.transform.transform.translation.x+15)<.01
        result['reordered_same_epoch']=True
        # Healthy-looking corrections never recover absent/RECOVERING LIO.
        n.emit_lio=False;n.until(lambda:not n.health.healthy and not n.validity.valid)
        n.spin(.4);assert not n.validity.valid
        n.mode='large';n.spin(.2);assert not n.validity.valid and not n.health.healthy
        n.mode='normal'
        result['both_untrusted']=True
        n.emit_lio=True;n.spin(.11);assert not n.validity.valid
        n.until(lambda:n.health.healthy and n.validity.valid)
        result['cases']['C11']=True
        n.emit_global=False;n.spin(1.4)
        assert n.health.healthy and not n.validity.valid
        n.emit_global=True;n.until(lambda:n.validity.valid);result['cases']['C12']=True
        n.move=True;n.spin(1.3);assert n.health.healthy and n.validity.valid
        n.move=False;n.spin(.2);result['moving_7_rad_s']=True
        # Gradual 2 m drift is accepted; one large drift requires explicit transaction.
        start=n.offset
        for _ in range(10):n.offset+=.2;n.spin(.15);assert n.validity.valid
        assert abs(n.transform.transform.translation.x-(start+2))<.02
        result['accumulated_drift_2m']=True
        # Interrupted transaction verification must restart, never count the old sample.
        n.emit_global=False;n.call(n.prepare);n.spin(.08);n.call(n.reset_backend)
        def one_sample():
            n.emit_global=True;n.publish();n.emit_global=False;n.spin(.08)
        one_sample();assert not n.validity.valid
        n.spin(1.2)
        one_sample();assert not n.validity.valid
        one_sample();assert not n.validity.valid
        one_sample();n.until(lambda:n.validity.valid)
        result['interrupted_transaction_restarts_consensus']=True
        owners=[x.node_name for x in n.get_publishers_info_by_topic('/localization/status')]
        assert owners==['map_odom_from_global_pose'],owners
        result['validity_owners']=owners;result['passed']=True
    except Exception as e:result['error']=str(e)
    finally:
        for child in children:child.send_signal(signal.SIGINT)
        exits=[]
        for child in children:
            try:exits.append(child.wait(timeout=8))
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait();exits.append('timeout')
        for log in logs:log.close()
        result['shutdown_exit_codes']=exits;result['passed'] &= all(c==0 for c in exits)
        result['diagnostic_states']=sorted(set(s.split(':')[0] for s in n.states))
        n.destroy_node();rclpy.try_shutdown()
        (output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return result['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    raise SystemExit(0 if run(p.parse_args().output) else 1)
