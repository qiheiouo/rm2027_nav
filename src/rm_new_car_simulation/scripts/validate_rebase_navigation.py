#!/usr/bin/env python3
"""Explicit SE(2) corrections in unified Gazebo, real Mission/Nav2 and cache layer.

Candidates are controlled physics-derived fixtures, not AMCL accuracy evidence.
The planner/controller delay decorators only change result/cancel timing.
"""
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
from ament_index_python.packages import get_package_prefix
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rm_competition_interfaces.msg import LocalizationStatus, LocalizationBarrier
from rm_competition_interfaces.srv import BeginRebase
from validate_correction_navigation import CorrectionProbe
from validate_event_replanning import Probe


def ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


class RebaseProbe(CorrectionProbe):
    def __init__(self, bias):
        super().__init__()
        self.bias=bias
        self.backend=None;self.transform=None;self.input_mode='normal'
        self.transforms=[];self.transactions=[];self.barriers=[];self.odometry=[]
        self.local_map=None;self.truth=None
        self.begin=self.create_client(BeginRebase,'/localization/begin_rebase')
        self.create_subscription(LocalizationStatus,'/localization/amcl_backend_valid/status',
                                 lambda m:setattr(self,'backend',m),100)
        self.create_subscription(TransformStamped,'/localization/map_to_odom',self.on_transform,100)
        self.create_subscription(Odometry,'/odometry/lio',self.on_odom,100)
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:setattr(self,'local_map',m),10)
        self.create_subscription(LocalizationBarrier,'/localization/map_cache_primed',
                                 lambda m:self.barriers.append([self.elapsed(),ns(m.started_at)]),100)

    def on_pose(self,m):
        Probe.on_pose(self,m)
        if not hasattr(self,'bias'):return
        self.truth=m
        self.raw.publish(m)
        p=PoseWithCovarianceStamped();p.header=copy.deepcopy(m.header)
        p.header.frame_id='map';p.pose=copy.deepcopy(m.pose)
        dx,dy,angle=self.bias
        # Rotation around the spawn point keeps the biased pose inside this test corridor.
        x,y=p.pose.pose.position.x+3,p.pose.pose.position.y+3
        p.pose.pose.position.x=-3+math.cos(angle)*x-math.sin(angle)*y+dx
        p.pose.pose.position.y=-3+math.sin(angle)*x+math.cos(angle)*y+dy
        yaw=2*math.atan2(p.pose.pose.orientation.z,p.pose.pose.orientation.w)+angle
        p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2)
        if self.input_mode=='nan':p.pose.pose.position.x=float('nan')
        elif self.input_mode=='outlier':p.pose.pose.position.x+=5
        if self.input_mode!='silent':self.global_pose.publish(p)

    def on_validity(self,m):
        super().on_validity(m)
        if hasattr(self,'transactions'):
            row=[m.transaction_phase,m.valid,ns(m.started_at),ns(m.committed_at)]
            if not self.transactions or self.transactions[-1][1:]!=row:
                self.transactions.append([self.elapsed(),*row])

    def on_transform(self,m):
        self.transform=m
        self.transforms.append([self.elapsed(),ns(m.header.stamp),m.transform.translation.x,
            m.transform.translation.y,2*math.atan2(m.transform.rotation.z,m.transform.rotation.w)])

    def on_odom(self,m):
        self.odometry.append([self.elapsed(),ns(m.header.stamp),m.pose.pose.position.x,m.pose.pose.position.y,
            m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])

    def active(self,name):
        return [key for key,value in self.actions[name].items() if value['statuses'][-1][1]<4]

    def request(self):
        self.until(self.begin.service_is_ready,5,'begin service')
        req=BeginRebase.Request(producer_id=self.validity.producer_id,
            expected_session=self.validity.started_at,candidate_stamp=self.backend.pose.header.stamp)
        future=self.begin.call_async(req);self.until(future.done,5,'begin response')
        return future.result()

    def rebase(self,bias,result):
        old_epoch=ns(self.validity.started_at)
        old_goals={name:set(self.actions[name]) for name in self.actions}
        old_c=self.transforms[-1][2:]
        start=self.elapsed();self.bias=bias
        self.spin(.35)  # three distinct, same-source quality samples at the new correction
        reply=self.request();assert reply.accepted,reply.detail
        epoch=ns(reply.transaction_session)
        assert epoch>old_epoch
        self.until(lambda:self.validity.valid and ns(self.validity.started_at)==epoch,10,'prime and active')
        commit=next(row[0] for row in self.transactions if row[1]==LocalizationStatus.PRIME_NEW_SESSION and row[3]==epoch)
        # All old upper/child actions must have terminal statuses before C changes.
        terminal={}
        for name,records in self.actions.items():
            for key,rec in records.items():
                if rec['time']>=commit:continue
                ends=[t for t,s in rec['statuses'] if s>=4]
                assert ends and ends[0]<=commit,(name,rec,commit)
                terminal[name]=max(terminal.get(name,0),ends[0])
        assert any(e==epoch for _,e in self.barriers),'missing actual obstacle-layer receipt'
        new_goals=[v for k,v in self.actions['navigate_to_pose'].items() if k not in old_goals['navigate_to_pose']]
        self.until(lambda:len(self.actions['navigate_to_pose'])>len(old_goals['navigate_to_pose']),6,'new Mission goal')
        new_goals=[v for k,v in self.actions['navigate_to_pose'].items() if k not in old_goals['navigate_to_pose']]
        assert all(v['time']>=commit for v in new_goals)
        # No canonical odometry reset or map-jump-derived velocity.
        samples=[x for x in self.odometry if start-.1<=x[0]<=self.elapsed()]
        max_step=max((math.hypot(b[2]-a[2],b[3]-a[3]) for a,b in zip(samples,samples[1:])),default=0)
        max_speed=max((math.hypot(x[4],x[5]) for x in samples),default=0)
        assert max_step<.15 and max_speed<2.,(max_step,max_speed)
        result.append({'epoch':epoch,'old_correction':old_c,'new_correction':self.transforms[-1][2:],
            'commit_time':commit,'old_terminal_times':terminal,'odom_max_step_m':max_step,
            'twist_max_speed_mps':max_speed,'map_cache_primed':True})


def run(output,scenario):
    output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(50+os.getpid()%100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['GZ_PARTITION']=f'rebase-{os.getpid()}'
    os.environ['ROS_LOG_DIR']=str(output/'ros_log')
    initial={'normal':(0.,0.,0.),'translation':(.8,0.,0.),'yaw':(0.,0.,.5),
             'combined':(.6,.3,.5),'planner':(.8,0.,0.),'controller':(.8,0.,0.),
             'event':(.8,0.,0.),'repeated':(1.,0.,0.),'invalid':(0.,0.,0.)}[scenario]
    rclpy.init();n=RebaseProbe(initial)
    delay=scenario if scenario in ('planner','controller') else 'none'
    log=(output/'launch.log').open('w')
    owner_log=(output/'tf_owner_probe.log').open('w')
    owner_probe=subprocess.Popen([str(Path(get_package_prefix('rm_new_car_simulation'))/
        'lib/rm_new_car_simulation/s2_tf_owner_probe'),'--ros-args','-p','use_sim_time:=true',
        '-p','output:='+str(output/'tf_owners.json')],stdout=owner_log,stderr=subprocess.STDOUT,start_new_session=True)
    child=subprocess.Popen(['ros2','launch','rm_navigation_bringup','simulation.launch.py',
        'use_rviz:=false','navigation_policy:=event_driven','localization_source:=rebase_fixture',
        'rebase_delay_fixture:='+delay,'output:='+str(output/'scene')],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result={'passed':False,'scenario':scenario,'hardware_validated':False,'rebases':[]}
    try:
        n.until(lambda:n.state and n.scan and n.costmap and n.validity and n.validity.valid,65,'startup')
        n.spin(.5)
        if scenario=='invalid':
            epoch=ns(n.validity.started_at);before=n.transforms[-1][2:]
            n.input_mode='nan';n.spin(.7)
            reply=n.request();assert not reply.accepted and ns(n.validity.started_at)==epoch
            assert n.transforms[-1][2:]==before
            result['invalid_candidate_rejected']=True
            n.input_mode='outlier';n.spin(.6)
            n.input_mode='normal';n.spin(.5)
            assert not n.validity.valid,'ordinary candidate cleared latch'
            n.target=[-1.,-3.];n.mode('pursuit')
            n.rebase((0.,0.,0.),result['rebases'])
            result['only_explicit_verified_request_clears_latch']=True
        else:
            n.target=[-1.,-3.];n.mode('pursuit')
            if scenario=='planner':
                n.until(lambda:n.active('compute_path_to_pose'),15,'pending planner')
            else:
                n.until(lambda:n.active('follow_path'),20,'active FollowPath')
            if scenario=='event':
                n.progress_failure_fixture()
                n.until(lambda:n.active('wait'),15,'event recovery Wait')
                client=AsyncParameterClient(n,'/controller_server')
                future=client.set_parameters([Parameter('progress_checker.required_movement_radius',value=.1),
                    Parameter('progress_checker.movement_time_allowance',value=3.)])
                n.until(future.done,5,'restore progress checker')
                assert all(x.successful for x in future.result().results)
            if scenario=='repeated':
                n.rebase((.45,0.,.35),result['rebases'])
                n.rebase((0.,0.,0.),result['rebases'])
            elif scenario!='normal':n.rebase((0.,0.,0.),result['rebases'])
        n.until(n.complete,60,'goal completion')
        result['goal_error_m']=math.hypot(n.pose.x+1,n.pose.y+3)
        assert result['goal_error_m']<.25,result
        result['plan_count']=len(n.actions['compute_path_to_pose'])
        assert result['plan_count']<=len(result['rebases'])+3,'event replan storm'
        result['validity_owners']=[x.node_name for x in n.get_publishers_info_by_topic('/localization/status')]
        assert result['validity_owners']==['map_odom_from_global_pose']
        owners=json.loads((output/'tf_owners.json').read_text())
        assert set(owners)=={'map>odom','odom>base_link'},owners
        result['tf_owner_counts']={edge:len(ids) for edge,ids in owners.items()}
        assert all(count==1 for count in result['tf_owner_counts'].values()),result['tf_owner_counts']
        result['passed']=True
    except Exception as error:result['error']=repr(error)
    finally:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=18)
        except subprocess.TimeoutExpired:
            result['shutdown_timeout']=True;os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
        log.close()
        owner_probe.send_signal(signal.SIGINT)
        try:result['owner_probe_exit']=owner_probe.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(owner_probe.pid,signal.SIGKILL);owner_probe.wait();result['owner_probe_exit']='timeout'
        owner_log.close()
        rows=subprocess.check_output(['ps','-eo','pid,pgid,stat,comm'],text=True).splitlines()[1:]
        residual=[line for line in rows if int(line.split()[1])==child.pid and 'Z' not in line.split()[2]]
        deaths=[line for line in (output/'launch.log').read_text().splitlines() if 'process has died' in line]
        result['clean_shutdown']=not residual and not deaths and not result.get('shutdown_timeout') and result['owner_probe_exit']==0
        result['process_deaths']=deaths
        result['passed'] &= result['clean_shutdown']
        if residual:os.killpg(child.pid,signal.SIGKILL)
        result['transactions']=n.transactions;result['actions']=n.actions;result['barriers']=n.barriers
        result['mission_states']=n.states;result['bt_events']=n.events
        n.destroy_node();rclpy.try_shutdown()
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in
            ('transactions','actions','barriers','mission_states','bt_events')},indent=2),flush=True)
    return result['passed']


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=['normal','translation','yaw','combined','planner','controller','event','repeated','invalid'],required=True)
    args=parser.parse_args();raise SystemExit(0 if run(args.output,args.scenario) else 1)
