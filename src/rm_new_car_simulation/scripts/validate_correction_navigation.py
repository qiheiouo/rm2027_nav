#!/usr/bin/env python3
"""Synthetic backend candidates in real unified Gazebo/Mission/Nav2 chain."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from validate_lio_health_navigation import HealthProbe


class CorrectionProbe(HealthProbe):
    def __init__(self):
        super().__init__()
        self.destroy_publisher(self.global_pose)
        self.global_pose=self.create_publisher(PoseWithCovarianceStamped,'/localization/amcl_pose_raw',10)
        self.bad_remaining=0;self.correction_fault=False;self.correction_states=[]
        self.prepare=self.create_client(Trigger,'/localization/reset_map_to_odom')
        self.backend_reset=self.create_client(Trigger,'/localization/amcl_backend_valid/reset_session')
        self.create_subscription(String,'/localization/correction_state',lambda m:self.correction_states.append(m.data),100)

    def on_pose(self,m):
        # Bypass HealthProbe injection: both input streams have the SAME physics timestamp.
        from validate_event_replanning import Probe
        Probe.on_pose(self,m)
        if not hasattr(self,'global_pose'):return
        self.raw.publish(m)
        p=PoseWithCovarianceStamped();p.header=copy.deepcopy(m.header);p.header.frame_id='map';p.pose=copy.deepcopy(m.pose)
        if self.correction_fault or self.bad_remaining:
            p.pose.pose.position.x+=5
            if self.bad_remaining:self.bad_remaining-=1
        self.global_pose.publish(p)

    def call(self,client):
        self.until(client.service_is_ready,5,'transaction service')
        future=client.call_async(Trigger.Request());self.until(future.done,5,'transaction reply')
        assert future.result().success,future.result().message

    def transaction(self):
        old=(self.validity.started_at.sec,self.validity.started_at.nanosec)
        count=len(self.actions['navigate_to_pose'])
        self.call(self.prepare);self.spin(.15)
        assert not self.validity.valid and self.health.healthy
        self.until(lambda:self.state.active_branch=='hold',5,'prepared validity revoked')
        self.correction_fault=False
        self.spin(.15);assert not self.validity.valid,'old backend committed prepared session'
        self.call(self.backend_reset)
        self.until(lambda:self.validity.valid,5,'new backend consensus commit')
        assert (self.validity.started_at.sec,self.validity.started_at.nanosec)>old
        self.until(lambda:len(self.actions['navigate_to_pose'])>count,8,'new session navigation')
        goals=sorted(self.actions['navigate_to_pose'].values(),key=lambda x:x['time'])
        previous,new=goals[-2:]
        terminal=[t for t,s in previous['statuses'] if s in (4,5,6)]
        assert terminal and terminal[0]<=new['time'], 'replacement before old action terminal'
        assert previous['statuses'][-1][1]==5,previous


def main(output):
    output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(50+os.getpid()%100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['GZ_PARTITION']=f'correction-{os.getpid()}';os.environ['ROS_LOG_DIR']=str(output/'ros_log')
    rclpy.init();n=CorrectionProbe();log=(output/'launch.log').open('w')
    child=subprocess.Popen(['ros2','launch','rm_navigation_bringup','simulation.launch.py',
        'use_rviz:=false','navigation_policy:=event_driven','localization_source:=correction_gate_fixture',
        'output:='+str(output/'scene')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result={'passed':False,'hardware_validated':False,'cases':{}}
    try:
        n.until(lambda:n.state and n.scan and n.costmap and n.validity and n.validity.valid,65,'profile startup')
        n.spin(1);n.target=[0.,-3.];n.mode('pursuit')
        n.until(lambda:n.paths and n.pose.x>-2.8,20,'active navigation')
        actions=len(n.actions['navigate_to_pose']);plans=len(n.actions['compute_path_to_pose'])
        for _ in range(3):
            n.bad_remaining=1;n.spin(.25)
            assert n.validity.valid and n.health.healthy
        assert len(n.actions['navigate_to_pose'])==actions
        assert len(n.actions['compute_path_to_pose'])==plans
        assert any(s.startswith('REJECT: residual') for s in n.correction_states)
        result['cases']['C13']={'reject_preserved_action':True,'extra_plans':0}
        n.transaction()
        result['cases']['C14']={'prepare_revoked_old_action':True,'terminal_before_replacement':True}
        n.until(n.complete,55,'goal completes after transaction')
        result['goal_error_m']=math.hypot(n.pose.x,n.pose.y+3)
        assert result['goal_error_m']<.25
        n.progress_failure_fixture();n.track_id='failure_wait';n.target=[2.,-3.]
        n.until(lambda:len(n.actions['wait'])>=1,20,'FollowPath failure Wait')
        n.correction_fault=True
        n.until(lambda:not n.validity.valid and n.state.active_branch=='hold',5,'fault latch veto')
        n.spin(1);plans=len(n.actions['compute_path_to_pose']);canonical=len(n.canonical);stop=[n.pose.x,n.pose.y]
        n.spin(2)
        assert n.health.healthy and len(n.canonical)>canonical
        assert len(n.actions['compute_path_to_pose'])==plans
        assert not any(abs(v)>.03 for c in n.commands[-10:] for v in c[1:])
        assert math.hypot(n.pose.x-stop[0],n.pose.y-stop[1])<.1
        n.correction_fault=False;n.spin(.4);assert not n.validity.valid,'good frame cleared latch'
        n.transaction();n.mode('hold');n.spin(1)
        result['cases']['C15']={'latched_during_wait':True,'replans_while_invalid':0,
            'healthy_lio_continued':True,'physical_stop':True,'explicit_reset_required':True}
        owners=[x.node_name for x in n.get_publishers_info_by_topic('/localization/status')]
        assert owners==['map_odom_from_global_pose'],owners
        result['validity_owners']=owners;result['passed']=True
    except Exception as e:result['error']=str(e)
    finally:
        result['actions']=n.actions;result['mission_states']=n.states;result['health_events']=n.health_events
        result['bt_events']=n.events
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=18)
        except subprocess.TimeoutExpired:
            result['passed']=False;result['shutdown_timeout']=True
            os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
        log.close()
        rows=subprocess.check_output(['ps','-eo','pid,pgid,stat,comm'],text=True).splitlines()[1:]
        residual=[line for line in rows if int(line.split()[1])==child.pid and 'Z' not in line.split()[2]]
        deaths=[line for line in (output/'launch.log').read_text().splitlines() if 'process has died' in line]
        result['clean_shutdown']=not residual and not deaths and not result.get('shutdown_timeout')
        result['process_deaths']=deaths;result['passed'] &= result['clean_shutdown']
        if residual:os.killpg(child.pid,signal.SIGKILL)
        n.destroy_node();rclpy.try_shutdown()
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('actions','mission_states','bt_events','health_events')},indent=2))
    return result['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    raise SystemExit(0 if main(p.parse_args().output) else 1)
