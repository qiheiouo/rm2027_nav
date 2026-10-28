#!/usr/bin/env python3
"""Physics truth -> injected raw LIO fixture -> actual adapter/bridge/Mission/Nav2.

This validates software isolation, not FAST-LIO performance or physical braking.
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import LioSourceStatus, LocalizationStatus
from validate_event_replanning import Probe


class HealthProbe(Probe):
    def __init__(self):
        super().__init__()
        self.fault = None
        self.validity = self.health = None
        self.health_events = []
        self.canonical = []
        self.raw = self.create_publisher(Odometry, '/odometry/fast_lio_raw', 10)
        self.global_pose = self.create_publisher(PoseWithCovarianceStamped, '/localization/global_pose', 10)
        self.create_subscription(LocalizationStatus, '/localization/status', self.on_validity, 100)
        self.create_subscription(LioSourceStatus, '/localization/lio_source_status', lambda m: setattr(self, 'health', m), 100)
        self.create_subscription(Odometry, '/odometry/lio', lambda m: self.canonical.append(
            [self.elapsed(), m.pose.pose.position.x, m.pose.pose.position.y]), 100)

    def on_validity(self, m):
        self.validity = m
        entry = [m.valid, m.started_at.sec, m.started_at.nanosec]
        if not self.health_events or self.health_events[-1][1:] != entry:
            self.health_events.append([self.elapsed(), *entry])

    def on_pose(self, m):
        super().on_pose(m)
        if not hasattr(self, 'raw'):
            return
        global_pose = PoseWithCovarianceStamped()
        global_pose.header = copy.deepcopy(m.header)
        global_pose.header.frame_id = 'map'
        global_pose.pose = copy.deepcopy(m.pose)
        self.global_pose.publish(global_pose)
        if self.fault == 'silence':
            return
        raw = copy.deepcopy(m)
        if self.fault == 'jump':
            raw.pose.pose.position.x += 10.0
        self.raw.publish(raw)


def fail_and_recover(n, fault, result):
    previous_goals = len(n.actions['navigate_to_pose'])
    start = n.elapsed()
    n.fault = fault
    n.until(lambda: n.validity and not n.validity.valid and n.state.active_branch == 'hold', 5, 'localization veto')
    invalid = n.elapsed()
    n.spin(1)
    assert not any(abs(v) > .03 for c in n.commands[-10:] for v in c[1:]), 'nonzero command after cancellation'
    count = len(n.actions['compute_path_to_pose'])
    canonical_count = len(n.canonical)
    stop = [n.pose.x, n.pose.y]
    n.spin(3)
    assert len(n.actions['compute_path_to_pose']) == count, 'replanning while localization invalid'
    assert len(n.canonical) == canonical_count, 'invalid source reached canonical odometry'
    assert math.hypot(n.pose.x - stop[0], n.pose.y - stop[1]) < .1, 'simulated runaway'
    assert max(p[1] for p in n.canonical) < 4, 'jump reached Nav2 input'
    n.fault = None
    n.until(lambda: n.validity.valid, 5, 'debounced source plus fresh global correction')
    n.until(lambda: len(n.actions['navigate_to_pose']) > previous_goals, 8, 'new session goal')
    goals = sorted(n.actions['navigate_to_pose'].values(), key=lambda x: x['time'])
    old, new = goals[-2:]
    terminal = [t for t, s in old['statuses'] if s in (4, 5, 6)]
    assert terminal and terminal[0] <= new['time'], 'old action survived recovery'
    assert old['statuses'][-1][1] == 5, old
    result.append({'fault': fault, 'invalid_to_hold_sec': round(invalid-start, 4),
                   'no_replan_during_invalid': True, 'bad_odom_isolated': True,
                   'terminal_before_new_goal': True, 'physical_fixture_stopped': True})


def main(output):
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['GZ_PARTITION'] = f'lio-health-{os.getpid()}'
    os.environ['ROS_LOG_DIR'] = str(output/'ros_log')
    rclpy.init(); n = HealthProbe()
    log = (output/'launch.log').open('w')
    child = subprocess.Popen(['ros2','launch','rm_navigation_bringup','simulation.launch.py',
        'use_rviz:=false','navigation_policy:=event_driven','localization_source:=lio_health_fixture',
        'output:='+str(output/'scene')], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'passed': False, 'hardware_validated': False, 'cases': []}
    try:
        n.until(lambda: n.state and n.scan and n.costmap and n.validity and n.validity.valid, 65, 'profile startup')
        n.spin(1)
        owners = [x.node_name for x in n.get_publishers_info_by_topic('/localization/status')]
        assert owners == ['map_odom_from_global_pose'], owners
        result['validity_owners'] = owners
        n.target = [-1., -3.]
        n.mode('pursuit')
        n.until(lambda: n.paths and n.pose.x > -2.8, 20, 'healthy navigation motion')
        fail_and_recover(n, 'jump', result['cases'])
        n.until(n.complete, 55, 'navigation completes after recovery')
        result['recovered_goal_error_m'] = math.hypot(n.pose.x+1, n.pose.y+3)
        assert result['recovered_goal_error_m'] < .25
        # Fault during an existing FollowPath failure / Wait interval must cancel
        # that recovery sequence before it can issue another ComputePath.
        n.progress_failure_fixture()
        n.track_id = 'failure_wait'; n.target = [2., -3.]
        n.until(lambda: len(n.actions['wait']) >= 1, 20, 'FollowPath failure recovery wait')
        fail_and_recover(n, 'silence', result['cases'])
        n.mode('hold'); n.spin(1)
        assert n.state.active_branch == 'hold'
        result['passed'] = True
    except Exception as e:
        result['error'] = str(e)
    finally:
        result['health_events'] = n.health_events
        result['actions'] = n.actions
        result['mission_states'] = n.states
        result['bt_events'] = n.events
        result['canonical_samples'] = len(n.canonical)
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=18)
        except subprocess.TimeoutExpired:
            result['passed'] = False; result['shutdown_timeout'] = True
            os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=5)
        log.close()
        rows = subprocess.check_output(['ps','-eo','pid,pgid,stat,comm'], text=True).splitlines()[1:]
        residual = [line.strip() for line in rows if int(line.split()[1]) == child.pid and 'Z' not in line.split()[2]]
        deaths = [line for line in (output/'launch.log').read_text().splitlines() if 'process has died' in line]
        result['clean_shutdown'] = not residual and not deaths and not result.get('shutdown_timeout')
        result['process_deaths'] = deaths
        result['passed'] &= result['clean_shutdown']
        if residual:
            os.killpg(child.pid, signal.SIGKILL)
        n.destroy_node(); rclpy.try_shutdown()
        (output/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('actions','mission_states','bt_events')}, indent=2), flush=True)
    return result['passed']


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--output', type=Path, required=True)
    raise SystemExit(0 if main(p.parse_args().output) else 1)
