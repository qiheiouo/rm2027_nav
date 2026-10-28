#!/usr/bin/env python3
"""Bounded unified-physics scenarios. Publishes Mission inputs, never Nav2 goals.

Artifacts are disposable JSON/logs outside source. Every scenario gets its own
ROS domain, Gazebo partition and process group; no hardware or serial is used.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from sensor_msgs.msg import LaserScan
from rm_competition_interfaces.msg import MissionState, PursuitCandidate
from rm_competition_interfaces.srv import SetMissionMode
from std_srvs.srv import Trigger


SCENARIOS = ('normal', 'blocked', 'no_route', 'removed', 'local', 'repeated',
             'session', 'home', 'safety', 'new_goal')


class Probe(Node):
    def __init__(self):
        super().__init__('event_replanning_probe', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.started = time.monotonic()
        self.state = None
        self.pose = None
        self.scan = None
        self.costmap = None
        self.events = []
        self.paths = []
        self.poses = []
        self.states = []
        self.commands = []
        self.actions = {name: {} for name in ('navigate_to_pose', 'compute_path_to_pose', 'follow_path', 'wait')}
        self.target = [2.0, -3.0]
        self.track_id = 'scenario'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MissionState, '/mission/state', self.on_state, latched)
        self.create_subscription(Odometry, '/odometry/simulation', self.on_pose, 20)
        self.create_subscription(NavPath, '/plan', self.on_path, 20)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, 100)
        self.create_subscription(LaserScan, '/simulation/navigation_scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, latched)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.commands.append(
            [self.elapsed(), m.linear.x, m.linear.y, m.angular.z]), 50)
        for name in self.actions:
            self.create_subscription(GoalStatusArray, f'/{name}/_action/status',
                                     lambda m, name=name: self.on_action(name, m), latched)
        self.candidate = self.create_publisher(PursuitCandidate, '/mission/pursuit_candidate', 10)
        self.create_timer(0.05, self.publish_target)
        self.mode_client = self.create_client(SetMissionMode, '/mission/set_mode')
        self.reset_client = self.create_client(Trigger, '/simulation/reset_localization_session')

    def elapsed(self):
        return round(time.monotonic() - self.started, 4)

    def on_state(self, message):
        self.state = message
        value = [message.active_branch, message.navigation_status, message.detail]
        if not self.states or self.states[-1][1:] != value:
            self.states.append([self.elapsed(), *value])

    def on_pose(self, message):
        self.pose = message.pose.pose.position
        self.poses.append([self.elapsed(), self.pose.x, self.pose.y,
                           message.twist.twist.linear.x, message.twist.twist.linear.y])

    def on_path(self, message):
        self.paths.append({'time': self.elapsed(), 'points': [
            [p.pose.position.x, p.pose.position.y] for p in message.poses]})

    def on_bt(self, message):
        self.events.extend([self.elapsed(), e.node_name, e.previous_status, e.current_status]
                           for e in message.event_log)

    def on_scan(self, message):
        self.scan = message

    def on_costmap(self, message):
        self.costmap = message

    def on_action(self, name, message):
        for entry in message.status_list:
            key = bytes(entry.goal_info.goal_id.uuid).hex()
            record = self.actions[name].setdefault(key, {'time': self.elapsed(), 'statuses': []})
            if not record['statuses'] or record['statuses'][-1][1] != entry.status:
                record['statuses'].append([self.elapsed(), entry.status])

    def publish_target(self):
        now = self.get_clock().now()
        message = PursuitCandidate()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'map'
        message.valid_until = (now + Duration(seconds=0.3)).to_msg()
        message.pose.position.x, message.pose.position.y = self.target
        message.pose.orientation.w = 1.0
        message.track_id = self.track_id
        message.source = PursuitCandidate.SOURCE_RADAR
        message.valid = True
        self.candidate.publish(message)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def until(self, condition, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.03)
            if condition():
                return
        raise AssertionError(f'timed out: {label}; state={self.states[-3:]}')

    def mode(self, mode):
        self.until(self.mode_client.service_is_ready, 5, 'Mission service')
        future = self.mode_client.call_async(SetMissionMode.Request(mode=mode, enable=True))
        self.until(future.done, 5, 'Mission reply')
        assert future.result().accepted

    def complete(self):
        return self.state is not None and self.state.navigation_status == 'goal_complete'

    def progress_failure_fixture(self):
        client = AsyncParameterClient(self, '/controller_server')
        self.until(client.services_are_ready, 5, 'controller parameters')
        result = client.set_parameters([
            Parameter('progress_checker.required_movement_radius', value=100.0),
            Parameter('progress_checker.movement_time_allowance', value=0.8)])
        self.until(result.done, 5, 'failure injection')
        assert all(r.successful for r in result.result().results)


def gz_service(node, service, request_type, request):
    # Keep Mission input leases alive during the external world mutation.
    process = subprocess.Popen(['gz', 'service', '-s', '/world/new_car_baseline/' + service,
        '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', request], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        node.until(lambda: process.poll() is not None, 8, 'Gazebo world mutation')
        out, err = process.communicate()
        assert process.returncode == 0 and 'true' in out, (out, err)
    finally:
        if process.poll() is None:
            process.kill();process.wait()


def obstacle(node, output, name, x, y, sx, sy):
    sdf = output / f'{name}.sdf'
    geometry = f'<geometry><box><size>{sx} {sy} 1.0</size></box></geometry>'
    sdf.write_text(f'<sdf version="1.9"><model name="{name}"><static>true</static>'
                   f'<pose>{x} {y} 0.5 0 0 0</pose><link name="body">'
                   f'<collision name="collision">{geometry}</collision>'
                   f'<visual name="visual">{geometry}</visual></link></model></sdf>')
    gz_service(node, 'create', 'gz.msgs.EntityFactory', f'sdf_filename: "{sdf}"')


def run_scenario(node, scenario, output, result):
    node.until(lambda: node.state and node.scan and node.pose and node.costmap, 60, 'sensor/mission/costmap startup')
    node.spin(2)
    assert node.state.active_branch == 'hold'
    owners = [n for n in node.get_node_names() if n == 'competition_mission_node']
    assert len(owners) == 1
    # Check graph action-client ownership, not just action-server goal counts.
    from rclpy.action import get_action_client_names_and_types_by_node
    clients = []
    for name, namespace in node.get_node_names_and_namespaces():
        for action, _ in get_action_client_names_and_types_by_node(node, name, namespace):
            if action == '/navigate_to_pose':
                clients.append(name)
    result['navigate_action_clients'] = clients
    # Nav2's built-in goal_pose subscription owns an internal self-client.
    # It is remapped away from the RViz shortcut in this Mission-only profile.
    assert sorted(clients) == ['bt_navigator', 'competition_mission_node'], clients
    if scenario == 'normal':
        node.target = [-1.0, -3.0]
    if scenario == 'repeated':
        node.progress_failure_fixture()
    node.mode('pursuit')
    node.until(lambda: len(node.paths) >= 1, 20, 'initial global plan')
    result['initial_path_time'] = node.paths[0]['time']
    result['injection_time'] = node.elapsed()
    if scenario in ('blocked', 'removed', 'session', 'home', 'safety', 'new_goal'):
        obstacle(node, output, 'blocker', -0.4, -3.0, 0.5, 2.4)
    elif scenario == 'local':
        # Inject AFTER planning: the retained route clips this small obstacle,
        # but MPPI can move sideways without a global route replacement.
        obstacle(node, output, 'local_obstacle', -0.5, -2.75, 0.3, 0.3)
    elif scenario == 'no_route':
        # Goal enclosure. The map was free when the first path was computed.
        for name, x, y, sx, sy in [('front', 1.2, -3, .25, 2.0),
                                  ('back', 2.8, -3, .25, 2.0),
                                  ('left', 2., -2., 1.85, .25),
                                  ('right', 2., -4., 1.85, .25)]:
            obstacle(node, output, name, x, y, sx, sy)
    if scenario in ('removed', 'session', 'home', 'safety', 'new_goal'):
        node.until(lambda: len(node.actions['wait']) > 0, 45, 'failure recovery wait')
        result['intervention_time'] = node.elapsed()
        if scenario == 'removed':
            gz_service(node, 'remove', 'gz.msgs.Entity', 'name: "blocker", type: MODEL')
        elif scenario == 'session':
            future = node.reset_client.call_async(Trigger.Request())
            node.until(future.done, 5, 'session reset')
            assert future.result().success
            node.until(lambda: 'stale_results_rejected=1' in node.state.detail, 8, 'old session terminal rejected')
            result['stale_callback_rejected'] = True
            # New session should compute its own path, after old action terminal.
        elif scenario == 'home':
            node.mode('home')
        elif scenario == 'safety':
            # Withdraw health evidence while physics/TF continue. This tests
            # the actual localization lease veto, not merely a mode command.
            health = AsyncParameterClient(node, '/new_car_simulation_truth_tf')
            node.until(health.services_are_ready, 5, 'oracle parameter service')
            future = health.set_parameters([Parameter('publish_localization_fixture', value=False)])
            node.until(future.done, 5, 'withdraw localization fixture')
            assert all(r.successful for r in future.result().results)
            node.spin(3)
            assert node.state.active_branch == 'hold'
            assert node.state.detail.startswith('safety_gate_closed')
            assert not any(abs(v) > 0.03 for c in node.commands[-10:] for v in c[1:])
            count = len(node.actions['compute_path_to_pose'])
            node.spin(2)
            assert len(node.actions['compute_path_to_pose']) == count
            result['safety_hold'] = True
            return
        elif scenario == 'new_goal':
            node.track_id = 'replacement'
            node.target = [-2.0, -5.0]
    if scenario in ('session', 'home', 'new_goal'):
        node.until(lambda: len(node.actions['navigate_to_pose']) >= 2, 10, 'Mission replacement after cancel')
        goals = sorted(node.actions['navigate_to_pose'].values(), key=lambda a: a['time'])
        terminal = [t for t, status in goals[0]['statuses'] if status in (4, 5, 6)]
        assert terminal and terminal[0] <= goals[1]['time'], 'replacement before old terminal'
        assert goals[0]['statuses'][-1][1] == 5, 'old goal was not canceled'
        result['terminal_before_replacement'] = True
    if scenario in ('no_route', 'repeated'):
        node.until(lambda: node.state.navigation_status == 'goal_retry_limit', 70, 'bounded exhaustion')
        counts = {k: len(v) for k, v in node.actions.items()}
        node.spin(5)
        assert counts == {k: len(v) for k, v in node.actions.items()}, 'retry storm'
        assert counts['navigate_to_pose'] == 1 and counts['compute_path_to_pose'] == 4, counts
        assert counts['wait'] == 3, counts
        result['bounded_exhaustion'] = True
    else:
        node.until(node.complete, 100, 'navigation completion')
        result['navigation_completed'] = True
        goal = [-3., -3.] if scenario == 'home' else node.target
        result['final_distance_m'] = math.hypot(node.pose.x-goal[0], node.pose.y-goal[1])
        assert result['final_distance_m'] < .25
        if scenario in ('normal', 'local'):
            assert len(node.actions['compute_path_to_pose']) == 1, 'unexpected replan'
            assert not node.actions['wait'], 'unexpected initial recovery'
        if scenario == 'local':
            result['local_lateral_deviation_m'] = max(abs(p[2]+3.0) for p in node.poses if -1.2 < p[1] < 0.2)
            assert result['local_lateral_deviation_m'] > 0.10, 'no measured local avoidance'
        if scenario == 'blocked':
            assert len(node.actions['navigate_to_pose']) == 1, 'Mission input lease interruption invalidates event test'
            assert len(node.actions['compute_path_to_pose']) >= 2, 'no global replan'
            assert any(e[1]=='FollowPath' and e[3]=='FAILURE' for e in node.events), 'no FollowPath failure'
            assert any(abs(y+3) > 1.3 for x, y in node.paths[-1]['points']), 'no alternate route'
            stopped = [p for p in node.poses if node.paths[0]['time']+1 < p[0] < node.paths[1]['time']
                       and math.hypot(p[3], p[4]) < .03]
            assert stopped, 'no observed stop before replan'
            result['stopped_before_replan'] = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=SCENARIOS, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['GZ_PARTITION'] = f'event-replan-{os.getpid()}'
    os.environ['ROS_LOG_DIR'] = str(args.output/'ros_log')
    rclpy.init()
    node = Probe()
    log = (args.output/'launch.log').open('w')
    child = subprocess.Popen(['ros2', 'launch', 'rm_navigation_bringup', 'simulation.launch.py',
                              'use_rviz:=false', 'navigation_policy:=event_driven',
                              'output:='+str(args.output/'scene')],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'scenario': args.scenario, 'passed': False, 'hardware_validated': False,
              'sensor': 'co-located horizontal Gazebo range fixture'}
    try:
        run_scenario(node, args.scenario, args.output, result)
        result['passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        result['actions'] = node.actions
        result['replan_count'] = max(0, len(node.actions['compute_path_to_pose']) - len(node.actions['navigate_to_pose']))
        result['paths'] = node.paths
        result['bt_events'] = node.events
        result['mission_states'] = node.states
        result['triggers'] = [e for e in node.events if e[3] == 'FAILURE' and e[1] in ('FollowPath', 'ComputePathToPose')]
        result['failure_to_new_path_sec'] = [round(next(p['time'] for p in node.paths if p['time'] > e[0])-e[0], 4)
            for e in result['triggers'] if any(p['time'] > e[0] for p in node.paths)]
        result['time_to_new_path_sec'] = [round(p['time'] - node.paths[i]['time'], 4) for i, p in enumerate(node.paths[1:])]
        result['poses'] = node.poses[::10]
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=18)
        except subprocess.TimeoutExpired:
            result['passed'] = False
            result['shutdown_timeout'] = True
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=5)
        log.close()
        # Check all process-group members, including orphaned Gazebo workers.
        rows = subprocess.check_output(['ps', '-eo', 'pid,pgid,stat,comm'], text=True).splitlines()[1:]
        residual = [line.strip() for line in rows if int(line.split()[1]) == child.pid and 'Z' not in line.split()[2]]
        result['shutdown_residuals'] = residual
        result['clean_shutdown'] = not residual and not result.get('shutdown_timeout')
        if residual:
            result['passed'] = False
            os.killpg(child.pid, signal.SIGKILL)
        result['launch_process_deaths'] = [line for line in (args.output/'launch.log').read_text().splitlines()
                                           if 'process has died' in line]
        if result['launch_process_deaths']:
            result['passed'] = False
        node.destroy_node()
        rclpy.try_shutdown()
        (args.output/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in
                         ('actions','paths','bt_events','mission_states','poses')}, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
