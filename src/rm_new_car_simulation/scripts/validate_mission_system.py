#!/usr/bin/env python3
"""Mission system interactions in the existing rebase-enabled physics composition.

No Nav2 goal client, posture producer or physical authority is introduced.
Shutdown evidence belongs to this process group and this Gazebo partition only.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from lifecycle_msgs.srv import ChangeState, GetState
from rm_competition_interfaces.msg import LocalizationStatus, RefereeState, V2MotionGuardState
from std_msgs.msg import Bool
from validate_rebase_navigation import RebaseProbe
from validate_event_replanning import obstacle

SCENARIOS = ('normal', 'patrol_blocked', 'planner_home', 'planner_session',
             'controller_home', 'replan_home', 'replan_guard', 'replan_new_target',
             'replan_lio', 'patrol_lio', 'costmap_unavailable', 'patrol_rebase', 'cancel_pending', 'long',
             'life_replan', 'life_rebase')


class SystemProbe(RebaseProbe):
    def __init__(self, bias, life_fixture=False):
        super().__init__(bias)
        self.guard_phase = V2MotionGuardState.PERMITTED
        self.guard = self.create_publisher(V2MotionGuardState, '/motion/v2_guard/state', 10)
        self.create_timer(.05, self.publish_guard)
        if life_fixture:
            self.hp = 400
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.referee = self.create_publisher(RefereeState, '/referee/state', qos)
            self.referee_valid = self.create_publisher(Bool, '/referee/state_valid', qos)
            self.create_timer(.05, self.publish_referee)
        self.state_clients = {name: self.create_client(GetState, name + '/get_state') for name in
            ('/planner_server', '/controller_server', '/bt_navigator', '/global_costmap/global_costmap')}

    def publish_guard(self):
        m = V2MotionGuardState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.endpoint_instance = 'simulation-veto-fixture'; m.phase = self.guard_phase
        self.guard.publish(m)

    def publish_referee(self):
        m = RefereeState(); m.header.stamp = self.get_clock().now().to_msg()
        m.valid = True; m.game_progress = 4; m.current_hp = self.hp
        self.referee.publish(m); self.referee_valid.publish(Bool(data=True))

    def dead(self):
        self.hp = 0
        self.until(lambda: 'life_state=dead' in self.state.detail, 3, 'DEAD authority')
        assert self.state.active_branch == 'hold' and 'navigation_authority=0' in self.state.detail

    def restore_progress(self):
        client = AsyncParameterClient(self, '/controller_server')
        future = client.set_parameters([
            Parameter('progress_checker.required_movement_radius', value=.1),
            Parameter('progress_checker.movement_time_allowance', value=3.)])
        self.until(future.done, 5, 'restore progress checker')
        assert all(x.successful for x in future.result().results)

    def lifecycle(self, name):
        client = self.state_clients[name]
        if not client.service_is_ready(): return 'unavailable'
        future = client.call_async(GetState.Request())
        try:
            self.until(future.done, 2, 'lifecycle state')
            return future.result().current_state.label
        except Exception as error:
            return 'unavailable:' + type(error).__name__


def owned_processes(group):
    rows = subprocess.check_output(['ps', '-eo', 'pid,pgid,stat,comm'], text=True).splitlines()[1:]
    return [line.strip() for line in rows if int(line.split()[1]) == group and 'Z' not in line.split()[2]]


def run_case(n, scenario, output, result):
    n.until(lambda: n.state and n.scan and n.costmap and n.validity and n.validity.valid, 65, 'startup')
    n.until(lambda: n.lifecycle('/bt_navigator') == 'active', 10, 'navigator lifecycle active')
    n.spin(.5)
    result['lifecycle_at_startup'] = {name: n.lifecycle(name) for name in n.state_clients}
    n.target = [2., -3.]
    if scenario == 'costmap_unavailable':
        client = n.create_client(ChangeState, '/planner_server/change_state')
        n.until(client.service_is_ready, 5, 'planner lifecycle')
        request = ChangeState.Request(); request.transition.id = 4  # DEACTIVATE
        future = client.call_async(request); n.until(future.done, 5, 'deactivate planner and its costmap')
        assert future.result().success
        result['inactive_costmap'] = n.lifecycle('/global_costmap/global_costmap')
        assert result['inactive_costmap'] == 'inactive'
        n.mode('patrol')
        n.until(lambda: n.state.navigation_status == 'goal_retry_limit', 20, 'bounded unavailable planner')
        before = len(n.actions['navigate_to_pose']); n.spin(2)
        assert len(n.actions['navigate_to_pose']) == before and not n.paths
        return
    n.mode('patrol')
    if scenario in ('planner_home', 'planner_session', 'cancel_pending', 'life_rebase'):
        n.until(lambda: n.active('compute_path_to_pose'), 15, 'pending planner')
    else:
        n.until(lambda: n.active('follow_path'), 20, 'FollowPath')
    if scenario == 'cancel_pending':
        n.mode('hold')
        n.until(lambda: 'cancel' in n.state.navigation_status, 3, 'cancel pending shutdown')
        return
    if scenario == 'life_rebase':
        def during_drain():
            n.until(lambda: n.validity.transaction_phase == LocalizationStatus.DRAIN_OLD_NAV, 3, 'rebase draining')
            n.dead()
        n.rebase((0., 0., 0.), result['rebases'], during_drain=during_drain, expect_resume=False)
        before = len(n.actions['navigate_to_pose']); n.spin(.5)
        assert len(n.actions['navigate_to_pose']) == before and not n.active('navigate_to_pose')
        assert 'life_state=dead' in n.state.detail and n.validity.valid
        n.mode('home'); n.hp = 400
        result['life_cases'] = ['D7', 'dead_during_drain', 'prime_does_not_revive_dead']
    elif scenario in ('patrol_rebase', 'planner_session'):
        n.rebase((0., 0., 0.), result['rebases'])
        n.mode('home')
    elif scenario in ('planner_home', 'controller_home'):
        n.mode('home')
    elif scenario == 'patrol_lio':
        old_goals = len(n.actions['navigate_to_pose'])
        n.input_mode = 'lio_silence'
        n.until(lambda: not n.validity.valid and n.state.active_branch == 'hold', 5, 'patrol LIO veto')
        n.until(lambda: not n.active('navigate_to_pose'), 5, 'patrol drained')
        before = len(n.actions['compute_path_to_pose']); n.spin(1)
        assert len(n.actions['compute_path_to_pose']) == before
        n.input_mode = 'normal'
        n.until(lambda: n.validity.valid and len(n.actions['navigate_to_pose']) > old_goals, 8, 'patrol rebuilt')
        assert n.state.active_branch == 'patrol'
        result['patrol_intent_survived_lio_loss'] = True
        n.mode('home')
    elif scenario.startswith('replan_') or scenario in ('patrol_blocked', 'long', 'life_replan'):
        if scenario != 'patrol_blocked':
            n.mode('pursuit')
            n.until(lambda: n.state.active_branch == 'pursuit' and n.active('follow_path'), 10, 'pursuit')
        if scenario == 'patrol_blocked':
            obstacle(n, output, 'patrol_blocker', -1.9, -3., .3, 1.0)
        n.progress_failure_fixture()
        n.until(lambda: n.active('wait'), 18, 'event Wait')
        n.restore_progress()
        result['intent_during_wait'] = n.state.active_branch
        if scenario == 'life_replan':
            n.dead()
            n.until(lambda: not any(n.active(name) for name in n.actions), 6, 'DEAD cancels Nav2 and children')
            before = {name: len(goals) for name, goals in n.actions.items()}; n.spin(2)
            assert before == {name: len(goals) for name, goals in n.actions.items()}
            n.mode('home'); n.hp = 400
            result['life_cases'] = ['D6', 'D8', 'no_replan_during_dead']
        if scenario in ('replan_home', 'long'):
            n.mode('home')
        if scenario in ('replan_lio', 'long'):
            n.input_mode = 'lio_silence'
            n.until(lambda: not n.validity.valid and n.state.active_branch == 'hold', 5, 'LIO veto')
            n.until(lambda: not n.active('navigate_to_pose'), 5, 'drain during LIO veto')
            before = len(n.actions['compute_path_to_pose']); n.spin(1.5)
            assert len(n.actions['compute_path_to_pose']) == before
            n.input_mode = 'normal'
            n.until(lambda: n.validity.valid, 6, 'fresh LIO recovery')
            n.mode('home')
        if scenario in ('replan_guard', 'long'):
            n.guard_phase = V2MotionGuardState.BLOCKED
            n.until(lambda: n.state.active_branch == 'hold' and not n.active('navigate_to_pose'), 5, 'guard veto')
            before = len(n.actions['compute_path_to_pose']); n.spin(1)
            assert len(n.actions['compute_path_to_pose']) == before
            n.guard_phase = V2MotionGuardState.PERMITTED; n.mode('home')
        if scenario == 'replan_new_target':
            n.track_id = 'replacement'; n.target = [-2., -5.]
        if scenario == 'patrol_blocked':
            before = len(n.actions['compute_path_to_pose'])
            n.until(lambda: len(n.actions['compute_path_to_pose']) > before, 8, 'patrol replan')
            assert n.state.active_branch == 'patrol' and len(n.actions['navigate_to_pose']) == 1
            n.mode('home')
    if scenario == 'normal':
        n.mode('home')
    n.until(n.complete, 65, 'current intent completion')
    if scenario == 'long':
        n.mode('patrol'); n.until(lambda: n.active('follow_path'), 15, 'post-veto patrol rebuild')
        n.mode('hold'); n.until(lambda: not n.active('navigate_to_pose'), 5, 'final drain')


def main(output, scenario, shutdown_backtrace=False):
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(50 + os.getpid() % 100),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', GZ_PARTITION=f'mission-system-{os.getpid()}',
        ROS_LOG_DIR=str(output / 'ros_log'))
    rclpy.init(); n = SystemProbe((.8, 0., 0.) if scenario in ('patrol_rebase', 'planner_session', 'life_rebase') else (0., 0., 0.),
        life_fixture=scenario.startswith('life_'))
    delay = 'planner' if scenario in ('planner_home', 'planner_session', 'cancel_pending', 'life_rebase') else (
        'controller' if scenario in ('controller_home', 'long') else 'none')
    log = (output / 'launch.log').open('w')
    child = subprocess.Popen(['ros2', 'launch', 'rm_navigation_bringup', 'simulation.launch.py',
        'use_rviz:=false', 'navigation_policy:=event_driven', 'localization_source:=rebase_fixture',
        'rebase_delay_fixture:=' + delay, 'output:=' + str(output / 'scene')],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = dict(scenario=scenario, functional_pass=False, hardware_validated=False, rebases=[])
    try:
        run_case(n, scenario, output, result)
        goals = sorted(n.actions['navigate_to_pose'].values(), key=lambda x: x['time'])
        for old, new in zip(goals, goals[1:]):
            ends = [t for t, status in old['statuses'] if status >= 4]
            assert ends and ends[0] <= new['time'], (old, new)
        result['max_active_mission_actions'] = 1 if goals else 0
        if scenario.startswith('life_'):
            dead_states = [row for row in n.states if 'life_state=dead' in row[-1]]
            assert dead_states and all('navigation_authority=0' in row[-1] for row in dead_states)
            result['max_dead_navigation_authority'] = 0
        result['functional_pass'] = True
    except Exception as error:
        result['error'] = repr(error)
    finally:
        result['state_before_shutdown'] = n.states[-1] if n.states else None
        result['lifecycle_before_shutdown'] = ({name: n.lifecycle(name) for name in n.state_clients}
            if scenario != 'cancel_pending' else 'skipped_to_preserve_pending_cancel')
        result['nodes_before_shutdown'] = sorted(n.get_node_names())
        if scenario != 'cancel_pending':
            # Continue feeding causal LIO/guard evidence during diagnostic IO.
            transport = subprocess.Popen(['gz', 'topic', '-l'], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True)
            try:
                n.until(lambda: transport.poll() is not None, 3, 'Gazebo topic inventory')
                result['gazebo_topics_before_shutdown'] = transport.communicate()[0].splitlines()
            except AssertionError:
                result['gazebo_transport_query_timeout'] = True
            finally:
                if transport.poll() is None:
                    transport.kill(); transport.wait(timeout=2)
        child.send_signal(signal.SIGINT)
        try: child.wait(timeout=6)
        except subprocess.TimeoutExpired:
            survivors = owned_processes(child.pid)
            result['survivors_after_6_sec'] = survivors
            waits = {}
            for row in survivors:
                pid = row.split()[0]
                try:
                    waits[pid] = {p.parent.name: p.read_text() for p in Path('/proc', pid, 'task').glob('*/wchan')}
                except (FileNotFoundError, PermissionError): pass
            result['thread_wait_channels'] = waits
            if shutdown_backtrace:
                for row in survivors:
                    pid, _, _, command = row.split()
                    if command not in ('ruby', 'gz', 'gz-sim-server'): continue
                    # Attach only AFTER the shutdown failure is observed, to this
                    # launch group's simulator; no locals or unrelated processes.
                    with (output / ('shutdown-stack-' + pid + '.txt')).open('w') as stack:
                        try:
                            subprocess.run(['gdb', '-q', '-batch', '-ex', 'set pagination off',
                                '-ex', 'thread apply all bt 8', '-p', pid], stdout=stack,
                                stderr=subprocess.STDOUT, timeout=4)
                        except subprocess.TimeoutExpired:
                            result['backtrace_timeout'] = True
            try: child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                result['shutdown_timeout'] = True; os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=3)
        log.close()
        result['launch_exit'] = child.returncode
        result['residual_processes'] = owned_processes(child.pid)
        result['process_deaths'] = [line for line in (output / 'launch.log').read_text().splitlines() if 'process has died' in line]
        result['clean_shutdown'] = not result['residual_processes'] and not result['process_deaths'] and not result.get('shutdown_timeout')
        if result['residual_processes']: os.killpg(child.pid, signal.SIGKILL)
        result['actions'] = n.actions; result['states'] = n.states; result['transactions'] = n.transactions
        result['bt_events'] = n.events
        n.destroy_node(); rclpy.try_shutdown()
        result['passed'] = result['functional_pass'] and result['clean_shutdown']
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k not in ('actions', 'states', 'transactions',
            'bt_events', 'nodes_before_shutdown', 'gazebo_topics_before_shutdown', 'thread_wait_channels')}, indent=2), flush=True)
    return result['passed']


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--scenario', choices=SCENARIOS, required=True)
    p.add_argument('--shutdown-backtrace', action='store_true', help='capture only this simulator if still alive after 6 seconds')
    p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    raise SystemExit(0 if main(args.output, args.scenario, args.shutdown_backtrace) else 1)
