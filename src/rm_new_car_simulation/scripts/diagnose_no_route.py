#!/usr/bin/env python3
"""Observe unchanged no_route acceptance; never send a Nav2 goal.

Run in a sourced workspace with --scenario no_route --output /tmp/<directory>.
The inherited validator retains its timeout and pass/fail decision. After a
failure with an active goal, request Mission HOLD and record cancellation
separately. Full feedback/maps/poses are disposable output, not Git evidence.
"""
import copy
import json
import time

from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import GetState
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose, Wait
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict as to_dict

import validate_event_replanning as base

OriginalProbe = base.Probe
original_run = base.run_scenario
TYPES = dict(follow_path=FollowPath, compute_path_to_pose=ComputePathToPose,
             navigate_to_pose=NavigateToPose, wait=Wait)


class Observe(OriginalProbe):
    def __init__(self):
        self.observation = dict(feedback={}, results={}, lifecycle=[],
                                costmap_updates={}, clock=[], goal_stamps={}, parameters={})
        self.result_requests = set()
        super().__init__()
        nodes = ['controller_server', 'planner_server', 'bt_navigator',
                 'global_costmap/global_costmap', 'local_costmap/local_costmap']
        self.life_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in nodes}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps = {}
        self.map_snapshots = {}
        for name in ['global_costmap', 'local_costmap']:
            self.create_subscription(OccupancyGrid, f'/{name}/costmap',
                                     lambda m, n=name: self.map_full(n, m), latched)
            self.create_subscription(OccupancyGridUpdate, f'/{name}/costmap_updates',
                                     lambda m, n=name: self.map_update(n, m), 10)
        self.parameters_client = AsyncParameterClient(self, '/controller_server')
        self.parameters_started = False
        self.create_timer(2., self.sample)

    def mode(self, mode):
        # Original ownership check runs first. ROS graph discovery classifies
        # even GetResult-only clients as action clients; these cannot send goals.
        if not hasattr(self, 'result_clients'):
            self.result_clients = {
                n: self.create_client(t.Impl.GetResultService, f'/{n}/_action/get_result')
                for n, t in TYPES.items()}
            for name, action_type in TYPES.items():
                self.create_subscription(action_type.Impl.FeedbackMessage,
                                         f'/{name}/_action/feedback',
                                         lambda m, n=name: self.feedback(n, m), 100)
        super().mode(mode)

    def feedback(self, name, message):
        key = bytes(message.goal_id.uuid).hex()
        row = [self.elapsed(), self.get_clock().now().nanoseconds / 1e9,
               to_dict(message.feedback)]
        self.observation['feedback'].setdefault(name, {}).setdefault(key, []).append(row)

    def on_action(self, name, message):
        super().on_action(name, message)
        for entry in message.status_list:
            key = bytes(entry.goal_info.goal_id.uuid).hex()
            self.observation['goal_stamps'][key] = to_dict(entry.goal_info.stamp)
            if (name, key) in self.result_requests or not hasattr(self, 'result_clients'):
                continue
            client = self.result_clients[name]
            if not client.service_is_ready():
                continue
            self.result_requests.add((name, key))
            request = TYPES[name].Impl.GetResultService.Request()
            request.goal_id = entry.goal_info.goal_id
            client.call_async(request).add_done_callback(
                lambda f, n=name, k=key: self.observation['results'].setdefault(n, {}).update(
                    {k: [self.elapsed(), to_dict(f.result())]}))

    def map_full(self, name, message):
        self.maps[name] = message
        self.map_tick(name, 'full', message.header.stamp)

    def map_update(self, name, message):
        grid = self.maps.get(name)
        if grid:
            for y in range(message.height):
                offset = (message.y + y) * grid.info.width + message.x
                grid.data[offset:offset + message.width] = message.data[
                    y * message.width:(y + 1) * message.width]
        self.map_tick(name, 'update', message.header.stamp)

    def map_tick(self, name, kind, stamp):
        self.observation['costmap_updates'].setdefault(name, []).append(
            [self.elapsed(), kind, stamp.sec + stamp.nanosec / 1e9])

    def sample(self):
        self.observation['clock'].append(
            [self.elapsed(), self.get_clock().now().nanoseconds / 1e9])
        for name, client in self.life_clients.items():
            if client.service_is_ready():
                client.call_async(GetState.Request()).add_done_callback(
                    lambda f, n=name: self.observation['lifecycle'].append(
                        [self.elapsed(), n, f.result().current_state.id,
                         f.result().current_state.label]))
        if not self.parameters_started and self.parameters_client.services_are_ready():
            self.parameters_started = True
            names = ['progress_checker_plugins', 'progress_checker.plugin',
                     'progress_checker.required_movement_radius',
                     'progress_checker.movement_time_allowance', 'failure_tolerance',
                     'costmap_update_timeout', 'FollowPath.regenerate_noises',
                     'controller_frequency']
            self.parameters_client.get_parameters(names).add_done_callback(
                lambda f: self.observation['parameters'].update(
                    dict(zip(names, [to_dict(v) for v in f.result().values]))))

    def capture_maps(self, phase):
        # Copy compact ROS arrays now; defer large JSON serialization until all
        # actions terminate so observation cannot interrupt the candidate lease.
        self.map_snapshots[phase] = copy.deepcopy(self.maps)

    def save(self, output):
        self.observation.update(poses_full=self.poses, commands_full=self.commands)
        for phase, maps in self.map_snapshots.items():
            self.observation['maps_' + phase] = {n: to_dict(m) for n, m in maps.items()}
        (output / 'observation.json').write_text(json.dumps(self.observation))


def run(node, scenario, output, result):
    assert scenario == 'no_route', 'This diagnostic is scoped to no_route'
    result['mission_package_prefix'] = get_package_prefix('rm_competition_mission')
    started = time.monotonic()
    try:
        original_run(node, scenario, output, result)
    finally:
        result['scenario_wall_time'] = time.monotonic() - started
        result['original_verdict_time'] = node.elapsed()
        result['original_actions'] = json.loads(json.dumps(node.actions))
        result['original_mission_state'] = node.states[-1] if node.states else None
        node.capture_maps('verdict')
        # Intervention AFTER the original verdict; no timeout extension and no
        # direct Nav2 cancel. This is a separate Mission -> BT -> controller check.
        if node.state and node.state.navigation_status == 'goal_active':
            result['hold_intervention_time'] = node.elapsed()
            try:
                node.mode('hold')
                node.until(lambda: all(v['statuses'][-1][1] in (4, 5, 6)
                                       for a in node.actions.values() for v in a.values()),
                           5, 'diagnostic hold cancellation terminal')
                node.spin(.5)
                result['hold_terminal_time'] = node.elapsed()
                result['hold_final_state'] = node.states[-1]
            except Exception as exc:
                # Preserve the original failure instead of masking it.
                result['diagnostic_hold_error'] = str(exc)
            node.capture_maps('after_hold')
        node.save(output)


if __name__ == '__main__':
    base.Probe = Observe
    base.run_scenario = run
    raise SystemExit(base.main())
