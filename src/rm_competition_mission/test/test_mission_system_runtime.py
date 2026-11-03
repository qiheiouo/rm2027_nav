"""System invariants with real Mission/pursuit and a causal delayed action server.

The fixture is the only safety/localization source, not a hardware authority.
Action request ordinals are observed commitment generations, not private C++ counters.
"""
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rm_competition_interfaces.msg import PursuitCandidate, V2MotionGuardState
import yaml

from test_navigation_arbitration_runtime import ArbitrationFixture, wait, wait_goal, stable_count


class SystemFixture(ArbitrationFixture):
    def __init__(self):
        super().__init__()
        self.abort = set()
        self.late_failure = False
        self.trace = []
        self.states = []
        self.action_ids = {}
        self.cases = []
        self.result_replies = []
        # Test-only wire fault injection against the installed Jazzy action API.
        # Production Mission still receives results through rclcpp_action.
        self.original_result_reply = self.nav._send_result_response
        def remember_result(header, future):
            self.result_replies.append((header, future))
            self.original_result_reply(header, future)
        self.nav._send_result_response = remember_result

    async def execute(self, handle):
        index = len(self.goals)
        self.goals.append((time.monotonic(), handle.request.pose.pose.position.x))
        self.action_ids[index] = bytes(handle.goal_id.uuid).hex()
        self.active.add(index)
        self.max_active = max(self.max_active, len(self.active))
        self.record('accepted', index)
        while not handle.is_cancel_requested and index not in self.complete and index not in self.abort:
            await self.delay(.01)
        canceled = handle.is_cancel_requested
        if canceled:
            await self.delay(self.cancel_delay)
        self.active.remove(index)
        self.terminals.append((time.monotonic(), index, canceled))
        result = 'aborted' if index in self.abort or (canceled and self.late_failure) else (
            'succeeded' if not canceled or self.late_success else 'canceled')
        getattr(handle, {'aborted': 'abort', 'succeeded': 'succeed', 'canceled': 'canceled'}[result])()
        self.record(result, index)
        return NavigateToPose.Result()

    def record(self, event, generation=None):
        self.trace.append(dict(time=time.monotonic(), event=event,
            observed_goal_generation=generation, active=sorted(self.active),
            action_uuid=self.action_ids.get(generation), owner='competition_mission_node',
            mission_detail=self.state.detail if self.state else None,
            localization_session=[self.started_at.sec, self.started_at.nanosec],
            target=self.track, branch=self.state.active_branch if self.state else None))

    def on_state(self, message):
        super().on_state(message)
        self.states.append(dict(time=time.monotonic(), branch=message.active_branch,
            status=message.navigation_status, detail=message.detail))

    def checkpoint(self, *cases):
        self.cases.extend(cases)
        self.record(','.join(cases))


@pytest.fixture
def system(request):
    output = Path(os.environ.get('MISSION_SYSTEM_OUTPUT', '/tmp/mission-system')) / request.node.name
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(160 + os.getpid() % 30)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_log')
    rclpy.init()
    node = SystemFixture()
    executor = SingleThreadedExecutor(); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    config = output / 'mission.yaml'
    overrides = getattr(request, 'param', {})
    config.write_text(yaml.safe_dump({'competition_mission_node': {'ros__parameters': {
        'default_mode': 'hold', 'allow_pursuit': True, 'home_pose': [0., 0., 0.],
        'patrol_waypoints': [1., 0., 0., 2., 0., 0.], 'tick_rate_hz': 20.,
        'goal_failure_retry_sec': .1, 'max_consecutive_goal_failures': 2, **overrides}}}))
    source = Path(os.environ['RM_COMPETITION_MISSION_SOURCE_DIR'].lstrip(os.pathsep))
    launch = source.parent / 'rm_navigation_bringup/launch/competition_2027_navigation.launch.py'
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', str(launch), 'enable_navigation_intents:=true',
        'radar_target_input:=' + overrides.get('radar_target_input', 'track'),
        'mission_config:=' + str(config)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        wait(lambda: node.state is not None, 'Mission startup', 10)
        if overrides.get('radar_target_input', 'track') == 'track':
            wait(lambda: any(info.node_name == 'pursuit_goal_planner' for info in
                 node.get_publishers_info_by_topic('/mission/pursuit_candidate')),
                 'pursuit planner startup', 10)
        yield node
        assert node.max_active <= 1, node.trace
        for index, replacement in enumerate(node.requests[1:]):
            terminal = next(t[0] for t in node.terminals if t[1] == index)
            assert terminal <= replacement[0], (index, terminal, replacement)
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3)
        log.close()
        deaths = [line for line in (output / 'launch.log').read_text().splitlines()
                  if 'process has died' in line]
        node.complete.update(range(len(node.goals)))
        wait(lambda: not node.active, 'fixture action cleanup', 4)
        (output / 'result.json').write_text(json.dumps(dict(cases=node.cases, max_active=node.max_active,
            requests=node.requests, cancels=node.cancels, terminals=node.terminals,
            trace=node.trace, states=node.states,
            launch_exit=process.returncode, process_deaths=deaths), indent=2))
        executor.shutdown(timeout_sec=3); thread.join(timeout=3)
        node.destroy_node(); rclpy.try_shutdown()
        assert process.returncode == 0 and not deaths, deaths


@pytest.mark.parametrize('boundary', ['radar', 'candidate'])
@pytest.mark.parametrize('invalid', ['future', 'negative', 'malformed'])
def test_timestamp_recovery(system, boundary, invalid):
    n = system
    n.mode('auto'); wait_goal(n, 1.)
    n.track = ('baseline', 5., True); wait_goal(n, 3.5, 1)
    n.track = None
    m = n.target('bad', 6.) if boundary == 'radar' else PursuitCandidate()
    if boundary == 'candidate':
        m.header.frame_id = 'map'; m.header.stamp = n.get_clock().now().to_msg()
        m.valid_until = (n.get_clock().now() + Duration(seconds=.3)).to_msg()
        m.track_id = 'bad'; m.source = m.SOURCE_RADAR; m.valid = True
        m.pose.position.x = 6.; m.pose.orientation.w = 1.
    if invalid == 'future': m.header.stamp.sec += 3600
    elif invalid == 'negative': m.header.stamp.sec = -1
    else: m.header.stamp.nanosec = 1000000000
    (n.radar_pub if boundary == 'radar' else n.candidate_pub).publish(m)
    time.sleep(.2)
    before = len(n.goals); n.track = ('recovered', 6., True)
    wait_goal(n, 4.5, before)
    n.checkpoint('T5' if invalid == 'future' else 'T6', boundary + '_' + invalid)


def test_modes_targets_and_failure_budget(system):
    n = system
    n.mode('hold'); stable_count(n, .2); assert not n.requests; n.checkpoint('M1')
    n.mode('patrol'); wait_goal(n, 1.); n.complete.add(0); wait_goal(n, 2., 1); n.checkpoint('M2')
    n.mode('home'); wait_goal(n, 0., 2); n.checkpoint('M3', 'M6', 'P1')
    n.track = ('alpha', 5., True); stable_count(n); n.checkpoint('T8')
    n.mode('auto'); wait_goal(n, 3.5, 3); n.checkpoint('M5', 'M9', 'T1')
    n.mode('patrol'); wait_goal(n, 2., 4)
    n.mode('pursuit'); wait_goal(n, 3.5, 5); n.checkpoint('M4', 'M7')
    before = len(n.goals); n.track = ('beta', 6., True); wait_goal(n, 4.5, before); n.checkpoint('T3')
    old = n.target('obsolete', 7.); old.header.stamp.sec -= 1
    n.radar_pub.publish(old); stable_count(n, .2); n.checkpoint('T4')
    n.track = None; wait(lambda: not n.active, 'TTL pursuit hold'); n.checkpoint('T2')
    before = len(n.goals); n.mode('auto'); wait_goal(n, 2., before)
    before = len(n.goals); n.track = ('cancel', 5., True); wait_goal(n, 3.5, before)
    before = len(n.goals); n.track = ('cancel', 5., False); wait_goal(n, 2., before); n.checkpoint('T7')
    before = len(n.goals); n.track = ('home_preempts', 5., True); wait_goal(n, 3.5, before)
    before = len(n.goals); n.mode('home'); wait_goal(n, 0., before); n.checkpoint('M8', 'P2')
    n.guard = V2MotionGuardState.BLOCKED; wait(lambda: not n.active, 'safety cancels home')
    stable_count(n); n.checkpoint('P3', 'T9')
    before = len(n.goals); n.track = None; n.guard = V2MotionGuardState.PERMITTED
    wait_goal(n, 0., before)
    before = len(n.goals); n.abort.add(before-1); wait_goal(n, 0., before)
    n.abort.add(len(n.goals)-1)
    wait(lambda: n.state.navigation_status == 'goal_retry_limit', 'bounded Mission failures')
    stable_count(n); n.checkpoint('bounded_action_failure')
    before = len(n.goals); n.mode('home'); wait_goal(n, 0., before); n.checkpoint('explicit_rearm')


@pytest.mark.parametrize('invalid', ['negative', 'malformed'])
def test_candidate_deadline(system, invalid):
    n = system; n.mode('auto'); wait_goal(n, 1.)
    m = PursuitCandidate(); m.header.stamp = n.get_clock().now().to_msg()
    m.header.frame_id = 'map'; m.valid = True; m.track_id = 'bad_deadline'
    m.source = m.SOURCE_RADAR; m.pose.orientation.w = 1.
    if invalid == 'negative': m.valid_until.sec = -1
    else: m.valid_until.nanosec = 1000000000
    n.candidate_pub.publish(m); time.sleep(.15)
    n.track = ('good_deadline', 5., True); wait_goal(n, 3.5, 1)
    n.checkpoint('T6_deadline_' + invalid)


def test_duplicate_terminal_response(system):
    n = system; n.mode('patrol'); wait_goal(n, 1.)
    n.complete.add(0); wait_goal(n, 2., 1)
    wait(lambda: bool(n.result_replies), 'cached result response')
    reply = n.result_replies[0]
    for _ in range(3): n.original_result_reply(*reply)
    stable_count(n, .3); assert n.goals[-1][1] == 2.
    n.mode('home'); wait_goal(n, 0., 2)
    n.original_result_reply(*reply); stable_count(n, .3)
    assert n.state.active_branch == 'home'
    n.checkpoint('duplicate_old_result_response_rejected')


def test_life_authority_without_posture_enum(system):
    from rm_competition_interfaces.msg import RobotPosture
    n = system
    assert not hasattr(RobotPosture, 'DEAD') and not hasattr(RobotPosture, 'REVIVED')
    subscriptions = n.get_subscriber_names_and_types_by_node('competition_mission_node', '/')
    assert not any(name == '/robot/posture/state' for name, _ in subscriptions)
    n.mode('patrol'); wait_goal(n, 1.)
    n.hp = 0; wait(lambda: not n.active and 'life_state=dead' in n.state.detail, 'HP0 revokes')
    stable_count(n, .3); assert len(n.requests) == 1 and n.state.active_branch == 'hold'
    n.localization_valid = False
    wait(lambda: n.state.detail.startswith('safety_gate_closed'), 'invalid localization')
    n.hp = 400
    wait(lambda: 'life_state=alive' in n.state.detail, 'revival evidence')
    stable_count(n, .3); assert n.state.active_branch == 'hold'
    n.checkpoint('D1_D5_HP_authority_without_posture_enum')


@pytest.mark.parametrize('seed', [7, 19, 41])
def test_deterministic_long_sequence(system, seed):
    n = system; rng = random.Random(seed)
    for cycle in range(3):
        n.mode('hold'); wait(lambda: not n.active, 'hold')
        n.track = None; n.guard = V2MotionGuardState.PERMITTED; n.localization_valid = True
        n.cancel_delay = rng.uniform(.25, .45)
        n.late_success = cycle != 1; n.late_failure = cycle == 1
        # Snapshot BEFORE the trigger; a goal may arrive before set_mode replies.
        before = len(n.goals); n.mode('patrol'); wait_goal(n, 1., before)
        before = len(n.goals)
        n.track = (f'{seed}-{cycle}', 5., True); n.mode('auto')
        wait_goal(n, 3.5, before)
        n.mode('home')
        wait(lambda: n.state.navigation_status.startswith('goal_canceling'), 'cancel pending')
        n.localization_valid = False
        wait(lambda: not n.active and n.state.active_branch == 'hold', 'invalid drains old result')
        stable_count(n, .15)
        before = len(n.goals)
        n.started_at = n.get_clock().now().to_msg(); n.sequence = 0; n.localization_valid = True
        wait_goal(n, 0., before)
        n.guard = V2MotionGuardState.BLOCKED
        wait(lambda: not n.active, 'guard veto')
        before = len(n.goals)
        n.track = None; n.mode('patrol'); n.guard = V2MotionGuardState.PERMITTED
        wait_goal(n, 1., before)
        # Duplicate/out-of-order source observations cannot renew a target lease.
        duplicate = n.target('duplicate', 6.)
        for _ in range(3): n.radar_pub.publish(duplicate)
        stable_count(n, .15)
        n.checkpoint('P4', 'P5', 'P6' if n.late_success else 'P7', 'L3', f'seed{seed}_cycle{cycle}')
    before = len(n.goals); n.mode('pursuit'); n.track = ('invalid_pursuit', 5., True)
    wait_goal(n, 3.5, before)
    n.localization_valid = False; wait(lambda: not n.active, 'invalid pursuit')
    n.checkpoint('L2')


def test_operator_over_track_and_radar_producer_isolation(system):
    from rm_competition_interfaces.msg import TargetIntent
    n = system; n.mode('auto'); wait_goal(n, 1.)
    n.track = ('track', 5., True); wait_goal(n, 3.5, 1)
    pub = n.create_publisher(TargetIntent, '/mission/target_intent', 10)
    def target(source, x, sequence):
        m = TargetIntent(); m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg(); m.source = source
        m.sequence = sequence; m.target_id = 'explicit_source'
        m.pose.orientation.w = 1.; m.pose.position.x = x
        m.ttl = Duration(seconds=1.).to_msg(); m.valid = True
        return m
    # The track profile must reject a second producer's Radar sequence domain.
    pub.publish(target(TargetIntent.SOURCE_RADAR, 9., 1000)); stable_count(n, .2)
    pub.publish(target(TargetIntent.SOURCE_OPERATOR, 6., 1)); wait_goal(n, 6., 2)
    wait_goal(n, 3.5, 3)
    assert not any(x == 9. for _, x in n.goals)
    n.checkpoint('track_radar_isolation', 'operator_over_track', 'track_fallback')
