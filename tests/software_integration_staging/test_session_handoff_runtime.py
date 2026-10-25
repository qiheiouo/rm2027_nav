"""Event-controlled Mission handoff; no sleep/timeout unlock in the implementation."""
from contextlib import contextmanager
import copy
import json
from pathlib import Path
import subprocess
import threading

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rm_competition_interfaces.msg import PursuitCandidate
import yaml
from ament_index_python.packages import get_package_prefix

from test_combination_runtime import ROOT, arbitration, stop


class HandoffFixture(arbitration.ArbitrationFixture):
    def __init__(self):
        self.accept_gate = threading.Event(); self.accept_gate.set()
        self.release = {}
        self.succeed = set()
        self.spin_goals = []
        self.spin_cancels = []
        self.spin_release = threading.Event()
        self.spin_active = False
        self.status_rows = []
        self.localization = {'id': 'canonical-A', 'epoch': 1, 'valid': True}
        super().__init__()
        publisher = self.loc_pub
        fixture = self

        class VersionedPublisher:
            def publish(self, message):
                context = fixture.localization.copy()
                message.session_id = context['id']; message.session_epoch = context['epoch']
                message.valid = context['valid']
                publisher.publish(message)
        self.loc_pub = VersionedPublisher()
        self.spin_server = ActionServer(self, Spin, '/spin',
            goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=self.cancel_spin,
            execute_callback=self.execute_spin, callback_group=ReentrantCallbackGroup())

    def on_state(self, message):
        super().on_state(message)
        self.status_rows.append(message)

    def accept(self, request):
        self.requests.append((self.get_clock().now().nanoseconds, request.pose.pose.position.x))
        assert self.accept_gate.wait(10), 'test acceptance gate was not released'
        return GoalResponse.ACCEPT

    def execute(self, handle):
        index = len(self.goals)
        self.goals.append((self.get_clock().now().nanoseconds, handle.request.pose.pose.position.x))
        self.active.add(index); self.max_active = max(self.max_active, len(self.active))
        release = self.release.setdefault(index, threading.Event())
        assert release.wait(10), 'test terminal gate was not released'
        canceled = handle.is_cancel_requested
        self.active.remove(index)
        self.terminals.append((self.get_clock().now().nanoseconds, index, canceled))
        if index in self.succeed or not canceled: handle.succeed()
        else: handle.canceled()
        return NavigateToPose.Result()

    def cancel_spin(self, _):
        self.spin_cancels.append(self.get_clock().now().nanoseconds)
        return CancelResponse.ACCEPT

    def execute_spin(self, handle):
        self.spin_goals.append(handle.request.target_yaw); self.spin_active = True
        assert self.spin_release.wait(10), 'test Spin terminal gate was not released'
        self.spin_active = False
        handle.succeed()  # success deliberately races with the observed cancel
        return Spin.Result()

    def finish(self, index, success=False):
        if success: self.succeed.add(index)
        self.release.setdefault(index, threading.Event()).set()

    def ticks(self, count=3):
        before = len(self.status_rows)
        arbitration.wait(lambda: len(self.status_rows) >= before+count, 'Mission ticks after boundary')


@contextmanager
def mission(tmp_path, pending=False, spin=False):
    rclpy.init(); node = HandoffFixture()
    if pending: node.accept_gate.clear()
    executor = MultiThreadedExecutor(num_threads=8); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    params = {'default_mode': 'auto', 'allow_pursuit': True, 'tick_rate_hz': 30.,
        'patrol_waypoints': [1., 0., 0., 2., 0., 0.], 'spin_failure_retry_sec': 0.}
    if spin:
        params['tree_xml'] = str(ROOT / 'src/rm_competition_mission/trees/competition_three_point_spin_test.xml')
    config = tmp_path / 'mission.yaml'
    config.write_text(yaml.safe_dump({'competition_mission_node': {'ros__parameters': params}}))
    log = (tmp_path / 'mission.log').open('w')
    tree = ROOT / 'src/rm_competition_mission/trees' / (
        'competition_three_point_spin_test.xml' if spin else 'competition_default.xml')
    binary = Path(get_package_prefix('rm_competition_mission'))/'lib/rm_competition_mission/competition_mission_node'
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config),
        '-p', 'tree_xml:='+str(tree)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        arbitration.wait(lambda: node.state is not None, 'Mission ready')
        node.mode('auto')
        arbitration.wait(lambda: bool(node.requests), 'pending request')
        if not pending: arbitration.wait_goal(node, 1.)
        yield node
        assert node.max_active <= 1
    finally:
        node.mode('hold'); node.accept_gate.set(); node.spin_release.set()
        for index in range(len(node.requests)+2): node.finish(index)
        arbitration.wait(lambda: not node.active and not node.spin_active, 'fixture action cleanup')
        stop(process)
        for timer in node.timers: timer.cancel()
        executor.remove_node(node); executor.shutdown(); thread.join(timeout=3)
        node.destroy_node(); rclpy.shutdown(); log.close()


@pytest.mark.parametrize('boundary', ['epoch', 'source', 'lost', 'rapid', 'pending'])
def test_retired_result_releases_barrier_without_progress(boundary, tmp_path):
    with mission(tmp_path, pending=boundary == 'pending') as node:
        initial_requests = len(node.requests)
        if boundary == 'source':
            node.localization = {'id': 'canonical-B', 'epoch': 2, 'valid': True}
        elif boundary == 'lost':
            node.localization = dict(node.localization, valid=False)
        else:
            node.localization = dict(node.localization, epoch=2)
        if boundary == 'pending':
            node.ticks(); assert not node.goals
            node.accept_gate.set()
        arbitration.wait(lambda: bool(node.cancels), 'retired action canceled')
        if boundary == 'rapid':
            for epoch in range(3, 103):
                node.localization = dict(node.localization, epoch=epoch)
                node.publish()  # authoritative callbacks, no tick/timeout ordering assumption
        if boundary == 'lost':
            # A map candidate cached while LOST must not survive the commit edge.
            candidate = PursuitCandidate()
            stamp = node.get_clock().now()
            candidate.header.stamp = stamp.to_msg(); candidate.header.frame_id = 'map'
            candidate.valid_until = (stamp+Duration(seconds=.5)).to_msg()
            candidate.track_id = 'lost-gap'; candidate.source = PursuitCandidate.SOURCE_RADAR
            candidate.pose.position.x = 8.; candidate.pose.orientation.w = 1.; candidate.valid = True
            node.candidate_pub.publish(candidate)
            node.localization = dict(node.localization, valid=True)
        node.ticks()
        assert len(node.requests) == initial_requests, 'cancel ACK released ownership before terminal'
        node.finish(0, success=True)
        arbitration.wait_goal(node, 1., after=1)  # retired success must not advance to patrol x=2
        assert node.requests[1][0] >= node.terminals[0][0]
        if boundary in ('source', 'rapid'):
            # A later sequence carrying a retired identity must not roll the consumer back.
            current = node.localization.copy()
            cancel_before = len(node.cancels)
            node.localization = {'id': 'canonical-A', 'epoch': 1, 'valid': True}
            node.ticks()
            node.localization = current
            assert len(node.requests) == 2
            assert len(node.cancels) == cancel_before, 'retired status canceled current-session work'
        (tmp_path / 'summary.json').write_text(json.dumps({
            'boundary': boundary, 'requests': node.requests, 'terminals': node.terminals,
            'cancel_count': len(node.cancels), 'max_active': node.max_active,
            'latest_epoch': node.localization['epoch'], 'retired_success_advanced_patrol': False,
        }, indent=2)+'\n')


def test_spin_session_change_retires_late_success(tmp_path):
    with mission(tmp_path, spin=True) as node:
        node.finish(0, success=True)
        arbitration.wait(lambda: bool(node.spin_goals), 'active patrol Spin')
        node.localization = dict(node.localization, epoch=2)
        arbitration.wait(lambda: bool(node.spin_cancels), 'retired Spin canceled')
        node.ticks(); assert len(node.requests) == 1
        node.spin_release.set()
        arbitration.wait_goal(node, 1., after=1)
        assert len(node.spin_goals) == 1
