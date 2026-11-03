"""Three-source latest-value arbitration through the real Mission action owner."""
import copy
import signal
import struct
import subprocess
import time

import pytest
from ament_index_python.packages import get_package_prefix
from pathlib import Path
from rclpy.duration import Duration
from rm_competition_interfaces.msg import TargetIntent
from std_msgs.msg import UInt8MultiArray

from test_mission_system_runtime import system
from test_navigation_arbitration_runtime import wait, wait_goal, stable_count
from test_life_authority_runtime import life, packet

pytestmark = pytest.mark.parametrize('system', [{
    'radar_target_input': 'serial_intent', 'retreat_projectile_threshold': 0,
}], indirect=True)
L, R, O = TargetIntent.SOURCE_LOCAL, TargetIntent.SOURCE_RADAR, TargetIntent.SOURCE_OPERATOR


class Inputs:
    def __init__(self, n):
        self.n = n
        self.pub = n.create_publisher(TargetIntent, '/mission/target_intent', 10)
        self.selected = None
        self.sub = n.create_subscription(TargetIntent, '/mission/selected_target', self.observe, 10)
        self.seq = {L: 0, R: 0, O: 0}
        self.current = {}
        self.last = {}
        self.timer = n.create_timer(.04, self.tick)

    def observe(self, value):
        self.selected = value

    def packet(self, source, x, *, ttl=.8, valid=True, clear=False, target_id=None):
        self.seq[source] += 1
        m = TargetIntent()
        m.header.stamp = self.n.get_clock().now().to_msg(); m.header.frame_id = 'map'
        m.source = source; m.sequence = self.seq[source]
        m.target_id = target_id or str(source)
        m.pose.position.x = float(x); m.pose.orientation.w = 1.
        m.ttl = Duration(seconds=ttl).to_msg(); m.valid = valid; m.clear = clear
        return m

    def set(self, source, x, **kwargs):
        self.current[source] = (x, kwargs)

    def stop(self, source):
        self.current.pop(source, None)

    def tick(self):
        for source, (x, options) in list(self.current.items()):
            m = self.packet(source, x, **options)
            self.last[source] = m; self.pub.publish(m)

    def once(self, source, x, **kwargs):
        self.stop(source)
        m = self.packet(source, x, **kwargs); self.pub.publish(m)
        return m

    def source(self, source):
        wait(lambda: self.selected and self.selected.valid and self.selected.source == source,
             f'selected source {source}')


def test_priority_fallback_and_latest_sequence(system):
    n = system; t = Inputs(n); n.mode('auto'); wait_goal(n, 1.)
    t.source(L)  # Built-in Local patrol also traverses the selector.
    t.set(L, 2.5); wait_goal(n, 2.5, 1)
    t.set(R, 4.); wait_goal(n, 4., 2)
    t.set(O, 6.); wait_goal(n, 6., 3); t.source(O)
    t.stop(O); wait_goal(n, 4., 4); t.source(R)
    t.stop(R); wait_goal(n, 2.5, 5); t.source(L)
    t.set(O, 6.); wait_goal(n, 6., 6)  # Operator directly preempts Local.
    t.once(O, 0., valid=False, clear=True); wait_goal(n, 2.5, 7)
    # Both higher sources expire during the same cancellation window.
    t.set(R, 4.); wait_goal(n, 4., 8)
    t.set(O, 6.); wait_goal(n, 6., 9)
    t.stop(R); t.stop(O); wait_goal(n, 2.5, 10)
    t.once(L, 0., valid=False, clear=True); wait_goal(n, 1., 11)
    stable_count(n, .2)
    assert n.max_active == 1
    n.checkpoint('local_radar_operator_priority', 'operator_to_radar_to_local', 'simultaneous_expiry')


def test_duplicates_stale_invalid_clear_and_timestamp_watermarks(system):
    n = system; t = Inputs(n); t.set(L, 2.5); n.mode('auto'); wait_goal(n, 2.5)
    t.seq[O] = 10  # Exercise a positive old sequence, not only the reserved zero.
    m = t.once(O, 6., ttl=.9); wait_goal(n, 6., 1)
    before = len(n.requests)
    bad = copy.deepcopy(m); bad.sequence -= 1; bad.pose.position.x = 9.
    bad.header.stamp = n.get_clock().now().to_msg(); t.pub.publish(bad)
    conflict = copy.deepcopy(m); conflict.pose.position.x = 10.; t.pub.publish(conflict)
    end = time.monotonic() + 1.
    while time.monotonic() < end:
        t.pub.publish(m); time.sleep(.04)
    wait_goal(n, 2.5, before)  # Identical repeats cannot extend TTL.
    assert not any(x in (9., 10.) for _, x in n.goals)
    # A well-formed fresh INVALID is a source withdrawal, not an ESTOP.
    t.set(O, 6.); wait_goal(n, 6., len(n.goals))
    old = copy.deepcopy(t.last[O]); t.once(O, 0., valid=False)
    wait_goal(n, 2.5, len(n.goals)); t.pub.publish(old); stable_count(n, .2)
    t.set(O, 7.); wait_goal(n, 7., len(n.goals))
    t.once(O, 0., valid=False, clear=True); wait_goal(n, 2.5, len(n.goals))
    # Malformed/future/expired high sequences must not poison the valid watermark.
    for kind in ('future', 'negative', 'malformed', 'expired', 'wrong_frame', 'nan', 'nonplanar'):
        bad = t.packet(O, 8.); bad.sequence += 100000
        if kind == 'future': bad.header.stamp.sec += 3600
        elif kind == 'negative': bad.header.stamp.sec = -1
        elif kind == 'malformed': bad.ttl.nanosec = 1000000000
        elif kind == 'expired': bad.header.stamp.sec -= 10
        elif kind == 'wrong_frame': bad.header.frame_id = 'odom'
        elif kind == 'nan': bad.pose.position.x = float('nan')
        else: bad.pose.orientation.x = .1
        t.pub.publish(bad)
    t.set(O, 6.); wait_goal(n, 6., len(n.goals)); stable_count(n, .3)
    n.checkpoint('stale_sequence', 'duplicate_no_renewal', 'clear_invalid', 'bad_watermark_recovery')


@pytest.mark.parametrize('source', [L, R, O])
def test_mandatory_and_safety_override_and_recovery(system, source):
    n = system; t = Inputs(n); t.set(source, 3. + source)
    n.mode('auto'); wait_goal(n, 3. + source)
    n.hp = 100; wait_goal(n, 0., 1)
    stable_count(n, .2); assert n.state.active_branch == 'home'
    n.hp = 400; wait_goal(n, 3. + source, 2)
    n.estop = True; wait(lambda: not n.active, 'ESTOP terminal')
    stable_count(n, .15); assert n.state.active_branch == 'hold'
    t.stop(source); n.estop = False
    wait_goal(n, 1., len(n.goals))  # Pre-veto sample cannot resurrect, even within TTL.
    t.set(source, 3. + source); wait_goal(n, 3. + source, len(n.goals))
    n.hp = 0; life(n, 'dead'); wait(lambda: not n.active, 'death terminal')
    t.stop(source)
    # A cached sample slightly ahead of ROS time must not cross a fast revival,
    # even though its stamp may be later than the last veto tick.
    future = t.packet(source, 8., ttl=1.)
    future.header.stamp = (n.get_clock().now() + Duration(seconds=.04)).to_msg()
    t.pub.publish(future); time.sleep(.01)
    before = len(n.goals)
    n.hp = 400; revived = packet(n, 400); revived.projectile_allowance_17mm = 100
    n.ref_pub.publish(revived); life(n, 'alive')
    wait_goal(n, 1., before); stable_count(n, .1)
    assert not any(x == 8. for _, x in n.goals)
    t.set(source, 3. + source); wait_goal(n, 3. + source, len(n.goals))
    n.checkpoint('mandatory_home', 'estop', 'dead_revived', f'source_{source}')


def test_projectile_return_and_late_action_coalescing(system):
    n = system; t = Inputs(n); n.mode('auto'); wait_goal(n, 1.)
    # Inject normalized lower-controller referee state with zero allowance.
    n.publish_referee = False
    def referee(ammo):
        from rm_competition_interfaces.msg import RefereeState
        m = RefereeState(); m.header.stamp = n.get_clock().now().to_msg()
        m.valid = True; m.game_progress = 4; m.current_hp = 400
        m.projectile_allowance_17mm = ammo; n.ref_pub.publish(m)
    referee(0); wait_goal(n, 0., 1)
    referee(100); n.publish_referee = True
    wait_goal(n, 1., 2)
    n.accept_delay = .3; n.cancel_delay = .4; n.late_success = True
    t.set(R, 4.); wait(lambda: n.state.navigation_status == 'goal_pending', 'Radar pending')
    t.set(O, 6.); time.sleep(.08); t.set(O, 7.)
    wait_goal(n, 7., len(n.goals))
    assert not any(x == 6. for _, x in n.goals)
    # TTL expiry while cancellation is draining must use current Local only.
    n.accept_delay = 0.; t.stop(O); t.stop(R)
    wait_goal(n, 1., len(n.goals)); stable_count(n, .2)
    assert n.max_active == 1
    assert len({name for name in n.get_node_names() if name == 'competition_mission_node'}) == 1
    subscriptions = n.get_subscriber_names_and_types_by_node('competition_mission_node', '/')
    assert not any(topic == '/robot/posture/request' for topic, _ in subscriptions)
    n.checkpoint('projectile_home', 'late_accept_success', 'coalesced_latest', 'single_owner')


def frame(kind, payload, sequence=1):
    data = struct.pack('<BBHH', 2, kind, len(payload), sequence) + payload
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8): crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return b'RM' + data + struct.pack('<H', crc)


def test_serial_radar_operator_distinction_and_delayed_packets(system, tmp_path):
    n = system; n.mode('auto'); wait_goal(n, 1.)
    pub = n.create_publisher(UInt8MultiArray, '/serial/v2/mock_rx', 10)
    observed = []
    sub = n.create_subscription(TargetIntent, '/mission/target_intent', observed.append, 10)
    log = (tmp_path / 'serial.log').open('w')
    executable = Path(get_package_prefix('rm_serial_driver')) / 'lib/rm_serial_driver/competition_v2_transport_node'
    process = subprocess.Popen([str(executable),
        '--ros-args', '-p', 'dry_run:=true', '-r', '/motion/v2_guard/state:=/test/serial_guard'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    started = time.monotonic(); boot = 123; seq = {R: 0, O: 0}
    def send(kind, payload): pub.publish(UInt8MultiArray(data=list(frame(kind, payload))))
    def heartbeat():
        ms = int((time.monotonic() - started) * 1000)
        send(3, struct.pack('<IIIB', ms, boot, 0xc0, 1))
        return ms
    def target(source, x, *, stale=False, clear=False, wrong_boot=False):
        ms = heartbeat(); seq[source] += 1
        sample = max(0, ms - 5000) if stale else ms
        payload = struct.pack('<QIIIHBfff', seq[source], source, boot + int(wrong_boot),
                              sample, 1500, 2 if clear else 1, x, 0., 0.)
        kind = 0x86 if source == R else 0x87
        send(kind, payload)
        if not stale and not wrong_boot:
            # DDS discovery is directional. Repeat the same original sample
            # until observed, as on the actual latest-value wire contract.
            deadline = time.monotonic() + .6
            while not any(m.source == source and m.sequence == seq[source] for m in observed):
                assert time.monotonic() < deadline, ('serial sample not observed', source, observed)
                time.sleep(.04); heartbeat(); send(kind, payload)
        return payload
    try:
        wait(lambda: pub.get_subscription_count() == 1, 'serial mock ingress')
        heartbeat(); time.sleep(.05)
        target(R, 4.); wait_goal(n, 4., 1)
        target(O, 6.); wait_goal(n, 6., 2)
        assert {m.source for m in observed} == {R, O}
        target(O, 0., clear=True); wait_goal(n, 4., 3)
        target(R, 0., clear=True); wait_goal(n, 1., 4)
        before = len(observed)
        # An old MCU sample must be rejected before publication, not RX stamped.
        time.sleep(1.6); heartbeat(); time.sleep(.05)
        target(O, 9., stale=True); target(R, 9., wrong_boot=True); time.sleep(.15)
        assert len(observed) == before
        target(O, 7.); wait_goal(n, 7., len(n.goals))
        wait_goal(n, 1., len(n.goals))  # Serial loss falls back to Local.
        assert n.max_active == 1
        n.checkpoint('wire_source_types', 'wire_freshness', 'wire_boot', 'wire_loss_fallback')
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=3)
        log.close()
        assert process.returncode == 0, (tmp_path / 'serial.log').read_text()
