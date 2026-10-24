"""Deterministic harness calling the actual adapter callbacks, without DDS/time waits."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_interfaces.msg import FrontendSession
from rclpy.time import Time
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import candidate_adapter as adapter
from session_window import SessionWindow


class Harness(NS):
    def __init__(self, monkeypatch):
        super().__init__()
        self.ros = 10_000_000_000
        self.steady = 100.
        monkeypatch.setattr(adapter.time, 'monotonic', lambda: self.steady)
        self.frontend = 'zlio'
        self.started = Time(nanoseconds=1_000_000_000)
        self.window = SessionWindow(self.started.nanoseconds)
        self.identity = 'test-process-1'
        self.sequence = 0
        self.native_identity = (10, 0)
        self.native_sequence = 0
        self.last_raw_ns = 0
        self.sensor = 'lio_imu_link'
        self.blocked = self.trusted = False
        self.lease = .5
        self.last_pose = PoseWithCovarianceStamped()
        self.last_pose_received = 0.
        self.states, self.poses = {}, {}
        self.output = []
        self.status_pub = NS(publish=lambda m: self.output.append(copy.deepcopy(m)))
        self.tf = NS(lookup_transform=lambda *_: NS(transform=NS(
            translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
        self.get_clock = lambda: NS(now=lambda: Time(nanoseconds=self.ros))
        self.get_logger = lambda: NS(error=lambda _: None, debug=lambda _: None)
        self.observe_publisher = lambda *_: self.check_clock()
        for name in ('block', 'revoke', 'check_clock', 'native_session', 'state', 'pose',
                     'drain', 'publish_status', 'tick', 'invalidate'):
            setattr(self, name, getattr(adapter.CandidateAdapter, name).__get__(self))

    def state_at(self, ns, generation=1, revision=1, trusted=True, epoch=0):
        self.state(String(data=(f'epoch={epoch};generation={generation};pose_revision={revision};'
            f'stamp_ns={ns};status={"NORMAL" if trusted else "LOST"};'
            f'pose_trusted={str(trusted).lower()};tf_frozen={str(not trusted).lower()};'
            f'tf_published={str(ns > 0).lower()};estimator_failure_count=0')))

    def pose_at(self, ns, x=1.):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = Time(nanoseconds=ns).to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.orientation.w = 1.
        for i in range(6):
            m.pose.covariance[7*i] = .01
        self.pose(m)

    def pair(self, ns=None, generation=1, revision=1, x=1.):
        ns = self.ros if ns is None else ns
        self.state_at(ns, generation, revision)
        self.pose_at(ns, x)


def test_loss_recovery_cannot_resurrect_delayed_pre_loss_pose(monkeypatch):
    h = Harness(monkeypatch)
    h.pair()
    assert h.output[-1].valid
    h.state_at(0, generation=2, trusted=False)
    assert not h.output[-1].valid
    h.state_at(h.ros, generation=3, revision=2)
    h.pose_at(h.ros, x=99.)  # late old source, even with a newer recovery heartbeat
    h.tick()
    assert not h.output[-1].valid
    h.ros += 10_000_000
    h.pair(generation=3, revision=2, x=12.)
    result = h.output[-1]
    assert result.valid and result.pose.pose.pose.position.x == 12.  # real jump allowed
    assert result.pose.pose.covariance[0] == pytest.approx(.01)
    assert result.pose.header.stamp == Time(nanoseconds=h.ros).to_msg()


def test_delayed_reordered_state_and_pose_do_not_overwrite_new_evidence(monkeypatch):
    h = Harness(monkeypatch)
    h.pair(generation=3, revision=4)
    h.state_at(0, generation=2, revision=3, trusted=False)
    h.pose_at(h.ros-10_000_000, x=99.)
    h.state_at(h.ros-10_000_000, generation=2, revision=3)
    h.tick()
    assert h.output[-1].valid and h.output[-1].pose.pose.pose.position.x == 1.
    # New source may arrive pose-first; cross-topic ordering does not require sleeps.
    h.ros += 10_000_000
    h.pose_at(h.ros, x=2.)
    h.state_at(h.ros, generation=4, revision=5)
    assert h.output[-1].valid and h.output[-1].pose.pose.pose.position.x == 2.


@pytest.mark.parametrize('resets', [1, 3, 100])
def test_single_multiple_rapid_reset_is_latched(monkeypatch, resets):
    h = Harness(monkeypatch)
    h.pair()
    for epoch in range(1, resets+1):
        h.state_at(0, generation=2, epoch=epoch, trusted=False)
    h.ros += 10_000_000
    h.pair(generation=3, revision=2)
    h.native_session(FrontendSession(instance_id=10, epoch=0))  # stale session cannot rearm
    h.tick()
    assert h.blocked and not h.output[-1].valid
    assert not h.output[-1].pose.header.stamp.sec
    assert [m.sequence for m in h.output] == sorted(set(m.sequence for m in h.output))
    assert all(m.producer_id == h.identity for m in h.output)


@pytest.mark.parametrize('change', ['epoch', 'instance_id', 'failed', 'reset_required'])
def test_session_or_old_session_cannot_mix_with_canonical_reference(monkeypatch, change):
    h = Harness(monkeypatch)
    h.pair()
    session = FrontendSession(instance_id=10)
    setattr(session, change, True if change in ('failed', 'reset_required') else 11)
    h.native_session(session)
    h.native_session(FrontendSession(instance_id=10))
    h.ros += 1_000_000
    h.pair()
    h.tick()
    assert h.blocked and not h.output[-1].valid


def test_pause_expiry_resume_requires_fresh_source(monkeypatch):
    h = Harness(monkeypatch)
    h.pair()
    h.steady += .51  # ROS clock unchanged, wall timer still runs
    h.tick()
    assert not h.output[-1].valid
    h.pair()  # same source heartbeat cannot extend evidence
    h.tick()
    assert not h.output[-1].valid
    h.ros += 10_000_000
    h.pair()
    assert h.output[-1].valid


def test_backwards_clock_cannot_recover_when_time_catches_up(monkeypatch):
    h = Harness(monkeypatch)
    h.pair()
    h.ros -= 1_000_000
    h.tick()
    assert h.blocked and not h.output[-1].valid
    h.ros += 2_000_000
    h.pair()
    h.tick()
    assert not h.output[-1].valid


def test_new_process_rejects_old_pre_start_pose(monkeypatch):
    h = Harness(monkeypatch)
    h.started = Time(nanoseconds=h.ros)
    h.window = SessionWindow(h.ros)
    h.identity = 'new-process-2'
    h.pair(h.ros-1_000_000)
    h.tick()
    assert not h.output[-1].valid
    h.ros += 1_000_000
    h.pair()
    assert h.output[-1].valid and h.output[-1].producer_id == 'new-process-2'


def test_new_group_does_not_adopt_already_reset_frontend(monkeypatch):
    h = Harness(monkeypatch)
    h.native_identity = None
    h.native_session(FrontendSession(instance_id=20, epoch=5))
    assert h.blocked and h.native_identity is None
