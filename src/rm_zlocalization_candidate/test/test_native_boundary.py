"""Fault cases at the native canonical handoff; ROS transport is covered by smoke."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from localization_interfaces.msg import FrontendFrame, FrontendSession
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from candidate_adapter import CandidateAdapter


def boundary():
    output = []
    node = SimpleNamespace(native_identity=None, native_sequence=0, last_raw_ns=0,
        raw_frame='zlio_odom', sensor='lio_imu_link', blocked=False,
        observe_publisher=lambda *_: True, native_raw_pub=SimpleNamespace(publish=output.append))
    node.block = lambda _: setattr(node, 'blocked', True)
    return node, output


@pytest.mark.parametrize('change', ['instance', 'epoch', 'failed', 'reset_required'])
def test_session_discontinuity_latches_before_new_canonical_input(change):
    node, _ = boundary()
    CandidateAdapter.native_session(node, FrontendSession(instance_id=10, epoch=0))
    next_session = FrontendSession(instance_id=10, epoch=0)
    setattr(next_session, {'instance': 'instance_id'}.get(change, change),
            True if change in ('failed', 'reset_required') else 11)
    CandidateAdapter.native_session(node, next_session)
    assert node.blocked


def test_atomic_frame_preserves_source_and_does_not_relabel_covariance():
    node, outputs = boundary()
    frame = FrontendFrame(instance_id=10, epoch=0, sequence=1)
    frame.odometry.header.frame_id = 'zlio_odom'
    frame.odometry.header.stamp.sec = 10
    frame.odometry.child_frame_id = 'lio_imu_link'
    frame.odometry.pose.pose.orientation.w = 1.0
    frame.odometry.pose.covariance[0] = .1
    frame.cloud.header = frame.odometry.header
    frame.cloud.header.frame_id = 'lio_imu_link'
    # Python ROS assignment aliases header; set the odometry frame back using a copy.
    import copy
    frame.odometry.header = copy.deepcopy(frame.cloud.header)
    frame.odometry.header.frame_id = 'zlio_odom'
    CandidateAdapter.native_frame(node, frame)
    assert not outputs, 'an observation cannot establish its own session identity'
    CandidateAdapter.native_session(node, FrontendSession(instance_id=10))
    CandidateAdapter.native_frame(node, frame)
    assert len(outputs) == 1 and not node.blocked
    assert outputs[0].header.stamp == frame.odometry.header.stamp
    assert frame.odometry.pose.covariance[0] == .1
    assert not any(outputs[0].pose.covariance)
    CandidateAdapter.native_frame(node, frame)
    assert node.blocked and len(outputs) == 1, 'duplicate sequence must not pass the session barrier'
