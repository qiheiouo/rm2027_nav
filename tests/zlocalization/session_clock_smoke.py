#!/usr/bin/env python3
"""Actual rclpy adapter + DDS + TF under pause/resume/backwards clock; no estimator."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from localization_interfaces.msg import FrontendSession
from rclpy.qos import DurabilityPolicy, QoSProfile
from rm_competition_interfaces.msg import LocalizationStatus
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    out = Path(p.parse_args().output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('canonical_clock_fixture')
    clock = node.create_publisher(Clock, '/clock', 10)
    state = node.create_publisher(String, '/zlocalization/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose = node.create_publisher(PoseWithCovarianceStamped, '/zlocalization/sensor_pose', 20)
    session = node.create_publisher(FrontendSession, '/zlocalization/frontend/session',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    received = []
    node.create_subscription(LocalizationStatus,
        '/localization/zlocalization_backend_valid/status', received.append, 100)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'lio_imu_link'
    tf.transform.rotation.w = 1.
    broadcaster.sendTransform(tf)
    executable = Path(__file__).resolve().parents[2] / 'src/rm_zlocalization_candidate/scripts/candidate_adapter.py'
    log = (out/'adapter.log').open('w')
    process = subprocess.Popen([sys.executable, str(executable), '--ros-args',
        '-p', 'frontend:=zlio', '-p', 'use_sim_time:=true'], stdout=log, stderr=log)

    def drive(seconds, ros_sec):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert process.poll() is None, 'adapter exited before test completed'
            c = Clock(); c.clock.sec = ros_sec; clock.publish(c)
            session.publish(FrontendSession(instance_id=10))
            state.publish(String(data=f'epoch=0;generation=1;pose_revision=1;stamp_ns={ros_sec*1000000000};'
                'status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;estimator_failure_count=0'))
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'; m.header.stamp.sec = ros_sec
            m.pose.pose.position.x = 4.; m.pose.pose.orientation.w = 1.
            for i in range(6): m.pose.covariance[7*i] = .01
            pose.publish(m)
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        drive(2., 10)
        assert any(m.valid for m in received), 'no initial valid source-time TF/pose pair'
        assert not received[-1].valid, 'steady timer did not expire paused source'
        count = len(received)
        drive(.3, 10)
        assert len(received) > count and not any(m.valid for m in received[count:])
        count = len(received)
        drive(.3, 11)
        fresh = [m for m in received[count:] if m.valid]
        assert fresh and fresh[-1].pose.header.stamp.sec == 11
        assert fresh[-1].pose.pose.pose.position.x == 4.
        assert abs(fresh[-1].pose.pose.covariance[0]-.01) < 1e-9
        drive(.3, 9)
        assert not received[-1].valid
        count = len(received)
        drive(.3, 12)
        assert len(received) > count, 'latched invalid heartbeat stopped'
        assert not any(m.valid for m in received[count:]), 'clock catch-up silently rearmed session'
        assert len({m.producer_id for m in received}) == 1
        assert all(b.sequence > a.sequence for a,b in zip(received, received[1:]))
        result = dict(paused_clock_expiry=True, invalid_heartbeat_continues=True,
            fresh_resume=True, backwards_latched=True, covariance_pose_tf_consistent=True,
            messages=len(received), scope='adapter DDS/TF fixture only; no real sensor/map evidence')
        (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, 'adapter shutdown failed'


if __name__ == '__main__':
    main()
