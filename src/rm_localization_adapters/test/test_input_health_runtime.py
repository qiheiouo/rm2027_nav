#!/usr/bin/env python3
"""Real adapter/bridge fault injection. Synthetic input; no hardware/backend claim."""
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
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import LioSourceStatus, LocalizationStatus


def ns(t):
    return t.sec * 10**9 + t.nanosec


class Fixture(Node):
    def __init__(self, upstream=False):
        super().__init__('lio_health_injection')
        self.mode = 'normal'
        self.upstream = upstream
        self.backend_epoch = self.get_clock().now().to_msg()
        self.backend_sequence = 0
        self.backend = self.create_publisher(LocalizationStatus, '/test/backend_valid/status', 10)
        self.health = self.validity = None
        self.canonical = []
        self.health_history = []
        self.x = self.yaw = 0.0
        self.raw = self.create_publisher(Odometry, '/odometry/fast_lio_raw', 10)
        self.global_pose = self.create_publisher(PoseWithCovarianceStamped, '/localization/global_pose', 10)
        self.replay = self.create_publisher(LioSourceStatus, '/localization/lio_source_status', 10)
        self.create_subscription(LioSourceStatus, '/localization/lio_source_status', self.on_health, 100)
        self.create_subscription(LocalizationStatus, '/localization/status', lambda m: setattr(self, 'validity', m), 100)
        self.create_subscription(Odometry, '/odometry/lio', lambda m: self.canonical.append(m), 100)
        self.create_timer(.05, self.publish)

    def on_health(self, m):
        self.health = m
        self.health_history.append(m)

    def publish(self):
        now = self.get_clock().now()
        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        if self.mode == 'spin':
            self.yaw += .35
            m.twist.twist.angular.z = 7.0
        m.pose.pose.position.x = self.x
        m.pose.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2)
        # Global correction stays healthy, even when the odometry source fails.
        p = PoseWithCovarianceStamped()
        p.header.stamp = now.to_msg()
        p.header.frame_id = 'map'
        p.pose = copy.deepcopy(m.pose)
        self.global_pose.publish(p)
        if self.upstream:
            status = LocalizationStatus()
            status.header.stamp = now.to_msg()
            status.producer_id = 'backend_fixture'
            status.started_at = self.backend_epoch
            self.backend_sequence += 1
            status.sequence = self.backend_sequence
            status.valid = True
            status.valid_until = (now + Duration(seconds=.8)).to_msg()
            status.pose = p
            self.backend.publish(status)
        if self.mode == 'silence':
            return
        if self.mode in ('stale', 'rollback'):
            m.header.stamp = (now - Duration(seconds=1.0 if self.mode == 'stale' else .15)).to_msg()
        elif self.mode == 'future':
            m.header.stamp = (now + Duration(seconds=5.0)).to_msg()
        elif self.mode == 'nan':
            m.pose.pose.position.x = float('nan')
        elif self.mode == 'inf':
            m.twist.twist.linear.x = float('inf')
        elif self.mode == 'jump':
            m.pose.pose.position.x += 10.0
        elif self.mode == 'frame':
            m.child_frame_id = 'changed_sensor'
        elif self.mode == 'freeze':
            m.twist.twist.linear.x = .5
        self.raw.publish(m)

    def spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)

    def until(self, predicate, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(.02)
            if predicate():
                return
        raise AssertionError(f'timeout: mode={self.mode}, health={self.health}, validity={self.validity}')


def run(output, upstream=False, finite_difference=False):
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_log')
    rclpy.init()
    n = Fixture(upstream)
    children, logs = [], []
    result = {'passed': False, 'cases': [], 'hardware_validated': False, 'selected_backend_status': upstream, 'finite_difference': finite_difference}
    try:
        for package, executable, params in [
            ('rm_localization_adapters', 'lio_adapter', ['input_health.enabled:=true', 'input_health.trust_raw_twist:=true'] +
             (['twist_mode:=finite_difference'] if finite_difference else [])),
            ('rm_relocalization_bridge', 'map_odom_from_global_pose', ['source_health_topic:=/localization/lio_source_status'] +
             (['upstream_valid_topic:=/test/backend_valid'] if upstream else [])),
        ]:
            log = (output / f'{executable}.log').open('w'); logs.append(log)
            command = [str(Path(get_package_prefix(package)) / 'lib' / package / executable), '--ros-args']
            for p in params:
                command += ['-p', p]
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        n.until(lambda: n.validity and n.validity.valid)
        n.spin(.6)
        assert n.health.healthy and n.validity.valid
        result['cases'].append({'case': 'normal', 'canonical_samples': len(n.canonical)})
        for mode in ('silence', 'stale', 'rollback', 'future', 'nan', 'inf', 'jump', 'frame', 'freeze'):
            retired = copy.deepcopy(n.health)
            old_epoch = ns(n.validity.started_at)
            start = time.monotonic()
            n.mode = mode
            n.until(lambda: n.health and not n.health.healthy and n.validity and not n.validity.valid)
            latency = time.monotonic() - start
            reason = n.health.reason
            n.spin(.1)
            count = len(n.canonical)
            n.spin(.45)
            assert len(n.canonical) == count, (mode, 'isolated output leaked')
            assert ns(n.validity.started_at) > old_epoch, 'session not retired'
            n.mode = 'normal'
            n.spin(.11)
            assert not n.validity.valid, 'single-frame recovery'
            n.until(lambda: n.validity.valid and n.health.healthy)
            epoch = ns(n.validity.started_at)
            # Replay both old healthy and invalid source-generation callbacks.
            n.replay.publish(retired)
            retired.healthy = False
            retired.sequence += 1
            n.replay.publish(retired)
            n.spin(.2)
            assert n.validity.valid and ns(n.validity.started_at) == epoch
            result['cases'].append({'case': mode, 'invalid_latency_sec': round(latency, 4),
                                    'reason': reason, 'isolation': True, 'recovery': True,
                                    'retired_generation_rejected': True})
        if upstream:
            n.backend_epoch = n.get_clock().now().to_msg()
            n.backend_sequence = 0
            n.spin(.4)
            assert n.validity.valid, 'new backend epoch sequence reset rejected'
            result['backend_epoch_sequence_reset'] = True
        n.mode = 'spin'
        n.spin(1.5)
        assert n.health.healthy and n.validity.valid
        n.mode = 'normal'
        n.spin(.2)
        result['cases'].append({'case': '7_rad_s_spin', 'valid': n.validity.valid})
        assert all(math.isfinite(m.pose.pose.position.x) and abs(m.pose.pose.position.x) < 1
                   for m in n.canonical)
        owners = [x.node_name for x in n.get_publishers_info_by_topic('/localization/status')]
        assert owners == ['map_odom_from_global_pose'], owners
        result['validity_owners'] = owners
        result['passed'] = True
    except Exception as e:
        result['error'] = str(e)
    finally:
        for child in children:
            child.send_signal(signal.SIGINT)
        exits = []
        for child in children:
            try:
                exits.append(child.wait(timeout=8))
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
                exits.append('timeout')
        for log in logs:
            log.close()
        result['shutdown_exit_codes'] = exits
        result['passed'] &= all(code == 0 for code in exits)
        n.destroy_node(); rclpy.try_shutdown()
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    return result['passed']


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--upstream', action='store_true')
    p.add_argument('--finite-difference', action='store_true')
    args = p.parse_args()
    raise SystemExit(0 if run(args.output, args.upstream, args.finite_difference) else 1)
