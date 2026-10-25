"""AUD-015 covariance policy at each Z/canonical/bridge entry; actual ROS nodes."""
import copy
import json
import math
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from rclpy.time import Time
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import LocalizationFrame, LocalizationStatus
from tf2_ros import StaticTransformBroadcaster

from test_combination_runtime import stop


def covariance(kind):
    c = [0.] * 36
    for i in range(6): c[7*i] = .1
    if kind == 'missing': return [0.] * 36
    if kind in ('correlated', 'asymmetric'): c[1] = c[6] = .08
    if kind == 'rank_deficient': c[1] = c[6] = .1
    if kind == 'near_psd': c[1] = c[6] = .1 * (1. + 2.e-11)
    if kind in ('indefinite', 'scaled_indefinite'): c[1] = c[6] = .2
    if kind in ('scaled_indefinite', 'scaled_valid'): c[14] = 1.e12
    if kind == 'scaled_valid': c[1] = c[6] = .08
    if kind == 'asymmetric': c[6] += 2.e-10
    if kind == 'near_symmetric': c[1], c[6] = .08, .08+2.e-12
    if kind == 'negative_variance': c[21] = -.1
    if kind == 'nan': c[21] = math.nan
    if kind == 'inf': c[21] = math.inf
    if kind == 'outside_psd_tolerance': c[1] = c[6] = .1 * (1. + 2.e-9)
    return c


def ns(stamp): return stamp.sec*1_000_000_000+stamp.nanosec


@pytest.mark.parametrize('entry', ['atomic_bridge', 'canonical_bridge', 'legacy_bridge', 'legacy_status_bridge'])
def test_reject_revoke_and_fresh_recovery(entry, tmp_path):
    rclpy.init(); node = rclpy.create_node('covariance_parity_fixture')
    statuses = []; processes = []; logs = []
    started_at = node.get_clock().now().to_msg()
    node.create_subscription(LocalizationStatus, '/localization/status', statuses.append, 100)
    atomic = node.create_publisher(LocalizationFrame,
        '/zlocalization/session_input' if entry == 'canonical_bridge' else '/zlocalization/canonical_frame', 100)
    poses = node.create_publisher(PoseWithCovarianceStamped, '/localization/global_pose', 100)
    odometry = node.create_publisher(Odometry, '/odometry/lio', 100)
    upstream = node.create_publisher(LocalizationStatus, '/fixture/z_valid/status', 100)
    tf = StaticTransformBroadcaster(node)
    mount = TransformStamped(); mount.header.frame_id = 'base_link'; mount.child_frame_id = 'lio_imu_link'
    mount.transform.rotation.w = 1.; tf.sendTransform(mount)
    case = {'kind': 'diagonal', 'replay': False}; sequence = 0; last = None; emits = []

    def start(package, executable, parameters):
        log = (tmp_path / (executable+'.log')).open('w'); logs.append(log)
        argv = [str(Path(get_package_prefix(package))/'lib'/package/executable), '--ros-args']
        for key, value in parameters.items(): argv += ['-p', key+':='+str(value)]
        processes.append(subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def emit():
        nonlocal sequence, last
        sequence += 1; stamp = node.get_clock().now().to_msg()
        if case['replay']:
            frame = copy.deepcopy(last)
            frame.sequence = frame.state_generation = frame.pose_revision = sequence
            frame.emitted_at = stamp; frame.global_pose.pose.covariance = covariance('diagonal')
        else:
            frame = LocalizationFrame(producer_id='covariance-source', epoch=1, sequence=sequence,
                state_generation=sequence, pose_revision=sequence, has_odometry=True, valid=True, emitted_at=stamp)
            frame.odometry.header.stamp = stamp
            frame.odometry.header.frame_id = 'zlio_odom' if entry == 'canonical_bridge' else 'odom'
            frame.odometry.child_frame_id = 'lio_imu_link' if entry == 'canonical_bridge' else 'base_link'
            frame.odometry.pose.pose.orientation.w = 1.
            frame.global_pose.header.stamp = stamp; frame.global_pose.header.frame_id = 'map'
            frame.global_pose.pose.pose.orientation.w = 1.
            frame.global_pose.pose.covariance = covariance(case['kind'])
            last = copy.deepcopy(frame)
        if entry.startswith('legacy'):
            odometry.publish(frame.odometry)
            if entry == 'legacy_status_bridge':
                status = LocalizationStatus(header=copy.deepcopy(frame.global_pose.header),
                    producer_id='legacy-z-fixture', started_at=started_at, sequence=sequence,
                    valid=True, valid_until=Time(nanoseconds=ns(frame.global_pose.header.stamp)+500_000_000).to_msg(),
                    pose=frame.global_pose)
                status.header.stamp = stamp
                upstream.publish(status)
            else: poses.publish(frame.global_pose)
        else: atomic.publish(frame)
        emits.append((case['kind'], ns(frame.global_pose.header.stamp)))

    def pump(predicate, label, timeout=5.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            assert all(p.poll() is None for p in processes), 'owner exited'
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate(): return
        raise AssertionError(label)

    results = []
    try:
        if entry == 'canonical_bridge':
            start('rm_localization_adapters', 'lio_adapter', {
                'canonical_session_transactions': 'true', 'expected_input_odom_frame': 'zlio_odom',
                'raw_odom_parent_frame_mode': 'sensor_initial', 'sensor_initial_reference_mode': 'first_source_sample',
                'twist_mode': 'finite_difference'})
        bridge_params = {'canonical_session_transactions': 'false' if entry.startswith('legacy') else 'true'}
        if entry == 'legacy_status_bridge': bridge_params['upstream_valid_topic'] = '/fixture/z_valid'
        start('rm_relocalization_bridge', 'map_odom_from_global_pose', bridge_params)
        timer = node.create_timer(.02, emit)
        pump(lambda: bool(statuses) and statuses[-1].valid, 'PSD positive control')
        for kind in ['missing', 'diagonal', 'correlated', 'rank_deficient', 'near_psd', 'near_symmetric', 'scaled_valid']:
            case.update(kind=kind, replay=False); before = len(emits)
            pump(lambda: len(emits) >= before+4 and statuses[-1].valid and
                ns(statuses[-1].pose.header.stamp) >= emits[before][1], 'valid compatibility: '+kind)
            results.append({'case': kind, 'accepted': True})
        for kind in ['indefinite', 'scaled_indefinite', 'asymmetric', 'negative_variance', 'nan', 'inf', 'outside_psd_tolerance']:
            case.update(kind=kind, replay=False); before = len(statuses); emitted_before = len(emits)
            # Fail if rejection relies only on the old 0.5s source lease expiring.
            pump(lambda: len(emits) > emitted_before and len(statuses) > before and not statuses[-1].valid,
                'invalid covariance must revoke immediately: '+kind, timeout=.35)
            assert ns(statuses[-1].pose.header.stamp) == 0, 'held bad/old pose was not cleared'
            case['replay'] = True; replay_before = len(emits); rows_before = len(statuses)
            pump(lambda: len(emits) >= replay_before+5, 'same-stamp replay delivered')
            assert not any(m.valid for m in statuses[rows_before:]), 'same-stamp corrected replay resurrected validity'
            case.update(kind='diagonal', replay=False); fresh_before = len(emits)
            pump(lambda: len(emits) > fresh_before and statuses[-1].valid and
                ns(statuses[-1].pose.header.stamp) > emits[replay_before][1], 'fresh PSD recovery')
            results.append({'case': kind, 'rejected': True, 'same_stamp_cannot_recover': True, 'fresh_recovered': True})
        (tmp_path/'summary.json').write_text(json.dumps({'entry': entry, 'results': results}, indent=2)+'\n')
    finally:
        if 'timer' in locals(): timer.cancel()
        for process in reversed(processes): stop(process)
        for log in logs: log.close()
        node.destroy_node(); rclpy.shutdown()
