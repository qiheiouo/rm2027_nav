#!/usr/bin/env python3
"""Deterministic post-filter fixtures through production C++ publishers and DDS.

No point-cloud/IMU simulator; no EKF statistical-performance assertion.
Run inside the isolated Humble container after sourcing its own install overlay.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import ChassisHeadingState
from std_msgs.msg import Float64MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]
INSTALL = ROOT / 'build/install_humble'
RUNTIME = ROOT / 'build/runtime_humble'
EVIDENCE = ROOT / 'docs/research/fastlio_covariance_ros_validation/evidence'


def hat(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp(v):
    a = np.linalg.norm(v)
    k = hat(v)
    return np.eye(3) + (math.sin(a)/a if a else 1.)*k + ((1.-math.cos(a))/a**2 if a else .5)*(k@k)


def quat(r):
    # Eigenvalue method avoids branch assumptions for 180-degree fixtures.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3.
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[3] >= 0 else -q


def transform(p=(0., 0., 0.), v=(0., 0., 0.)):
    t = np.eye(4)
    t[:3, :3] = exp(np.array(v))
    t[:3, 3] = p
    return t


def message_transform(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v = np.array([q.x, q.y, q.z])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + 2*q.w*hat(v) + 2*hat(v)@hat(v)
    t[:3, 3] = [p.x, p.y, p.z]
    return t


def stamp(seconds):
    # Match production get_ros_time(double) truncation, including floating-point error.
    sec = math.floor(seconds)
    return sec, int((seconds-sec)*1e9)


def ns(msg):
    return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec


def pose_into(t, p):
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat(t[:3, :3]))


def known_covariance():
    l = np.array([[1,0,0,0,0,0], [.2,2,0,0,0,0], [-.3,.4,3,0,0,0],
                  [.5,-.2,.1,4,0,0], [-.1,.3,.2,.4,5,0], [.6,-.4,.2,-.1,.3,6.]])
    return .001*l@l.T


def conditional_valid(msg):
    p = np.array(msg.pose.covariance).reshape(6, 6)
    return bool(msg.header.frame_id == 'odom' and np.isfinite(p).all() and
                np.max(np.abs(p-p.T)) < 1e-10 and np.max(np.abs(p)) > 0 and
                np.linalg.eigvalsh(p).min() >= -1e-10)


def finite_difference_jacobian(s, e, parent_rotation=False):
    # Independent differentiation of the pose conversion, not covariance helper code.
    convert = lambda x: e@x@np.linalg.inv(e) if parent_rotation else x@np.linalg.inv(e)
    b = convert(s)
    j = np.zeros((6, 6))
    h = 1e-6
    for i in range(6):
        differences = []
        for sign in [1., -1.]:
            perturbed = s.copy()
            if i < 3:
                perturbed[i, 3] += sign*h
            else:
                d = np.zeros(3)
                d[i-3] = sign*h
                perturbed[:3, :3] = exp(d)@s[:3, :3]
            out = convert(perturbed)
            r = out[:3, :3]@b[:3, :3].T
            rotation_delta = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]])/2
            differences.append(np.r_[out[:3, 3]-b[:3, 3], rotation_delta])
        j[:, i] = (differences[0]-differences[1])/(2*h)
    return j


class Test:
    def __init__(self, phase):
        self.phase = phase
        self.results = {'phase': phase, 'domain': os.environ['ROS_DOMAIN_ID'],
                        'rmw': os.environ['RMW_IMPLEMENTATION'], 'checks': [], 'observations': []}
        self.node = rclpy.create_node('covariance_contract_receiver_'+phase)
        self.processes = []
        self.subscriptions = []
        self.static = StaticTransformBroadcaster(self.node)
        self.dynamic = TransformBroadcaster(self.node)
        self.tf_sub = self.node.create_subscription(
            __import__('tf2_msgs.msg', fromlist=['TFMessage']).TFMessage, '/tf', self.on_tf, 100)
        self.tfs = []

    def on_tf(self, msg):
        self.tfs.extend(msg.transforms)

    def spin(self, duration=.2):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(.02, max(0., end-time.monotonic())))

    def wait(self, predicate, description, timeout=5.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.spin(.02)
        raise AssertionError('timeout: '+description)

    def check(self, label, success, **evidence):
        self.results['checks'].append(dict(label=label, passed=bool(success), **evidence))
        if not success:
            raise AssertionError(label+' '+str(evidence))

    def start(self, package, binary, name, params=None, remaps=None, params_file=None):
        if self.phase == 'baseline' and package == 'rm_localization_adapters' and binary == 'lio_adapter':
            package, binary = 'rm_fastlio_covariance_ros_validation', 'baseline_lio_adapter'
        args = [str(INSTALL/package/'lib'/package/binary), '--ros-args', '-r', '__node:='+name]
        if params_file:
            args += ['--params-file', str(params_file)]
        for key, value in (params or {}).items():
            args += ['-p', key+':='+str(value).lower() if isinstance(value, bool) else key+':='+str(value)]
        for key, value in (remaps or {}).items():
            args += ['-r', key+':='+value]
        log = open(RUNTIME/(self.phase+'_'+name+'.log'), 'w')
        p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((p, log))
        self.spin(.6)
        self.check(name+'_alive', p.poll() is None)
        return p

    def stop(self, p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()

    def collect(self, topic, kind=Odometry):
        records = []
        sub = self.node.create_subscription(kind, topic, lambda m: records.append((time.monotonic(), m)), 100)
        self.subscriptions.append(sub)
        return records

    def boundary(self, mode, label, parent='odom'):
        prefix = '/covariance_validation/'+self.phase+'/'+label
        records = self.collect(prefix+'/raw')
        pub = self.node.create_publisher(Float64MultiArray, prefix+'/fixture', 10)
        process = self.start('rm_fastlio_covariance_ros_validation', 'covariance_'+mode, label,
                             dict(raw_topic=prefix+'/raw', command_topic=prefix+'/fixture', parent_frame=parent))
        self.wait(lambda: pub.get_subscription_count() == 1, label+' discovery')
        self.spin(.3)
        return pub, records, process, prefix

    def send(self, pub, records, source_time, t, p, zero=False):
        d = Float64MultiArray()
        d.data = list(map(float, np.r_[source_time, t[:3, 3], quat(t[:3, :3]), float(zero), p.flatten()]))
        count = len(records)
        before = time.monotonic()
        pub.publish(d)
        self.wait(lambda: len(records) > count, 'raw receive')
        self.check('single_raw_delivery', len(records) == count+1)
        return records[-1][1], (records[-1][0]-before)*1000.

    def raw(self, mode):
        pub, records, process, _ = self.boundary(mode, 'raw_'+mode)
        s = transform([1., 2., 3.], [.3, -.2, .7])
        fixtures = [('first_identity', 100., np.eye(4), np.diag([.01,.02,.03,.04,.05,.06]), False),
                    ('dense_rotated', 100.1, s, known_covariance(), False),
                    ('consecutive', 100.2, transform([4,3,2], [-.1,.4,.8]), known_covariance()*2, False),
                    ('duplicate_stamp', 100.2, s, known_covariance()*3, False),
                    ('reverse_stamp', 99.9, s, known_covariance()*4, False),
                    ('future_stamp', time.time()+30., s, known_covariance(), False),
                    ('zero_stamp', 0., s, known_covariance(), False)]
        if mode == 'bundle':
            fixtures += [('zero_wait_first', 101., s, known_covariance(), True),
                         ('zero_activation', 101.1, transform([2,3,4], [0,0,math.pi/2]), known_covariance(), True),
                         ('zero_following', 101.2, transform([4,5,6], [.2,.1,1.8]), known_covariance()*5, True),
                         ('zero_disabled', 101.3, s, known_covariance(), False)]
        g = np.eye(4)
        zero_count = 0
        captured = []
        latencies = []
        for label, ts, t, p, zero in fixtures:
            if zero:
                zero_count += 1
                if zero_count == 2:
                    g = np.linalg.inv(t)
            basis = g if zero and zero_count >= 2 else np.eye(4)
            expected_pose = basis@t
            j = np.zeros((6, 6))
            j[:3, :3] = basis[:3, :3]
            j[3:, 3:] = expected_pose[:3, :3]
            expected = j@p@j.T
            tf_count = len(self.tfs)
            msg, delay = self.send(pub, records, ts, t, p, zero)
            actual = np.array(msg.pose.covariance).reshape(6, 6)
            error = float(np.max(np.abs(actual-expected)))
            pose_error = float(np.max(np.abs(message_transform(msg)-expected_pose)))
            self.check(mode+'_'+label, error < 1e-11 and pose_error < 1e-11 and
                       (msg.header.stamp.sec, msg.header.stamp.nanosec) == stamp(ts) and
                       msg.header.frame_id == 'odom' and msg.child_frame_id == 'body',
                       max_covariance_error=error, max_pose_error=pose_error)
            # Raw TF is emitted immediately after Odometry; verify same sampled state.
            target_ns = ns(msg)
            self.wait(lambda: any(ns(x) == target_ns and x.child_frame_id == 'body' for x in self.tfs[tf_count:]), 'raw TF')
            tf = [x for x in self.tfs[tf_count:] if ns(x) == target_ns and x.child_frame_id == 'body'][-1]
            tf_pose = Odometry()
            tf_pose.pose.pose.position.x = tf.transform.translation.x
            tf_pose.pose.pose.position.y = tf.transform.translation.y
            tf_pose.pose.pose.position.z = tf.transform.translation.z
            tf_pose.pose.pose.orientation = tf.transform.rotation
            self.check(mode+'_'+label+'_tf_frame_stamp_pose', tf.header.frame_id == msg.header.frame_id and
                       np.max(np.abs(message_transform(tf_pose)-expected_pose)) < 1e-11)
            captured.append(dict(label=label, stamp_ns=ns(msg), frame=msg.header.frame_id,
                                 pose_matrix=message_transform(msg).tolist(), covariance=list(msg.pose.covariance)))
            latencies.append(delay)
        self.spin(.1)
        self.check(mode+'_no_extra_delivery', len(records) == len(fixtures))
        self.results['observations'].append(dict(mode=mode, received=captured,
            roundtrip_ms=dict(min=min(latencies), max=max(latencies), median=float(np.median(latencies))),
            note='command->publisher->DDS->subscriber roundtrip, not pure DDS latency; source timestamps preserved, not freshness gated'))
        self.stop(process)

    def smoke(self, mode):
        topic = '/covariance_validation/smoke_'+mode
        records = self.collect(topic)
        p = self.start('fast_lio_multi', 'laserMapping_'+mode, 'production_'+mode,
                       {'common.multi_lidar': False, 'common.map_frame': 'odom', 'common.publish_tf_results': False},
                       {'/Odometry': topic}, ROOT/'src/fast_lio_multi/config/multi.yaml')
        self.spin(.7)
        names = [x[0] for x in self.node.get_node_names_and_namespaces()]
        self.check(mode+'_production_executor_discovered', 'production_'+mode in names and not records)
        self.results['observations'].append(dict(production_entry=mode, startup='PASS', sensor_updates='NOT_RUN',
                                                reason='no credible MID360 per-point timestamps and synchronized IMU/deskew input'))
        self.stop(p)

    def tf(self, label, e, source_time, dynamic=False):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_'+label, 'sensor_'+label
        t.header.stamp.sec, t.header.stamp.nanosec = stamp(source_time)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, e[:3, 3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, quat(e[:3, :3]))
        (self.dynamic if dynamic else self.static).sendTransform(t)
        self.spin(.2)

    def adapter(self, label, kind='conditional', parent='odom', dynamic=False, bad=None):
        pub, raw, source, prefix = self.boundary('bundle', label, parent)
        output = self.collect(prefix+'/canonical')
        params = dict(raw_odom_topic=prefix+'/raw', output_odom_topic=prefix+'/canonical',
                      expected_input_odom_frame=parent, odom_frame='odom', base_frame='base_'+label,
                      input_sensor_frame='sensor_'+label, publish_tf=False,
                      backend_child_frame_alias_enabled=True, backend_child_frame_alias_target='sensor_'+label,
                      pose_covariance_mode='unavailable' if kind in ('default', 'zero_start') else 'conditional_sensor_tf')
        if kind == 'sensor_initial':
            params['raw_odom_parent_frame_mode'] = 'sensor_initial'
        if kind == 'latest':
            params['use_latest_transform'] = True
        if kind == 'placeholder':
            params['use_tf_sensor_to_base'] = False
        heading_pub = None
        if kind == 'heading':
            params.update({'pose_conversion_mode': 'chassis_heading_fusion', 'twist_mode': 'finite_difference',
                           'heading_fusion.initial_alignment_confirmed': True,
                           'heading_fusion.heading_topic': prefix+'/heading',
                           'heading_fusion.derived_gimbal_topic': prefix+'/derived'})
            heading_pub = self.node.create_publisher(ChassisHeadingState, prefix+'/heading', 10)
        adapter = self.start('rm_localization_adapters', 'lio_adapter', 'adapter_'+label, params)
        self.spin(.4)
        s = transform([1,2,3], [.1,-.2,.6])
        p = known_covariance()
        if kind == 'heading':
            p = np.diag([.09,.01,.04,.02,.03,.04])
            p[0,5] = p[5,0] = .054  # Valid SPD; replacing only yaw diagonal makes it indefinite.
            s = np.eye(4)
        if bad == 'asymmetric': p[0, 1] += .1
        if bad == 'indefinite': p[0, 1] = p[1, 0] = 2.
        if bad == 'zero': p.fill(0.)
        if bad == 'nan': p[0, 0] = np.nan
        errors = []
        outputs = []
        for i in range(2):
            ts = 0. if kind == 'zero_stamp' and i == 0 else 200.+i*.1
            e = transform([.4,-.2,.3], [.2,.1,(.3+i*.5) if dynamic else .3])
            if kind not in ('placeholder', 'heading'):
                self.tf(label, e, ts, dynamic)
            if heading_pub is not None:
                h = ChassisHeadingState()
                h.header.stamp.sec, h.header.stamp.nanosec = stamp(ts)
                h.online, h.valid, h.source_boot_id = True, True, 7
                h.sample_sequence, h.mcu_time_ms = i+1, 1000+i*100
                h.yaw_rad = 0.
                heading_pub.publish(h)
                self.spin(.15)
            count = len(output)
            raw_msg, _ = self.send(pub, raw, ts, s, p, zero=(kind == 'zero_start'))
            current_s = message_transform(raw_msg)
            if kind == 'heading' and i == 0:
                self.spin(.2)
                self.check(label+'_first_frame_warmup', len(output) == 0)
                continue
            self.wait(lambda: len(output) > count, label+' adapter receive')
            msg = output[-1][1]
            actual = np.array(msg.pose.covariance).reshape(6, 6)
            raw_p = np.array(raw_msg.pose.covariance).reshape(6, 6)
            available = (kind == 'conditional' or (kind == 'zero_stamp' and i == 1)) and parent == 'odom' and bad is None
            if self.phase == 'baseline':
                expected = raw_p.copy()
                if kind == 'heading': expected[5,5] = .01
                self.check(label+'_baseline_copy_or_yaw_only', np.allclose(actual, expected, equal_nan=True))
                if bad is None and kind != 'heading':
                    j = finite_difference_jacobian(current_s, e, kind == 'sensor_initial')
                    error = float(np.max(np.abs(actual-j@raw_p@j.T)))
                    errors.append(error)
                if kind == 'heading':
                    minimum = float(np.linalg.eigvalsh(actual).min())
                    self.check('heading_false_confidence_counterexample', minimum < -.01,
                               raw_min_eigenvalue=float(np.linalg.eigvalsh(raw_p).min()), output_min_eigenvalue=minimum)
            elif available:
                j = finite_difference_jacobian(current_s, e)
                expected = j@raw_p@j.T
                error = float(np.max(np.abs(actual-expected)))
                self.check(label+'_conditional_full_covariance', error < 1e-9 and conditional_valid(msg),
                           independent_finite_difference_error=error)
                errors.append(error)
            else:
                self.check(label+'_unavailable_all36', np.isnan(actual).all() and not conditional_valid(msg))
            self.check(label+'_output_frame_stamp', ns(msg) == ns(raw_msg) and
                       msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_'+label)
            if kind != 'heading' and kind != 'placeholder':
                expected_pose = e@current_s@np.linalg.inv(e) if kind == 'sensor_initial' else current_s@np.linalg.inv(e)
                self.check(label+'_pose_unchanged', np.max(np.abs(message_transform(msg)-expected_pose)) < 1e-9)
            outputs.append(dict(stamp_ns=ns(msg), covariance=list(msg.pose.covariance), consumer_accepts=conditional_valid(msg)))
        if kind == 'heading':
            h.header.stamp.sec, h.header.stamp.nanosec = stamp(200.2)
            h.sample_sequence, h.reset_counter = 3, 1
            heading_pub.publish(h)
            self.spin(.15)
            count = len(output)
            self.send(pub, raw, 200.2, s, p)
            self.spin(.4)
            self.check('heading_reset_epoch_latched', len(output) == count)
        self.results['observations'].append(dict(adapter_case=label, max_covariance_error=max(errors) if errors else None,
                                                outputs=outputs, dynamic_tf_conditioned=dynamic))
        self.stop(adapter)
        self.stop(source)

    def temporal_adapter(self):
        topic = '/covariance_validation/'+self.phase+'/temporal'
        pub = self.node.create_publisher(Odometry, topic+'/raw', 10)
        out = self.collect(topic+'/canonical')
        p = self.start('rm_localization_adapters', 'lio_adapter', 'temporal_adapter',
                       dict(raw_odom_topic=topic+'/raw', output_odom_topic=topic+'/canonical', publish_tf=False))
        self.wait(lambda: pub.get_subscription_count() == 1, 'temporal adapter discovery')
        source_times = [300., 300., 299., time.time()+30., 0.]
        for ts in source_times:
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.header.stamp.sec, msg.header.stamp.nanosec = stamp(ts)
            msg.pose.pose.orientation.w = 1.
            msg.pose.covariance = known_covariance().flatten().tolist()
            count = len(out)
            pub.publish(msg)
            self.wait(lambda: len(out) > count, 'temporal adapter output')
        self.check('passthrough_not_freshness_gate', [ns(m) for _, m in out] == [a*10**9+b for a,b in map(stamp, source_times)])
        self.results['observations'].append(dict(temporal_adapter='stale/duplicate/reverse/future/zero timestamps forwarded',
                                                status='Modify: no freshness or numerical-basis epoch gate in passthrough'))
        self.stop(p)

    def consumers(self):
        prefix = '/covariance_validation/'+self.phase+'/consumer'
        odom_pub = self.node.create_publisher(Odometry, prefix+'/odom', 10)
        pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, prefix+'/global', 10)
        corrections = self.collect(prefix+'/correction', TransformStamped)
        bridge = self.start('rm_relocalization_bridge', 'map_odom_from_global_pose', 'bridge_consumer',
                            dict(odom_topic=prefix+'/odom', global_pose_topic=prefix+'/global',
                                 map_to_odom_topic=prefix+'/correction', publish_tf=False))
        self.wait(lambda: odom_pub.get_subscription_count() == 1 and pose_pub.get_subscription_count() == 1, 'bridge discovery')
        for index, covariance in enumerate([known_covariance(), np.full((6,6), np.nan)]):
            ts = time.time()
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp.sec, odom.header.stamp.nanosec = stamp(ts)
            pose_into(transform([1,0,0]), odom.pose.pose)
            odom.pose.covariance = covariance.flatten().tolist()
            global_pose = PoseWithCovarianceStamped()
            global_pose.header = copy.deepcopy(odom.header)
            global_pose.header.frame_id = 'map'
            pose_into(transform([3,0,0]), global_pose.pose.pose)
            count = len(corrections)
            odom_pub.publish(odom)
            self.spin(.08)
            pose_pub.publish(global_pose)
            self.wait(lambda: len(corrections) > count, 'bridge correction')
            self.check('bridge_pose_only_'+str(index), abs(corrections[-1][1].transform.translation.x-2.) < 1e-9)
        self.stop(bridge)
        gate_in = self.node.create_publisher(PoseWithCovarianceStamped, prefix+'/gate_in', 10)
        accepted = self.collect(prefix+'/gate_out', PoseWithCovarianceStamped)
        gate = self.start('rm_relocalization_bridge', 'global_pose_gate_node', 'gate_consumer',
                         dict(input_topic=prefix+'/gate_in', output_topic=prefix+'/gate_out', valid_topic=prefix+'/valid'))
        self.wait(lambda: gate_in.get_subscription_count() == 1, 'gate discovery')
        gate_cases = []
        for label in ['valid', 'indefinite_positive_diagonal', 'nan']:
            p = known_covariance()
            if label == 'indefinite_positive_diagonal': p[0,1] = p[1,0] = .5
            if label == 'nan': p[0,0] = np.nan
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp.sec, msg.header.stamp.nanosec = stamp(time.time())
            msg.pose.pose.orientation.w = 1.
            msg.pose.covariance = p.flatten().tolist()
            count = len(accepted)
            gate_in.publish(msg)
            self.spin(.15)
            forwarded = len(accepted) > count
            self.check('global_gate_'+label, forwarded == (label != 'nan'))
            gate_cases.append(dict(case=label, accepted=forwarded))
        self.results['observations'].append(dict(consumer='production map_odom bridge is pose-only; global gate checks finite+diagonal, not full PSD',
                                                global_gate=gate_cases, information_matrix_inversion='none in exercised path'))
        self.stop(gate)

    def run(self, resume_consumers=False):
        try:
            if resume_consumers:
                self.results = json.loads((EVIDENCE/(self.phase+'_ros.json')).read_text())
                assert any(c['label'] == 'passthrough_not_freshness_gate' and c['passed'] for c in self.results['checks'])
            else:
                for mode in ['bundle', 'adaptive', 'async']:
                    self.smoke(mode)
                    self.raw(mode)
                for label, kw in [('lever', {}), ('dynamic_gimbal', {'dynamic': True}),
                                  ('parent_rotation', {'kind': 'sensor_initial'}),
                                  ('heading', {'kind': 'heading'}), ('default', {'kind': 'default'}),
                                  ('rename', {'parent': 'raw_parent'}), ('latest', {'kind': 'latest'}),
                                  ('placeholder', {'kind': 'placeholder'}),
                                  ('zero_stamp', {'kind': 'zero_stamp'}), ('zero_start', {'kind': 'zero_start'})]:
                    self.adapter(label, **kw)
                for bad in ['asymmetric', 'indefinite', 'zero', 'nan']:
                    self.adapter('invalid_'+bad, bad=bad)
                self.temporal_adapter()
            self.consumers()
            self.results['completed'] = True
        finally:
            for p, log in self.processes:
                self.stop(p)
                log.close()
            self.node.destroy_node()
            EVIDENCE.mkdir(parents=True, exist_ok=True)
            def clean(value):
                if isinstance(value, float) and not math.isfinite(value): return None
                if isinstance(value, list): return [clean(v) for v in value]
                if isinstance(value, dict): return {k: clean(v) for k,v in value.items()}
                return value
            (EVIDENCE/(self.phase+'_ros.json')).write_text(json.dumps(clean(self.results), indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=['baseline', 'candidate'], required=True)
    parser.add_argument('--resume-consumers', action='store_true', help='resume only after an incomplete downstream build')
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        Test(args.phase).run(args.resume_consumers)
    finally:
        rclpy.shutdown()
