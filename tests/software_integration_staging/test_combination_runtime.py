"""Bounded cross-candidate regressions: real Mission/Nav2/smoother, no devices."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('arbitration_fixture', ROOT / 'src/rm_competition_mission/test/test_navigation_arbitration_runtime.py')
arbitration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arbitration)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
    assert process.returncode == 0, f'abnormal child exit {process.returncode}'


@pytest.mark.parametrize('boundary', ['epoch', 'source'])
def test_mission_retires_pre_session_navigation(boundary, tmp_path):
    """A depth-1 status consumer must handle a new valid session even if it missed invalid."""
    rclpy.init()
    node = arbitration.ArbitrationFixture()
    real_publisher = node.loc_pub
    session = {'id': 'canonical-A', 'epoch': 1, 'valid': True}
    observed = []

    class VersionedPublisher:
        def publish(self, message):
            message.session_id = session['id']
            message.session_epoch = session['epoch']
            message.valid = session['valid']
            real_publisher.publish(message)
            observed.append((message.sequence, message.session_id, message.session_epoch))

    node.loc_pub = VersionedPublisher()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    params = tmp_path / 'mission.yaml'
    params.write_text(yaml.safe_dump({'competition_mission_node': {'ros__parameters': {
        'default_mode': 'auto', 'allow_pursuit': False, 'tick_rate_hz': 20.,
        'patrol_waypoints': [1., 0., 0.],
    }}}))
    log = (tmp_path / 'mission.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', str(ROOT / 'src/rm_navigation_bringup/launch/competition_2027_navigation.launch.py'),
        'enable_navigation_intents:=true',
        f'mission_config:={params}'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        arbitration.wait(lambda: node.state is not None, 'mission ready')
        node.mode('auto')
        arbitration.wait_goal(node, 1.)
        before = len(node.cancels)
        if boundary == 'epoch':
            session['epoch'] += 1
        else:
            session['id'] = 'canonical-B'
            session['epoch'] += 1
        arbitration.wait(lambda: observed[-1][1:] == (session['id'], session['epoch']), 'new session emitted')
        # Same bridge process, strictly increasing emission/sequence, unexpired lease.
        # The old Nav2 action was computed using the retired canonical basis.
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline and len(node.cancels) == before:
            time.sleep(.01)
        session_canceled = len(node.cancels) > before
        # Positive control: the same action/server responds to invalid status.
        session['valid'] = False
        arbitration.wait(lambda: len(node.cancels) > before, 'invalid status cancels old action')
        (tmp_path / 'session_result.json').write_text(json.dumps({
            'boundary': boundary, 'new_session': session, 'session_canceled': session_canceled,
            'invalid_status_canceled': True, 'requests': node.requests, 'cancels': node.cancels,
            'new_session_statuses': sum(row[1:] == (session['id'], session['epoch']) for row in observed),
            'max_active': node.max_active,
        }, indent=2) + '\n')
        assert session_canceled, 'new canonical session left old navigation active'
        assert node.max_active <= 1
    finally:
        node.mode('hold')
        arbitration.wait(lambda: not node.active, 'fixture terminal cleanup')
        stop(process)
        for timer in node.timers: timer.cancel()
        executor.remove_node(node)
        executor.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def test_controller_cancel_reaches_unique_final_smoother(tmp_path):
    """Real AUD-016 controller cancellation through the A single-owner velocity path."""
    rclpy.init()
    node = rclpy.create_node('combined_cancel_velocity_probe')
    tf = TransformBroadcaster(node)
    rows, events = [], []
    node.create_subscription(Twist, '/cmd_vel', lambda m: rows.append((time.monotonic(), m.linear.x)), 100)
    node.create_subscription(String, '/fixture/events', lambda m: events.append(m.data), 100)
    config = tmp_path / 'controller.yaml'
    config.write_text(yaml.safe_dump({
        'controller_server': {'ros__parameters': {
            'controller_plugins': ['Fixture'], 'Fixture.plugin': 'fixture::Controller',
            'controller_frequency': 20., 'odom_topic': '/fixture/odom'}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'base_link', 'resolution': .05,
            'width': 6, 'height': 6, 'rolling_window': True, 'update_frequency': 20.,
            'plugins': ['gate'], 'gate.plugin': 'fixture::Layer'}}},
    }))
    processes, logs = [], []

    def start(package, executable, arguments):
        log = (tmp_path / (executable + '.log')).open('w'); logs.append(log)
        binary = str(Path(get_package_prefix(package)) / 'lib' / package / executable)
        processes.append(subprocess.Popen([binary, '--ros-args', *arguments], stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True))

    def spin(predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), 'child exited'
            t = TransformStamped(); t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
            t.header.stamp = node.get_clock().now().to_msg(); t.transform.rotation.w = 1.
            tf.sendTransform(t)
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate(): return
        raise AssertionError('runtime predicate timed out')

    def call(client, request):
        spin(client.service_is_ready)
        future = client.call_async(request); spin(future.done)
        assert future.result().success

    def transition(name, value):
        client = node.create_client(ChangeState, '/' + name + '/change_state')
        request = ChangeState.Request(); request.transition.id = value
        call(client, request); node.destroy_client(client)

    action = ActionClient(node, FollowPath, '/follow_path')
    gate = node.create_client(SetBool, '/fixture/current')
    try:
        start('nav2_controller', 'controller_server', ['--params-file', str(config), '-r', 'cmd_vel:=/cmd_vel_nav'])
        start('nav2_velocity_smoother', 'velocity_smoother', ['--params-file',
            str(ROOT / 'src/rm_nav_config/config/nav2_phase2f_deployment.yaml'),
            '-r', 'cmd_vel:=/cmd_vel_nav', '-r', 'cmd_vel_smoothed:=/cmd_vel'])
        for name in ('controller_server', 'velocity_smoother'):
            transition(name, 1); transition(name, 3)
        assert {e.node_name for e in node.get_publishers_info_by_topic('/cmd_vel')} == {'velocity_smoother'}
        assert {e.node_name for e in node.get_publishers_info_by_topic('/cmd_vel_nav')} == {'controller_server'}
        call(gate, SetBool.Request(data=True))
        spin(action.server_is_ready)
        pose = PoseStamped(); pose.header.frame_id = 'map'; pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = 1.; pose.pose.orientation.w = 1.
        path = NavPath(header=pose.header, poses=[pose])
        sent = action.send_goal_async(FollowPath.Goal(path=path, controller_id='Fixture'))
        spin(sent.done); handle = sent.result(); assert handle.accepted
        terminal = handle.get_result_async()
        spin(lambda: any(v > .01 for _, v in rows))
        call(gate, SetBool.Request(data=False))
        cancel = handle.cancel_goal_async(); spin(cancel.done); assert cancel.result().goals_canceling
        spin(terminal.done); assert terminal.result().status == 5
        terminal_at = time.monotonic()
        compute_at_terminal = events.count('compute')
        # Smoother deceleration is bounded by the unchanged acceleration config.
        spin(lambda: rows and abs(rows[-1][1]) < 1e-9, timeout=2.)
        zero_at = rows[-1][0]
        call(gate, SetBool.Request(data=True))
        end = time.monotonic() + .35
        spin(lambda: time.monotonic() >= end)
        assert all(abs(v) < 1e-9 for at, v in rows if at >= zero_at)
        assert events.count('compute') == compute_at_terminal
        (tmp_path / 'velocity_result.json').write_text(json.dumps({
            'terminal_status': terminal.result().status,
            'final_publishers': sorted(e.node_name for e in node.get_publishers_info_by_topic('/cmd_vel')),
            'observed_terminal_to_zero_sec': max(0., zero_at-terminal_at),
            'old_compute_after_restore': events.count('compute')-compute_at_terminal,
            'nonzero_after_observed_zero': sum(abs(v) > 1e-9 for at, v in rows if at >= zero_at),
            'scope': 'software smoother deceleration and action retirement; no physical stop SLA',
        }, indent=2) + '\n')
    finally:
        for process in reversed(processes): stop(process)
        for log in logs: log.close()
        action.destroy(); node.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('entry', ['z_localization_candidate.launch.py', 'zlio_shadow.launch.py'])
def test_repeated_default_off_starts_no_process(entry, tmp_path):
    for trial in range(3):
        result = subprocess.run(['ros2', 'launch', 'rm_zlocalization_candidate', entry],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=12)
        assert result.returncode == 0, result.stdout
        assert 'process started with pid' not in result.stdout
        (tmp_path / f'{trial}.log').write_text(result.stdout)


def test_native_bridge_rejects_finite_indefinite_covariance(tmp_path):
    """The Z atomic path should meet A's global-pose covariance validity contract."""
    from rm_competition_interfaces.msg import LocalizationFrame, LocalizationStatus
    rclpy.init()
    node = rclpy.create_node('combined_covariance_probe')
    publisher = node.create_publisher(LocalizationFrame, '/zlocalization/canonical_frame', 100)
    statuses = []
    node.create_subscription(LocalizationStatus, '/localization/status', statuses.append, 100)
    binary = Path(get_package_prefix('rm_relocalization_bridge')) / 'lib/rm_relocalization_bridge/map_odom_from_global_pose'
    log = (tmp_path / 'bridge.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '-p', 'canonical_session_transactions:=true'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    sequence = 0

    def emit(indefinite):
        nonlocal sequence
        sequence += 1
        stamp = node.get_clock().now().to_msg()
        frame = LocalizationFrame(producer_id='canonical-covariance-fixture', epoch=1,
            sequence=sequence, state_generation=sequence, pose_revision=sequence,
            has_odometry=True, valid=True, emitted_at=stamp)
        frame.odometry.header.frame_id = 'odom'; frame.odometry.child_frame_id = 'base_link'
        frame.odometry.header.stamp = stamp; frame.odometry.pose.pose.orientation.w = 1.
        frame.global_pose.header.frame_id = 'map'; frame.global_pose.header.stamp = stamp
        frame.global_pose.pose.pose.orientation.w = 1.
        for i in range(6): frame.global_pose.pose.covariance[7*i] = 1.
        if indefinite:
            frame.global_pose.pose.covariance[1] = frame.global_pose.pose.covariance[6] = 2.
        publisher.publish(frame)
        return stamp

    try:
        deadline = time.monotonic() + 5.
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert publisher.get_subscription_count() == 1
        while not any(m.valid for m in statuses) and time.monotonic() < deadline:
            emit(False); rclpy.spin_once(node, timeout_sec=.02)
        assert any(m.valid for m in statuses), 'positive PSD control never accepted'
        before = len(statuses)
        end = time.monotonic() + .25
        while time.monotonic() < end:
            emit(True); rclpy.spin_once(node, timeout_sec=.01)
        bad = [m for m in statuses[before:] if m.valid and m.pose.pose.covariance[1] == 2.]
        (tmp_path / 'covariance_result.json').write_text(json.dumps({
            'positive_psd_accepted': True, 'indefinite_valid_statuses': len(bad),
            'bad_2x2_determinant': -3., 'accepted_covariance': list(bad[-1].pose.pose.covariance) if bad else [],
        }, indent=2) + '\n')
        assert not bad, 'native bridge marked a finite covariance with eigenvalue -1 valid'
    finally:
        stop(process); log.close(); node.destroy_node(); rclpy.shutdown()
