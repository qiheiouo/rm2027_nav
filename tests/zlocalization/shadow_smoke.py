#!/usr/bin/env python3
"""Actual native sensor processing through the RM shadow launch, no hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from localization_interfaces.msg import FrontendFrame, FrontendSession
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import SetBool, Trigger
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    upstream = ROOT / 'external/z_localization'
    spec = importlib.util.spec_from_file_location('upstream_sensor_fixture',
        upstream / 'src/frontend/test/ros/test_sensor_pipeline.py')
    fixture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    config = out / 'backend.yaml'
    config.write_text(yaml.safe_dump({'reset': {'min_gap': 0.0}}))
    rclpy.init()
    node = rclpy.create_node('native_shadow_smoke')
    imus = node.create_publisher(Imu, '/test/imu', 1000)
    clouds = node.create_publisher(PointCloud2, '/test/lidar', 20)
    frames, sessions, odometry = [], [], []
    node.create_subscription(FrontendFrame, '/zlocalization/shadow/frontend/frame', frames.append, 100)
    node.create_subscription(FrontendSession, '/zlocalization/shadow/frontend/session', sessions.append,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry, '/zlocalization/shadow/sensor_odometry', odometry.append, 100)
    log = (out / 'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'rm_zlocalization_candidate', 'zlio_shadow.launch.py',
        'enabled:=true', 'frontend_config:=' + str(upstream / 'config/lio.yaml'),
        'config_file:=' + str(config), 'imu_acceleration_scale:=1.0', 'input_type:=pointcloud2',
        'lidar_topic:=/test/lidar', 'imu_topic:=/test/imu', 'use_sim_time:=false',
        'wait_for_start:=true', 'visualization:=true', 'world_cloud:=true'],
        env=dict(os.environ), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            assert process.poll() is None, 'shadow launch exited; see launch.log'

    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(request)
        end = time.monotonic() + 30
        while not future.done() and time.monotonic() < end:
            spin(.02)
        assert future.done() and future.result().success, name
        return future.result()

    try:
        spin(2)
        assert sessions and not sessions[-1].failed
        call(SetBool, '/zlocalization/shadow/start', SetBool.Request(data=True))
        cloud = fixture.room_message('pointcloud2')
        start = node.get_clock().now().nanoseconds
        for step in range(1, 801):
            stamp = start + step * 10_000_000
            imu = Imu()
            fixture.set_stamp(imu.header.stamp, stamp)
            imu.linear_acceleration.z = 9.81
            imus.publish(imu)
            if step % 10 == 0:
                fixture.set_stamp(cloud.header.stamp, stamp - 100_000_000)
                clouds.publish(cloud)
            spin(.01)
            if step > 450 and frames and odometry:
                break
        spin(1)
        assert frames and odometry, 'no native sensor estimates'
        assert frames[-1].odometry.header.frame_id == 'zlio_odom'
        assert frames[-1].cloud.header.frame_id == 'lio_imu_link'
        assert odometry[-1].header.frame_id == 'map'
        assert not node.get_publishers_info_by_topic('/tf'), 'shadow leaked public TF'
        assert not node.get_publishers_info_by_topic('/odometry/lio')
        assert not node.get_publishers_info_by_topic('/localization/status')
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        call(Trigger, '/zlocalization/shadow/stop', Trigger.Request())
        previous = sessions[-1].epoch
        call(Trigger, '/zlocalization/shadow/upstream_reset_request', Trigger.Request())
        spin(1)
        assert sessions[-1].epoch > previous, 'coordinated native reset did not advance epoch'
        result = dict(sensor_frames=len(frames), map_sensor_odometry=len(odometry),
            native_epoch_after_reset=sessions[-1].epoch, public_tf_publishers=0,
            canonical_or_command_publishers=0,
            scope='synthetic stationary room; native software functionality only; no accuracy or Hardware Go')
        (out / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        assert 'process has died' not in (out/'launch.log').read_text(), 'child process failed; see launch.log'


if __name__ == '__main__':
    main()
