#!/usr/bin/env python3
"""Bounded integration check. Run only in an isolated, no-device ROS domain."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from tf2_msgs.msg import TFMessage


def quaternion(roll, pitch, yaw):
    sr, cr = math.sin(roll / 2), math.cos(roll / 2)
    sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def main():
    # The supplied container runner provides the isolation; never infer it from ROS_DOMAIN_ID.
    if os.environ.get("RM_DEMO_ISOLATED_BENCH") != "1":
        raise RuntimeError("Use run_bench_container.sh check: this check injects synthetic inputs.")
    rclpy.init()
    node = rclpy.create_node("demo_bench_check")
    samples, joints, raw, frames, errors = [], [], [], [], []
    edges = set()
    expected_q = quaternion(0.12, 0.22, 0.4)

    def odometry(msg):
        samples.append(time.monotonic())
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            errors.append("noncanonical odometry frames")
        if max(abs(p.x - 1.0), abs(p.y - 2.0), abs(p.z - 0.3)) > 1e-5:
            errors.append("lever-arm/base translation mismatch")
        dot = sum(a * b for a, b in zip((q.x, q.y, q.z, q.w), expected_q))
        if abs(abs(dot) - 1.0) > 1e-8:
            errors.append("full base orientation mismatch")

    def transform(msg):
        for tf in msg.transforms:
            edges.add((tf.header.frame_id, tf.child_frame_id))

    def serial(msg):
        data = bytes(msg.data)
        if len(data) < 10 or data[:3] != b"RM\x02":
            errors.append("malformed V2 frame")
            return
        if crc16(data[2:-2]) != int.from_bytes(data[-2:], "little"):
            errors.append("invalid V2 CRC")
        if data[3] == 1:
            frames.append(time.monotonic())
            if len(data) != 23 or struct.unpack("<fffB", data[8:21]) != (0.0, 0.0, 0.0, 0):
                errors.append("chassis frame is not zero-disabled")

    subscriptions = [
        node.create_subscription(Odometry, "/odometry/lio", odometry, 50),
        node.create_subscription(Odometry, "/odometry/fast_lio_raw", lambda _: raw.append(time.monotonic()), 50),
        node.create_subscription(JointState, "/joint_states", lambda _: joints.append(time.monotonic()), 50),
        node.create_subscription(TFMessage, "/tf", transform, 100),
        node.create_subscription(TFMessage, "/tf_static", transform,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(UInt8MultiArray, "/serial/v2/mock_tx", serial, 100),
    ]
    command = node.create_publisher(Twist, "/cmd_vel", 10)
    process = None
    result = {}
    try:
        # Default profile must exit without creating any runtime nodes.
        default = subprocess.run(
            ["ros2", "launch", "rm_demo_platform", "demo_bench.launch.py"],
            capture_output=True, text=True, timeout=15)
        if default.returncode or "zero nodes" not in default.stdout:
            raise RuntimeError("default inventory did not exit cleanly: " + default.stdout + default.stderr)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if set(node.get_node_names()) != {node.get_name()}:
            raise RuntimeError("domain is not empty after inventory")

        log_path = Path(os.environ.get("ROS_LOG_DIR", tempfile.gettempdir())) / "demo_bench_launch.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w") as log:
            process = subprocess.Popen([
                "ros2", "launch", "rm_demo_platform", "demo_bench.launch.py",
                "enable_test:=true", "gimbal_duration_sec:=8.0",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            start = time.monotonic()
            # Nonzero generic commands must not obtain V2 authority.
            nonzero = Twist()
            nonzero.linear.x, nonzero.linear.y, nonzero.angular.z = 0.1, -0.1, 0.2
            while time.monotonic() - start < 14.0:
                if process.poll() is not None:
                    raise RuntimeError("bench exited early: " + log_path.read_text())
                command.publish(nonzero)
                rclpy.spin_once(node, timeout_sec=0.02)

            now = time.monotonic()
            if len(samples) < 20 or len(joints) < 20 or len(frames) < 20:
                raise RuntimeError(f"missing positive chain: odom={len(samples)} joints={len(joints)} frames={len(frames)}")
            if now - samples[-1] < 1.0 or now - joints[-1] < 1.0:
                raise RuntimeError("gimbal dropout did not stop canonical/joint publication")
            if not raw or now - raw[-1] > 0.2 or now - frames[-1] > 0.2:
                raise RuntimeError("dropout was not isolated: raw or zero-disabled serial also stopped")
            expected_edges = {
                ("map", "odom"): "map_odom_stub",
                ("odom", "base_link"): "lio_adapter",
                ("base_link", "gimbal_yaw_link"): "robot_state_publisher",
                ("gimbal_yaw_link", "mid360_left_frame"): "robot_state_publisher",
                ("mid360_left_frame", "lio_imu_link"): "robot_state_publisher",
            }
            # Humble rclpy callbacks do not expose publisher GIDs per message.
            # Check the live publisher set and tree; edge responsibility also
            # follows the unchanged production nodes selected by this launch.
            for topic, expected_names in {
                "/tf": {"map_odom_stub", "lio_adapter", "robot_state_publisher"},
                "/tf_static": {"robot_state_publisher"},
            }.items():
                endpoints = node.get_publishers_info_by_topic(topic)
                if len(endpoints) != len(expected_names) or {e.node_name for e in endpoints} != expected_names:
                    errors.append(f"unexpected TF publishers on {topic}")
            if edges != set(expected_edges):
                errors.append("unexpected TF edge")
            if len(node.get_publishers_info_by_topic("/odometry/lio")) != 1:
                errors.append("canonical odometry publisher count")
            forbidden = ("livox", "fast_lio_multi_backend", "controller_server", "mission", "bt_navigator")
            if any(any(token in name for token in forbidden) for name in node.get_node_names()):
                errors.append("unexpected hardware/navigation node")
            if errors:
                raise RuntimeError("; ".join(sorted(set(errors))))
            result = {"decision": "SOFTWARE_BENCH_PASS", "canonical_samples": len(samples),
                      "joint_samples": len(joints), "zero_disabled_frames": len(frames),
                      "tf_edges": len(edges), "expected_tf_publishers": "pass",
                      "tilted_se3_and_lever_arm": "pass",
                      "source_dropout": "pass", "default_zero_nodes": "pass",
                      "hardware_acceptance": False}
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                code = process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise RuntimeError("bench shutdown timed out")
            if code != 0:
                raise RuntimeError(f"bench shutdown returned {code}")
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
