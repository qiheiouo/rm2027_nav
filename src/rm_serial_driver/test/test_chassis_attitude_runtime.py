"""No-hardware lifecycle test for the V2 chassis attitude host adapter."""
import os
from pathlib import Path
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rm_competition_interfaces.msg import ChassisAttitudeStatus, V2TimestampProvenance
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray


def frame(kind, sequence, payload):
    body = struct.pack("<BBHH", 2, kind, len(payload), sequence) + payload
    crc = 0xFFFF
    for byte in body:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return b"RM" + body + struct.pack("<H", crc)


def heartbeat(sequence, uptime, boot):
    return frame(0x03, sequence, struct.pack("<IIIB", uptime, boot, 1 << 8, 1))


def attitude(frame_sequence, sample_sequence, tick, boot, reset, flags=0x0F):
    payload = struct.pack(
        "<ffffffffffIIIHB",
        0.0, 0.0, 0.0, 1.0,
        0.125, -0.25, 0.5,
        0.0, 0.0, 9.80665,
        sample_sequence, tick, boot, reset, flags,
    )
    return frame(0x88, frame_sequence, payload)


def test_chassis_attitude_lifecycle_and_fail_closed_output():
    os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 20)
    executable = (
        Path(get_package_prefix("rm_serial_driver"))
        / "lib/rm_serial_driver/competition_v2_transport_node"
    )
    process = subprocess.Popen(
        [
            str(executable), "--ros-args",
            "-p", "dry_run:=true",
            "-p", "connection_timeout_sec:=60.0",
            "-p", "chassis_attitude_timeout_sec:=0.35",
            "-p", "health_rate_hz:=20.0",
            "-p", "enable_timestamp_provenance:=true",
            "-p", "provenance_session_id:=attitude-lifecycle-fixture",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    rclpy.init()
    node = rclpy.create_node("chassis_attitude_runtime_probe")
    statuses, imus, traces = [], [], []
    latched = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(ChassisAttitudeStatus, "/chassis/attitude_status", statuses.append, latched)
    node.create_subscription(Imu, "/chassis/imu", imus.append, qos_profile_sensor_data)
    node.create_subscription(
        V2TimestampProvenance,
        "/serial/v2/timestamp_provenance",
        traces.append,
        QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    rx = node.create_publisher(UInt8MultiArray, "/serial/v2/mock_rx", 10)

    def wait(predicate, seconds=5.0):
        deadline = time.monotonic() + seconds
        while not predicate():
            assert process.poll() is None, "transport exited early"
            assert time.monotonic() < deadline, "fixture timeout"
            rclpy.spin_once(node, timeout_sec=0.02)

    def send(data):
        message = UInt8MultiArray()
        message.data = list(data)
        rx.publish(message)

    try:
        wait(lambda: rx.get_subscription_count() == 1)
        send(heartbeat(10, 1000, 42) + attitude(200, 1, 1010, 42, 0))
        wait(lambda: len(imus) == 1 and statuses[-1].current_valid)
        first_epoch = statuses[-1].lifecycle_epoch
        assert first_epoch == 1
        assert imus[-1].header.frame_id == "chassis_imu_link"
        assert abs(imus[-1].linear_acceleration.z - 9.80665) < 1e-5
        assert all(value == 0.0 for value in imus[-1].orientation_covariance)

        send(attitude(201, 1, 1011, 42, 0))
        wait(lambda: any(s.reason == "sample_sequence_duplicate_or_replay" for s in statuses))
        assert len(imus) == 1

        # Packet loss is allowed: a strictly newer sample need not be contiguous.
        send(attitude(202, 10, 1020, 42, 0))
        wait(lambda: len(imus) == 2)

        send(attitude(203, 11, 1000, 42, 0))
        wait(lambda: any(s.reason == "mcu_time_regression" for s in statuses))
        assert len(imus) == 2

        send(attitude(204, 12, 1030, 42, 0, flags=0x0E))
        wait(lambda: any(s.reason == "source_component_invalid" for s in statuses))
        assert len(imus) == 2

        send(attitude(205, 13, 1040, 42, 0, flags=0x07))
        wait(lambda: any(s.reason == "source_offline" for s in statuses))
        assert len(imus) == 2

        send(attitude(3, 0xFFFFFFFF, 0xFFFFFFFE, 42, 1))
        wait(lambda: len(imus) == 3 and statuses[-1].current_valid and
             statuses[-1].reset_counter == 1)
        assert statuses[-1].current_valid
        assert statuses[-1].lifecycle_epoch == first_epoch + 1

        # Both sample identity and uint32 MCU time may wrap within an epoch.
        send(attitude(4, 0, 0, 42, 1))
        wait(lambda: len(imus) == 4)

        send(attitude(206, 3, 1050, 42, 0))
        wait(lambda: any(s.reason == "reset_counter_replay" for s in statuses))
        assert len(imus) == 4

        send(heartbeat(11, 5, 43))
        wait(lambda: any(s.reason == "lower-controller restart" for s in statuses))
        send(attitude(207, 4, 1060, 42, 1))
        wait(lambda: any(s.reason == "source_boot_id_mismatch" for s in statuses))
        assert len(imus) == 4

        send(attitude(1, 0, 10, 43, 0))
        wait(lambda: len(imus) == 5 and statuses[-1].current_valid and
             statuses[-1].source_boot_id == 43)
        assert statuses[-1].source_boot_id == 43
        assert statuses[-1].lifecycle_epoch == first_epoch + 2

        # A delayed heartbeat from a retired boot must not re-authorize its samples.
        send(heartbeat(12, 2000, 42) + attitude(2, 1, 11, 43, 0))
        wait(lambda: len(imus) == 6 and statuses[-1].current_valid and
             statuses[-1].source_boot_id == 43)
        assert statuses[-1].source_boot_id == 43
        wait(lambda: any(t.invalid_reason == "retired_boot_id_replay" for t in traces))

        accepted_count = len(imus)
        wait(lambda: statuses and statuses[-1].stale, 3.0)
        assert not statuses[-1].current_valid
        assert statuses[-1].reason == "stale"
        assert len(imus) == accepted_count
        attitude_traces = [t for t in traces if t.kind == t.ATTITUDE and t.output_valid]
        assert len(attitude_traces) >= 6
        assert all(t.raw_orientation_valid and t.raw_angular_velocity_valid and
                   t.raw_linear_acceleration_valid for t in attitude_traces)
        assert all(t.output_publish_ros_ns > 0 and t.output_publish_steady_ns > 0
                   for t in attitude_traces)
    finally:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=3.0)
        print(output)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
