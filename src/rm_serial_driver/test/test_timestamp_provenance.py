"""Exercise the real V2 decoder/handlers through existing dry-run RX only."""
import os
from pathlib import Path
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rm_competition_interfaces.msg import ChassisHeadingState, GimbalState, V2TimestampProvenance
from std_msgs.msg import UInt8MultiArray


def frame(kind, sequence, payload):
    body = struct.pack('<BBHH', 2, kind, len(payload), sequence) + payload
    crc = 0xffff
    for byte in body:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return b'RM' + body + struct.pack('<H', crc)


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def exercise(enabled):
    executable = Path(get_package_prefix('rm_serial_driver')) / 'lib/rm_serial_driver/competition_v2_transport_node'
    proc = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'dry_run:=true',
        '-p', 'use_sim_time:=true', '-p', 'connection_timeout_sec:=60.0',
        '-p', 'gimbal_timeout_sec:=60.0', '-p', 'chassis_heading_timeout_sec:=60.0',
        '-p', f'enable_timestamp_provenance:={str(enabled).lower()}',
        '-p', 'provenance_session_id:=controlled-fixture',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = rclpy.create_node('timestamp_provenance_probe')
    heading, gimbal, traces = [], [], []
    node.create_subscription(ChassisHeadingState, '/chassis/heading', heading.append, 50)
    node.create_subscription(GimbalState, '/gimbal/state', gimbal.append, 50)
    node.create_subscription(V2TimestampProvenance, '/serial/v2/timestamp_provenance', traces.append,
                             QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT))
    rx = node.create_publisher(UInt8MultiArray, '/serial/v2/mock_rx', 10)
    clock = node.create_publisher(Clock, '/clock', 10)

    def wait(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while not predicate():
            assert proc.poll() is None, 'transport exited'
            assert time.monotonic() < deadline, 'fixture timeout'
            c = Clock(); c.clock.sec = 100; clock.publish(c)
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        wait(lambda: rx.get_subscription_count() == 1 and node.count_subscribers('/clock') >= 1)
        end = time.monotonic() + 0.4
        wait(lambda: time.monotonic() >= end)
        heading.clear(); gimbal.clear(); traces.clear()
        hb = lambda seq, up, boot: frame(3, seq, struct.pack('<IIIB', up, boot, 36, 1))
        h = lambda seq, sample, tick, reset: frame(0x85, seq, struct.pack('<ffIIHB', .25, .125, sample, tick, reset, 3))
        g = lambda seq, sample, tick, valid=3: frame(0x82, seq, struct.pack('<ffIIB', -.5, .0625, sample, tick, valid))
        data = b''.join([
            h(10, 1, 10, 0), g(11, 1, 11),
            hb(12, 1000, 42), h(13, 2, 1010, 0), g(14, 2, 1011),
            hb(15, 2000, 42), h(16, 3, 2020, 1), g(17, 3, 2021),
            hb(18, 5, 43), h(19, 1, 10, 0), g(20, 1, 11, 2),
            frame(0x85, 21, b''),  # Valid frame CRC, invalid payload: no semantic output.
        ])
        message = UInt8MultiArray(); message.data = list(data); rx.publish(message)
        wait(lambda: len([m for m in heading if m.sample_sequence]) >= 4 and
             len([m for m in gimbal if m.sample_sequence]) >= 4)
        if enabled:
            wait(lambda: any(t.has_frame and t.frame_sequence == 21 for t in traces))
            heartbeats = {t.offset_revision: t for t in traces if t.kind == t.HEARTBEAT}
            assert {r: t.frame_sequence for r, t in heartbeats.items()} == {1: 12, 2: 15, 3: 18}
            assert [t.record_sequence for t in traces] == sorted(t.record_sequence for t in traces)
            for t in traces:
                assert t.session_id == 'controlled-fixture' and t.transport_instance
                assert not t.has_read_batch_completion  # Mock RX is not physical serial I/O.
                assert t.handler_ros_ns == 100 * 10**9
                assert t.handler_steady_ns > 0 and t.handler_system_ns > 0
                if t.has_output and t.payload_decoded:
                    outputs = heading if t.kind == t.HEADING else gimbal
                    matches = [m for m in outputs if m.sample_sequence == t.sample_sequence and
                               m.mcu_time_ms == t.mcu_time_ms and ns(m.header.stamp) == ns(t.output_header.stamp) and
                               m.valid == t.output_valid and m.online == t.output_online]
                    assert len(matches) == 1, 'ambiguous or absent output association'
                    if t.offset_available:
                        basis = heartbeats[t.offset_revision]
                        assert t.offset_heartbeat_frame_sequence == basis.frame_sequence
                        assert t.associated_boot_id == basis.associated_boot_id
                        assert t.offset_ns == basis.offset_ns
                        assert t.offset_ns == t.offset_heartbeat_ros_ns - t.offset_heartbeat_uptime_ms * 10**6
                        assert ns(t.output_header.stamp) == t.offset_ns + t.mcu_time_ms * 10**6
                    else:
                        assert not t.output_valid and ns(t.output_header.stamp) == 100 * 10**9
            reset = next(t for t in traces if t.has_frame and t.frame_sequence == 16)
            assert reset.heading_reset_counter == 1
            assert any(t.kind == t.INVALID_HEADING and 'restart' in t.invalid_reason for t in traces)
            invalid = next(t for t in traces if t.has_frame and t.frame_sequence == 20)
            assert invalid.associated_boot_id == 43 and invalid.invalid_reason == 'source_invalid'
            assert not next(t for t in traces if t.has_frame and t.frame_sequence == 21).payload_decoded
        else:
            assert not traces
            assert node.count_publishers('/serial/v2/timestamp_provenance') == 0
        # Ignore startup/timer markers; all four actual source samples must be
        # field-identical with diagnostics off/on, including exact headers.
        # CDR alignment padding is not part of ROS message semantics.
        return [[m for m in stream if m.sample_sequence] for stream in (heading, gimbal)]
    finally:
        proc.terminate()
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=3)
        node.destroy_node()


def test_output_offset_heartbeat_chain_and_unchanged_semantics():
    os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 30)
    rclpy.init()
    try:
        assert exercise(False) == exercise(True)
    finally:
        rclpy.shutdown()
