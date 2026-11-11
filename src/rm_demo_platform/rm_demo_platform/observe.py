"""Bounded passive ROS observation plus optional explicit whitelist rosbag capture."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .phase1 import read


class StreamStats:
    def __init__(self):
        self.count = 0
        self.first = self.last = None
        self.max_gap_sec = 0.0
        self.stamps = {}
        self.frames = set()
        self.nonmonotonic_stamps = 0
        self.sequence_gaps = 0
        self.duplicate_sequences = 0
        self.backward_sequences = 0
        self.sequences = {}
        self.invalid = self.offline = self.incomplete = 0
        self.identities = {}
        self.events = []
        self.event_overflow = 0
        self.latest_counters = {}

    def update(self, msg, arrival):
        self.count += 1
        if self.first is None:
            self.first = arrival
        if self.last is not None:
            self.max_gap_sec = max(self.max_gap_sec, arrival - self.last)
        self.last = arrival
        # Provenance may multiplex cloud/IMU or gimbal/heading into one topic.
        stream = (getattr(msg, 'output_topic', ''), getattr(msg, 'kind', ''))
        header = getattr(msg, 'header', getattr(msg, 'output_header', None))
        if header is not None:
            stamp = header.stamp.sec * 10**9 + header.stamp.nanosec
            self.frames.add(header.frame_id)
            if stamp > 0:
                if stream in self.stamps and stamp <= self.stamps[stream]:
                    self.nonmonotonic_stamps += 1
                self.stamps[stream] = stamp
        identities = tuple(getattr(msg, key, None) for key in
            ('transport_instance','driver_instance','associated_boot_id','source_boot_id','heading_reset_counter','reset_counter'))
        for key, bits in [('sample_sequence',32), ('record_sequence',64), ('output_sequence',64)]:
            if not hasattr(msg, key):
                continue
            if key == 'sample_sequence' and (getattr(msg, 'has_frame', True) is False or getattr(msg, 'frame_type', None) == 3):
                continue
            if key == 'output_sequence' and getattr(msg, 'has_output', True) is False:
                continue
            # record_sequence is global per producer; output/sample sequences are per stream.
            grouping = () if key == 'record_sequence' else (getattr(msg, 'frame_type', None), stream[0])
            identity = (key, grouping, identities)
            current = getattr(msg, key)
            if identity in self.sequences:
                delta = (current - self.sequences[identity]) % (1 << bits)
                if delta == 0:
                    self.duplicate_sequences += 1
                elif delta < (1 << (bits - 1)):
                    self.sequence_gaps += delta - 1
                else:
                    self.backward_sequences += 1
            self.sequences[identity] = current
        for key, dest in [('valid','invalid'), ('online','offline'), ('complete','incomplete')]:
            if hasattr(msg, key) and not getattr(msg, key):
                setattr(self, dest, getattr(self, dest) + 1)
        for key in ('remote_boot_id','source_boot_id','associated_boot_id','reset_counter',
                    'heading_reset_counter','transport_instance','driver_instance','offset_revision'):
            if hasattr(msg, key):
                value = getattr(msg, key)
                identity = (stream, key)
                if self.identities.get(identity) != value:
                    event = dict(field=key, value=value, arrival_monotonic_sec=arrival, stream=list(stream))
                    if len(self.events) < 200:
                        self.events.append(event)
                    else:
                        self.event_overflow += 1
                    self.identities[identity] = value
        for key in ('valid_frame_count','crc_error_count','framing_error_count','unknown_message_count'):
            if hasattr(msg, key):
                self.latest_counters[key] = getattr(msg, key)

    def summary(self):
        span = 0 if self.first is None else self.last - self.first
        return dict(count=self.count, receive_rate_hz=(self.count - 1) / span if span > 0 else None,
                    max_receive_gap_sec=self.max_gap_sec, frames=sorted(self.frames),
                    nonmonotonic_stamps=self.nonmonotonic_stamps, observed_sequence_gaps=self.sequence_gaps,
                    duplicate_sequences=self.duplicate_sequences, backward_sequences=self.backward_sequences,
                    invalid=self.invalid, offline=self.offline, incomplete_provenance=self.incomplete,
                    events=self.events, omitted_events=self.event_overflow, latest_counters=self.latest_counters)


def observe(session, duration, record):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    import yaml

    session = Path(session).resolve()
    metadata = read(session / 'session.yaml')
    topics = read(session / 'topics.yaml')['required']
    result_path = session / 'observations.json'
    if result_path.exists():
        raise ValueError('observations already exist; prepare a fresh session for every run')
    bag = Path(metadata['bag_directory'])
    if record and (bag != session / 'bag' or bag.exists()):
        raise ValueError('bag must be the new session/bag directory')
    # Python SIGINT raises KeyboardInterrupt; keep ROS context alive for final inventory.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('demo_phase1_observer')
    stats = {topic: StreamStats() for topic in topics}
    subscriptions, unavailable = [], {}
    process = None
    command = ['ros2','bag','record','--output',str(bag),'--qos-profile-overrides-path',str(session / 'qos.yaml'), *topics]
    start = time.monotonic()
    report = dict(result='OBSERVATION_ONLY', hardware_go=False, bag_requested=record,
                  bag_command=command if record else None, started_utc=datetime.now(timezone.utc).isoformat(),
                  manual_checks_required=['units','physical_direction','gravity','reset causality','MCU stop time','TF edge owner'])
    try:
        for topic, type_name in topics.items():
            try:
                msg_type = get_message(type_name)
            except (ImportError, AttributeError, ModuleNotFoundError, ValueError) as error:
                unavailable[topic] = str(error)
                continue
            qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static' else DurabilityPolicy.VOLATILE)
            subscriptions.append(node.create_subscription(msg_type, topic,
                lambda msg, name=topic: stats[name].update(msg, time.monotonic()), qos))
        if record:
            with (session / 'recorder.log').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = start + duration
        while time.monotonic() < deadline:
            if process and process.poll() is not None:
                report['recorder_early_exit'] = process.returncode
                break
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        report['interrupted'] = True
    finally:
        if process:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                    report['recorder_forced_kill'] = True
            report['recorder_exit_code'] = process.returncode
            report['bag_metadata_present'] = (bag / 'metadata.yaml').is_file()
        inventory = {}
        for topic in topics:
            inventory[topic] = [dict(node=f'{ep.node_namespace}/{ep.node_name}', type=ep.topic_type,
                                    gid=list(ep.endpoint_gid), qos=str(ep.qos_profile))
                                for ep in node.get_publishers_info_by_topic(topic)]
        (session / 'topic_inventory.json').write_text(json.dumps(inventory, indent=2))
        report.update(duration_sec=time.monotonic()-start, topics={k:v.summary() for k,v in stats.items()},
                      missing_topics=[k for k,v in stats.items() if not v.count], unavailable_types=unavailable)
        metadata['software'].update(ros_distro=os.environ.get('ROS_DISTRO'), rmw=os.environ.get('RMW_IMPLEMENTATION', 'default (see inventory)'), domain_id=os.environ.get('ROS_DOMAIN_ID','0'))
        metadata['runtime']['topic_type_qos_publisher_inventory'] = str(session / 'topic_inventory.json')
        metadata['runtime']['launch_records'] = [json.loads(p.read_text()) for p in sorted(session.glob('launch_*.json'))]
        metadata['session_evidence']['valid_window_and_gaps'] = str(result_path)
        metadata['session_evidence']['completeness_checks_and_recorder_warnings'] = str(session / 'recorder.log') if record else 'NO_BAG_REQUESTED'
        # Never promote observation to test PASS or motion authority.
        (session / 'session.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True))
        result_path.write_text(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--duration-sec', type=float, required=True)
    parser.add_argument('--record', action='store_true', help='explicitly record whitelisted topics; otherwise observe only')
    args = parser.parse_args()
    if not 0 < args.duration_sec <= 86400:
        parser.error('duration must be finite, in (0, 86400] seconds')
    report = observe(args.session_dir, args.duration_sec, args.record)
    print(json.dumps({k:report[k] for k in ('result','hardware_go','missing_topics','unavailable_types')}))
    if report['missing_topics'] or report['unavailable_types'] or report.get('recorder_early_exit') is not None or report.get('recorder_forced_kill') or (args.record and (not report.get('bag_metadata_present') or report.get('recorder_exit_code') not in (0, -2, 130))):
        return 2
    return 0
