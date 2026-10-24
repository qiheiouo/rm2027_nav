#!/usr/bin/env python3
"""Exercise real ROS byte encoding and source expiry with a paused bag clock."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('paused_clock_diagnostics_fixture')
    pub = node.create_publisher(String, '/diagnostic_fixture/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock = node.create_publisher(Clock, '/clock', 10)
    received = []
    node.create_subscription(DiagnosticArray, '/diagnostic_fixture/diagnostics', received.append, 20)
    log = (out / 'launch.log').open('w')
    executable = str(Path(get_package_prefix('rm_zlocalization_candidate')) /
                     'lib/rm_zlocalization_candidate/runtime_diagnostics.py')
    process = subprocess.Popen([executable,
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'state_topic:=/diagnostic_fixture/state',
        '-p', 'diagnostics_topic:=/diagnostic_fixture/diagnostics', '-p', "adapter_status_topic:=''"],
        stdout=log, stderr=subprocess.STDOUT)
    text = ('status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;'
        'epoch=0;generation=1;pose_revision=1;stamp_ns=1000000000;'
        'estimator_failure_count=0;last_estimator_failure=;reason=fixture')
    message = Clock()
    message.clock.sec = 1

    def drive(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, 'diagnostic process exited'
            clock.publish(message)
            pub.publish(String(data=text))
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        drive(2)
        assert any(m.status[0].level == b'\x00' for m in received), 'NORMAL was never emitted'
        assert received[-1].status[0].level == b'\x03', 'heartbeats renewed unchanged source evidence'
        assert received[-1].header.stamp.sec == 1, 'clock unexpectedly advanced'
        # Malformed external text must become an error diagnostic, not crash.
        text = 'epoch=broken'
        drive(.4)
        assert received[-1].status[0].level == b'\x02'
        result = dict(paused_clock_expiry=True, malformed_state_error=True,
                      diagnostic_only=True, messages=len(received))
        (out / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0


if __name__ == '__main__':
    main()
