#!/usr/bin/env python3
"""Expose upstream failure/version/source age without publishing TF or commands."""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rm_competition_interfaces.msg import LocalizationStatus
from std_msgs.msg import Header, String

from diagnostic_health import parse_state, health


class RuntimeDiagnostics(Node):
    def __init__(self):
        super().__init__('zlocalization_diagnostics')
        state_topic = self.declare_parameter('state_topic', '/zlocalization/state').value
        topic = self.declare_parameter('diagnostics_topic', '/zlocalization/diagnostics').value
        adapter = self.declare_parameter('adapter_status_topic',
            '/localization/zlocalization_backend_valid/status').value
        self.timeout = self.declare_parameter('source_timeout_sec', .5).value
        if self.timeout <= 0:
            raise ValueError('source_timeout_sec must be positive')
        self.fields = None
        self.received = self.source_received = 0.0
        self.source_stamp = 0
        self.error = ''
        self.adapter = None
        self.adapter_received = 0.0
        self.publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.state_subscription = self.create_subscription(String, state_topic, self.state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.adapter_subscription = (self.create_subscription(LocalizationStatus, adapter,
            self.status, 10) if adapter else None)
        # Keep reporting input/process loss when a bag is paused at a fixed ROS time.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.2, self.tick, clock=self.steady_clock)

    def status(self, message):
        self.adapter = message
        self.adapter_received = time.monotonic()

    def state(self, message):
        try:
            self.fields = parse_state(message.data)
            stamp = int(self.fields['stamp_ns'])
            self.received = time.monotonic()
            if stamp > self.source_stamp:
                self.source_stamp = stamp
                self.source_received = self.received
            self.error = ''
        except (KeyError, ValueError) as error:
            self.error = str(error)

    def tick(self):
        now = self.get_clock().now()
        wall = time.monotonic()
        values = dict(self.fields or {})
        level, message = health(self.fields, now_ns=now.nanoseconds,
            receive_age=wall - self.received, source_receive_age=wall - self.source_received,
            timeout=self.timeout)
        if self.error:
            level, message = 2, 'malformed state: ' + self.error
        if self.adapter is not None:
            stamp = self.adapter.valid_until
            deadline = stamp.sec * 1_000_000_000 + stamp.nanosec
            accepted = (self.adapter.valid and wall - self.adapter_received < self.timeout
                        and now.nanoseconds < deadline)
            values['rm_adapter_valid'] = str(bool(accepted)).lower()
            if not accepted and level < 1:
                level, message = 1, 'RM adapter invalid or stale'
        values['authority'] = 'diagnostic_only'
        values['source_receive_age_sec'] = str(wall - self.source_received)
        # diagnostic_msgs uses ROS `byte` (bytes in Humble Python), not uint8.
        status = DiagnosticStatus(level=bytes([level]), name='zlocalization/runtime',
            hardware_id='software_candidate', message=message,
            values=[KeyValue(key=key, value=value) for key, value in sorted(values.items())])
        self.publisher.publish(DiagnosticArray(header=Header(stamp=now.to_msg()), status=[status]))


def main():
    rclpy.init()
    node = RuntimeDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
