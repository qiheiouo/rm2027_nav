"""Exercise the actual monitor callback with source-stamped odometry."""
import time
import rclpy
from nav_msgs.msg import Odometry
from rm_system_monitor.readiness_monitor import ReadinessMonitor


def test_monitor_odom_callback_does_not_refresh_old_or_future_source():
    rclpy.init()
    monitor = ReadinessMonitor()
    try:
        def fresh():
            return monitor._odom_lease.fresh(monitor.get_clock().now().nanoseconds, time.monotonic())
        message = Odometry()
        monitor._on_odom(message)
        assert not fresh()
        message.header.stamp = monitor.get_clock().now().to_msg()
        monitor._on_odom(message)
        assert fresh()
        monitor._on_odom(message)
        assert not fresh()
        message.header.stamp.sec -= 10
        monitor._on_odom(message)
        assert not fresh()
        message.header.stamp.sec += 20
        monitor._on_odom(message)
        assert not fresh()
        message.header.stamp = monitor.get_clock().now().to_msg()
        monitor._on_odom(message)
        assert fresh()
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
