#!/usr/bin/env python3
"""Private raw-input aliases and source-time sensor -> base status adapter.

No TF or commands are published here. Legacy FAST-LIO has no reset/session
protocol: observed discontinuities require restarting the complete candidate.
"""
import copy
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from localization_interfaces.msg import FrontendFrame, FrontendSession
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rm_competition_interfaces.msg import LocalizationStatus
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from geometry import matrix, quaternion, body_pose
from diagnostic_health import parse_state
from session_window import SessionWindow


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class CandidateAdapter(Node):
    def __init__(self):
        super().__init__('zlocalization_adapter')
        self.frontend = self.declare_parameter('frontend', 'legacy').value
        if self.frontend not in ('legacy', 'zlio'):
            raise ValueError('frontend must be legacy or zlio')
        self.native_identity = None
        self.native_sequence = 0
        self.sensor = self.declare_parameter('sensor_frame', 'lio_imu_link').value
        self.raw_frame = self.declare_parameter('raw_frame', 'odom').value
        self.raw_child = self.declare_parameter('raw_child', 'body').value
        self.lease = self.declare_parameter('source_timeout_sec', .5).value
        self.started = self.get_clock().now()
        self.identity = str(uuid.uuid4())
        self.window = SessionWindow(self.started.nanoseconds)
        self.sequence = 0
        self.blocked = False
        self.gids = {}
        self.last_raw_ns = 0
        self.last_pose = PoseWithCovarianceStamped()
        self.last_pose_received = 0.
        self.trusted = False
        self.states = {}
        self.poses = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.odom_pub = self.create_publisher(Odometry, '/zlocalization/fastlio_odom', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/zlocalization/cloud_registered', 10)
        self.native_raw_pub = self.create_publisher(Odometry, '/zlocalization/zlio_raw', 50)
        self.status_pub = self.create_publisher(
            LocalizationStatus, '/localization/zlocalization_backend_valid/status', 1)
        self.subscriptions_owned = [
            self.create_subscription(PoseWithCovarianceStamped, '/zlocalization/sensor_pose', self.pose, 20),
            self.create_subscription(String, '/zlocalization/state', self.state,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        if self.frontend == 'legacy':
            self.subscriptions_owned.extend([
                self.create_subscription(Odometry, '/odometry/fast_lio_raw', self.raw, qos_profile_sensor_data),
                self.create_subscription(PointCloud2, '/lio/cloud_registered', self.cloud, qos_profile_sensor_data),
            ])
        else:
            self.subscriptions_owned.extend([
                self.create_subscription(FrontendFrame, '/zlocalization/frontend/frame', self.native_frame, 50),
                self.create_subscription(FrontendSession, '/zlocalization/frontend/session', self.native_session,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            ])
        self.reset_service = self.create_service(Trigger, '/zlocalization/invalidate', self.invalidate)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def revoke(self):
        self.window.revoke()
        self.trusted = False
        self.states.clear()
        self.poses.clear()
        self.last_pose = PoseWithCovarianceStamped()

    def check_clock(self):
        if not self.window.observe_clock(self.get_clock().now().nanoseconds):
            self.block('ROS time moved backwards')
            return False
        return not self.blocked

    def block(self, reason):
        if not self.blocked:
            frontend_name = 'ZLIO' if self.frontend == 'zlio' else 'FAST-LIO'
            self.get_logger().error(reason + '; restart ' + frontend_name + ', canonical adapter and candidate together')
        self.blocked = True
        self.revoke()
        self.publish_status()

    def observe_publisher(self, name, topic):
        if not self.check_clock():
            return False
        # This Humble rclpy exposes endpoint GIDs, not per-callback MessageInfo.
        publishers = self.get_publishers_info_by_topic(topic)
        if len(publishers) > 1:
            self.block(name + ' has multiple publishers')
        if len(publishers) != 1:
            return False
        gid = bytes(publishers[0].endpoint_gid)
        previous = self.gids.setdefault(name, gid)
        if previous != gid:
            self.block(name + ' publisher changed')
        return not self.blocked

    def raw(self, message):
        if not self.observe_publisher('FAST-LIO', '/odometry/fast_lio_raw'):
            return
        ns = stamp_ns(message.header.stamp)
        if ns < self.last_raw_ns:
            self.block('FAST-LIO source time moved backwards')
            return
        if ns == self.last_raw_ns:
            return
        if message.header.frame_id != self.raw_frame or message.child_frame_id != self.raw_child:
            self.block('FAST-LIO raw frame changed or differs from candidate contract')
            return
        self.last_raw_ns = ns
        output = copy.deepcopy(message)
        # Same numeric sensor-initial basis, now explicitly private to this backend.
        output.header.frame_id = 'fast_lio_odom'
        output.child_frame_id = self.sensor
        # Current bundle publishes an older P with incompatible rotation semantics.
        # Zero means MISSING to upstream, which uses its configured noise model.
        output.pose.covariance = [0.] * 36
        self.odom_pub.publish(output)

    def native_session(self, message):
        if not self.observe_publisher('ZLIO session', '/zlocalization/frontend/session'):
            return
        identity = (message.instance_id, message.epoch)
        if message.failed or message.reset_required or message.epoch != 0:
            self.block('ZLIO failed or requested a coordinated reset')
        elif self.native_identity is not None and identity != self.native_identity:
            self.block('ZLIO instance/epoch changed; canonical startup reference must restart')
        elif message.instance_id:
            self.native_identity = identity

    def native_frame(self, message):
        if self.blocked or self.native_identity is None:
            return
        if not self.observe_publisher('ZLIO frames', '/zlocalization/frontend/frame'):
            return
        if (message.instance_id, message.epoch) != self.native_identity:
            self.block('ZLIO observation crossed the canonical session boundary')
            return
        odom = message.odometry
        ns = stamp_ns(odom.header.stamp)
        if (message.sequence <= self.native_sequence or ns <= self.last_raw_ns):
            self.block('ZLIO observation sequence or source time did not increase')
            return
        if (odom.header.frame_id != self.raw_frame or odom.child_frame_id != self.sensor or
                message.cloud.header.frame_id != self.sensor or
                message.cloud.header.stamp != odom.header.stamp):
            self.block('ZLIO atomic frame geometry or timestamp mismatch')
            return
        self.native_sequence = message.sequence
        self.last_raw_ns = ns
        # Preserve model covariance on the original atomic/native outputs.
        # The existing canonical adapter does not propagate pose covariance
        # through its startup basis and moving lever arm. Mark that separate
        # canonical input covariance missing rather than relabeling it.
        output = copy.deepcopy(odom)
        output.pose.covariance = [0.] * 36
        # IMU-frame twist is not chassis twist; lio_adapter derives the latter.
        self.native_raw_pub.publish(output)

    def cloud(self, message):
        if self.blocked:
            return
        if not self.observe_publisher('FAST-LIO cloud', '/lio/cloud_registered'):
            return
        if message.header.frame_id != self.raw_frame:
            self.block('registered cloud is not in the raw odometry basis')
            return
        output = copy.deepcopy(message)
        output.header.frame_id = 'fast_lio_odom'
        self.cloud_pub.publish(output)

    def state(self, message):
        if not self.observe_publisher('Z-Localization', '/zlocalization/state'):
            return
        try:
            fields = parse_state(message.data)
            epoch, ns = int(fields['epoch']), int(fields['stamp_ns'])
            generation, revision = int(fields['generation']), int(fields['pose_revision'])
        except (KeyError, ValueError):
            self.block('malformed backend state')
            return
        if epoch != 0:
            self.block('backend epoch changed without a canonical reset transaction')
            return
        previous_version = self.window.version
        if not self.window.observe_state(generation, revision, ns):
            return
        if self.window.version != previous_version:
            self.states.clear()
        self.trusted = (fields['pose_trusted'] == 'true' and fields['tf_frozen'] == 'false')
        if not self.trusted:
            self.revoke()
            self.publish_status()
            return
        if ns > 0 and fields['tf_published'] == 'true' and self.window.fresh(ns):
            self.states[ns] = fields
            self.drain()

    def pose(self, message):
        if not self.observe_publisher('Z-Localization pose', '/zlocalization/sensor_pose'):
            return
        if not self.blocked and message.header.frame_id == 'map':
            ns = stamp_ns(message.header.stamp)
            self.window.observe_source(ns)
            if not self.window.fresh(ns):
                return
            self.poses[ns] = message
            self.drain()

    def drain(self):
        if not self.check_clock():
            return
        if (stamp_ns(self.last_pose.header.stamp) > 0 and
                time.monotonic() - self.last_pose_received >= self.lease):
            self.revoke()
        now_ns = self.get_clock().now().nanoseconds
        for cache in (self.states, self.poses):
            for ns in list(cache):
                if ns <= now_ns - int(self.lease * 1.e9) or not self.window.fresh(ns):
                    del cache[ns]
            while len(cache) > 50:
                del cache[min(cache)]
        if self.blocked or not self.trusted:
            return
        for ns in sorted(self.states.keys() & self.poses.keys()):
            if (not self.window.fresh(ns) or ns <= stamp_ns(self.last_pose.header.stamp) or
                    ns > now_ns + 100000000):
                continue
            message = self.poses[ns]
            try:
                tf = self.tf.lookup_transform('base_link', self.sensor, Time.from_msg(message.header.stamp))
                t, q = tf.transform.translation, tf.transform.rotation
                base_from_sensor = matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                result, covariance = body_pose(pose_matrix(message.pose.pose), base_from_sensor,
                    np.array(message.pose.covariance).reshape(6, 6))
                if not np.isfinite(covariance).all():
                    raise ValueError('nonfinite backend covariance')
            except (TransformException, ValueError) as error:
                self.get_logger().debug(str(error))
                continue
            output = copy.deepcopy(message)
            p, q = output.pose.pose.position, output.pose.pose.orientation
            p.x, p.y, p.z = map(float, result[:3, 3])
            q.x, q.y, q.z, q.w = map(float, quaternion(result[:3, :3]))
            output.pose.covariance = covariance.reshape(-1).tolist()
            self.last_pose = output
            self.last_pose_received = time.monotonic()
        self.publish_status()

    def publish_status(self):
        now = self.get_clock().now()
        if self.started.nanoseconds == 0 and now.nanoseconds > 0:
            self.started = now
        ns = stamp_ns(self.last_pose.header.stamp)
        deadline = ns + int(self.lease * 1.e9)
        valid = (not self.blocked and self.trusted and ns > 0 and
            (self.frontend == 'legacy' or self.native_identity is not None) and
            ns <= now.nanoseconds + 100000000 and now.nanoseconds < deadline and
            time.monotonic() - self.last_pose_received < self.lease)
        self.sequence += 1
        status = LocalizationStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = 'map'
        status.started_at = self.started.to_msg()
        status.producer_id = self.identity
        status.sequence = self.sequence
        status.valid = valid
        status.valid_until = Time(nanoseconds=max(0, deadline)).to_msg()
        status.pose = self.last_pose
        self.status_pub.publish(status)

    def tick(self):
        self.drain()
        self.publish_status()

    def invalidate(self, request, response):
        self.block('candidate explicitly invalidated')
        response.success = True
        response.message = 'Candidate paused. No frontend reset performed; restart the complete session.'
        return response


def main():
    rclpy.init()
    node = CandidateAdapter()
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
