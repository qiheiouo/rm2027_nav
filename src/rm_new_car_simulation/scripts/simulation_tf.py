#!/usr/bin/env python3
"""Explicit simulation oracle. Never claims to be LIO or wheel odometry."""
import os
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rm_competition_interfaces.msg import LocalizationStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SimulationTF(Node):
    def __init__(self):
        super().__init__('new_car_simulation_truth_tf')
        self.publish_tf=self.declare_parameter('publish_truth_tf',True).value
        self.fixture=self.declare_parameter('publish_localization_fixture',False).value
        self.epoch=None;self.sequence=0;self.latest_odom=None
        if self.fixture:
            self.status_pub=self.create_publisher(LocalizationStatus,'/localization/status',1)
            self.create_timer(0.1,self.status)
            self.create_service(Trigger,'/simulation/reset_localization_session',self.reset_session)
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
        if self.publish_tf:self.static.sendTransform(t)
        self.pub=self.create_publisher(Odometry,'/odometry/simulation',20)
        self.sub=self.create_subscription(Odometry,'/simulation/ground_truth/odom',self.odom,20)
        self.get_logger().info(f'SIMULATION ORACLE: TF publishing={self.publish_tf}; full SE(3) physics truth. Not localization validation.')

    def odom(self,msg):
        self.latest_odom=msg
        msg.header.frame_id='odom';msg.child_frame_id='base_link';self.pub.publish(msg)
        t=TransformStamped();t.header=msg.header;t.child_frame_id='base_link'
        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=msg.pose.pose.position.z
        t.transform.rotation=msg.pose.pose.orientation
        if self.publish_tf:self.tf.sendTransform(t)

    def reset_session(self, request, response):
        self.epoch=self.get_clock().now().to_msg();self.sequence=0
        response.success=True;response.message='Simulation oracle fixture session reset'
        return response

    def status(self):
        if not self.get_parameter('publish_localization_fixture').value:return
        now=self.get_clock().now()
        if now.nanoseconds<=0 or self.latest_odom is None:return
        if self.epoch is None:self.epoch=now.to_msg()
        msg=LocalizationStatus();msg.header.stamp=now.to_msg();msg.header.frame_id='map'
        msg.started_at=self.epoch;msg.producer_id=f'simulation_truth_fixture:{os.getpid()}'
        self.sequence+=1;msg.sequence=self.sequence
        # Do not renew a valid lease if physics truth has stopped arriving.
        msg.valid=0.0 <= (now-rclpy.time.Time.from_msg(self.latest_odom.header.stamp)).nanoseconds*1e-9 < 0.5
        msg.valid_until=(now+Duration(seconds=0.5)).to_msg()
        msg.pose.header=msg.header;msg.pose.pose=self.latest_odom.pose
        self.status_pub.publish(msg)


def main():
    # Avoid raising KeyboardInterrupt inside an rclpy/pybind subscription take.
    # The handler only requests a stop; destroy the context after spin returns.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=SimulationTF()
    stopping=False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGTERM,request_stop)
    try:
        while not stopping:rclpy.spin_once(node,timeout_sec=0.1)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()
