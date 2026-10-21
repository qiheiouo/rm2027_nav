#!/usr/bin/env python3
"""Shared simulated input + explicit transport faults; no truth subscriptions."""
import copy
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rm_competition_interfaces.msg import ChassisHeadingState
from common import stamp
class Ingress(Node):
    def __init__(self):
        super().__init__('shadow_input_ingress');self.fault={};self.sequence=0;self.history=[]
        self.raw=self.create_publisher(Odometry,'/shadow/input/raw',100)
        self.heading=self.create_publisher(ChassisHeadingState,'/shadow/input/heading',100)
        self.create_subscription(Odometry,'/shadow/observation_packet',self.packet,100)
        self.create_subscription(String,'/shadow/injection',self.inject,10)
    def inject(self,m):self.fault=json.loads(m.data)
    def packet(self,p):
        self.sequence+=1;now=stamp(p.header.stamp)
        self.history.append((now,p.twist.twist.angular.z,p.twist.twist.angular.y))
        self.history=self.history[-100:]
        f=self.fault;h=ChassisHeadingState();h.header=copy.deepcopy(p.header);h.header.frame_id='sim_gravity_heading'
        h.yaw_rad=p.twist.twist.angular.z;h.yaw_rate_rad_s=p.twist.twist.angular.y
        delay=int(f.get('delay_ns',0))
        if delay:
            past=min(self.history,key=lambda v:abs(v[0]-(now-delay)))
            h.yaw_rad,h.yaw_rate_rad_s=past[1:];ht=now-delay
            h.header.stamp.sec,h.header.stamp.nanosec=divmod(ht,1_000_000_000)
            if f.get('lie_stamp'):h.header.stamp=copy.deepcopy(p.header.stamp)
        offset=int(f.get('stamp_offset_ns',0))
        if offset:
            ht=stamp(h.header.stamp)+offset
            h.header.stamp.sec,h.header.stamp.nanosec=divmod(ht,1_000_000_000)
        if f.get('zero_rate'):h.yaw_rate_rad_s=0.
        h.source_boot_id=1;h.reset_counter=int(f.get('heading_epoch',0));h.sample_sequence=self.sequence
        h.valid=not f.get('invalid',False);h.online=not f.get('offline',False)
        raw=copy.deepcopy(p);raw.twist.twist.angular.z=0.;raw.twist.twist.angular.y=0.
        if f.get('raw_epoch'):raw.header.frame_id='sensor_epoch_2'
        if f.get('frame'):raw.child_frame_id='wrong_sensor'
        if not f.get('missing'):self.heading.publish(h)
        self.raw.publish(raw)
def main():
    rclpy.init();n=Ingress()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
