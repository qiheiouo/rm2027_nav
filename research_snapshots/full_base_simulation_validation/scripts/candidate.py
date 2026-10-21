#!/usr/bin/env python3
"""Heading-only full-base shadow node. No truth, joint, TF, or command imports."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rm_competition_interfaces.msg import ChassisHeadingState
from adapter import solve
from common import calibration,samples,startup,stamp,set_pose,REFERENCE
from input_contract import HeadingCache

class Candidate(Node):
    def __init__(self):
        super().__init__('full_base_shadow');self.pending={};self.start=None;self.last=0;self.latched=None
        self.cal=calibration(json.loads(Path(self.declare_parameter('manifest','').value).read_text()))
        self.rp=self.declare_parameter('startup_roll_pitch',[float('nan'),float('nan')]).value
        self.home=self.declare_parameter('startup_home_confirmed',False).value
        self.missing=self.declare_parameter('missing_startup',False).value
        self.parent=self.declare_parameter('expected_raw_parent','sensor_epoch_1').value
        self.headings=HeadingCache(self.declare_parameter('expected_heading_reference',REFERENCE).value)
        self.pub=self.create_publisher(Odometry,'/shadow/full_base/odom',100)
        self.status=self.create_publisher(String,'/shadow/full_base/status',100)
        self.create_subscription(Odometry,'/shadow/input/raw',self.raw,100)
        self.create_subscription(ChassisHeadingState,'/shadow/input/heading',self.heading,100)
        self.create_timer(.005,self.drain)

    def raw(self,m):
        t=stamp(m.header.stamp)
        if self.start is not None and m.header.frame_id!=self.start.raw.raw_parent:
            self.latched='raw epoch change requires explicit restart and re-home'
        if self.latched or self.headings.latched:
            self.emit(t,'INVALID_TIME',self.latched or self.headings.latched);return
        if m.header.frame_id!=self.parent or m.child_frame_id!=self.cal.sensor_frame:
            self.emit(t,'INVALID_INPUT','unexpected raw parent or sensor frame');return
        self.pending.setdefault(t,(m,time.monotonic()));self.drain()

    def heading(self,m):
        self.headings.add(m,time.monotonic());self.drain()

    def emit(self,t,status,reason,**extra):
        self.status.publish(String(data=json.dumps(dict(stamp_ns=t,status=status,reason=reason,**extra),allow_nan=False)))

    def drain(self):
        for t,(m,arrived) in sorted(list(self.pending.items())):
            if self.latched or self.headings.latched:
                del self.pending[t]
                self.emit(t,'INVALID_TIME',self.latched or self.headings.latched);continue
            h,problem=self.headings.match(t,arrived,time.monotonic())
            if h is None and problem is None:break
            del self.pending[t]
            if t<=self.last:
                self.latched='nonmonotonic source time requires explicit restart and re-home'
                self.emit(t,'INVALID_TIME',self.latched);continue
            self.last=t
            if problem:
                self.emit(t,*problem);continue
            try:
                # Startup is transactional: invalid frame/time/reference/pose
                # cannot poison the reference for subsequent valid input.
                reference=self.start or startup(m,h,self.cal,self.rp,self.missing,self.home)
                raw,heading=samples(m,h)
                result=solve(raw,heading,self.cal,reference) # deliberately NO joint
            except ValueError as e:
                self.emit(t,'INVALID_INPUT','invalid pose/startup: '+str(e));continue
            if self.start is None and result.T_odom_base is not None:
                self.start=reference;self.headings.bind(h)
            self.emit(t,result.status.value,result.reason,diagnostics=result.diagnostics)
            if result.T_odom_base is not None:
                out=Odometry();out.header=m.header;out.header.frame_id='shadow_full_odom';out.child_frame_id='shadow_full_base'
                set_pose(out.pose.pose,result.T_odom_base)
                # Pose-only candidate; covariance/twist are not a validated estimate.
                out.pose.covariance[0]=-1.;out.twist.covariance[0]=-1.
                self.pub.publish(out)

def main():
    rclpy.init();n=Candidate()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
