#!/usr/bin/env python3
"""Compare representative sweep outputs with the unchanged V ROS executable.

Private domain + private topics; no bridge, TF broadcaster, LIO or cmd publisher.
odom == base is an explicit synthetic identity assumption, not a vehicle profile.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, Header
import yaml

from run_baseline import SOURCE_PIN, read_pcd, command


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=198)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    if command(['git', 'diff', SOURCE_PIN, '--', 'src'], cwd=root):
        raise SystemExit('Fixed baseline source has changed')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    # Keep ROS bookkeeping inside this isolated, writable research tree.
    os.environ['ROS_LOG_DIR'] = str(root/'build/l1_ros_logs')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = {r['case_id']: r for r in csv.DictReader((out/'results.csv').open())}
    manifest = json.loads((out/'manifest.json').read_text())
    cases = {c['id']: c for c in manifest['case_inputs']}
    cfg = yaml.safe_load((root/'src/rm_gicp_relocalization/config/gicp_3d.yaml').read_text())
    params = cfg['gicp_relocalization']['ros__parameters']
    executable = root/'build/l1_ros/rm_gicp_relocalization/gicp_relocalization_node'
    rclpy.init()
    node = rclpy.create_node('l1_native_verifier')
    evidence = {'source_pin': SOURCE_PIN, 'ros_domain_id': args.domain,
                'frames': {'map': 'l1_map', 'odom': 'l1_odom', 'base': 'l1_odom'},
                'scope': 'synthetic identity odom/base; GICP raw outputs only; no TF or bridge',
                'cases': []}
    def spin_until(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return True
        return False
    try:
        for case_id, frames in [('identity',1), ('duplicate_wrong_12m',3),
                                ('no_seed',3), ('wrong_far_xy',1), ('yaw_-3.1416',1)]:
            case = cases[case_id]
            prefix = '/l1/' + case_id.replace('-','minus').replace('.','p')
            local = dict(params, prior_pcd_file=str(root/case['target']), map_id=case_id,
                         map_frame='l1_map', odom_frame='l1_odom', base_frame='l1_odom',
                         initial_pose_topic=prefix+'/seed', input_cloud_topic=prefix+'/cloud',
                         raw_pose_topic=prefix+'/raw', score_topic=prefix+'/score',
                         valid_topic=prefix+'/valid')
            argv = [str(executable), '--ros-args', '-r', '__node:=l1_gicp_probe',
                    '-r', '/localization/gicp_map_id:='+prefix+'/map_id',
                    '-r', '/localization/reset_gicp:='+prefix+'/reset']
            for key,value in local.items():
                argv += ['-p', key+':='+str(value).lower() if isinstance(value,bool)
                         else key+':='+str(value)]
            poses, scores = [], []
            raw = node.create_subscription(PoseWithCovarianceStamped,prefix+'/raw',poses.append,10)
            score = node.create_subscription(Float64,prefix+'/score',lambda m:scores.append(m.data),10)
            seed_pub = node.create_publisher(PoseWithCovarianceStamped,prefix+'/seed',10)
            from sensor_msgs.msg import PointCloud2
            cloud_pub = node.create_publisher(PointCloud2,prefix+'/cloud',10)
            log_path = root/'build'/('l1_native_'+case_id+'.log')
            with log_path.open('w') as log:
                process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
                try:
                    ready = spin_until(lambda: seed_pub.get_subscription_count()>0 and
                                       cloud_pub.get_subscription_count()>0, 4)
                    if not ready:
                        raise RuntimeError('DDS input discovery failed: '+case_id+'; see '+str(log_path))
                    # Graph discovery can precede the best-effort data endpoint.
                    spin_until(lambda:False,.4)
                    if case['has_seed']:
                        seed = PoseWithCovarianceStamped()
                        seed.header.frame_id = 'l1_map'
                        seed.header.stamp = node.get_clock().now().to_msg()
                        seed.pose.pose.position.x,seed.pose.pose.position.y = map(float,case['seed'][:2])
                        seed.pose.pose.orientation.z = math.sin(case['seed'][2]/2)
                        seed.pose.pose.orientation.w = math.cos(case['seed'][2]/2)
                        seed_pub.publish(seed)
                        spin_until(lambda:False,.15)
                    points = read_pcd(root/case['source']).astype(np.float32)
                    for i in range(frames):
                        before = len(poses)
                        header = Header(frame_id='l1_odom',stamp=node.get_clock().now().to_msg())
                        cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                        if case_id in ('no_seed','wrong_far_xy'):
                            spin_until(lambda:False,.8)
                        elif not spin_until(lambda:len(poses)>before,2):
                            raise RuntimeError('Missing raw pose: '+case_id)
                    spin_until(lambda:False,.1)
                    output = []
                    for message in poses:
                        p = message.pose.pose
                        yaw = math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                                         1-2*(p.orientation.y**2+p.orientation.z**2))
                        output.append({'x':p.position.x,'y':p.position.y,'z':p.position.z,'yaw':yaw})
                    reference = rows[case_id]
                    if case_id in ('no_seed','wrong_far_xy'):
                        assert not poses, case_id
                    else:
                        assert len(poses)==frames, (case_id,len(poses))
                        first=output[0]
                        assert abs(first['x']-float(reference['T03']))<1e-4
                        assert abs(first['y']-float(reference['T13']))<1e-4
                        assert abs(first['yaw']-math.atan2(float(reference['T10']),float(reference['T00'])))<1e-4
                        assert abs(scores[0]-float(reference['fitness']))<1e-6
                    if case_id=='no_seed':
                        assert not scores
                    evidence['cases'].append(dict(case_id=case_id,cloud_frames=frames,
                                                  raw_pose_count=len(poses),scores=scores,poses=output))
                    print(case_id,'raw_pose_count=',len(poses),'scores=',scores,flush=True)
                finally:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill();process.wait()
            for obj in (raw,score): node.destroy_subscription(obj)
            for obj in (seed_pub,cloud_pub): node.destroy_publisher(obj)
    finally:
        node.destroy_node();rclpy.shutdown()
    evidence['crosscheck']='PASS: representative first-pose and fitness agree with offline probe'
    (out/'native_crosscheck.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    main()
