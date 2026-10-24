"""AUD-016 correctness regression, real Nav2 servers and ROS actions."""
"""Actual planner cancellation while original StaticLayer awaits first map."""
import json, subprocess, time, os, sys
import yaml
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathThroughPoses, FollowPath
from nav_msgs.msg import Path as NavPath
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
mode = sys.argv[1]
action_type = {'pose': ComputePathToPose, 'poses': ComputePathThroughPoses, 'controller': FollowPath}[mode]
server_package = 'nav2_controller' if mode == 'controller' else 'nav2_planner'
server_name = 'controller_server' if mode == 'controller' else 'planner_server'
E = Path(os.environ['EVIDENCE_OUTPUT'])
E.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = Node('audit_planner_cancel_observer')
tfpub = TransformBroadcaster(node)
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
mappub = node.create_publisher(OccupancyGrid, '/map', q)
client = ActionClient(node, action_type, {'pose': '/compute_path_to_pose', 'poses': '/compute_path_through_poses', 'controller': '/follow_path'}[mode])
events, velocities = [], []
node.create_subscription(String, '/fixture/events', lambda m: events.append(m.data), 10)
node.create_subscription(Twist, '/fixture/cmd_vel', lambda m: velocities.append(m.linear.x), 10)
map_on = False
process = None
log = None
r = {'mode': mode, 'scope': 'Real Nav2 action/costmap; synthetic TF/map or explicitly gated test layer/controller; no hardware', 'events': {}}

def inputs():
    stamp = node.get_clock().now().to_msg()
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    t.transform.rotation.w = 1.0
    tfpub.sendTransform(t)
    if map_on:
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.info.resolution = 0.05
        m.info.width = 120
        m.info.height = 120
        m.info.origin.position.x = -3.0
        m.info.origin.position.y = -3.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * 14400
        mappub.publish(m)

def spin(sec, pred=None):
    end = time.monotonic() + sec
    last = 0.0
    while time.monotonic() < end:
        if time.monotonic() - last > 0.04:
            inputs()
            last = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.003)
        assert process is None or process.poll() is None, 'unexpected planner exit'
        if pred and pred():
            return True
    return pred is None or pred()

def goal():
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 0.5
    pose.pose.orientation.w = 1.0
    g = action_type.Goal()
    if mode == 'pose':
        g.goal = pose
        g.use_start = False
        g.planner_id = 'GridBased'
    elif mode == 'poses':
        g.goals = [pose]
        g.use_start = False
        g.planner_id = 'GridBased'
    else:
        g.path = NavPath()
        g.path.header = pose.header
        g.path.poses = [pose]
        g.controller_id = 'Fixture'
    return g

try:
    r['versions'] = subprocess.check_output(['dpkg-query', '-W', 'ros-humble-nav2-planner', 'ros-humble-nav2-navfn-planner', 'ros-humble-nav2-costmap-2d'], text=True).splitlines()
    log = (E / 'planner.log').open('w')
    config = Path(__file__).resolve().parents[2] / 'src/rm_nav_config/config/nav2_phase2f_deployment.yaml'
    if mode == 'controller':
        params = {server_name: {'ros__parameters': {
          'controller_plugins': ['Fixture'], 'Fixture.plugin': 'fixture::Controller',
          'controller_frequency': 20., 'odom_topic': '/fixture/odom'}},
          'local_costmap': {'local_costmap': {'ros__parameters': {
          'global_frame': 'map', 'robot_base_frame': 'base_link', 'resolution': 0.05,
          'width': 6, 'height': 6, 'rolling_window': True, 'update_frequency': 20.,
          'plugins': ['gate'], 'gate.plugin': 'fixture::Layer'}}}}
        config = E / 'controller.yaml'
        config.write_text(yaml.safe_dump(params))
    process = subprocess.Popen([get_package_prefix(server_package) + '/lib/' + server_package + '/' + server_name,
      '--ros-args', '--params-file', str(config), '-r', 'cmd_vel:=/fixture/cmd_vel'], stdout=log, stderr=subprocess.STDOUT)
    c = node.create_client(ChangeState, '/' + server_name + '/change_state')
    assert spin(5, lambda: c.service_is_ready())
    for tr in [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]:
        req = ChangeState.Request()
        req.transition.id = tr
        f = c.call_async(req)
        assert spin(8, lambda: f.done()) and f.result().success
    assert spin(3, lambda: client.server_is_ready())
    send = client.send_goal_async(goal())
    assert spin(3, lambda: send.done())
    h = send.result()
    assert h.accepted
    f = h.get_result_async()
    if mode == 'controller':
        assert spin(3, lambda: 'set_plan' in events)
    else:
        spin(0.1)  # bounded observation of the initial-map wait, not a race workaround
    assert not f.done()
    r['events']['initial_map_absent'] = {'goal_accepted': True, 'result_done': False}
    at = time.monotonic()
    cancel = h.cancel_goal_async()
    assert spin(2, lambda: cancel.done())
    ack = cancel.result()
    r['events']['cancel_ack'] = {'return_code': ack.return_code, 'goals_canceling': len(ack.goals_canceling), 'delay_sec': time.monotonic() - at}
    assert ack.return_code == 0 and ack.goals_canceling
    # Terminal cancellation must arrive while map/currentness remains withheld.
    assert spin(2, lambda: f.done()), 'cancel remained blocked on costmap'
    res = f.result()
    assert res.status == 5, f'expected CANCELED, got {res.status}'
    if mode == 'controller':
        assert 'compute' not in events and not any(velocities), (events, velocities)
    else:
        assert not res.result.path.poses
    r['events']['canceled_without_map'] = {'status': res.status, 'seconds': time.monotonic() - at,
      'compute_calls': events.count('compute'), 'nonzero_commands': sum(v != 0 for v in velocities)}
    map_on = True
    if mode == 'controller':
        gate = node.create_client(SetBool, '/fixture/current')
        assert spin(3, lambda: gate.service_is_ready())
        req = SetBool.Request(); req.data = True
        future = gate.call_async(req)
        assert spin(3, lambda: future.done()) and future.result().success
    send = client.send_goal_async(goal())
    assert spin(3, lambda: send.done())
    healthy = send.result()
    assert healthy.accepted
    hf = healthy.get_result_async()
    if mode == 'controller':
        assert spin(3, lambda: 'compute' in events)
        hc = healthy.cancel_goal_async()
        assert spin(2, lambda: hc.done())
        assert spin(2, lambda: hf.done()) and hf.result().status == 5
        r['events']['healthy_control'] = {'compute_calls': events.count('compute'), 'status': hf.result().status}
    else:
        assert spin(3, lambda: hf.done())
        assert hf.result().status == 4 and hf.result().result.path.poses
        r['events']['healthy_control'] = {'status': hf.result().status, 'poses': len(hf.result().result.path.poses)}
    r['verdict'] = 'PASS: canceled before map/currentness restore; fresh request works after restore'

finally:
    map_on = True
    if process is not None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        r['planner_exit'] = process.returncode
    if log:
        log.close()
    (E / 'result.json').write_text(json.dumps(r, indent=2) + '\n')
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()
assert r['planner_exit'] == 0, 'planner required forced or abnormal cleanup'
print(json.dumps(r, indent=2))
