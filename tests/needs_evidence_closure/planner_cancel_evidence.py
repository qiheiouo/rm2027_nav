# Adapted from the historical AUD-016 actual-planner fixture.
"""Actual planner cancellation while original StaticLayer awaits first map."""
import json, subprocess, time, os
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
E = Path(os.environ['EVIDENCE_OUTPUT'])
E.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = Node('audit_planner_cancel_observer')
tfpub = TransformBroadcaster(node)
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
mappub = node.create_publisher(OccupancyGrid, '/map', q)
client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
map_on = False
process = None
log = None
r = {'scope': 'Actual Humble Nav2 planner/Navfn/globalcostmap + current unchanged phase2f config; known-defect characterization. Current synthetic TF, StaticLayer initially has no map, bounded map restoration. Direct planning action only, no mission/BT/controller/smoother/velocity/robot/MCU. No source/plugin/threshold patch.', 'events': {}}

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
    g = ComputePathToPose.Goal()
    g.goal = PoseStamped()
    g.goal.header.stamp = node.get_clock().now().to_msg()
    g.goal.header.frame_id = 'map'
    g.goal.pose.position.x = 0.5
    g.goal.pose.orientation.w = 1.0
    g.planner_id = 'GridBased'
    g.use_start = False
    return g
try:
    r['versions'] = subprocess.check_output(['dpkg-query', '-W', 'ros-humble-nav2-planner', 'ros-humble-nav2-navfn-planner', 'ros-humble-nav2-costmap-2d'], text=True).splitlines()
    log = (E / 'planner.log').open('w')
    process = subprocess.Popen([get_package_prefix('nav2_planner') + '/lib/nav2_planner/planner_server', '--ros-args', '--params-file', str(Path(__file__).resolve().parents[2] / 'src/rm_nav_config/config/nav2_phase2f_deployment.yaml')], stdout=log, stderr=subprocess.STDOUT)
    c = node.create_client(ChangeState, '/planner_server/change_state')
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
    spin(0.6)
    assert not f.done()
    r['events']['initial_map_absent'] = {'goal_accepted': True, 'result_done': False}
    at = time.monotonic()
    cancel = h.cancel_goal_async()
    assert spin(2, lambda: cancel.done())
    ack = cancel.result()
    r['events']['cancel_ack'] = {'return_code': ack.return_code, 'goals_canceling': len(ack.goals_canceling), 'delay_sec': time.monotonic() - at}
    assert ack.return_code == 0 and ack.goals_canceling
    spin(0.8)
    r['events']['map_absent_after_cancel'] = {'seconds_after_request': time.monotonic() - at, 'result_done': f.done()}
    assert not f.done()
    map_on = True
    restored = time.monotonic()
    assert spin(3, lambda: f.done())
    res = f.result()
    assert res.status == 4 and res.result.path.poses
    r['events']['map_restore'] = {'seconds_after_cancel': time.monotonic() - at, 'seconds_after_map_restore': time.monotonic() - restored, 'result_status': res.status, 'poses': len(res.result.path.poses)}
    send = client.send_goal_async(goal())
    assert spin(3, lambda: send.done())
    healthy = send.result()
    assert healthy.accepted
    hf = healthy.get_result_async()
    assert spin(3, lambda: hf.done())
    r['events']['healthy_map_control'] = {'result_status': hf.result().status, 'poses': len(hf.result().result.path.poses)}
    assert hf.result().status == 4 and hf.result().result.path.poses
    r['verdict'] = 'Accepted cancellation remains uncompleted while initial static map absent; map restore releases pending action. Terminal status recorded without assuming result order. Healthy-map next planning succeeds. No velocity output tested.'
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
