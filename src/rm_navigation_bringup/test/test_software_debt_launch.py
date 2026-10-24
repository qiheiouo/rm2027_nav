"""Controlled ROS boundary probes; no device or simulator required."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rm_competition_interfaces.msg import GimbalState
from sensor_msgs.msg import JointState, PointCloud2
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[3]


def spin_until(node, predicate, timeout=10.0, publish=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if publish:
            publish()
        rclpy.spin_once(node, timeout_sec=0.03)
        if predicate():
            return
    raise AssertionError("runtime observation timed out")


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


@pytest.mark.parametrize("override,topic,expected", [
    ([], "/gimbal/state_derived", 0.4),
    (["gimbal_use_input:=false"], "/gimbal/state_derived", 0.0),
    (["gimbal_use_input:=true", "gimbal_input_topic:=/probe/gimbal"], "/probe/gimbal", 0.4),
])
def test_phase2a_gimbal_yaml_and_cli_reach_joint_tf(override, topic, expected, tmp_path):
    rclpy.init()
    node = rclpy.create_node("gimbal_boundary_probe")
    joints, transforms = [], []
    node.create_subscription(JointState, "/joint_states", joints.append, 10)
    node.create_subscription(TFMessage, "/tf", lambda msg: transforms.extend(msg.transforms), 20)
    pub = node.create_publisher(GimbalState, topic, 10)
    config = ROOT / "src/rm_localization_adapters/config/gimbal_state_adapter_derived.yaml"
    # Only these packages' launch/config assets are used: driver stays disabled.
    index = tmp_path / "share/ament_index/resource_index/packages"
    index.mkdir(parents=True)
    for package in ("rm_mid360_driver_bridge", "rm_nav_config"):
        (index / package).touch()
        (tmp_path / "share" / package).symlink_to(ROOT / "src" / package)
    env = dict(os.environ, AMENT_PREFIX_PATH=str(tmp_path) + ":" + os.environ["AMENT_PREFIX_PATH"])
    with (tmp_path / "phase2a.log").open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "rm_navigation_bringup", "phase2a_lio_bringup.launch.py",
            "use_map_odom_stub:=false", f"gimbal_state_adapter_config:={config}", *override,
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        def publish():
            msg = GimbalState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.valid = msg.online = True
            msg.relative_yaw_rad = 0.4
            pub.publish(msg)
        try:
            spin_until(node, lambda: any(
                msg.name == ["gimbal_yaw_joint"] and abs(msg.position[0] - expected) < 1e-6
                for msg in joints), publish=publish)
            spin_until(node, lambda: any(
                tf.child_frame_id == "gimbal_yaw_link"
                and abs(tf.transform.rotation.z - math.sin(expected / 2)) < 1e-6
                for tf in transforms), publish=publish)
            assert process.poll() is None
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


def test_single_startup_reference_is_stable_across_40_samples(tmp_path):
    import yaml
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import StaticTransformBroadcaster
    backend = yaml.safe_load((ROOT / "src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml").read_text())
    assert backend["/**"]["ros__parameters"]["mapping"]["zero_start_pose"] is False
    rclpy.init()
    node = rclpy.create_node("startup_reference_probe")
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = "base_link", "lio_imu_link"
    tf.transform.translation.x, tf.transform.translation.y = 0.2, 0.1
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(0.2), math.cos(0.2)
    broadcaster.sendTransform(tf)
    rows = []
    node.create_subscription(Odometry, "/odometry/lio", rows.append, 100)
    pub = node.create_publisher(Odometry, "/odometry/fast_lio_raw", 10)
    config = ROOT / "src/rm_lio_bringup/config/lio_adapter_fast_lio_multi.yaml"
    with (tmp_path / "startup.log").open("w") as log:
        process = subprocess.Popen(["ros2", "run", "rm_localization_adapters", "lio_adapter",
            "--ros-args", "--params-file", str(config)], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            spin_until(node, lambda: pub.get_subscription_count() > 0)
            for index in range(45):
                # R(t) = W * G(t) * B, with nonidentity W and lever arm B.
                yaw = 0.001 * index
                gx, gy = 0.002 * index, 0.0
                sx = gx + math.cos(yaw)*0.2 - math.sin(yaw)*0.1
                sy = gy + math.sin(yaw)*0.2 + math.cos(yaw)*0.1
                msg = Odometry()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id, msg.child_frame_id = "odom", "body"
                msg.pose.pose.position.x = 3 + math.cos(0.7)*sx - math.sin(0.7)*sy
                msg.pose.pose.position.y = -1 + math.sin(0.7)*sx + math.cos(0.7)*sy
                msg.pose.pose.orientation.z = math.sin((0.7 + yaw + 0.4)/2)
                msg.pose.pose.orientation.w = math.cos((0.7 + yaw + 0.4)/2)
                before = len(rows)
                pub.publish(msg)
                if index:
                    spin_until(node, lambda: len(rows) > before)
                    assert rows[-1].header.stamp == msg.header.stamp
                    assert abs(rows[-1].pose.pose.position.x - gx) < 1e-7
                    assert abs(rows[-1].pose.pose.position.y - gy) < 1e-7
                    assert abs(rows[-1].pose.pose.orientation.z - math.sin(yaw/2)) < 1e-7
                else:
                    end = time.monotonic() + 0.1
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.01)
            assert len(rows) == 44
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


def test_generic_gicp_rejects_missing_or_incompatible_producer():
    import importlib.util
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location("generic_navigation", ROOT / "src/rm_navigation_bringup/launch/navigation.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    valid = dict(relocalization_backend="gicp_3d", global_localization_mode="external_pose",
                 use_map_server="true", use_lio_backend="true", sensor_mode="single",
                 update_method="bundle", gicp_input_cloud_topic="/lio/cloud_registered_body")
    context = LaunchContext()
    context.launch_configurations.update(valid)
    assert module._validate_localization_backend(context) == []
    for key, value in (("use_lio_backend", "false"), ("sensor_mode", "dual"),
                       ("update_method", "async"), ("gicp_input_cloud_topic", "/lio/cloud_registered")):
        context.launch_configurations.update(valid)
        context.launch_configurations[key] = value
        with pytest.raises(RuntimeError):
            module._validate_localization_backend(context)


def test_gicp_body_cloud_uses_canonical_sensor_tf(tmp_path):
    import random
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
    randomizer = random.Random(61)
    points = [(randomizer.uniform(-2, 2), randomizer.uniform(-2, 2), randomizer.uniform(-0.4, 0.4))
              for _ in range(300)]
    # canonical odom<-sensor = odom<-base * base<-sensor, both nonidentity.
    tx = 1 + math.cos(0.3)*0.2 - math.sin(0.3)*0.1
    ty = 0.5 + math.sin(0.3)*0.2 + math.cos(0.3)*0.1
    prior = [(tx + math.cos(0.7)*x - math.sin(0.7)*y,
              ty + math.sin(0.7)*x + math.cos(0.7)*y, z) for x, y, z in points]
    pcd = tmp_path / "prior.pcd"
    pcd.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 300\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 300\nDATA ascii\n" +
                   "\n".join(" ".join(map(str, p)) for p in prior) + "\n")
    rclpy.init()
    node = rclpy.create_node("gicp_body_probe")
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, x, y, yaw in (("odom", "base_link", 1., 0.5, 0.3),
                                    ("base_link", "lio_imu_link", 0.2, 0.1, 0.4)):
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    cloud_pub = node.create_publisher(PointCloud2,
                                      "/probe/body_cloud", 10)
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    rows = []
    node.create_subscription(PoseWithCovarianceStamped, "/localization/gicp_pose_raw", rows.append, 10)
    with (tmp_path / "gicp.log").open("w") as log:
        process = subprocess.Popen(["ros2", "run", "rm_gicp_relocalization", "gicp_relocalization_node",
            "--ros-args", "-p", f"prior_pcd_file:={pcd}", "-p", "use_fast_lio_body_cloud:=true",
            "-p", "input_cloud_topic:=/probe/body_cloud", "-p", "map_leaf_size:=0.05",
            "-p", "source_leaf_size:=0.05", "-p", "registration_rate_hz:=10.0"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def publish(frame):
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = "map"
            seed.header.stamp = node.get_clock().now().to_msg()
            seed.pose.pose.position.x, seed.pose.pose.position.y = 1., 0.5
            seed.pose.pose.orientation.z, seed.pose.pose.orientation.w = math.sin(0.15), math.cos(0.15)
            seed_pub.publish(seed)
            header = Header(frame_id=frame, stamp=node.get_clock().now().to_msg())
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        try:
            spin_until(node, lambda: cloud_pub.get_subscription_count() > 0 and seed_pub.get_subscription_count() > 0)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                publish("odom")  # A raw world cloud cannot enter the body contract.
                rclpy.spin_once(node, timeout_sec=0.03)
            assert not rows
            spin_until(node, lambda: bool(rows), publish=lambda: publish("body"))
            pose = rows[-1].pose.pose
            assert abs(pose.position.x - 1.) < 0.002
            assert abs(pose.position.y - 0.5) < 0.002
            assert abs(pose.orientation.z - math.sin(0.15)) < 0.002
            assert rows[-1].header.frame_id == "map"
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()


def test_nav2_behavior_and_controller_share_smoother_input(tmp_path):
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import ChangeState
    rclpy.init()
    node = rclpy.create_node("velocity_route_probe")
    rows = []
    node.create_subscription(Twist, "/cmd_vel", rows.append, 10)
    pub = node.create_publisher(Twist, "/cmd_vel_nav", 10)
    params = ROOT / "src/rm_nav_config/config/nav2_phase2f_deployment.yaml"
    with (tmp_path / "nav2.log").open("w") as log:
        process = subprocess.Popen(["ros2", "launch", "rm_navigation_bringup",
            "nav2_velocity_routed.launch.py", f"params_file:={params}", "autostart:=false"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def transition(name, identifier):
            client = node.create_client(ChangeState, "/" + name + "/change_state")
            assert client.wait_for_service(timeout_sec=10)
            req = ChangeState.Request()
            req.transition.id = identifier
            future = client.call_async(req)
            spin_until(node, future.done)
            assert future.result().success
            node.destroy_client(client)
        try:
            for name in ("controller_server", "behavior_server", "velocity_smoother"):
                transition(name, 1)
            transition("velocity_smoother", 3)
            spin_until(node, lambda: {e.node_name for e in node.get_publishers_info_by_topic("/cmd_vel_nav")}
                       >= {"controller_server", "behavior_server"})
            assert {e.node_name for e in node.get_publishers_info_by_topic("/cmd_vel")} == {"velocity_smoother"}
            assert any(e.node_name == "velocity_smoother" for e in node.get_subscriptions_info_by_topic("/cmd_vel_nav"))
            command = Twist()
            command.linear.x = 0.2
            spin_until(node, lambda: any(msg.linear.x > 0.01 for msg in rows), publish=lambda: pub.publish(command))
            # The final publisher never subscribes to its own output.
            assert not any(e.node_name == "velocity_smoother" for e in node.get_subscriptions_info_by_topic("/cmd_vel"))
            for name in ("src/rm_navigation_bringup/launch/navigation.launch.py",
                         "src/rm_simulation/launch/phase1_5_gazebo.launch.py"):
                assert '"nav2_velocity_routed.launch.py"' in (ROOT / name).read_text()
        finally:
            stop(process)
            node.destroy_node()
            rclpy.shutdown()
