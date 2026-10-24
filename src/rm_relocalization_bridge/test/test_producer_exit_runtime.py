"""Registered AUD-008 regression: real gate/bridge, synthetic poses, no devices."""
import json
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rm_competition_interfaces.msg import LocalizationStatus


def test_gate_death_replay_and_restart_cannot_renew_bridge_evidence(tmp_path):
    rclpy.init()
    node = rclpy.create_node("producer_exit_regression")
    statuses, upstream, corrections = [], [], []
    node.create_subscription(LocalizationStatus, "/localization/status",
                             lambda m: statuses.append((time.monotonic(), m)), 100)
    node.create_subscription(LocalizationStatus, "/evidence/gate_valid/status",
                             lambda m: upstream.append(m), 100)
    node.create_subscription(TransformStamped, "/evidence/correction",
                             lambda m: corrections.append((time.monotonic(), m)), 100)
    odom_pub = node.create_publisher(Odometry, "/evidence/odom", 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/evidence/raw", 10)
    replay_pub = node.create_publisher(LocalizationStatus, "/evidence/gate_valid/status", 10)
    processes, logs = [], []
    result = {}
    publish_pose = True
    replay = None

    def start(executable, parameters):
        log = (tmp_path / f"{executable}_{len(processes)}.log").open("w")
        logs.append(log)
        command = [get_package_prefix("rm_relocalization_bridge") +
                   "/lib/rm_relocalization_bridge/" + executable, "--ros-args"]
        for key, value in parameters.items():
            command.extend(["-p", f"{key}:={value}"])
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def gate():
        return start("global_pose_gate_node", dict(input_topic="/evidence/raw",
                     output_topic="/evidence/global", valid_topic="/evidence/gate_valid"))

    def spin(seconds, predicate=None):
        end, last_publish = time.monotonic() + seconds, 0.
        while time.monotonic() < end:
            if time.monotonic() - last_publish >= .04:
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = stamp, "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)
                if publish_pose:
                    pose = PoseWithCovarianceStamped()
                    pose.header.stamp, pose.header.frame_id = stamp, "map"
                    pose.pose.pose.position.x = 2.
                    pose.pose.pose.orientation.w = 1.
                    pose_pub.publish(pose)
                if replay is not None:
                    replay_pub.publish(replay)
                last_publish = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate and predicate():
                return True
        return predicate is None or predicate()

    try:
        bridge = start("map_odom_from_global_pose", dict(
            upstream_valid_topic="/evidence/gate_valid", odom_topic="/evidence/odom",
            global_pose_topic="/evidence/global", map_to_odom_topic="/evidence/correction"))
        producer = gate()
        assert spin(5., lambda: statuses and statuses[-1][1].valid and corrections and upstream)
        spin(.4)
        assert statuses[-1][1].valid
        assert abs(corrections[-1][1].transform.translation.x - 2.) < 1e-6
        old_id = upstream[-1].producer_id
        cached = upstream[-1]
        killed_at = time.monotonic()
        producer.kill()
        producer.wait(timeout=3.)
        assert producer.returncode == -signal.SIGKILL
        assert spin(1.6, lambda: statuses and not statuses[-1][1].valid)
        result["gate_death_to_invalid_sec"] = time.monotonic() - killed_at
        # Keep raw poses and odometry arriving after the gate process is gone.
        spin(1.2)
        cutoff = time.monotonic()
        replay = cached
        spin(.5)
        assert all(not msg.valid for t, msg in statuses if t > cutoff)
        assert not any(t > cutoff for t, _ in corrections)
        result["expired_replay_accepted"] = False
        replay, publish_pose = None, False
        producer = gate()
        spin(.5)
        assert not statuses[-1][1].valid
        assert upstream[-1].producer_id != old_id
        publish_pose = True
        assert spin(3., lambda: statuses[-1][1].valid)
        assert bridge.poll() is None and producer.poll() is None
        result["fresh_new_instance_recovers"] = True
    finally:
        forced_kills = 0
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    forced_kills += 1
                    process.kill()
                    process.wait(timeout=3.)
        result["exit_codes"] = [p.returncode for p in processes]
        result["forced_cleanup_kills"] = forced_kills
        (tmp_path / "evidence.json").write_text(json.dumps(result, indent=2) + "\n")
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    assert forced_kills == 0
    assert result["exit_codes"] == [0, -signal.SIGKILL, 0]
