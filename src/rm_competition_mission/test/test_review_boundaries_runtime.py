"""Independent review regressions for rejected cancellation and input recovery."""
from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rm_competition_interfaces.msg import PursuitCandidate
from rclpy.action import CancelResponse

from test_navigation_arbitration_runtime import ArbitrationFixture, stable_count, wait, wait_goal


@contextmanager
def mission(tmp_path):
    os.environ["ROS_DOMAIN_ID"] = str(70 + os.getpid() % 20)
    rclpy.init()
    node = ArbitrationFixture()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    config = tmp_path / "mission.yaml"
    config.write_text('''competition_mission_node:
  ros__parameters:
    default_mode: auto
    allow_pursuit: true
    tick_rate_hz: 20.0
    home_pose: [0.0, 0.0, 0.0]
    patrol_waypoints: [1.0, 0.0, 0.0, 2.0, 0.0, 0.0]
''')
    launch = Path(__file__).resolve().parents[2] / "rm_navigation_bringup/launch/competition_2027_navigation.launch.py"
    with (tmp_path / "mission.log").open("w+") as log:
        process = subprocess.Popen(["ros2", "launch", str(launch),
                                    "enable_navigation_intents:=true", f"mission_config:={config}"],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait(lambda: node.state is not None, "mission startup")
            node.mode("auto")
            wait_goal(node, 1.0)
            yield node, process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            node.complete.update(range(len(node.goals)))
            wait(lambda: not node.active, "fixture cleanup")
            log.seek(0)
            print(log.read())
            executor.shutdown(timeout_sec=3)
            node.destroy_node()
            rclpy.shutdown()


def candidate(node):
    msg = PursuitCandidate()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.valid_until = (node.get_clock().now() + Duration(seconds=0.3)).to_msg()
    msg.track_id = "review"
    msg.source = msg.SOURCE_RADAR
    msg.valid = True
    msg.pose.position.x = 5.0
    msg.pose.orientation.w = 1.0
    return msg


@pytest.mark.parametrize("field", ["header", "deadline", "nanosec", "future"])
def test_malformed_and_future_input_cannot_kill_or_poison_owner(tmp_path, field):
    with mission(tmp_path) as (node, process):
        bad = candidate(node)
        if field == "header":
            bad.header.stamp.sec = -1
        elif field == "nanosec":
            bad.header.stamp.nanosec = 1_000_000_000
        elif field == "deadline":
            bad.valid_until.sec = -1
        else:
            bad.header.stamp.sec += 3600
            bad.valid_until.sec += 3600
        node.candidate_pub.publish(bad)
        stable_count(node, 0.15)
        before = len(node.goals)
        until = time.monotonic() + 2.0
        while time.monotonic() < until and len(node.goals) == before:
            node.candidate_pub.publish(candidate(node))
            time.sleep(0.025)
        wait_goal(node, 5.0, before)
        wait_goal(node, 1.0, len(node.goals))  # source lease expires
        assert process.poll() is None


def test_rejected_cancel_and_success_result_do_not_transfer_owner_early(tmp_path):
    with mission(tmp_path) as (node, process):
        node.nav.register_cancel_callback(lambda _: CancelResponse.REJECT)
        before = len(node.requests)
        node.mode("home")
        time.sleep(0.25)
        assert len(node.requests) == before
        assert node.active
        node.complete.add(0)  # Old goal succeeds after cancellation was requested.
        wait_goal(node, 0.0, before)
        node.nav.register_cancel_callback(node.cancel)
        node.mode("auto")
        wait_goal(node, 1.0, len(node.goals))  # canceled patrol success cannot advance index
        assert node.max_active == 1
        for index in range(len(node.goals) - 1):
            terminal = next(t[0] for t in node.terminals if t[1] == index)
            assert terminal <= node.requests[index + 1][0]
        assert process.poll() is None


def test_planner_bad_stamp_is_rejected_before_watermark():
    from rm_pursuit.pursuit_goal_planner import PursuitGoalPlanner
    rclpy.init()
    planner = PursuitGoalPlanner()
    try:
        from rm_competition_interfaces.msg import TargetTrack
        msg = TargetTrack()
        msg.header.stamp.sec = -1
        planner._handle_track(msg)
        msg = TargetTrack()
        msg.header.stamp = (planner.get_clock().now() + Duration(seconds=3600)).to_msg()
        planner._handle_track(msg)
        good = TargetTrack()
        good.header.stamp = planner.get_clock().now().to_msg()
        planner._handle_track(good)
        assert planner._latest_track == good
    finally:
        planner.destroy_node()
        rclpy.shutdown()
