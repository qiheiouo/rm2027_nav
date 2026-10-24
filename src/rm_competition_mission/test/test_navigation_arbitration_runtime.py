"""Real Mission + pursuit planner, fake Nav2, isolated ROS graph; no hardware."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rm_competition_interfaces.msg import (
    ChassisMode, LocalizationStatus, MissionState, PursuitCandidate,
    RefereeState, TargetTrack, V2MotionGuardState,
)
from rm_competition_interfaces.srv import SetMissionMode
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


class ArbitrationFixture(Node):
    def __init__(self):
        super().__init__("arbitration_fixture")
        self.requests = []
        self.goals = []
        self.cancels = []
        self.terminals = []
        self.active = set()
        self.max_active = 0
        self.complete = set()
        self.accept_delay = 0.0
        self.cancel_delay = 0.15
        self.hp = 400
        self.estop = False
        self.guard = V2MotionGuardState.PERMITTED
        self.publish_guard = True
        self.publish_chassis = True
        self.track = None
        self.research_track = False
        self.state = None
        self.sequence = 0
        self.started_at = self.get_clock().now().to_msg()
        self.nav = ActionServer(
            self, NavigateToPose, "/navigate_to_pose",
            goal_callback=self.accept, cancel_callback=self.cancel,
            execute_callback=self.execute, callback_group=ReentrantCallbackGroup(),
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.loc_pub = self.create_publisher(LocalizationStatus, "/localization/status", 1)
        self.ref_pub = self.create_publisher(RefereeState, "/referee/state", latched)
        self.ref_valid = self.create_publisher(Bool, "/referee/state_valid", latched)
        self.mode_pub = self.create_publisher(ChassisMode, "/chassis/mode", latched)
        self.guard_pub = self.create_publisher(V2MotionGuardState, "/motion/v2_guard/state", 1)
        self.radar_pub = self.create_publisher(TargetTrack, "/radar/target_track", 10)
        self.generic_pub = self.create_publisher(TargetTrack, "/perception/target_track", 10)
        self.candidate_pub = self.create_publisher(PursuitCandidate, "/mission/pursuit_candidate", 10)
        self.legacy_pub = self.create_publisher(PoseStamped, "/mission/pursuit_goal", 10)
        self.legacy_valid = self.create_publisher(Bool, "/mission/pursuit_goal_valid", latched)
        self.create_subscription(MissionState, "/mission/state", self.on_state, latched)
        self.set_mode = self.create_client(SetMissionMode, "/mission/set_mode")
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.create_timer(0.025, self.publish)

    def on_state(self, message):
        self.state = message

    def accept(self, request):
        self.requests.append((time.monotonic(), request.pose.pose.position.x))
        time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def cancel(self, _):
        self.cancels.append(time.monotonic())
        return CancelResponse.ACCEPT

    def execute(self, handle):
        index = len(self.goals)
        self.goals.append((time.monotonic(), handle.request.pose.pose.position.x))
        self.active.add(index)
        self.max_active = max(self.max_active, len(self.active))
        while rclpy.ok() and not handle.is_cancel_requested and index not in self.complete:
            time.sleep(0.01)
        canceled = handle.is_cancel_requested
        if canceled:
            # ACK is deliberately earlier than the terminal result.
            time.sleep(self.cancel_delay)
        self.active.remove(index)
        self.terminals.append((time.monotonic(), index, canceled))
        if canceled:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def target(self, track_id="alpha", x=5.0, valid=True):
        message = TargetTrack()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.track_id = track_id
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.confidence = 0.95
        message.valid = valid
        return message

    def publish(self):
        now = self.get_clock().now()
        loc = LocalizationStatus()
        loc.header.stamp = now.to_msg()
        loc.started_at = self.started_at
        loc.producer_id = "arbitration_fixture"
        self.sequence += 1
        loc.sequence = self.sequence
        loc.valid = True
        loc.valid_until = (now + Duration(seconds=1.0)).to_msg()
        self.loc_pub.publish(loc)
        referee = RefereeState()
        referee.header.stamp = now.to_msg()
        referee.valid = True
        referee.game_progress = 4
        referee.current_hp = self.hp
        self.ref_pub.publish(referee)
        self.ref_valid.publish(Bool(data=True))
        mode = ChassisMode()
        mode.header.stamp = now.to_msg()
        mode.online = True
        mode.autonomous_enabled = not self.estop
        mode.emergency_stop = self.estop
        mode.mode = mode.MODE_ESTOP if self.estop else mode.MODE_AUTONOMOUS
        if self.publish_chassis:
            self.mode_pub.publish(mode)
        guard = V2MotionGuardState()
        guard.header.stamp = now.to_msg()
        guard.endpoint_instance = "fixture_endpoint"
        guard.phase = self.guard
        if self.publish_guard:
            self.guard_pub.publish(guard)
        if self.track is not None:
            track_id, x, valid = self.track
            publisher = self.generic_pub if self.research_track else self.radar_pub
            publisher.publish(self.target(track_id, x, valid))

    def mode(self, mode, enabled=True):
        assert self.set_mode.wait_for_service(timeout_sec=5.0)
        future = self.set_mode.call_async(SetMissionMode.Request(mode=mode, enable=enabled))
        wait(lambda: future.done(), "set mode response")
        assert future.result().accepted


def wait(predicate, label, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out: {label}")


def stable_count(node, duration=0.65):
    count = len(node.requests)
    time.sleep(duration)
    assert len(node.requests) == count, (count, node.requests)


def wait_goal(node, x, after=0):
    wait(lambda: len(node.goals) > after and math.isclose(node.goals[-1][1], x, abs_tol=0.02),
         f"goal x={x} after {after}; received {node.goals}")


def test_s1_to_s10_and_action_handover():
    os.environ["ROS_DOMAIN_ID"] = str(160 + os.getpid() % 30)
    rclpy.init()
    node = ArbitrationFixture()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes = []
    with tempfile.TemporaryDirectory(prefix="rm2027_arbitration_") as temporary:
        config = Path(temporary) / "mission.yaml"
        config.write_text(yaml.safe_dump({"competition_mission_node": {"ros__parameters": {
            "default_mode": "auto", "allow_pursuit": True,
            "home_pose": [0.0, 0.0, 0.0],
            "patrol_waypoints": [1.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            "tick_rate_hz": 20.0,
            # Attempted YAML overrides must lose to the composition policy.
            "radar_only": False, "require_motion_guard": False,
        }}}))
        log = open(Path(temporary) / "launch.log", "w+")
        source = Path(os.environ["RM_COMPETITION_MISSION_SOURCE_DIR"].lstrip(os.pathsep))
        launch = str(source.parent / "rm_navigation_bringup" / "launch" /
                     "competition_2027_navigation.launch.py")
        process = subprocess.Popen([
            "ros2", "launch", launch, "enable_navigation_intents:=true",
            f"mission_config:={config}",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        try:
            wait(lambda: node.state is not None, "mission startup")
            stable_count(node, 0.2)
            assert not node.requests
            node.mode("auto")
            wait_goal(node, 1.0)
            node.complete.add(0)
            wait_goal(node, 2.0, 1)
            print("S1 PASS: idle -> patrol; reached waypoint 0, now index 1")

            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, 2)
            assert node.cancels
            print("S2 PASS: patrol terminal cancellation precedes radar pursuit")

            node.track = None
            wait_goal(node, 2.0, 3)
            print("S3 PASS: stale radar -> resume patrol index 1")
            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, 4)
            node.track = ("alpha", 5.0, False)
            wait_goal(node, 2.0, 5)
            print("S3 LOST PASS: explicit invalid target -> same patrol waypoint")

            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, 6)
            node.hp = 100
            wait_goal(node, 0.0, 7)
            print("S4 PASS: low-health return preempts pursuit")
            node.track = ("beta", 6.0, True)
            stable_count(node)
            assert node.state.active_branch == "home"
            print("S5 PASS: new radar target cannot preempt home")

            node.guard = V2MotionGuardState.BLOCKED
            wait(lambda: not node.active, "guard denial cancels home")
            stable_count(node)
            print("S6 PASS: final guard denied -> cancel, zero replacement goals")
            node.track = None
            node.hp = 400
            node.guard = V2MotionGuardState.PERMITTED
            wait_goal(node, 2.0, 8)
            assert node.state.active_branch == "patrol"
            print("S7 PASS: restored guard resumes patrol, never old tactical target")

            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, 9)
            before = len(node.cancels)
            for i in range(20):
                node.track = ("alpha", 5.0 + 0.01 * (i % 2), True)
                time.sleep(0.03)
            assert len(node.cancels) == before
            stable_count(node, 0.2)
            moving_start = len(node.goals)
            started = time.monotonic()
            while time.monotonic() - started < 1.6:
                node.track = ("alpha", 5.0 + (time.monotonic() - started), True)
                time.sleep(0.025)
            assert len(node.goals) - moving_start <= 4
            stable_target = node.track[1]
            time.sleep(0.65)
            print(f"S8 PASS: 40Hz source, jitter cancels=0, moving updates="
                  f"{len(node.goals) - moving_start} over 2.25s")

            before = len(node.goals)
            node.track = ("replacement", stable_target, True)
            wait(lambda: len(node.goals) == before + 1, "identity replacement at same coordinates")
            stable_count(node)
            print("S9 PASS: different track_id at identical pose replaces exactly once")

            node.track = ("completion", stable_target, True)
            wait(lambda: len(node.goals) == before + 2, "fresh pursuit before completion")
            sent_at = node.requests[-1][0]
            count = len(node.goals)
            node.complete.add(count - 1)
            node.track = ("completion", stable_target + 0.5, True)
            wait_goal(node, stable_target - 1.0, count)
            assert node.requests[-1][0] - sent_at >= 0.45
            print("COMPLETE UPDATE PASS: same-ID update respects rate limit after success")

            node.track = None
            wait_goal(node, 2.0, len(node.goals))
            before = len(node.requests)
            end = time.monotonic() + 0.65
            while time.monotonic() < end:
                node.generic_pub.publish(node.target("teammate", 7.0))
                candidate = PursuitCandidate()
                candidate.header.stamp = node.get_clock().now().to_msg()
                candidate.header.frame_id = "map"
                candidate.valid_until = (node.get_clock().now() + Duration(seconds=0.3)).to_msg()
                candidate.track_id = "teammate"
                candidate.source = PursuitCandidate.SOURCE_RESEARCH
                candidate.valid = True
                candidate.pose.orientation.w = 1.0
                candidate.pose.position.x = 7.0
                node.candidate_pub.publish(candidate)
                node.legacy_valid.publish(Bool(data=True))
                node.legacy_pub.publish(PoseStamped())
                time.sleep(0.025)
            assert len(node.requests) == before
            print("S10 PASS: generic teammate, research candidate and legacy Bool/pose ignored")

            # Stopped planner cannot leave a cached true candidate alive.
            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, len(node.goals))
            planner = [n for n in node.get_node_names() if n == "pursuit_goal_planner"]
            assert len(planner) == 1
            # Freeze its input timestamp while republishing; deadline must not slide.
            node.track = None
            frozen = node.target("alpha", 5.0)
            count = len(node.goals)
            end = time.monotonic() + 0.8
            while time.monotonic() < end:
                node.radar_pub.publish(frozen)
                time.sleep(0.025)
            wait_goal(node, 2.0, count)
            print("LEASE PASS: repeated source sample cannot renew its original deadline")

            node.track = ("alpha", 5.0, True)
            wait_goal(node, 3.5, len(node.goals))
            owned_processes = subprocess.check_output(
                ["ps", "-eo", "pid,ppid,args"], text=True
            ).splitlines()
            rows = [line.split(None, 2) for line in owned_processes[1:]]
            descendants = {process.pid}
            while True:
                expanded = descendants | {int(row[0]) for row in rows
                                          if int(row[1]) in descendants}
                if expanded == descendants:
                    break
                descendants = expanded
            planner_pids = [int(row[0]) for row in rows
                            if int(row[0]) in descendants
                            and "/rm_pursuit/pursuit_goal_planner" in row[2]]
            assert len(planner_pids) == 1, owned_processes
            count = len(node.goals)
            os.kill(planner_pids[0], signal.SIGKILL)
            wait_goal(node, 2.0, count)
            print("PLANNER EXIT PASS: cached true Bool cannot sustain pursuit after producer death")

            # Pending response must be canceled before a replacement can be sent.
            node.mode("hold")
            wait(lambda: not node.active, "hold cancellation")
            node.accept_delay = 0.35
            before = len(node.requests)
            node.mode("auto")
            wait(lambda: len(node.requests) > before, "delayed goal request")
            node.hp = 100
            wait_goal(node, 0.0, len(node.goals))
            node.accept_delay = 0.0
            print("PENDING PASS: late accepted patrol canceled before home")

            node.estop = True
            wait(lambda: not node.active, "emergency cancellation")
            stable_count(node)
            node.estop = False
            wait_goal(node, 0.0, len(node.goals))
            node.publish_guard = False
            wait(lambda: not node.active, "guard producer disappearance")
            stable_count(node)
            node.publish_guard = True
            wait_goal(node, 0.0, len(node.goals))
            node.publish_chassis = False
            wait(lambda: not node.active, "chassis producer disappearance")
            stable_count(node)
            print("SAFETY LEASE PASS: ESTOP, vanished guard and vanished chassis cancel")

            # A configured retreat condition with no reviewed home must hold,
            # rather than falling through to a lower-priority pursuit/patrol.
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
            parameters = yaml.safe_load(config.read_text())
            del parameters["competition_mission_node"]["ros__parameters"]["home_pose"]
            config.write_text(yaml.safe_dump(parameters))
            node.state = None
            node.publish_chassis = True
            node.track = ("alpha", 5.0, True)
            process = subprocess.Popen([
                "ros2", "launch", launch, "enable_navigation_intents:=true",
                f"mission_config:={config}",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            wait(lambda: node.state is not None and not node.state.enabled, "restart without home")
            node.mode("auto")
            wait(lambda: node.state.active_branch == "hold_missing_home", "missing home hold")
            stable_count(node)
            node.hp = 400
            wait_goal(node, 3.5, len(node.goals))
            node.hp = 100
            wait(lambda: not node.active, "missing home cancels pursuit")
            stable_count(node)
            assert node.state.active_branch == "hold_missing_home"
            print("MISSING HOME PASS: return condition holds instead of falling through to pursuit")

            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
            node.hp = 400
            node.state = None
            node.research_track = True
            for command in (
                ["ros2", "launch", "rm_pursuit", "pursuit.launch.py",
                 "enable_pursuit_boundary:=true", "radar_only:=false"],
                ["ros2", "launch", "rm_competition_mission", "competition_mission.launch.py",
                 "enable_mission_node:=true", "radar_only:=false", f"mission_config:={config}"],
            ):
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                processes.append(process)
            wait(lambda: node.state is not None and not node.state.enabled, "research startup")
            node.mode("auto")
            wait_goal(node, 3.5, len(node.goals))
            stable_count(node)
            node.track = None
            wait_goal(node, 1.0, len(node.goals))
            print("RESEARCH PASS: explicit non-radar mode retains generic pursuit -> patrol")

            assert node.max_active == 1, node.goals
            for previous, replacement in zip(node.goals, node.goals[1:]):
                index = node.goals.index(previous)
                terminal = next(t[0] for t in node.terminals if t[1] == index)
                assert terminal <= replacement[0]
                assert terminal <= node.requests[index + 1][0]
            print(f"OWNER PASS: maximum active Nav2 goals={node.max_active}; "
                  f"goals={len(node.goals)}, cancels={len(node.cancels)}")
        finally:
            for process in processes:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.0)
            log.seek(0)
            print(log.read())
            log.close()
            # Let fake actions finish before removing their executor.
            node.complete.update(range(len(node.goals)))
            wait(lambda: not node.active, "fixture cleanup")
            executor.shutdown(timeout_sec=3.0)
            node.destroy_node()
            rclpy.shutdown()
