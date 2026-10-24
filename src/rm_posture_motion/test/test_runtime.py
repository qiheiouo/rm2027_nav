"""Real Humble smoother + selector + V2 dry-run; synthetic MPPI parameter peer."""
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rm_competition_interfaces.msg import PostureState
from rm_posture_motion.profiles import load_profiles
from std_msgs.msg import UInt8MultiArray
import yaml

SOURCE = Path(__file__).resolve().parents[1]
BASELINE = SOURCE.parent / "rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml"
PRODUCTION = SOURCE / "config/profiles.yaml"
TEST = SOURCE / "test/fixtures/profiles_test_only.yaml"


def binary(package, name):
    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def wait(predicate, label, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timeout: " + label)


class ControllerPeer(Node):
    def __init__(self, baseline):
        super().__init__("controller_server")
        self.transactions = []
        for key, value in baseline.mppi("FollowPath").items():
            self.declare_parameter(key, value)
        self.add_on_set_parameters_callback(self.record)
        self.create_service(GetState, "/controller_server/get_state", self.state)

    def record(self, parameters):
        self.transactions.append({p.name: p.value for p in parameters})
        return SetParametersResult(successful=True)

    def state(self, _, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        return response


class Fixture(Node):
    def __init__(self):
        super().__init__("posture_fixture")
        self.actual = None
        self.valid = True
        self.competition_stream = False
        self.diagnostic = None
        self.commands = []
        self.frames = []
        self.goal_active = False
        self.goal_cancels = 0
        self.end_goal = False
        self.test_pub = self.create_publisher(PostureState, "/test/posture/state", 10)
        self.raw_pub = self.create_publisher(UInt8MultiArray, "/serial/v2/mock_rx", 10)
        self.sequence = 0
        self.boot_started = time.monotonic()
        self.cmd_pub = self.create_publisher(Twist, "/test/cmd_vel_nav", 10)
        self.create_subscription(Twist, "/cmd_vel", self.command, 100)
        self.create_subscription(UInt8MultiArray, "/serial/v2/mock_tx", self.frame, 100)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticArray, "/posture_motion/status", self.status, qos)
        self.create_timer(0.025, self.publish)
        self.change = self.create_client(ChangeState, "/velocity_smoother/change_state")
        self.set_limits = self.create_client(SetParametersAtomically,
                                            "/velocity_smoother/set_parameters_atomically")
        self.read_limits = self.create_client(GetParameters, "/velocity_smoother/get_parameters")
        self.nav = ActionServer(self, NavigateToPose, "/navigate_to_pose",
                                execute_callback=self.execute,
                                goal_callback=lambda _: GoalResponse.ACCEPT,
                                cancel_callback=self.cancel,
                                callback_group=ReentrantCallbackGroup())
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def command(self, message):
        self.commands.append((time.monotonic(), message.linear.x, message.linear.y, message.angular.z))

    def frame(self, message):
        data = bytes(message.data)
        if len(data) == 23 and data[:4] == b"RM\x02\x01":
            self.frames.append(struct.unpack("<fffB", data[8:21]))

    def status(self, message):
        if message.status:
            self.diagnostic = {v.key: v.value for v in message.status[0].values}

    def publish(self):
        cmd = Twist()
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 1.0
        self.cmd_pub.publish(cmd)
        if self.actual is not None:
            state = PostureState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.posture_actual.value = self.actual
            # Deliberately different requested/ACK state: only actual is authoritative.
            state.posture_target.value = 3
            state.completed = False
            state.ack_matches_request = False
            state.valid = self.valid
            state.online = True
            if self.competition_stream:
                elapsed_ms = int((time.monotonic() - self.boot_started) * 1000)
                self.send_frame(0x03, struct.pack("<IIIB", elapsed_ms, 0x20270042, 63, 1))
                self.send_frame(0x81, struct.pack("<IIBBBH", 0, 0, 3, self.actual, 3, 0))
            else:
                self.test_pub.publish(state)

    def send_frame(self, kind, payload):
        data = b"RM" + struct.pack("<BBHH", 2, kind, len(payload), self.sequence & 0xFFFF) + payload
        self.sequence += 1
        crc = 0xFFFF
        for byte in data[2:]:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
        self.raw_pub.publish(UInt8MultiArray(data=list(data + struct.pack("<H", crc))))

    def execute(self, handle):
        self.goal_active = True
        while not self.end_goal and not handle.is_cancel_requested:
            time.sleep(0.01)
        self.goal_active = False
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def cancel(self, _):
        self.goal_cancels += 1
        return CancelResponse.ACCEPT

    def applied(self, name):
        return (self.diagnostic and self.diagnostic["applied_profile"] == name
                and self.diagnostic["selected_profile"] == name
                and self.diagnostic["profile_generation"] == self.diagnostic["applied_generation"]
                and not self.diagnostic["backend_error"])

    def lifecycle(self, transition):
        assert self.change.wait_for_service(timeout_sec=5.0)
        future = self.change.call_async(ChangeState.Request(transition=Transition(id=transition)))
        wait(future.done, "smoother lifecycle")
        assert future.result().success


def test_s1_to_s12_with_real_smoother_and_final_denial():
    os.environ["ROS_DOMAIN_ID"] = str(130 + os.getpid() % 30)
    rclpy.init()
    baseline, profiles, _ = load_profiles(TEST, BASELINE, test_mode=True)
    node, peer = Fixture(), ControllerPeer(baseline)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes = []
    with tempfile.TemporaryDirectory(prefix="rm2027_posture_") as temporary:
        log = open(Path(temporary) / "runtime.log", "w+")
        smoother_params = Path(temporary) / "smoother.yaml"
        smoother_params.write_text(yaml.safe_dump({"velocity_smoother": {"ros__parameters": {
            **baseline.smoother(), "smoothing_frequency": 20.0, "feedback": "OPEN_LOOP",
            "deadband_velocity": [0.0]*3, "velocity_timeout": 1.0,
        }}}))

        def spawn(command):
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        def selector(mode, config):
            return spawn([binary("rm_posture_motion", "posture_profile_selector"), "--ros-args",
                          "-p", f"source_mode:={mode}", "-p", f"profiles_file:={config}",
                          "-p", f"baseline_file:={BASELINE}"])

        try:
            spawn([binary("nav2_velocity_smoother", "velocity_smoother"), "--ros-args",
                   "--params-file", str(smoother_params), "-r", "cmd_vel:=/test/cmd_vel_nav",
                   "-r", "cmd_vel_smoothed:=/cmd_vel"])
            spawn([binary("rm_serial_driver", "competition_v2_transport_node"),
                   "--ros-args", "-p", "dry_run:=true", "-p", "motion_test_mode:=false"])
            node.lifecycle(Transition.TRANSITION_CONFIGURE)
            node.lifecycle(Transition.TRANSITION_ACTIVATE)
            process = selector("test", TEST)
            wait(lambda: node.applied("fallback"), "S1 startup fallback")
            print("S1 PASS: no posture -> FALLBACK")
            assert node.nav_client.wait_for_server(timeout_sec=2.0)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.orientation.w = 1.0
            future = node.nav_client.send_goal_async(goal)
            wait(future.done, "active goal")
            assert future.result().accepted
            wait(lambda: node.goal_active, "goal execution")

            def select(actual, name):
                node.actual = actual
                wait(lambda: node.applied(name), f"applied {name}")
                assert peer.transactions[-1] == profiles[name].mppi("FollowPath")
                expected = profiles[name].smoother()
                values = node.read_limits.call_async(GetParameters.Request(names=list(expected)))
                wait(values.done, "read actual smoother parameters")
                assert [list(v.double_array_value) for v in values.result().values] == list(expected.values())
                wait(lambda: node.commands and all(
                    abs(node.commands[-1][i + 1] - profiles[name].max_velocity[i]) < 1e-4
                    for i in range(3)), f"real smoother output reaches {name}")
                assert node.goal_active and node.goal_cancels == 0
                return dict(node.diagnostic)

            mobile = select(1, "mobile")
            print("S2 PASS: FALLBACK -> MOBILE")
            select(5, "enhanced_mobile")
            mobile_again = select(1, "mobile")
            assert int(mobile_again["profile_generation"]) > int(mobile["profile_generation"])
            print("S3 PASS: MOBILE -> ENHANCED_MOBILE -> MOBILE; target/ACK ignored")
            select(0, "attack")
            print("S4 PASS: MOBILE -> ATTACK")
            select(3, "enhanced_attack")
            select(0, "attack")
            print("S5 PASS: ATTACK -> ENHANCED_ATTACK -> ATTACK")
            select(2, "defense")
            print("S6 PASS: ATTACK -> DEFENSE")
            select(4, "enhanced_defense")
            select(2, "defense")
            print("S7 PASS: DEFENSE -> ENHANCED_DEFENSE -> DEFENSE")
            node.actual = None
            wait(lambda: node.applied("fallback"), "S8 stale fallback")
            assert node.diagnostic["current_effective_posture"] == "STALE"
            wait(lambda: node.commands[-1][1] <= 0.05001, "fallback deceleration")
            print("S8 PASS: source lost -> STALE -> FALLBACK, smoother decelerates")
            select(5, "enhanced_mobile")
            transactions = len(peer.transactions)
            writes = node.diagnostic["parameter_transactions"]
            generation = node.diagnostic["profile_generation"]
            time.sleep(1.0)
            assert len(peer.transactions) == transactions
            assert node.diagnostic["parameter_transactions"] == writes
            assert node.diagnostic["profile_generation"] == generation
            print("S9 PASS: 40Hz identical posture -> zero parameter churn")
            assert node.goal_active and node.goal_cancels == 0
            print("S10 PASS: same Nav2 goal stays active throughout posture transitions")
            wait(lambda: len(node.frames) > 100, "real V2 disabled frames")
            assert all(frame == (0.0, 0.0, 0.0, 0) for frame in node.frames)
            assert any(cmd[1] > 0.0 for cmd in node.commands)
            print(f"S11 PASS: final V2 denial preserved over {len(node.frames)} encoded frames")
            names = [v["FollowPath.vx_max"] for v in peer.transactions]
            assert set(p.max_velocity[0] for name, p in profiles.items() if name != "fallback") <= set(names)
            print("S12 PASS: six distinct test slots reach MPPI parameters and real smoother output")

            # Unknown enum and explicit invalid state cannot retain enhanced limits.
            node.actual = 255
            wait(lambda: node.applied("fallback"), "unknown enum")
            assert node.diagnostic["fallback_reason"] == "unknown_enum"
            select(5, "enhanced_mobile")
            node.valid = False
            wait(lambda: node.applied("fallback"), "invalid posture")
            node.valid = True
            print("INVALID PASS: unknown/invalid -> FALLBACK")

            # Restart production consumer with restored baseline; test topic must not leak.
            process.terminate()
            process.wait(timeout=3.0)
            request = SetParametersAtomically.Request(parameters=[
                Parameter(key, value=value).to_parameter_msg()
                for key, value in baseline.smoother().items()
            ])
            future = node.set_limits.call_async(request)
            wait(future.done, "restore fixture baseline")
            assert future.result().result.successful
            peer.set_parameters_atomically([
                Parameter(key, value=value) for key, value in baseline.mppi("FollowPath").items()
            ])
            node.actual = 5
            node.diagnostic = None
            process = selector("competition_v2", PRODUCTION)
            wait(lambda: node.applied("fallback"), "production ignores test source")
            time.sleep(0.7)
            assert node.applied("fallback")
            writes = node.diagnostic["parameter_transactions"]
            for actual, name in [(1, "mobile"), (5, "enhanced_mobile"), (0, "attack"),
                                 (3, "enhanced_attack"), (2, "defense"), (4, "enhanced_defense")]:
                node.competition_stream = True
                node.actual = actual
                wait(lambda: node.applied(name), "production independent slot " + name)
                assert node.diagnostic["current_effective_posture"] == name.upper()
            assert node.diagnostic["parameter_transactions"] == writes == "0"
            assert node.goal_active and node.goal_cancels == 0
            assert all(frame == (0.0, 0.0, 0.0, 0) for frame in node.frames)
            print("PRODUCTION PASS: mock topic ignored; real V2 decodes six actual states; equal-valued slots independent; zero writes")
            process.terminate()
            process.wait(timeout=3.0)
            peer.set_parameters_atomically([Parameter("FollowPath.vx_max", value=0.49)])
            node.diagnostic = None
            selector("competition_v2", PRODUCTION)
            wait(lambda: node.diagnostic and node.diagnostic["backend_error"], "baseline mismatch veto")
            assert "runtime baseline differs" in node.diagnostic["backend_error"]
            assert node.diagnostic["parameter_transactions"] == "0"
            assert node.diagnostic["applied_profile"] == "unconfirmed"
            assert node.goal_active and node.goal_cancels == 0
            print("BASELINE MISMATCH PASS: report error without writing constraints or canceling goal")
        finally:
            node.end_goal = True
            for process in reversed(processes):
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=4.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
            log.seek(0)
            print(log.read())
            log.close()
            executor.shutdown(timeout_sec=3.0)
            node.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()
