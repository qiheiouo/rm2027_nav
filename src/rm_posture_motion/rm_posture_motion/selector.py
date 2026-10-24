import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rm_competition_interfaces.msg import PostureState

from .profiles import PostureSelection, load_profiles


class PostureProfileSelector(Node):
    def __init__(self):
        super().__init__("posture_profile_selector")
        mode = self.declare_parameter("source_mode", "competition_v2").value
        if mode not in ("competition_v2", "test"):
            raise ValueError("source_mode must be competition_v2 or test")
        self.source_mode = mode
        profiles_file = self.declare_parameter("profiles_file", "").value
        baseline_file = self.declare_parameter("baseline_file", "").value
        self.plugin = self.declare_parameter("controller_plugin", "FollowPath").value
        self.baseline, self.profiles, self.revision = load_profiles(
            profiles_file, baseline_file, self.plugin, test_mode=mode == "test")
        timeout = float(self.declare_parameter("posture_timeout_sec", 0.5).value)
        debounce = float(self.declare_parameter("posture_debounce_sec", 0.05).value)
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(debounce) or debounce < 0:
            raise ValueError("invalid posture freshness/debounce settings")
        self.selection = PostureSelection(self.get_clock().now().nanoseconds, timeout, debounce)
        self.parameter_peers = {}
        for target, default in (("smoother", "/velocity_smoother"),
                                ("controller", "/controller_server")):
            node_name = self.declare_parameter(f"{target}_node", default).value.rstrip("/")
            self.parameter_peers[target] = {
                "state": self.create_client(GetState, node_name + "/get_state"),
                "get": self.create_client(GetParameters, node_name + "/get_parameters"),
                "set": self.create_client(SetParametersAtomically,
                                          node_name + "/set_parameters_atomically"),
            }
        self.current = {}
        self.startup = 0
        self.future = None
        self.operation = None
        self.state_token = None
        self.error = ""
        self.applied_profile = "unconfirmed"
        self.applied_generation = 0
        self.parameter_transactions = 0
        self.last_diagnostic = 0.0
        self.last_clock_ns = self.get_clock().now().nanoseconds
        self.topic = "/robot/posture/state" if mode == "competition_v2" else "/test/posture/state"
        self.subscription = self.create_subscription(PostureState, self.topic, self.observe, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(DiagnosticArray, "/posture_motion/status", qos)
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.timer_clock)

    def observe(self, message):
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1000000000:
            now_ns = self.get_clock().now().nanoseconds
            self.selection.observe(255, now_ns, False, False, False, False,
                                   now_ns, time.monotonic())
            return
        self.selection.observe(
            message.posture_actual.value, stamp.sec * 1000000000 + stamp.nanosec,
            message.valid, message.online, message.transitioning, message.fault,
            self.get_clock().now().nanoseconds, time.monotonic())

    def payload(self, target, profile):
        return profile.smoother() if target == "smoother" else profile.mppi(self.plugin)

    def request(self, target, kind, payload=None):
        client = self.parameter_peers[target][kind]
        if not client.service_is_ready():
            return False
        if kind == "state":
            request = GetState.Request()
        elif kind == "get":
            request = GetParameters.Request(names=list(self.payload(target, self.baseline)))
            payload = self.payload(target, self.baseline)
        else:
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=value).to_parameter_msg() for name, value in payload.items()
            ])
            self.parameter_transactions += 1
        self.operation = (target, kind, payload, self.selection.generation)
        self.future = client.call_async(request)
        self.sent_at = time.monotonic()
        return True

    def finish(self):
        target, kind, payload, generation = self.operation
        response = self.future.result()
        self.future = None
        if kind == "state":
            if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                raise ValueError(f"{target} is not ACTIVE; restart selector after lifecycle recovery")
            if self.startup < 4:
                self.startup += 1
            else:
                self.state_token = (target, generation)
        elif kind == "get":
            actual = []
            for value in response.values:
                if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    actual.append(list(value.double_array_value))
                elif value.type == ParameterType.PARAMETER_DOUBLE:
                    actual.append(value.double_value)
                else:
                    raise ValueError(f"{target}: required baseline parameter missing/wrong type")
            if actual != list(payload.values()):
                raise ValueError(f"{target}: runtime baseline differs from selected Nav2 file")
            self.current[target] = payload
            self.startup += 1
        else:
            if not response.result.successful:
                raise ValueError(f"{target} rejected profile: {response.result.reason}")
            self.current[target] = payload
            self.state_token = None

    def tick(self):
        now_ns, steady = self.get_clock().now().nanoseconds, time.monotonic()
        if now_ns < self.last_clock_ns:
            self.selection.start_ns = max(self.selection.start_ns, self.last_clock_ns)
        self.last_clock_ns = max(self.last_clock_ns, now_ns)
        self.selection.choose(now_ns, steady)
        try:
            if not self.error:
                if self.future is not None:
                    if self.future.done():
                        self.finish()
                    elif steady - self.sent_at > 2.0:
                        raise ValueError("profile RPC timeout; outcome unconfirmed, restart selector")
                if self.future is None:
                    self.advance()
        except Exception as error:
            # Never report an unconfirmed/partial application as an applied profile.
            # This control-plane failure does not grant/revoke chassis authority.
            self.error = str(error)
            self.applied_profile = "unconfirmed"
            self.get_logger().error(self.error)
        if steady - self.last_diagnostic >= 0.1:
            self.diagnose()
            self.last_diagnostic = steady

    def advance(self):
        if self.startup < 4:
            target = "smoother" if self.startup < 2 else "controller"
            kind = "state" if self.startup % 2 == 0 else "get"
            self.request(target, kind)
            return
        profile = self.profiles[self.selection.selected]
        for target in ("smoother", "controller"):
            payload = self.payload(target, profile)
            if self.current[target] != payload:
                if self.state_token != (target, self.selection.generation):
                    self.request(target, "state")
                else:
                    self.request(target, "set", payload)
                return
        self.applied_profile = self.selection.selected
        self.applied_generation = self.selection.generation

    def diagnose(self):
        status = DiagnosticStatus()
        status.name = "posture_motion"
        status.hardware_id = "upper_profile_selector"
        status.level = status.ERROR if self.error else (
            status.OK if self.applied_generation == self.selection.generation else status.WARN)
        status.message = self.error or self.selection.reason or "effective_posture"
        values = {
            "current_effective_posture": self.selection.posture.name,
            "posture_freshness": ("STALE" if self.selection.posture.name == "STALE" else
                                  "UNKNOWN" if self.selection.posture.value < 0 else "VALID"),
            "selected_profile": self.selection.selected,
            "applied_profile": self.applied_profile,
            "profile_generation": self.selection.generation,
            "applied_generation": self.applied_generation,
            "profile_revision": self.revision,
            "fallback_reason": self.selection.reason,
            "parameter_transactions": self.parameter_transactions,
            "source_mode": self.source_mode,
            "source_topic": self.topic,
            "backend_error": self.error,
        }
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PostureProfileSelector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble may raise a context-invalid wait-set error during SIGTERM.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
