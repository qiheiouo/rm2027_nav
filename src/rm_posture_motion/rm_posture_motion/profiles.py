"""Six independent profile slots; no posture decision or power model."""
from dataclasses import dataclass
from enum import IntEnum
import hashlib
import math
from pathlib import Path
import yaml


class EffectivePosture(IntEnum):
    UNKNOWN = -1
    STALE = -2
    ATTACK = 0
    MOBILE = 1                 # Existing wire/ROS RobotPosture.MOVE, unchanged.
    DEFENSE = 2
    ENHANCED_ATTACK = 3
    ENHANCED_DEFENSE = 4
    ENHANCED_MOBILE = 5        # Existing ENHANCED_MOVE, unchanged.


SLOTS = ("mobile", "attack", "defense", "enhanced_mobile", "enhanced_attack",
         "enhanced_defense", "fallback")
LIMITS = ("max_velocity", "min_velocity", "max_accel", "max_decel")
MPPI_KEYS = ("vx_max", "vx_min", "vy_max", "wz_max")


@dataclass(frozen=True)
class MotionProfile:
    max_velocity: tuple
    min_velocity: tuple
    max_accel: tuple
    max_decel: tuple

    @classmethod
    def parse(cls, values):
        arrays = [tuple(float(v) for v in values[key]) for key in LIMITS]
        if any(len(a) != 3 or not all(math.isfinite(v) for v in a) for a in arrays):
            raise ValueError("motion arrays must each contain three finite values")
        maximum, minimum, accel, decel = arrays
        if any(maximum[i] <= 0 or minimum[i] >= 0 or accel[i] <= 0 or decel[i] >= 0
               for i in range(3)):
            raise ValueError("invalid velocity/acceleration/deceleration signs")
        if any(not math.isclose(minimum[i], -maximum[i]) for i in (1, 2)):
            raise ValueError("MPPI requires symmetric lateral and angular bounds")
        return cls(*arrays)

    def smoother(self):
        return {key: list(getattr(self, key)) for key in LIMITS}

    def mppi(self, plugin):
        values = (self.max_velocity[0], self.min_velocity[0],
                  self.max_velocity[1], self.max_velocity[2])
        return {f"{plugin}.{key}": value for key, value in zip(MPPI_KEYS, values)}


def load_profiles(profiles_path, baseline_path, plugin="FollowPath", test_mode=False):
    raw = Path(profiles_path).read_bytes()
    baseline_raw = Path(baseline_path).read_bytes()
    document = yaml.safe_load(raw)
    nav = yaml.safe_load(baseline_raw)
    controller = nav["controller_server"]["ros__parameters"][plugin]
    if controller["plugin"] != "nav2_mppi_controller::MPPIController":
        raise ValueError("this bounded profile adapter supports MPPI only")
    baseline = MotionProfile.parse(nav["velocity_smoother"]["ros__parameters"])
    expected = baseline.mppi("")
    if any(not math.isclose(float(controller[key]), expected[f".{key}"])
           for key in MPPI_KEYS):
        raise ValueError("baseline MPPI/smoother velocity limits disagree")
    if document["kind"] == "test":
        if not test_mode:
            raise ValueError("TEST ONLY profiles are forbidden in competition mode")
    elif document["kind"] != "production" or document["calibration"] != "HARDWARE_TBD":
        raise ValueError("unsupported production calibration contract")
    if set(document["profiles"]) != set(SLOTS):
        raise ValueError("all six independent posture slots and fallback are required")
    profiles = {}
    for name in SLOTS:
        entry = document["profiles"][name]
        if entry == {"inherit": "baseline"}:
            profiles[name] = baseline
        else:
            if document["kind"] == "production":
                raise ValueError("uncalibrated production profiles must inherit baseline")
            profiles[name] = MotionProfile.parse(entry)
        value = profiles[name]
        for key in LIMITS:
            if any(abs(v) > abs(b) + 1e-9
                   for v, b in zip(getattr(value, key), getattr(baseline, key))):
                raise ValueError("profiles may not exceed the selected baseline")
    revision = document["revision"] + ":" + hashlib.sha256(raw + baseline_raw).hexdigest()[:12]
    return baseline, profiles, revision


class PostureSelection:
    def __init__(self, start_ns, timeout=0.5, debounce=0.05):
        self.start_ns = start_ns
        self.timeout = timeout
        self.debounce = debounce
        self.received = False
        self.last_stamp = 0
        self.last_progress = 0.0
        self.posture = EffectivePosture.UNKNOWN
        self.reason = "no_posture"
        self.candidate = "fallback"
        self.candidate_since = 0.0
        self.selected = "fallback"
        self.generation = 1

    def observe(self, actual, stamp_ns, valid, online, transitioning, fault, now_ns, steady):
        # ACK/target/completed deliberately play no part in effective-state selection.
        self.received = True
        if stamp_ns < self.last_stamp:
            return
        if stamp_ns > self.last_stamp:
            self.last_progress = steady
        self.last_stamp = stamp_ns
        self.posture = EffectivePosture.UNKNOWN
        self.reason = "invalid_state"
        if valid and online and not transitioning and not fault:
            try:
                self.posture = EffectivePosture(actual)
                if self.posture.value < 0:
                    self.posture = EffectivePosture.UNKNOWN
            except ValueError:
                pass
            self.reason = "unknown_enum" if self.posture == EffectivePosture.UNKNOWN else ""
        return self.choose(now_ns, steady)

    def choose(self, now_ns, steady):
        age = (now_ns - self.last_stamp) * 1e-9
        if (self.received and
                (self.last_stamp <= self.start_ns or age < -0.05 or age > self.timeout
                 or steady - self.last_progress > self.timeout)):
            self.posture = EffectivePosture.STALE
            self.reason = "stale_source"
        candidate = self.posture.name.lower() if self.posture.value >= 0 else "fallback"
        if candidate != self.candidate:
            self.candidate = candidate
            self.candidate_since = steady
        if candidate == "fallback" or steady - self.candidate_since >= self.debounce:
            if self.selected != candidate:
                self.selected = candidate
                self.generation += 1
        return self.selected
