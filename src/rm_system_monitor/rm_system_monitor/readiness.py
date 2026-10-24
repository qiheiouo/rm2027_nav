from dataclasses import dataclass
from typing import List, Set, Tuple


@dataclass(frozen=True)
class RequirementPolicy:
    require_lio: bool = True
    require_obstacle_input: bool = True
    require_localization: bool = True
    require_nav2: bool = True
    require_referee: bool = False
    require_chassis_mode: bool = False
    require_serial_transport: bool = False


def evaluate_readiness(
    policy: RequirementPolicy, available: Set[str]
) -> Tuple[bool, bool, List[str]]:
    navigation_requirements = (
        (policy.require_lio, "lio"),
        (policy.require_obstacle_input, "obstacle_input"),
        (policy.require_localization, "global_localization"),
        (policy.require_nav2, "nav2_action"),
    )
    mission_requirements = (
        (policy.require_referee, "referee_state"),
        (policy.require_chassis_mode, "chassis_authority"),
        (policy.require_serial_transport, "serial_transport"),
    )
    missing_navigation = [
        name for required, name in navigation_requirements if required and name not in available
    ]
    missing_mission = [
        name for required, name in mission_requirements if required and name not in available
    ]
    missing = missing_navigation + missing_mission
    return not missing_navigation, not missing, missing


class SourceLease:
    """Require increasing source stamps and bounded ROS + steady ages."""

    def __init__(self, timeout_sec):
        self.timeout_ns = int(timeout_sec * 1_000_000_000)
        self.source_ns = 0
        self.received = None
        self.last_clock_ns = 0

    def _clock_valid(self, now_ns):
        if now_ns < self.last_clock_ns:
            self.received = None
            return False
        self.last_clock_ns = now_ns
        return True

    def observe(self, source_ns, now_ns, steady_now):
        if (not self._clock_valid(now_ns) or source_ns <= self.source_ns
                or source_ns <= 0 or not 0 <= now_ns - source_ns <= self.timeout_ns):
            self.received = None
            return False
        self.source_ns, self.received = source_ns, steady_now
        return True

    def fresh(self, now_ns, steady_now):
        return (self._clock_valid(now_ns) and self.received is not None
                and 0 <= now_ns - self.source_ns <= self.timeout_ns
                and 0 <= steady_now - self.received <= self.timeout_ns * 1e-9)
