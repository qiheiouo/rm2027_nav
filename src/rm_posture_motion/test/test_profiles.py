from pathlib import Path

import pytest
from rm_competition_interfaces.msg import RobotPosture
from rm_posture_motion.profiles import (
    EffectivePosture, MotionProfile, PostureSelection, SLOTS, load_profiles,
)

SOURCE = Path(__file__).resolve().parents[1]
BASELINE = SOURCE.parent / "rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml"
PRODUCTION = SOURCE / "config/profiles.yaml"
TEST = SOURCE / "test/fixtures/profiles_test_only.yaml"


@pytest.mark.parametrize("wire,logical", [
    ("MOVE", "MOBILE"), ("ATTACK", "ATTACK"), ("DEFENSE", "DEFENSE"),
    ("ENHANCED_MOVE", "ENHANCED_MOBILE"), ("ENHANCED_ATTACK", "ENHANCED_ATTACK"),
    ("ENHANCED_DEFENSE", "ENHANCED_DEFENSE"),
])
def test_six_native_enum_mappings(wire, logical):
    assert EffectivePosture(getattr(RobotPosture, wire)).name == logical
    assert len({p.value for p in EffectivePosture if p.value >= 0}) == 6


def test_production_inherits_exact_baseline_without_collapsing_slots():
    baseline, profiles, revision = load_profiles(PRODUCTION, BASELINE)
    assert tuple(profiles) == SLOTS
    assert all(profile == baseline for profile in profiles.values())
    assert baseline.max_velocity == (0.5, 0.5, 1.2)
    assert baseline.min_velocity == (-0.3, -0.5, -1.2)
    assert baseline.max_accel == (0.8, 0.8, 2.0)
    assert baseline.max_decel == (-0.8, -0.8, -2.0)
    assert len(revision.split(":")[-1]) == 12
    selected = PostureSelection(1, debounce=0)
    names = []
    for i, posture in enumerate([1, 5, 0, 3, 2, 4]):
        selected.observe(posture, (i + 1) * 10**9, True, True, False, False,
                         (i + 1) * 10**9, float(i))
        names.append(selected.selected)
    assert len(set(names)) == 6 and selected.generation == 7


def test_test_profiles_are_distinct_and_not_loadable_in_competition():
    with pytest.raises(ValueError, match="TEST ONLY"):
        load_profiles(TEST, BASELINE)
    baseline, profiles, _ = load_profiles(TEST, BASELINE, test_mode=True)
    assert len({profiles[name] for name in SLOTS[:-1]}) == 6
    assert all(p.max_velocity[0] <= baseline.max_velocity[0] for p in profiles.values())


def test_freshness_debounce_repetition_loss_and_invalid_enum():
    state = PostureSelection(1)
    assert state.choose(10**9, 1.0) == "fallback"
    state.observe(5, 10**9, True, True, False, False, 10**9, 1.0)
    assert state.choose(1060000000, 1.06) == "enhanced_mobile"
    generation = state.generation
    for i in range(1, 41):
        now = 10**9 + i * 25000000
        state.observe(5, now, True, True, False, False, now, 1.0 + i * 0.025)
    assert state.generation == generation
    assert state.choose(2600000000, 2.6) == "fallback"
    assert state.posture == EffectivePosture.STALE
    state.observe(255, 2700000000, True, True, False, False, 2700000000, 2.7)
    assert state.selected == "fallback" and state.reason == "unknown_enum"


@pytest.mark.parametrize("valid,online,transitioning,fault", [
    (False, True, False, False), (True, False, False, False),
    (True, True, True, False), (True, True, False, True),
])
def test_untrusted_actual_state_falls_back(valid, online, transitioning, fault):
    state = PostureSelection(1, debounce=0)
    state.observe(5, 10**9, True, True, False, False, 10**9, 1.0)
    state.observe(5, 1100000000, valid, online, transitioning, fault, 1100000000, 1.1)
    assert state.selected == "fallback"


def test_replayed_source_timestamp_and_zero_stamp_cannot_renew():
    state = PostureSelection(1, debounce=0)
    state.observe(5, 10**9, True, True, False, False, 10**9, 1.0)
    state.observe(5, 10**9, True, True, False, False, 10**9, 1.6)
    assert state.selected == "fallback"
    empty = PostureSelection(1, debounce=0)
    empty.observe(5, 0, True, True, False, False, 10**9, 1.0)
    assert empty.selected == "fallback"


def test_invalid_limits_rejected_before_parameter_rpc():
    values = {"max_velocity": [0.1]*3, "min_velocity": [-0.1]*3,
              "max_accel": [0.2]*3, "max_decel": [-0.2]*3}
    values["max_accel"][1] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        MotionProfile.parse(values)
