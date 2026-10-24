from rm_system_monitor.readiness import RequirementPolicy, evaluate_readiness


def test_navigation_and_mission_requirements_are_separate():
    policy = RequirementPolicy(require_referee=True, require_chassis_mode=True)
    navigation_ready, mission_ready, missing = evaluate_readiness(
        policy,
        {"lio", "obstacle_input", "global_localization", "nav2_action"},
    )
    assert navigation_ready is True
    assert mission_ready is False
    assert missing == ["referee_state", "chassis_authority"]


def test_all_required_inputs_pass():
    policy = RequirementPolicy(
        require_referee=True,
        require_chassis_mode=True,
        require_serial_transport=True,
    )
    available = {
        "lio",
        "obstacle_input",
        "global_localization",
        "nav2_action",
        "referee_state",
        "chassis_authority",
        "serial_transport",
    }
    assert evaluate_readiness(policy, available) == (True, True, [])


def test_odom_source_lease_rejects_replay_stale_future_and_clock_rollback():
    from rm_system_monitor.readiness import SourceLease
    lease = SourceLease(0.5)
    assert not lease.observe(0, 2_000_000_000, 1.)
    assert not lease.observe(1_000_000_000, 2_000_000_000, 1.)
    assert not lease.observe(3_000_000_000, 2_000_000_000, 1.)
    assert lease.observe(2_000_000_000, 2_000_000_000, 1.)
    assert lease.fresh(2_100_000_000, 1.1)
    assert not lease.observe(2_000_000_000, 2_100_000_000, 1.1)
    assert not lease.fresh(2_100_000_000, 1.1)
    assert lease.observe(2_200_000_000, 2_200_000_000, 1.2)
    assert not lease.fresh(2_200_000_000, 1.8)  # paused ROS clock cannot extend lease
    assert not lease.fresh(2_100_000_000, 1.3)
    assert not lease.observe(2_150_000_000, 2_150_000_000, 1.4)
    assert lease.observe(2_300_000_000, 2_300_000_000, 1.5)
    assert not lease.fresh(2_900_000_000, 1.6)
