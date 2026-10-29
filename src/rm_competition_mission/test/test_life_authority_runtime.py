"""Life authority at the normalized RefereeState boundary; no firmware claims."""
import time

import pytest
from rclpy.duration import Duration
from rm_competition_interfaces.msg import PursuitCandidate, RefereeState, V2MotionGuardState

from test_mission_system_runtime import system
from test_navigation_arbitration_runtime import wait, wait_goal, stable_count


def detail(n, key):
    return dict(part.strip().split('=', 1) for part in n.state.detail.split(';')[1:])[key]


def life(n, value):
    wait(lambda: detail(n, 'life_state') == value, 'life ' + value)
    if value == 'dead':
        assert detail(n, 'navigation_authority') == '0'
        assert n.state.active_branch == 'hold'


def packet(n, hp):
    m = RefereeState(); m.header.stamp = n.get_clock().now().to_msg()
    m.valid = True; m.game_progress = 4; m.current_hp = hp
    return m


@pytest.mark.parametrize('mode,x', [('patrol', 1.), ('pursuit', 3.5), ('home', 0.)])
def test_dead_revokes_each_branch(system, mode, x):
    n = system; n.track = ('initial', 5., True)
    n.mode(mode); wait_goal(n, x)
    n.cancel_delay = .4; n.hp = 0; life(n, 'dead')
    assert n.cancels or n.active  # Authority is revoked even while terminal is pending.
    wait(lambda: not n.active, 'dead terminal')
    before = len(n.requests); n.track = ('while_dead', 6., True)
    stable_count(n, .4); assert len(n.requests) == before == 1
    n.checkpoint('D1', 'D8', mode)


@pytest.mark.parametrize('terminal', ['success', 'failure'])
def test_revive_before_old_terminal(system, terminal):
    n = system; n.mode('patrol'); wait_goal(n, 1.)
    old_generation = int(detail(n, 'mission_generation'))
    n.cancel_delay = .6; n.late_success = terminal == 'success'; n.late_failure = terminal == 'failure'
    n.mode('home')
    wait(lambda: n.state.navigation_status.startswith('goal_canceling'), 'preemption pending')
    n.hp = 0; life(n, 'dead'); dead_generation = int(detail(n, 'life_generation'))
    n.hp = 100; life(n, 'alive')
    assert int(detail(n, 'life_generation')) == dead_generation + 1
    stable_count(n, .15)
    wait_goal(n, 0., 1)
    assert int(detail(n, 'mission_generation')) > old_generation
    wait(lambda: int(detail(n, 'stale_results_rejected')) >= 1, 'old life rejected')
    # Old success must not advance patrol; old failure must not spend new budget.
    before = len(n.goals); n.hp = 400; n.mode('patrol'); wait_goal(n, 1., before)
    n.checkpoint('D2', 'D3', 'D4', 'late_' + terminal)


def test_revive_requires_new_localization_and_fresh_target(system):
    n = system; n.mode('auto'); wait_goal(n, 1.)
    n.track = ('pursuit', 5., True); wait_goal(n, 3.5, 1)
    n.hp = 0; life(n, 'dead'); wait(lambda: not n.active, 'dead drained')
    n.track = ('seen_while_dead', 6., True); time.sleep(.1); n.track = None
    n.localization_valid = False; time.sleep(.1)
    n.hp = 400; life(n, 'alive'); stable_count(n, .2)
    assert n.state.active_branch == 'hold'
    # A valid heartbeat in the SAME session cannot release this revival gate.
    n.localization_valid = True; stable_count(n, .2)
    assert n.state.active_branch == 'hold'
    before = len(n.goals)
    n.started_at = n.get_clock().now().to_msg(); n.sequence = 0
    wait_goal(n, 1., before)
    before = len(n.goals); n.track = ('after_revival', 5., True); wait_goal(n, 3.5, before)
    n.checkpoint('D5', 'D9')


def test_repeated_and_out_of_order_life_packets(system):
    n = system; n.mode('hold'); n.publish_referee = False; time.sleep(.06)
    alive = packet(n, 400); n.ref_pub.publish(alive); time.sleep(.04)
    dead = packet(n, 0); n.ref_pub.publish(dead); life(n, 'dead')
    generation = int(detail(n, 'life_generation'))
    # Fresh-in-age but older ALIVE, equal-stamp conflicting ALIVE, and future data.
    n.ref_pub.publish(alive)
    dead.current_hp = 400; n.ref_pub.publish(dead)
    future = packet(n, 400); future.header.stamp.sec += 3600; n.ref_pub.publish(future)
    time.sleep(.08); assert detail(n, 'life_state') == 'dead'
    assert int(detail(n, 'life_generation')) == generation
    revived = packet(n, 400); n.ref_pub.publish(revived); life(n, 'alive')
    dead.current_hp = 0; n.ref_pub.publish(dead); time.sleep(.08)
    assert detail(n, 'life_state') == 'alive'
    assert int(detail(n, 'life_generation')) == generation + 1
    # Identical packets do not renew the alive lease indefinitely.
    n.mode('patrol'); wait_goal(n, 1.)
    for _ in range(15):
        n.ref_pub.publish(revived); time.sleep(.05)
    wait(lambda: not n.active, 'duplicate lease expires')
    assert n.state.active_branch == 'hold'
    assert int(detail(n, 'life_generation')) == generation + 1
    n.checkpoint('D11', 'D12', 'duplicate_does_not_renew_lease')


def test_revive_discards_target_even_before_its_deadline(system):
    n = system; n.mode('auto'); wait_goal(n, 1.)
    n.hp = 0; life(n, 'dead'); wait(lambda: not n.active, 'dead terminal')
    candidate = PursuitCandidate(); now = n.get_clock().now()
    candidate.header.stamp = now.to_msg(); candidate.header.frame_id = 'map'
    candidate.valid_until = (now + Duration(seconds=.45)).to_msg()
    candidate.valid = True; candidate.track_id = 'cached_dead_target'
    candidate.source = candidate.SOURCE_RADAR; candidate.pose.position.x = 4.5
    candidate.pose.orientation.w = 1.
    sent = time.monotonic(); n.candidate_pub.publish(candidate); time.sleep(.05)
    n.hp = 400; life(n, 'alive'); wait_goal(n, 1., 1)
    assert n.requests[1][0] - sent < .45, 'test must observe revival BEFORE target TTL expires'
    assert len(n.requests) == 2  # No intermediate pursuit commitment.
    n.checkpoint('D4', 'D9', 'unexpired_pre_revival_target_rejected')


def test_rapid_life_changes_and_deterministic_sequence(system):
    n = system; n.cancel_delay = .4; n.late_success = True
    n.mode('patrol'); wait_goal(n, 1.)
    n.track = ('long_pursuit', 5., True); n.mode('auto'); wait_goal(n, 3.5, 1)
    n.mode('home'); wait_goal(n, 0., 2)
    n.hp = 0; life(n, 'dead')
    n.hp = 400; life(n, 'alive')
    n.hp = 0; life(n, 'dead')
    wait(lambda: not n.active, 'rapid cycle old success')
    assert len(n.requests) == 3
    n.track = ('during_dead', 6., True); stable_count(n, .2); n.track = None
    n.hp = 400; life(n, 'alive'); wait_goal(n, 0., 3)
    n.localization_valid = False
    wait(lambda: not n.active and n.state.active_branch == 'hold', 'localization lost')
    n.mode('patrol'); before = len(n.goals)
    n.started_at = n.get_clock().now().to_msg(); n.sequence = 0; n.localization_valid = True
    wait_goal(n, 1., before)
    n.guard = V2MotionGuardState.BLOCKED; wait(lambda: not n.active, 'final Safety veto')
    assert len(n.requests) == len(n.terminals) == 5
    assert n.max_active == 1
    for state in n.states:
        if 'life_state=dead' in state['detail']:
            assert 'navigation_authority=0' in state['detail']
    n.checkpoint('D3', 'D4', 'D8', 'D10', 'deterministic_life_sequence')
