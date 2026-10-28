#!/usr/bin/env python3
"""Ordering counterexample: real bridge/Mission, delayed software Nav2 terminal.

No hardware, no second localization publisher; the fake action holds cancellation
for two seconds to distinguish cancel ACK from relinquishing navigation authority.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rm_competition_interfaces.msg import MissionState, LocalizationBarrier, LocalizationStatus
from rm_competition_interfaces.srv import BeginRebase
from std_srvs.srv import Trigger
from test_correction_runtime import Fixture, ns


class OrderingFixture(Fixture):
    def __init__(self):
        super().__init__()
        self.begin = self.create_client(BeginRebase, '/localization/begin_rebase')
        self.cache = self.create_publisher(LocalizationBarrier, '/localization/map_cache_primed', 10)
        self.sleepers = []
        self.goals = []
        self.terminals = []
        self.active = set()
        self.cancels = []
        self.mission = None
        self.prime_enabled = True
        self.child_cancel=self.create_service(CancelGoal, '/test_child/_action/cancel_goal',
            lambda request,response: response)
        self.nav = ActionServer(self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self.cancel, callback_group=ReentrantCallbackGroup())
        self.create_subscription(MissionState, '/mission/state',
            lambda m: setattr(self, 'mission', m), 10)
        self.create_timer(.01, self.wake)

    def wake(self):
        # Unit fixture substitutes ONLY the costmap receipt. Physics tests use the real layer.
        if self.prime_enabled and self.validity and self.validity.transaction_phase == LocalizationStatus.PRIME_NEW_SESSION:
            message=LocalizationBarrier()
            message.header.stamp=self.get_clock().now().to_msg()
            message.producer_id=self.validity.producer_id
            message.started_at=self.validity.started_at
            self.cache.publish(message)
        for deadline, future in list(self.sleepers):
            if time.monotonic() >= deadline:
                self.sleepers.remove((deadline, future))
                future.set_result(None)

    async def delay(self, seconds):
        future = Future()
        self.sleepers.append((time.monotonic()+seconds, future))
        await future

    def cancel(self, _):
        self.cancels.append(time.monotonic())
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        index = len(self.goals)
        self.goals.append([time.monotonic(), handle.request.pose.pose.position.x])
        self.active.add(index)
        while not handle.is_cancel_requested:
            await self.delay(.01)
        await self.delay(2.)
        self.active.remove(index)
        self.terminals.append([time.monotonic(), index])
        # Deliberately late success: must never advance the old patrol waypoint.
        handle.succeed()
        return NavigateToPose.Result()


def run(output, expect_gap):
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(50+os.getpid()%100)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(output/'ros_log')
    rclpy.init(); n=OrderingFixture(); children=[]; logs=[]
    result={'passed': False, 'expected_gap': expect_gap}
    try:
        tree=str(Path(get_package_share_directory('rm_competition_mission'))/'trees/competition_default.xml')
        nodes=[
            ('rm_localization_adapters','lio_adapter',['input_health.enabled:=true','input_health.trust_raw_twist:=true']),
            ('rm_relocalization_bridge','global_pose_gate_node',[]),
            ('rm_relocalization_bridge','map_odom_from_global_pose',[
                'source_health_topic:=/localization/lio_source_status',
                'upstream_valid_topic:=/localization/amcl_backend_valid','correction_gate.enabled:=true',
                'rebase.enabled:='+str(not expect_gap).lower()]),
            ('rm_competition_mission','competition_mission_node',[
                'tree_xml:='+tree,'startup_enabled:=true','default_mode:=patrol',
                'rebase_child_actions:=[/test_child]',
                'require_referee_state:=false','require_game_running:=false','require_chassis_mode:=false',
                'patrol_waypoints:=[5.0, 0.0, 0.0, 7.0, 0.0, 0.0]'])]
        for package,exe,params in nodes:
            log=(output/f'{exe}.log').open('w');logs.append(log)
            cmd=[str(Path(get_package_prefix(package))/'lib'/package/exe),'--ros-args']
            for param in params:cmd+=['-p',param]
            children.append(subprocess.Popen(cmd, stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        n.until(lambda: n.validity and n.validity.valid and n.active, 10)
        old_epoch=[n.validity.started_at.sec,n.validity.started_at.nanosec]
        before=n.transform.transform.translation.x
        if expect_gap:
            n.call(n.prepare)
            n.offset=-15.
            n.call(n.reset_backend)
        else:
            future=n.prepare.call_async(Trigger.Request());n.until(future.done)
            assert not future.result().success, 'legacy prepare bypassed rebase barrier'
            n.offset=-15.;n.spin(.4)
            request=BeginRebase.Request(producer_id=n.validity.producer_id,
                expected_session=n.validity.started_at,candidate_stamp=n.backend.pose.header.stamp)
            future=n.begin.call_async(request);n.until(future.done)
            assert future.result().accepted,future.result().detail
            n.spin(.2)
            assert 0 in n.active and n.transform.transform.translation.x == before
            assert not n.validity.valid
            result['legacy_bypass_rejected']=True
            result['held_old_correction_until_terminal']=True
        n.until(lambda: n.transform and abs(n.transform.transform.translation.x+15)<.01, 6)
        commit=time.monotonic()
        result.update(old_correction=before, new_correction=n.transform.transform.translation.x,
                      commit_observed=commit, cancel_ack_times=n.cancels,
                      old_action_active_at_commit=0 in n.active,
                      old_epoch=old_epoch,
                      new_epoch=[n.validity.started_at.sec,n.validity.started_at.nanosec])
        n.until(lambda: len(n.goals)>=2, 6)
        result['terminal_times']=n.terminals
        result['replacement_after_terminal']=n.terminals[0][0] <= n.goals[1][0]
        result['late_success_did_not_advance_patrol']=n.goals[1][1] == 5.
        result['commit_before_terminal_sec']=n.terminals[0][0]-commit
        assert result['old_action_active_at_commit'] == expect_gap, result
        assert result['replacement_after_terminal'] and result['late_success_did_not_advance_patrol'],result
        if not expect_gap:
            first=copy.deepcopy(n.validity)
            n.offset=4.;n.spin(.35)
            request=BeginRebase.Request(producer_id=n.validity.producer_id,
                expected_session=n.validity.started_at,candidate_stamp=n.backend.pose.header.stamp)
            future=n.begin.call_async(request);n.until(future.done)
            assert future.result().accepted,future.result().detail
            n.spin(.1)
            second_epoch=ns(n.validity.started_at)
            assert second_epoch>ns(first.started_at)
            # A receipt for transaction 1 cannot authorize transaction 2.
            replay=n.create_publisher(LocalizationBarrier,'/localization/navigation_drained',10)
            old=LocalizationBarrier(producer_id=first.producer_id,started_at=first.started_at)
            old.header.stamp=n.get_clock().now().to_msg();replay.publish(old);n.spin(.25)
            assert 1 in n.active and abs(n.transform.transform.translation.x+15)<.01
            busy=BeginRebase.Request(producer_id=n.validity.producer_id,
                expected_session=n.validity.started_at,candidate_stamp=n.backend.pose.header.stamp)
            future=n.begin.call_async(busy);n.until(future.done);assert not future.result().accepted
            n.until(lambda:n.validity.valid and len(n.goals)>=3,6)
            assert abs(n.transform.transform.translation.x-4)<.01
            assert n.goals[2][1]==5. and n.terminals[1][0]<=n.goals[2][0]
            result['ordered_repeat_old_receipt_and_busy_request_rejected']=True
            # No verified/fresh observation => no transaction or epoch mutation.
            n.emit_global=False;n.spin(.7);epoch=ns(n.validity.started_at)
            bad=BeginRebase.Request(producer_id=n.validity.producer_id,
                expected_session=n.validity.started_at,candidate_stamp=n.backend.pose.header.stamp)
            future=n.begin.call_async(bad);n.until(future.done)
            assert not future.result().accepted and ns(n.validity.started_at)==epoch
            result['stale_candidate_does_not_start_transaction']=True
            n.emit_global=True;n.until(lambda:n.validity.valid and bool(n.active),6)
            n.prime_enabled=False;n.offset=6.;n.spin(.35)
            request=BeginRebase.Request(producer_id=n.validity.producer_id,
                expected_session=n.validity.started_at,candidate_stamp=n.backend.pose.header.stamp)
            future=n.begin.call_async(request);n.until(future.done);assert future.result().accepted
            n.offset=6.39
            n.until(lambda:n.validity.transaction_phase==LocalizationStatus.PRIME_NEW_SESSION,6)
            n.spin(.08);assert abs(n.transform.transform.translation.x-6.39)<.01
            n.offset=5.61;n.spin(.25);n.prime_enabled=True
            n.until(lambda:n.validity.transaction_phase==LocalizationStatus.REBASE_FAILED,5)
            assert not n.validity.valid and abs(n.transform.transform.translation.x-6.39)<.01
            n.spin(.3);assert not n.validity.valid
            result['prime_cannot_smuggle_second_large_correction']=True

        result['passed']=True
    except Exception as error:
        result['error']=repr(error)
    finally:
        for child in children:child.send_signal(signal.SIGINT)
        exits=[]
        for child in children:
            try:exits.append(child.wait(timeout=8))
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL);child.wait();exits.append('timeout')
        for log in logs:log.close()
        result['shutdown_exit_codes']=exits
        result['passed'] &= all(code == 0 for code in exits)
        n.destroy_node();rclpy.try_shutdown()
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
    return result['passed']


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--expect-gap',action='store_true')
    args=parser.parse_args()
    raise SystemExit(0 if run(args.output,args.expect_gap) else 1)
