"""Real Mission/planner through production composition, delayed fake Nav2."""
import copy
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rm_competition_interfaces.msg import V2MotionGuardState
from rm_competition_interfaces.srv import SetTemporaryMission
import yaml

from test_navigation_arbitration_runtime import ArbitrationFixture, stable_count, wait, wait_goal


SOURCE = Path(__file__).resolve().parents[1]


def test_minimal_mission_s1_to_s14():
    os.environ["ROS_DOMAIN_ID"] = str(110 + os.getpid() % 30)
    rclpy.init()
    node = ArbitrationFixture()
    node.temporary_client = node.create_client(SetTemporaryMission, "/mission/set_temporary")
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes = []
    sequence = 0
    with tempfile.TemporaryDirectory(prefix="rm2027_mission_") as temporary:
        config = Path(temporary) / "mission.yaml"
        parameters = {"default_mode": "auto", "allow_pursuit": True, "tick_rate_hz": 20.0,
                      "home_pose": [0.0, 0.0, 0.0], "supply_pose": [3.0, 0.0, 0.0],
                      "recovery_pose": [4.0, 0.0, 0.0],
                      "patrol_waypoints": [1.0, 0.0, 0.0, 2.0, 0.0, 0.0]}
        log = open(Path(temporary) / "mission.log", "w+")
        launch = SOURCE.parent / "rm_navigation_bringup/launch/competition_2027_navigation.launch.py"

        def start():
            config.write_text(yaml.safe_dump({"competition_mission_node": {"ros__parameters": parameters}}))
            process = subprocess.Popen(["ros2", "launch", str(launch),
                                        "enable_navigation_intents:=true", f"mission_config:={config}"],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            wait(lambda: node.state is not None and not node.state.enabled, "mission startup")
            assert node.temporary_client.wait_for_service(timeout_sec=5.0)
            return process

        def request(kind, mission_id, seconds=10.0, x=6.0, cancel=False):
            nonlocal sequence
            sequence += 1
            command = SetTemporaryMission.Request(sequence=sequence, kind=kind,
                                                  mission_id=mission_id, cancel=cancel)
            command.valid_until = (node.get_clock().now() + Duration(seconds=seconds)).to_msg()
            if kind == "special":
                command.target.header.frame_id = "map"
                command.target.pose.position.x = x
                command.target.pose.orientation.w = 1.0
            return command

        def send(command, accepted=True):
            future = node.temporary_client.call_async(command)
            wait(future.done, "temporary service response")
            response = future.result()
            assert response.accepted == accepted, response.message
            return response

        def arrive():
            index = len(node.goals) - 1
            node.complete.add(index)
            return len(node.goals)

        def temporary(kind, mission_id, seconds=10.0, x=6.0):
            before = len(node.goals)
            command = request(kind, mission_id, seconds, x)
            send(command)
            expected = {"supply": 3.0, "recovery": 4.0, "special": x}[kind]
            wait_goal(node, expected, before)
            return command

        try:
            process = start()
            assert not node.requests
            node.mode("auto")
            wait_goal(node, 1.0)
            wait_goal(node, 2.0, arrive())
            print("S1 PASS: idle -> patrol; current index=1")

            for number, kind in [(2, "supply"), (3, "recovery"), (4, "special")]:
                command = temporary(kind, f"arrival_{kind}")
                wait_goal(node, 2.0, arrive())
                assert f"{kind}:{command.mission_id}:completed" in node.state.detail
                stable_count(node, 0.15)
                before = len(node.goals)
                send(command)  # A retransmission after completion cannot resurrect it.
                stable_count(node, 0.15)
                assert len(node.goals) == before
                print(f"S{number} PASS: patrol -> {kind} -> completed -> same patrol index")

            for kind in ("supply", "recovery", "special"):
                node.track = ("radar_alpha", 5.0, True)
                wait_goal(node, 3.5, len(node.goals))
                command = temporary(kind, f"over_radar_{kind}")
                stable_count(node, 0.15)
                assert node.state.active_branch == kind
                node.track = None
                send(request(kind, command.mission_id, cancel=True))
                wait_goal(node, 2.0, len(node.goals))
            print("S5 PASS: each explicit mission deterministically preempts radar pursuit")

            for number, kind in [(6, "supply"), (7, "recovery"), (8, "special")]:
                command = temporary(kind, f"home_preempts_{kind}")
                before = len(node.goals)
                if kind == "recovery":
                    node.mode("home")
                else:
                    node.hp = 100
                wait_goal(node, 0.0, before)
                send(request("supply", f"rejected_during_home_{kind}"), accepted=False)
                arrive()
                wait(lambda: not node.active, "home arrival")
                node.hp = 400
                stable_count(node, 0.25)
                assert node.state.active_branch == "home"
                node.mode("auto")  # Only an explicit mode change releases persistent home.
                wait_goal(node, 2.0, len(node.goals))
                print(f"S{number} PASS: {kind} -> home; arrival holds, explicit release resumes patrol")

            command = temporary("special", "cancel_me")
            send(request("special", command.mission_id, cancel=True))
            wait_goal(node, 2.0, len(node.goals))
            print("S9 PASS: cancel temporary -> resume valid long-term patrol")

            command = temporary("supply", "expire_me", seconds=0.8)
            wait_goal(node, 2.0, len(node.goals))
            assert ":expired" in node.state.detail
            send(command)
            stable_count(node, 0.2)
            print("S10 PASS: expiration resumes patrol; old command cannot resurrect expired goal")

            command = temporary("supply", "denied_then_expires", seconds=1.0)
            node.guard = V2MotionGuardState.BLOCKED
            wait(lambda: not node.active, "authority cancel")
            stable_count(node, 0.3)
            print("S11 PASS: denied -> terminal cancellation, zero replacement while denied")
            time.sleep(0.8)
            replacement = request("recovery", "valid_while_denied", seconds=4.0)
            send(replacement)
            before = len(node.requests)
            stable_count(node, 0.15)
            node.guard = V2MotionGuardState.PERMITTED
            wait_goal(node, 4.0, len(node.goals))
            assert all(x != 3.0 for _, x in node.requests[before:])
            wait_goal(node, 2.0, arrive())
            print("S12 PASS: restored authority recomputes valid recovery, never expired supply")

            command = temporary("special", "no_churn", seconds=5.0)
            count, cancels = len(node.requests), len(node.cancels)
            futures = []
            for _ in range(30):
                futures.append(node.temporary_client.call_async(command))
                time.sleep(0.025)
            # Even a changed transport sequence must not mutate/renew the same logical task.
            for _ in range(10):
                sequence += 1
                duplicate = copy.deepcopy(command)
                duplicate.sequence = sequence
                futures.append(node.temporary_client.call_async(duplicate))
                time.sleep(0.025)
            wait(lambda: all(f.done() for f in futures), "repeat command responses")
            assert all(f.result().accepted for f in futures)
            assert len(node.requests) == count and len(node.cancels) == cancels
            mutated = copy.deepcopy(command)
            sequence += 1
            mutated.sequence = sequence
            mutated.target.pose.position.x = 7.0
            send(mutated, accepted=False)
            send(request("special", command.mission_id, cancel=True))
            wait_goal(node, 2.0, len(node.goals))
            print("S14 PASS: 40Hz identical mission command -> zero cancel/resend; mutation rejected")

            # Same priority uses latest valid different mission, with no task stack.
            temporary("supply", "replace_old")
            temporary("recovery", "replace_new")
            wait_goal(node, 2.0, arrive())
            stable_count(node, 0.2)
            print("REPLACEMENT PASS: latest valid explicit task wins; replaced task is never resumed")

            # Restart with no supply/recovery coordinates and keep current long-term goal.
            node.mode("hold")
            wait(lambda: not node.active, "hold before restart")
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
            parameters.pop("supply_pose")
            parameters.pop("recovery_pose")
            node.state = None
            process = start()
            node.mode("auto")
            wait_goal(node, 1.0, len(node.goals))
            before = len(node.requests)
            send(request("supply", "missing_supply"), accepted=False)
            send(request("recovery", "missing_recovery"), accepted=False)
            malformed = request("special", "bad_frame")
            malformed.target.header.frame_id = "odom"
            send(malformed, accepted=False)
            send(request("special", "past_deadline", seconds=-1.0), accepted=False)
            stable_count(node, 0.25)
            assert len(node.requests) == before
            print("S13 PASS: missing config/bad frame/expired request rejected; no unsafe substitute goal")

            assert node.max_active == 1
            for index in range(len(node.goals) - 1):
                terminal = next(event[0] for event in node.terminals if event[1] == index)
                assert terminal <= node.requests[index + 1][0]
            print(f"OWNER PASS: max active=1; {len(node.goals)} goals, "
                  f"{len(node.cancels)} cancellations; every replacement follows terminal result")
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2.0)
            log.seek(0)
            print(log.read())
            log.close()
            node.complete.update(range(len(node.goals)))
            wait(lambda: not node.active, "fixture cleanup")
            executor.shutdown(timeout_sec=3.0)
            node.destroy_node()
            rclpy.shutdown()
