"""Fixture/full-episode experiment. Discrete-event execution during planner lateness.

Only evaluation uses scene truth. Both consumers receive measured state, the real
immutable queue and the same source-age-corrected ideal CV observations as run.py.
"""
import argparse
import copy
import gzip
import json
import math
import os
from pathlib import Path
import time
from run import Consumer, hold_ttc
from world import advance, actors_at, clearance, observed, static_grid, stats

ROOT = Path(__file__).resolve().parent
FIXTURE = json.loads((ROOT / "results/failure_fixture.json").read_text())
START = json.loads((ROOT / "results/failure_diagnosis.json").read_text())["first_no_new_candidate_epoch"]
PROTOCOL = json.loads((ROOT / "reserve_protocol.json").read_text())


def at(events, stamp, initial):
    value = list(initial)
    for event in events:
        if event[0] <= stamp + 1e-8:
            value = list(event[1:3])
    return value


def question(state, target, queue, epoch, actors, version, slot, wall=None):
    return dict(state=state, target=target, queue=queue, epoch=epoch,
                actors=actors, wall=FIXTURE["wall"] if wall is None else wall, queue_version=version,
                source_epoch=epoch-.1, source_version=slot, dynamics=[.01,.5,.8,.355,.330])


def compact(backup, epoch):
    if backup is None:
        return None
    return {k: backup[k] for k in ("id", "epoch", "start_state", "immutable_queue_version", "immutable_queue",
                                  "controls", "backup_controls", "switch_time", "safe_rest", "assumptions", "prediction")} | {
        "execution_cursor": sum(e[0] <= epoch+1e-8 for e in backup["controls"]),
        "lineage": backup.get("lineage", backup["id"]), "acquired_epoch": backup.get("acquired_epoch", backup["epoch"]),
        "prediction_version": backup["prediction"]["version"]}


def sanity(build, out):
    """Protect baseline parity, numeric certificate checks and source/queue invalidation."""
    old = Consumer(build/"search_baseline", "search", out/"parity_old.log")
    current = Consumer(build/"search_consumer", "search", out/"parity_current.log")
    checker = Consumer(build/"reserve_consumer", "reserve", out/"sanity_reserve.log")
    try:
        q = {k: FIXTURE[k] for k in ("mode", "state", "target", "queue", "actors", "wall", "previous")}
        q["grid"], _ = static_grid(FIXTURE["scene"])
        a, b = old.call(q), current.call(q)
        for key in ("command", "verified", "reason", "prefix_safe", "viable", "expanded", "schedule"):
            assert a[key] == b[key], key
        assert b["reason"] == "reused" and b["viable"] == 0
        rq = question(FIXTURE["state"], FIXTURE["target"],
                      [[START+e[0], *e[1:]] for e in FIXTURE["queue"]], START, FIXTURE["actors"], 0, 0)
        cert = checker.call(rq | {"op": "init"})["certificate"]
        if cert is None:
            # Unavailable is a valid scientific outcome, not a failed test to bypass.
            state = rq["state"].copy()
            for _ in range(10): state = advance(state, rq["target"])
            archived = current.call(dict(recourse_probe=True, state=state, epoch=.1,
                                         actors=rq["actors"], wall=rq["wall"]))
            assert not archived["backup_exists"]
            library = []
            from world import cap
            for direction in [None, [-1,-1], [-1,0], [-1,1], [0,-1], [0,1], [1,-1], [1,0], [1,1]]:
                s = state.copy(); tau = .1
                if direction is not None:
                    for _ in range(140): s = advance(s, cap(direction, .5)); tau += .01
                for _ in range(math.ceil(math.hypot(*s[2:])/.8/.01)+30):
                    s = advance(s, [0,0]); tau += .01
                probe = current.call(dict(recourse_probe=True, state=s, epoch=tau,
                                          actors=rq["actors"], wall=rq["wall"]))
                # Existing head-on fixture only: compute closest approach on its CV ray,
                # using observed actor state, never the scene's future evaluator.
                actor = rq["actors"][0]
                closest_tau = max(tau, (s[0]-actor[0])/actor[2])
                cv_actor = [[actor[0]+actor[2]*closest_tau, actor[1]+actor[3]*closest_tau, *actor[2:]]]
                library.append(dict(direction=direction, rest_state=s, relative_rest_epoch=tau,
                                    stationary_refuge=probe["stationary_refuge"],
                                    cv_rest_physical_clearance_m=clearance(s, cv_actor, rq["wall"])))
            result = dict(baseline_parity=True, initialization_available=False,
                          archived_library_also_unavailable=True, library=library,
                          note="Persistence/invalidation execution tests not reached; no bootstrap motion authorized")
            (out/"sanity.json").write_text(json.dumps(result, indent=2)+"\n")
            print("sanity: unchanged proposal; new and archived library both unavailable", flush=True)
            return
        vq = rq | {"op": "validate", "backup": cert}
        assert checker.call(vq)["reason"] == "valid"
        tests = [("source_epoch", START-.2, "stale_observation"),
                 ("state", [rq["state"][0]+.01, *rq["state"][1:]], "state_deviation"),
                 ("actors", [], "obstacle_set_changed"),
                 ("queue", [[START+.05, 0, 0]], "immutable_queue_mismatch")]
        for key, value, reason in tests:
            assert checker.call(vq | {key: value})["reason"] == reason
        # Independently replay the exact certified commands through the common Python plant.
        state, target = rq["state"].copy(), rq["target"].copy()
        minimum = math.inf
        for k, expected in enumerate(cert["reference_states"][1:]):
            epoch = START+k*.01
            target = at(cert["controls"], epoch+.005, target)
            state = advance(state, target)
            assert max(abs(x-y) for x, y in zip(state, expected)) < 1e-10
            minimum = min(minimum, clearance(state, actors_at(FIXTURE["scene"], epoch+.01), FIXTURE["wall"]))
        result = dict(baseline_parity=True, initialization_available=True,
                      independent_rollout_min_clearance_m=minimum, invalidation_checks=len(tests),
                      certificate_rest=cert["safe_rest"])
        (out/"sanity.json").write_text(json.dumps(result, indent=2)+"\n")
        print("sanity", result, flush=True)
    finally:
        for c in (old, current, checker):
            c.close()


def trial(build, out, arm, condition, full_episode=False, cfg=None, observation_source=None):
    name = f"{arm}_{condition}"
    planner = Consumer(build/"search_baseline", "search", out/f"{name}_planner.log")
    checker = None if arm == "B" else Consumer(build/"reserve_consumer", "reserve", out/f"{name}_reserve.log")
    cpu0 = planner.usage()[0] + (checker.usage()[0] if checker else 0)
    own_cpu = time.process_time()
    if cfg is not None and not full_episode:
        raise ValueError("scenario selection requires an original full episode")
    cfg = FIXTURE["scene"] if cfg is None else cfg
    grid, _ = static_grid(cfg)
    def input_at(epoch):
        if observation_source is None:
            current, actors = observed(cfg, epoch)
            return current, actors, {}
        return observation_source.sample(epoch)
    def truth_at(epoch):
        return actors_at(cfg, epoch) if observation_source is None else observation_source.truth(epoch)
    start_epoch = 0. if full_episode else START
    state = [0.,0.,0.,0.] if full_episode else FIXTURE["state"].copy()
    target = [0.,0.] if full_episode else FIXTURE["target"].copy()
    queue = [] if full_episode else [[START+e[0], *e[1:]] for e in FIXTURE["queue"]]
    previous = [] if full_episode else [[START+e[0], *e[1:]] for e in FIXTURE["previous"]]
    backup = None; backup_id = 0; queue_version = 0
    epoch = start_epoch; slot = 0; busy_until = -math.inf; arrival = None; collision = False
    minimum = math.inf; worst = None; wait = 0.; stop_run = 0.; stops = 0
    calls = 0; intrinsic = []; response_lat = []; planner_lat = []; validator_lat = []; maintenance_lat = []
    injected = 0; intrinsic_misses = 0; observed_misses = 0; invalid = []; accepted = 0; rejected = 0
    active_exec = 0; late_exec = 0; timeout_brakes = 0; replacements = []; events = []
    sequence_rms = []; previous_schedule = None; flips = 0; last_sign = [0, 0]
    accelerations = []; jerks = []; last_acc = [0., 0.]; ttc_values = []
    initial = None
    lifecycle = []; owners = {}; owner = None; phase = None; held_cycles = 0
    revalidation_lat = []; acquisition_lat = []; total_cycle_lat = []
    def transition(state_name, reason, **extra):
        lifecycle.append(dict(t=epoch, state=state_name, reason=reason,
                              backup_id=backup["id"] if backup else None, **extra))
    try:
        if checker:
            if arm == "C1": transition("INITIALIZING", "episode_start" if full_episode else "fixture_start")
            _, initial_actors, input_metadata = input_at(epoch)
            rq = question(state, target, queue, epoch, initial_actors, queue_version, slot, cfg["wall"])
            if input_metadata:
                rq.update(source_epoch=input_metadata["source_epoch"], source_version=input_metadata["source_version"])
            t0 = time.perf_counter(); initial = checker.call(rq | {"op": "init"})
            init_ms = (time.perf_counter()-t0)*1000
            intrinsic.append(init_ms)
            if initial["available"] and init_ms <= 50:
                backup = initial["certificate"] if arm == "C1" else None
                if backup is not None:
                    backup["id"] = backup_id
                    backup["lineage"] = backup_id; backup["acquired_epoch"] = epoch
                    transition("HELD", "initial_certificate_acquired", certificate=compact(backup, epoch))
                    phase = "HELD"
            else:
                invalid.append({"t": epoch, "reason": "initialization_unavailable"})
                if arm == "C1":
                    transition("UNAVAILABLE", "initial_certificate_unavailable")
                    result = dict(arm=arm, condition=condition, status="initialization_unavailable",
                                  executed=False, collision=None, completion=False, minimum_clearance_m=None,
                                  traversal_s=None, arrival_s=None, initialization_ms=init_ms,
                                  calls=0, injected_calls=0, intrinsic_ms=stats([init_ms]),
                                  response_ms=stats([]), planner_ms=stats([]), backup_validation_ms=stats([init_ms]),
                                  intrinsic_deadline_misses=int(init_ms>50), response_deadline_misses=0,
                                  timeout_brakes=0, backup_dispatches=0, late_backup_dispatches=0,
                                  invalid=invalid, persistence_tested=False, lifecycle=lifecycle,
                                  note="No executable reserve at exact fixture start. No plant rollout or lateness exposure.")
                    (out/f"{name}.json").write_text(json.dumps(result, indent=2)+"\n")
                    print(name, "UNAVAILABLE; no rollout", flush=True)
                    return result
        with gzip.open(out/f"{name}.jsonl.gz", "wt") as trace:
            while (epoch < 40 if arrival is None else epoch < arrival+3-1e-8):
                begin = time.perf_counter()
                _, actors, input_metadata = input_at(epoch)
                rq = question(state, target, queue, epoch, actors, queue_version, slot, cfg["wall"])
                if input_metadata:
                    rq.update(source_epoch=input_metadata["source_epoch"], source_version=input_metadata["source_version"])
                validity_ms = 0.; validation_ms = 0.; validity = "not_held"
                lifecycle_start = len(lifecycle)
                if arm == "C1" and backup is not None:
                    vr = checker.call(rq | {"op": "validate", "backup": backup})
                    validity_ms = vr["roundtrip_ms"]; validity = vr["reason"]
                    if validity != "valid":
                        invalid.append({"t": epoch, "reason": validity, "id": backup["id"]})
                        transition("INVALID", validity)
                        backup = None
                    else:
                        held_cycles += 1
                        backup["execution_cursor"] = sum(e[0] <= epoch+1e-8 for e in backup["controls"])
                        transition("REVALIDATED", "measured_state_queue_and_CV_agree",
                                   cursor=backup["execution_cursor"], observation_version=slot)
                        actual_phase = "SAFE_REST" if vr["stationary_refuge"] else (
                            "EXECUTING_BACKUP" if owner and owner["backup"] else "HELD")
                        if actual_phase != phase:
                            transition(actual_phase, "current_refuge_certified" if actual_phase=="SAFE_REST" else "actual_command_owner",
                                       executed_owner=owner)
                            phase = actual_phase
                revalidation_lat.append(validity_ms)
                # Everything below prepares a transaction; the plant has not consumed it yet.
                old_backup, old_previous = backup, previous
                old_queue, old_queue_version = queue.copy(), queue_version
                proposal = None; candidate = None; reason = "planner_busy"
                response_ms = None; plan_ms = None; admitted = False
                available = epoch >= busy_until-1e-8
                if available:
                    q = dict(mode="search", state=state, target=target,
                             queue=[[e[0]-epoch, *e[1:]] for e in queue], actors=actors,
                             wall=cfg["wall"], grid=grid,
                             previous=[[e[0]-epoch, *e[1:]] for e in previous])
                    proposal = planner.call(q); plan_ms = proposal["roundtrip_ms"]
                    forced = condition == "late" and slot % 6 == 5
                    # Late output is never used for admission, even though its values can be logged.
                    if checker and not forced and plan_ms <= 50:
                        cr = checker.call(rq | {"op": "admit", "command": proposal["command"], "backup": backup})
                        validation_ms = cr["roundtrip_ms"]
                        candidate = cr["certificate"]; reason = cr["reason"]
                    elif arm == "B":
                        reason = "baseline"
                    compute_ms = (time.perf_counter()-begin)*1000
                    response_ms = max(compute_ms, 80. if forced else 0.)
                    calls += 1; injected += int(forced)
                    intrinsic.append(compute_ms); response_lat.append(response_ms)
                    planner_lat.append(plan_ms); validator_lat.append(validity_ms+validation_ms)
                    intrinsic_misses += int(compute_ms > 50)
                    observed_misses += int(response_ms > 50)
                    late = response_ms > 50
                    busy_until = epoch+response_ms/1000
                    if late:
                        reason = "planner_deadline_rejected"
                        if arm != "C1":
                            queue.append([epoch+response_ms/1000+.05, 0., 0.]); queue_version += 1
                            timeout_brakes += 1
                            previous = [[epoch+response_ms/1000+.05, 0., 0.]]
                    elif arm == "B" or candidate is not None:
                        admitted = True; accepted += 1
                        previous = [[epoch+e[0], *e[1:]] for e in proposal["schedule"]]
                        if arm == "C1":
                            old = compact(backup, epoch); backup_id += 1
                            backup = candidate; backup["id"] = backup_id
                            backup["lineage"] = old["lineage"] if old and reason=="splice_old" else backup_id
                            backup["acquired_epoch"] = old["acquired_epoch"] if old and reason=="splice_old" else epoch
                            replacements.append({"t": epoch, "old_switch": old["switch_time"] if old else None,
                                                 "new_switch": backup["switch_time"], "reason": reason})
                            transition("REPLACED", reason, parent_id=old["id"] if old else None,
                                       lineage=backup["lineage"], old_switch=old["switch_time"] if old else None,
                                       new_switch=backup["switch_time"])
                        else:
                            queue.append([epoch+.1, *proposal["command"]]); queue_version += 1
                    else:
                        rejected += 1
                        if arm == "C0":
                            queue.append([epoch+.1, 0., 0.]); queue_version += 1
                    if not admitted:
                        events.append({"t": epoch, "reason": reason, "backup_id": backup["id"] if backup else None})
                        if arm == "C1" and backup is not None:
                            transition("HELD", "proposal_rejected_keep_reserve", rejection=reason)
                else:
                    maintenance_lat.append((time.perf_counter()-begin)*1000)
                if arm == "C1":
                    if backup is not None:
                        command = at(backup["controls"], epoch+.1, backup["initial_target"])
                        from_backup = epoch+.1 >= backup["switch_time"]-1e-8
                        active_exec += int(from_backup); late_exec += int(from_backup and (not available or response_ms and response_ms > 50))
                    else:
                        command = [0., 0.]; from_backup = False
                    queue.append([epoch+.1, *command]); queue_version += 1
                    owners[round(epoch+.1,10)] = dict(backup=from_backup, id=backup["id"] if backup else None,
                                                    lineage=backup["lineage"] if backup else None)
                else:
                    from_backup = False
                queue.sort(key=lambda e: e[0])
                # Include transaction and command preparation, but not evaluator/logging cost.
                full_ms = (time.perf_counter()-begin)*1000
                if available:
                    intrinsic[-1] = full_ms
                    intrinsic_misses += int(full_ms>50)-int(compute_ms>50)
                    response_ms = max(full_ms, 80. if forced else 0.)
                    response_lat[-1] = response_ms
                    if admitted and full_ms>50:
                        # Roll back the still-private draft, never an already executed queue entry.
                        backup, previous = old_backup, old_previous
                        queue, queue_version = old_queue, old_queue_version
                        admitted = False; accepted -= 1; observed_misses += int(not late)
                        reason = "transaction_deadline_rejected"
                        if arm == "C1":
                            replacements.pop()
                            lifecycle[:] = [e for e in lifecycle[:lifecycle_start]] + [e for e in lifecycle[lifecycle_start:] if e["state"]!="REPLACED"]
                            command = at(backup["controls"],epoch+.1,backup["initial_target"]) if backup else [0.,0.]
                            from_backup = backup is not None and epoch+.1>=backup["switch_time"]-1e-8
                            queue.append([epoch+.1,*command]);queue_version += 1
                            owners[round(epoch+.1,10)] = dict(backup=from_backup,id=backup["id"] if backup else None,
                                                            lineage=backup["lineage"] if backup else None)
                            transition("HELD", reason)
                        else:
                            queue.append([epoch+response_ms/1000+.05,0.,0.]);queue_version += 1;timeout_brakes += 1
                        queue.sort(key=lambda e:e[0])
                    busy_until = epoch+response_ms/1000
                elif maintenance_lat:
                    maintenance_lat[-1] = full_ms
                acquisition_lat.append(validation_ms)
                total_cycle_lat.append((time.perf_counter()-begin)*1000)
                schedule = backup["controls"] if backup else previous
                if previous_schedule is not None:
                    sequence_rms.append(math.sqrt(sum(math.dist(at(schedule, epoch+.1+.2*k, target),
                                                               at(previous_schedule, epoch+.1+.2*k, target))**2 for k in range(10))/10))
                previous_schedule = copy.deepcopy(schedule)
                ttc = hold_ttc(state, target, actors, cfg["wall"], [(e[0]-epoch, e[1:]) for e in queue])
                if ttc is not None: ttc_values.append(ttc-.1)
                record = dict(t=epoch, state=state.copy(), target=target.copy(), queue_version=queue_version,
                              immutable_queue=copy.deepcopy(queue), proposed=proposal,
                              accepted=admitted, committed_action=proposal["command"] if admitted else None,
                              active_backup=compact(backup, epoch), candidate_available=candidate is not None,
                              reason=reason, validity=validity, planner_ms=plan_ms,
                              revalidation_ms=validity_ms, acquisition_replacement_ms=validation_ms,
                              backup_validation_ms=validity_ms+validation_ms, response_ms=response_ms,
                              total_cycle_ms=total_cycle_lat[-1], lifecycle=lifecycle[lifecycle_start:],
                              backup_dispatch=from_backup, executed=[], executed_owners=[])
                if input_metadata:
                    record["observation"] = input_metadata
                    record["input_actors"] = actors
                    record["hold_ttc_margin_s"] = ttc-.1 if ttc is not None else None
                for k in range(5):
                    while queue and queue[0][0] <= epoch+.005+1e-9:
                        e = queue.pop(0); target = e[1:]
                        owner = owners.pop(round(e[0],10), None)
                    for j in range(2):
                        sign = 0 if abs(target[j]) < .03 else (1 if target[j] > 0 else -1)
                        if sign and last_sign[j] and sign != last_sign[j]: flips += 1
                        if sign: last_sign[j] = sign
                    old_state = state; state = advance(state, target)
                    acc = [(state[j+2]-old_state[j+2])/.01 for j in range(2)]
                    accelerations.append(math.hypot(*acc)); jerks.append(math.dist(acc, last_acc)/.01); last_acc = acc
                    epoch = round(epoch+.01, 10)
                    record["executed"].append([epoch, *target, *state])
                    record["executed_owners"].append(owner)
                    if math.hypot(*state[2:]) < .03:
                        wait += .01; stop_run += .01
                        if abs(stop_run-.3) < .005: stops += 1
                    else: stop_run = 0.
                    value = clearance(state, truth_at(epoch), cfg["wall"])
                    if value < minimum: minimum = value; worst = dict(t=epoch, state=state.copy(), clearance_m=value)
                    if value < 0: collision = True
                    if arrival is None and math.dist(state[:2], cfg["goal"]) <= .15: arrival = epoch
                    if collision: break
                trace.write(json.dumps(record, separators=(",", ":"), allow_nan=False)+"\n")
                slot += 1
                if collision or (arm == "C1" and invalid): break
        completed = arrival is not None and epoch >= arrival+3-1e-8 and math.dist(state[:2], cfg["goal"]) <= .15
        usage = [planner.usage()] + ([checker.usage()] if checker else [])
        result = dict(arm=arm, condition=condition, collision=collision, completion=completed,
                      minimum_clearance_m=minimum, worst=worst, arrival_s=arrival if completed else None,
                      traversal_s=arrival-start_epoch if completed else None, start_epoch_s=start_epoch,
                      end_s=epoch, wait_s=wait, stop_count=stops,
                      unnecessary_stop_s=None, deadlock=None, calls=calls, control_slots=slot,
                      accepted=accepted, rejected=rejected, initialized=initial["available"] if initial else None,
                      backup_dispatches=active_exec, late_backup_dispatches=late_exec, invalid=invalid,
                      timeout_brakes=timeout_brakes, injected_calls=injected,
                      intrinsic_deadline_misses=intrinsic_misses, response_deadline_misses=observed_misses,
                      intrinsic_ms=stats(intrinsic), response_ms=stats(response_lat), planner_ms=stats(planner_lat),
                      backup_validation_ms=stats(validator_lat), executor_only_ms=stats(maintenance_lat),
                      revalidation_ms=stats(revalidation_lat), acquisition_replacement_ms=stats(acquisition_lat),
                      total_cycle_ms=stats(total_cycle_lat), held_cycles=held_cycles,
                      lifecycle_counts={s:sum(e["state"]==s for e in lifecycle) for s in
                                        ("INITIALIZING","HELD","REVALIDATED","REPLACED","EXECUTING_BACKUP","SAFE_REST","INVALID","UNAVAILABLE")},
                      sequence_rms_mps=stats(sequence_rms), command_sign_flips=flips,
                      acceleration_mps2=stats(accelerations), jerk_mps3=stats(jerks),
                      hold_ttc_margin_s=stats(ttc_values), replacement_count=len(replacements),
                      first_rejections=events[:8], consumer_cpu_s=sum(u[0] for u in usage)-cpu0,
                      consumer_rss_peak_sum_mib=sum(u[1] for u in usage), harness_cpu_s=time.process_time()-own_cpu,
                      planner_update_hz=calls/(epoch-start_epoch), executor_update_hz=slot/(epoch-start_epoch),
                      initialization_ms=initial["roundtrip_ms"] if initial else None)
        (out/f"{name}_lifecycle.json").write_text(json.dumps(lifecycle, separators=(",",":"))+"\n")
        (out/f"{name}.json").write_text(json.dumps(result, indent=2)+"\n")
        print(name, "collision", collision, "complete", completed, "clear", round(minimum, 6),
              "arrival", arrival, "p99", round(result["intrinsic_ms"]["p99"], 2), "invalid", invalid[:1], flush=True)
        return result
    finally:
        planner.close()
        if checker: checker.close()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--build", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True); ap.add_argument("--sanity-only", action="store_true")
    args = ap.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    sanity(args.build, args.output)
    if not args.sanity_only:
        results = [trial(args.build, args.output, arm, condition) for condition in ("normal", "late") for arm in ("B", "C0", "C1")]
        (args.output/"results.json").write_text(json.dumps(results, indent=2)+"\n")


if __name__ == "__main__": main()
