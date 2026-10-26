"""Deterministic A–H contract gate. No navigation episode, no future truth."""
import argparse
import copy
import json
from pathlib import Path
import time
from run import Consumer
from reserve_run import question
from certificate_lifecycle import CertificateLifecycle


def contracts(checker):
    passed = []
    admissions = []

    def setup():
        q = question([0.,0.,0.,0.], [0.,0.], [], 1., [[3.,0.,0.,0.,.3]], 0, 10)
        m = CertificateLifecycle(checker)
        assert m.initialize(q)["available"]
        return m, q

    def replace(m, q, command=(.1,0.)):
        before = m.certificate
        m.begin(q)
        r = m.prepare(list(command))
        assert m.certificate is before  # prepare never publishes or discards old
        result = m.finish(r["certificate"])
        admissions.append(dict(accepted=result["accepted"],valid=result["authorized"]))
        if result["accepted"]:
            assert result["authorized"]
            post = q | {"queue": result["queue"], "queue_version": result["queue_version"]}
            assert checker.call(post | {"op":"validate", "backup":m.certificate})["reason"] == "valid"
        return result

    m,q = setup(); changed = q | {"actors":[[3.005,0.,0.,0.,.3]], "source_version":11}
    assert m.begin(changed)["reason"] == "prediction_changed"
    assert replace(m,changed)["accepted"]
    passed.append("A_fresh_prediction_atomic_replacement")

    for actors in ([], [[3.,0.,0.,0.,.3],[3.,1.,0.,0.,.3]]):
        m,q = setup(); changed = q | {"actors":actors,"source_version":11,"obstacle_set_version":1}
        assert m.begin(changed)["reason"] == "obstacle_set_changed"
        assert replace(m,changed)["accepted"]
    passed.append("B_fresh_set_add_and_remove")

    m,q = setup(); old = m.certificate
    blocked = q | {"actors":[[0.,0.,0.,0.,.3]],"source_version":11}
    r = replace(m,blocked)
    assert not r["accepted"] and not r["authorized"] and r["state"] == "INVALID"
    assert r["reason"] == "unsafe_prefix"
    assert m.certificate is old and r["queue"] == q["queue"]
    passed.append("C_construction_failure_no_motion_admission")

    m,q = setup(); old = m.certificate
    stale = q | {"epoch":1.1}  # original .9 source now 200 ms old
    for op in ("init","admit"):
        r = checker.call(stale | {"op":op,"command":[.1,0.],"backup":old})
        assert not r["available"] and r["certificate"] is None and r["reason"] == "stale_observation"
    assert checker.call(stale | {"op":"validate","backup":old})["reason"] == "stale_observation"
    r = replace(m,stale)
    assert not r["accepted"] and not r["authorized"] and m.certificate is old
    passed.append("D_stale_generation_revalidation_replacement_admission_blocked")

    fresh = stale | {"source_epoch":1.,"receive_time":1.1,"source_version":11}
    r = replace(m,fresh)
    assert r["accepted"] and m.certificate["prediction"]["version"] == 11
    passed.append("E_stale_to_fresh_recovery")

    m,q = setup(); assert replace(m,q | {"source_version":12})["accepted"]
    saved = m.certificate
    reordered = q | {"source_version":11,"queue_version":1,"queue":[[1.1,.1,0.]]}
    assert m.begin(reordered)["reason"] == "prediction_version_regressed"
    assert not replace(m,reordered)["accepted"] and m.certificate is saved
    passed.append("F_late_observation_cannot_replace_newer")

    m,q = setup(); assert replace(m,q | {"source_version":12,"obstacle_set_version":2})["accepted"]
    saved = m.certificate
    reordered = q | {"source_version":13,"obstacle_set_version":1,"queue_version":1,"queue":[[1.1,.1,0.]]}
    assert m.begin(reordered)["reason"] == "obstacle_set_version_regressed"
    assert not replace(m,reordered)["accepted"] and m.certificate is saved
    passed.append("G_obstacle_set_version_cannot_roll_back")

    m,q = setup()
    assert checker.call(q | {"queue_version":1,"op":"validate","backup":m.certificate})["reason"] == "queue_version_changed"
    assert not m.begin(q | {"queue_version":1})["reason"] == "valid"
    # An intentional transaction append binds exactly its new queue generation.
    m,q = setup(); r = replace(m,q)
    assert r["queue_version"] == 1 and m.certificate["immutable_queue_version"] == 1
    assert m.certificate["immutable_queue"] == r["queue"]
    assert checker.call(q | {"queue_version":1,"queue":[],"op":"validate","backup":m.certificate})["reason"] == "immutable_queue_mismatch"
    try:
        m.finish(None)
        assert False, "duplicate publication must fail"
    except RuntimeError as e:
        assert 'only once' in str(e)
    passed.append("H_queue_generation_binding_and_transaction_append")

    for field,reason in (("frame_epoch","coordinate_reset"),("clock_epoch","time_reset")):
        m,q = setup(); changed = q | {field:1,"source_version":11}
        assert m.begin(changed)["reason"] == reason
        assert not replace(m,changed)["accepted"]
    passed.append("coordinate_and_clock_reset_fail_closed")

    # Timeout retains old absolute timing and executes only its verified command.
    m,q = setup(); original = copy.deepcopy(m.certificate)
    m.begin(q); draft = m.prepare([.1,0.])["certificate"]
    r = m.finish(draft,deadline=time.perf_counter()-1.)
    assert not r["accepted"] and r["authorized"]
    assert m.certificate["controls"] == original["controls"]
    assert m.certificate["epoch"] == original["epoch"]
    assert r["queue"] == [[1.1,0.,0.]]
    passed.append("deadline_keeps_old_absolute_reserve")

    m,q = setup()
    for age,valid in ((.15,True),(.15001,False)):
        aq = q | {"source_epoch":1.-age}
        assert checker.call(aq | {"op":"init"})["available"] == valid
    passed.append("freshness_threshold_unchanged_150ms")
    m,q=setup()
    for change in ({'source_epoch':1.01},{'receive_time':1.01},{'receive_time':.8}):
        r=checker.call(q | change | {'op':'init'})
        assert not r['available'] and r['reason']=='observation_time_invalid'
    passed.append('timestamp_order_checked')
    return dict(passed=passed,count=len(passed),admission_checks=len(admissions),
                accepted_without_valid_backup=sum(r['accepted'] and not r['valid'] for r in admissions),
                note="Assertions at every positive/negative admission; no episode safety claim")


if __name__ == "__main__":
    ap=argparse.ArgumentParser();ap.add_argument("--binary",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    c=Consumer(args.binary,"reserve",args.output/"consumer.log")
    try:
        result=contracts(c)
        (args.output/"contracts.json").write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps(result,indent=2))
    finally:c.close()
