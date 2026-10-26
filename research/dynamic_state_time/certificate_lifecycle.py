"""Single-threaded commit boundary; no planner, plant or backup-library changes.

The owned certificate is retained even when invalid. Only `authorized` permits
dispatch. finish() publishes certificate + immutable queue together; a draft is
never visible to the executor. Call begin() once per measured control snapshot.
"""
import copy
import time


class CertificateLifecycle:
    def __init__(self, checker):
        self.checker = checker
        self.certificate = None
        self.authorized = False
        self.state = "UNAVAILABLE"
        self.latest = None
        self.draft = None
        self.q = None
        self.finished = False

    def _call(self, q):
        r = self.checker.call(q)
        if "error" in r:
            raise RuntimeError(r["error"])
        return r

    def initialize(self, q):
        self.begin(q)
        if self.input_reason != "valid":
            return dict(available=False, certificate=None, reason=self.input_reason)
        r = self._call(q | {"op": "init"})
        if r["certificate"] is not None:
            assert self._call(q | {"op": "validate", "backup": r["certificate"]})["reason"] == "valid"
            self.certificate = r["certificate"]
            self.authorized = True
            self.state = "OLD_VALID"
        return r

    def _ordering(self, q):
        old = self.latest
        if old is None:
            return "valid"
        for key, reason in (("frame_epoch", "coordinate_reset"), ("clock_epoch", "time_reset")):
            if q[key] != old[key]:
                return reason  # New frame/clock requires an explicit new session.
        if q["epoch"] < old["epoch"] - 1e-9:
            return "time_reset"
        if q["source_version"] < old["source_version"] or q["source_epoch"] < old["source_epoch"] - 1e-9:
            return "prediction_version_regressed"
        if q["obstacle_set_version"] < old["obstacle_set_version"]:
            return "obstacle_set_version_regressed"
        if q["source_version"] == old["source_version"]:
            dt = q["epoch"] - old["epoch"]
            projected = [[a[0]+dt*a[2], a[1]+dt*a[3], *a[2:]] for a in old["actors"]]
            if (q["source_epoch"] != old["source_epoch"] or
                q["obstacle_set_version"] != old["obstacle_set_version"] or
                len(projected) != len(q["actors"]) or
                any(abs(x-y)>1e-7 for a,b in zip(projected,q["actors"]) for x,y in zip(a,b))):
                return "observation_version_conflict"
        return "valid"

    def begin(self, q):
        self.q = copy.deepcopy(q)
        self.finished = False
        self.draft = None
        self.input_reason = self._ordering(q)
        self.preparation_reason = None
        # The native service owns the one numerical freshness threshold.
        if self.input_reason == "valid":
            fresh = self._call(q | {"op": "freshness"})
            self.input_reason = fresh["reason"]
        if self.input_reason == "valid":
            self.latest = copy.deepcopy(q)
            r = self._call(q | {"op": "validate", "backup": self.certificate})
        else:
            r = dict(reason=self.input_reason, stationary_refuge=False)
        self.validity = r["reason"]
        self.authorized = self.validity == "valid"
        self.state = "OLD_VALID" if self.authorized else ("INVALID" if self.certificate else "UNAVAILABLE")
        return r

    def prepare(self, command):
        if self.input_reason != "valid":
            return dict(certificate=None, available=False, reason=self.input_reason, roundtrip_ms=0.)
        self.state = "PREPARING_NEW"
        # Invalid old objects are retained for audit, never spliced or authorized.
        r = self._call(self.q | {"op": "admit", "command": command,
                                "backup": self.certificate if self.authorized else None})
        self.draft = r["certificate"]
        self.preparation_reason = r["reason"]
        return r

    def finish(self, candidate=None, deadline=None):
        """Return the only queue the executor may publish; no irrevocable edit before this.

        Rebinding queue_version is restricted to exactly one appended command
        already in the verified schedule. External queue changes require begin()
        and new certification; they cannot silently rebind an old object.
        """
        if self.finished:
            raise RuntimeError("a snapshot may be committed only once")
        q = self.q
        accepted = candidate is not None and candidate is self.draft
        reason = self.preparation_reason or self.input_reason
        if accepted:
            checked = self._call(q | {"op": "validate", "backup": candidate})["reason"]
            accepted = checked == "valid"
            if not accepted:
                reason = checked
        if accepted and deadline is not None and time.perf_counter() > deadline:
            accepted = False
            reason = "transaction_deadline_rejected"
        cert = candidate if accepted else (self.certificate if self.authorized else None)
        queue, version = copy.deepcopy(q["queue"]), q["queue_version"]
        from_backup = False
        if cert is not None:
            stamp = q["epoch"] + .1
            command = cert["initial_target"]
            for e in cert["controls"]:
                if e[0] <= stamp+1e-8:
                    command = e[1:3]
            queue.append([stamp, *command])
            queue.sort(key=lambda e: e[0])
            version += 1
            bound = copy.deepcopy(cert)
            bound["immutable_queue_version"] = version
            bound["immutable_queue"] = copy.deepcopy(queue)
            bound["execution_cursor"] = sum(e[0]<=q["epoch"]+1e-8 for e in bound["controls"])
            check = self._call(q | {"queue": queue, "queue_version": version,
                                   "op": "validate", "backup": bound})["reason"]
            if check != "valid":
                raise RuntimeError("private dispatch failed validation: " + check)
            # No code can observe a new command with the old certificate.
            if accepted and deadline is not None and time.perf_counter() > deadline:
                retained = self.finish(None)
                retained["reason"] = "transaction_deadline_rejected"
                return retained
            self.certificate, self.authorized, self.state = bound, True, "NEW_VALID" if accepted else "OLD_VALID"
            from_backup = stamp >= bound["switch_time"]-1e-8
        else:
            self.authorized = False
            self.state = "INVALID" if self.certificate else "UNAVAILABLE"
        self.draft = None
        self.finished = True
        return dict(accepted=accepted, certificate=self.certificate, authorized=self.authorized,
                    queue=queue, queue_version=version, from_backup=from_backup,
                    state=self.state, reason=reason, source_epoch=q["source_epoch"],
                    receive_time=q["receive_time"], age_s=q["epoch"]-q["source_epoch"],
                    certificate_source_version=self.certificate["prediction"]["version"] if self.certificate else None)
