"""Source evidence admission inside one fixed canonical process group.

This never rebases poses and never rearms a reset. ROS-message pairing is kept in
CandidateAdapter; this reducer makes revocation and version order deterministic.
"""


class SessionWindow:
    def __init__(self, started_ns):
        self.clock_ns = started_ns
        self.seen_ns = started_ns
        self.revoked_ns = started_ns
        self.version = (-1, -1)

    def observe_clock(self, now_ns):
        backwards = now_ns < self.clock_ns
        self.clock_ns = max(self.clock_ns, now_ns)
        return not backwards

    def observe_source(self, ns):
        self.seen_ns = max(self.seen_ns, ns)

    def revoke(self):
        self.revoked_ns = max(self.revoked_ns, self.seen_ns)

    def observe_state(self, generation, revision, ns):
        # Counters are independently monotonic within epoch 0. A delayed old
        # status must not restore trust or revoke a newer accepted generation.
        if generation < self.version[0] or revision < self.version[1]:
            return False
        self.version = (generation, revision)
        self.observe_source(ns)
        return True

    def fresh(self, ns):
        return ns > self.revoked_ns
