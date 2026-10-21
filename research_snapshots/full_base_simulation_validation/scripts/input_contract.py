"""Observable heading metadata checks, without inferring hidden measurement time.

The 60 ms receipt window is the existing research transport budget, not a
hardware clock bound. Sequence is compared only within the heading stream.
"""
from dataclasses import dataclass
import math
from common import stamp

WAIT_SEC = .06


@dataclass
class Received:
    message: object
    arrived: float
    fingerprint: tuple


class HeadingCache:
    def __init__(self, reference):
        self.reference = reference
        self.entries = {}
        self.latest = None
        self.bound_epoch = None
        self.latched = None
        self.problem = ('INVALID_INPUT', 'missing heading')

    @staticmethod
    def epoch(m):
        return m.source_boot_id, m.reset_counter

    def bind(self, m):
        self.bound_epoch = self.epoch(m)

    def reject(self, status, reason):
        self.entries.clear()
        self.problem = status, reason

    def add(self, m, arrived):
        # A reset invalidates the reference even in an invalid/offline packet
        # or a packet that would never be selected for a raw timestamp.
        if self.bound_epoch is not None and self.epoch(m) != self.bound_epoch:
            self.latched = 'heading epoch change requires explicit restart and re-home'
        if self.latched:
            self.reject('INVALID_TIME', self.latched)
            return
        t = stamp(m.header.stamp)
        if m.header.frame_id != self.reference:
            self.reject('INVALID_INPUT', 'unexpected heading reference frame')
            return
        if not m.valid or not m.online or not all(map(math.isfinite, [m.yaw_rad, m.yaw_rate_rad_s])):
            self.reject('INVALID_INPUT', 'invalid, offline or nonfinite heading')
            return
        if t <= 0:
            self.reject('INVALID_TIME', 'heading requires positive source time')
            return
        fingerprint = (self.epoch(m), m.sample_sequence, t, m.yaw_rad,
                       m.yaw_rate_rad_s, m.mcu_time_ms, m.header.frame_id)
        for received in self.entries.values():
            if (self.epoch(received.message), received.message.sample_sequence) == (self.epoch(m), m.sample_sequence):
                if fingerprint != received.fingerprint:
                    self.reject('INVALID_TIME', 'reused heading sequence with changed sample')
                return  # Keep the first receipt for cached historical duplicates too.
        if self.latest is not None and self.epoch(m) == self.epoch(self.latest.message):
            previous = self.latest.message
            delta = (m.sample_sequence - previous.sample_sequence) & 0xffffffff
            if delta == 0:
                if fingerprint != self.latest.fingerprint:
                    self.reject('INVALID_TIME', 'reused heading sequence with changed sample')
                # Exact duplicates cannot refresh the original receipt time,
                # nor restore a stream cleared by an invalid/offline packet.
                return
            if delta >= 0x80000000:
                if t >= stamp(previous.header.stamp):
                    self.reject('INVALID_TIME', 'old heading sequence with advancing source time')
                elif self.problem is None and t not in self.entries:
                    # Legitimate heading-stream reordering can fill an exact
                    # match for a waiting raw packet. It cannot restore a stream
                    # cleared by an invalid packet or refresh any cached receipt.
                    self.entries[t] = Received(m, arrived, fingerprint)
                    self.trim()
                return
            if t <= stamp(previous.header.stamp):
                self.reject('INVALID_TIME', 'advancing heading sequence with nonmonotonic source time')
                return
        elif self.latest is not None:
            self.entries.clear()  # Before startup only, no reference to preserve.
        self.latest = Received(m, arrived, fingerprint)
        self.entries[t] = self.latest
        self.problem = None
        self.trim()

    def trim(self):
        while len(self.entries) > 100:  # Same bound as the subscription history.
            del self.entries[min(self.entries)]

    def match(self, t, raw_arrived, now):
        """Return (heading, problem); None/None means the existing wait is open."""
        exact = self.entries.get(t)
        if exact is None and now - raw_arrived < WAIT_SEC:
            return None, None
        item = exact or (min(self.entries.values(), key=lambda r: abs(stamp(r.message.header.stamp) - t))
                         if self.entries else None)
        if item is None:
            return None, self.problem or ('INVALID_INPUT', 'missing heading')
        if abs(item.arrived - raw_arrived) > WAIT_SEC:
            return None, ('INVALID_TIME', 'raw/heading receipt separation exceeds transport window')
        return item.message, None
