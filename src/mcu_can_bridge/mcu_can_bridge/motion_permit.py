"""A task lease cannot re-arm after revocation, expiry or clock regression."""
import math


class MotionPermit:
    def __init__(self, timeout=.25):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('permit timeout must be positive')
        self.timeout, self.last, self.armed, self.revoked = timeout, None, False, False

    def update(self, enabled, now):
        if self.armed and (self.last is None or not 0 <= now-self.last <= self.timeout):
            self.revoked = True
        if not math.isfinite(now) or (not enabled and self.armed):
            self.revoked = True
        if enabled and not self.revoked:
            self.armed, self.last = True, now

    def valid(self, now):
        if self.armed and not 0 <= now-self.last <= self.timeout:
            self.revoked = True
        return self.armed and not self.revoked
