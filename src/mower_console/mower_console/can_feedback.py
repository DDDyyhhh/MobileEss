"""Receive timestamps and unchanged stop criteria for 64-bit Linux SocketCAN."""
import socket
import struct


# SO_TIMESTAMPNS_OLD / SCM_TIMESTAMPNS, verified on the RK3588 kernel headers.
# Python 3.10 on the board does not export this socket option.
TIMESTAMPNS = 35


def arrival_time(ancillary, monotonic_now, realtime_now_ns):
    for level, kind, value in ancillary:
        if level != socket.SOL_SOCKET or kind != TIMESTAMPNS or len(value) != 16:
            continue
        seconds, nanoseconds = struct.unpack('=qq', value)
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            return None
        age = (realtime_now_ns - seconds * 1_000_000_000 - nanoseconds) / 1e9
        # A backwards wall-clock step must not make queued data look fresh.
        if age < -.05:
            return None
        return monotonic_now - age
    return None


def stationary_evidence(events, now, api_zero=True):
    channels = {}

    def rejected(reason, channel):
        return dict(ready=False, reason=reason, channel=channel, channels=channels, evaluated_at=now)

    for cid, max_age, limit_gap, min_count in ((0x507, .25, .25, 5), (0x50A, .15, .15, 10)):
        channel = hex(cid)
        rows = [(t, data) for t, c, data in events if c == cid and t >= now - 1.3]
        before = [i for i, (t, _) in enumerate(rows) if t <= now - 1.]
        details = dict(age_s=now - rows[-1][0] if rows else None, samples=len(rows))
        channels[channel] = details
        if not before:
            return rejected('history', channel)
        if now - rows[-1][0] > max_age:
            return rejected('stale', channel)
        rows = rows[before[-1]:]
        details.update(samples=len(rows), max_gap_s=max((b[0]-a[0] for a, b in zip(rows, rows[1:])), default=0.))
        if len(rows) < min_count:
            return rejected('sample_count', channel)
        if details['max_gap_s'] > limit_gap:
            return rejected('frame_gap', channel)
        if cid == 0x507 and api_zero and any(data[4] != 0 for _, data in rows):
            return rejected('api_mode', channel)
        if cid == 0x50A:
            details['max_abs_wheel_mm_s'] = max(abs(v) for _, data in rows for v in struct.unpack('>hhhh', data))
            if details['max_abs_wheel_mm_s'] > 10:
                return rejected('wheel_speed', channel)
    return dict(ready=True, reason='confirmed', channel=None, channels=channels, evaluated_at=now)
