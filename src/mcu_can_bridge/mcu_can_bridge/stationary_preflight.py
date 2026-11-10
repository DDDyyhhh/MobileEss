"""Receive-only CAN readiness verdict; never opens a socket or sends a frame."""
import math
import struct


def stationary_preflight(events, now, wheel_limit_mm_s=10):
    """Require a fresh, continuous one-second stop before an operator-owned trial.

    The caller may wait at most eight seconds for readiness. A control frame or
    nonzero API mode anywhere in this capture invalidates that entire capture.
    Earlier wheel transients reset the stable window; they are never discarded
    from the evidence trace.
    """
    failed = lambda reason: {'ready': False, 'reason': reason}
    if wheel_limit_mm_s not in (10, 50):
        return failed('unsupported startup wheel tolerance')
    if not math.isfinite(now) or now < 0:
        return failed('invalid clock')
    status, wheels = [], []
    previous = -1.
    try:
        for event in events:
            t = event['t_s']
            if not math.isfinite(t) or t < previous or not 0 <= t <= now:
                return failed('invalid capture timestamps')
            previous = t
            cid = event['can_id']
            if cid == '0x5':
                return failed('control frame present')
            if cid not in ('0x507', '0x50a'):
                continue
            data = bytes.fromhex(event['payload'])
            if len(data) != 8:
                return failed('invalid telemetry payload')
            if cid == '0x507':
                if data[4] != 0:
                    return failed('API not exited during capture')
                status.append((t, data[4]))
            else:
                wheels.append((t, struct.unpack('>hhhh', data)))
    except (KeyError, ValueError, TypeError, struct.error):
        return failed('invalid capture')

    def window(rows, minimum_count, maximum_gap):
        if not rows or now-rows[-1][0] > maximum_gap:
            return None
        # Include the receipt just before the boundary, proving coverage.
        preceding = [i for i, row in enumerate(rows) if row[0] <= now-1.]
        if not preceding:
            return None
        recent = rows[preceding[-1]:]
        if len(recent) < minimum_count:
            return None
        if any(not 0 < b[0]-a[0] <= maximum_gap for a,b in zip(recent,recent[1:])):
            return None
        return recent

    if window(status, 5, .25) is None:
        return failed('status not continuous and fresh')
    recent = window(wheels, 10, .15)
    if recent is None:
        return failed('wheel feedback not continuous and fresh')
    peak = max(abs(v) for _, row in recent for v in row)
    if peak > wheel_limit_mm_s:
        return failed('waiting for continuous stationary window')
    return {'ready': True, 'reason': 'continuous stationary window',
            'stable_window_s': 1., 'wheel_samples': len(recent),
            'startup_wheel_limit_mm_s': wheel_limit_mm_s,
            'maximum_recent_wheel_mm_s': peak,
            'last_wheel_mm_s': list(recent[-1][1])}
