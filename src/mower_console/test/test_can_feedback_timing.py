"""Exercise the actual receive loop with queued, kernel-timestamped CAN frames."""
from collections import deque
import socket
import struct
import threading

import pytest

from mower_console.ros_adapter import RosAdapter


def receive_backlog(monkeypatch, *, last=10., wheel_speed=0, gap=False, timestamps=True, api=0):
    # All frames were already on the wire, but Python handles the backlog at t=10.
    frames = []
    for index in range(27):
        arrival = last - 1.3 + index * .05
        if gap and last - .7 < arrival < last - .4:
            continue
        for cid, data in ((0x507, b'\0\0\0\0' + bytes([api, 0, 0, 0])),
                          (0x50A, struct.pack('>hhhh', 0, 0, 0, wheel_speed))):
            kernel_ns = round((990. + arrival) * 1_000_000_000)
            seconds, nanoseconds = divmod(kernel_ns, 1_000_000_000)
            ancillary = [(socket.SOL_SOCKET, 35, struct.pack('=qq', seconds, nanoseconds))] if timestamps else []
            frames.append((struct.pack('=IB3x8s', cid, 8, data), ancillary, 0, None))
    adapter = RosAdapter.__new__(RosAdapter)
    adapter.closed = threading.Event()
    adapter.lock = threading.RLock()
    adapter.events = deque(maxlen=2000)
    adapter.times = {}
    adapter.audit_file = None
    adapter.can_receive_lag_s = {}
    adapter.can_max_receive_lag_s = {}
    adapter.can_timestamp_errors = 0

    class Backlog:
        def next(self):
            value = frames.pop(0)
            if not frames:
                adapter.closed.set()
            return value

        def recv(self, _):
            return self.next()[0]

        def recvmsg(self, *_):
            return self.next()

    adapter.can = Backlog()
    monkeypatch.setattr('mower_console.ros_adapter.time.monotonic', lambda: 10.)
    monkeypatch.setattr('mower_console.ros_adapter.time.time_ns', lambda: 1_000_000_000_000)
    adapter.receive_can()
    return adapter


def test_backlog_does_not_fabricate_a_gap_in_continuous_stationary_feedback(monkeypatch):
    adapter = receive_backlog(monkeypatch)
    assert adapter.stationary(), 'processing a receive backlog must preserve real arrival history'
    assert adapter.events[0][0] < 9.


def test_queued_old_feedback_cannot_renew_freshness(monkeypatch):
    adapter = receive_backlog(monkeypatch, last=8.)
    assert 10. - adapter.times['wheels'] >= 1.99
    assert not adapter.stationary()


def test_missing_kernel_timestamp_does_not_refresh_feedback(monkeypatch):
    adapter = receive_backlog(monkeypatch, timestamps=False)
    assert not adapter.times and not adapter.events
    assert adapter.can_timestamp_errors > 0


@pytest.mark.parametrize('speed,ready', [(10, True), (11, False), (-13, False)])
def test_actual_wheel_speed_limit_is_preserved(monkeypatch, speed, ready):
    adapter = receive_backlog(monkeypatch, wheel_speed=speed)
    assert adapter.stationary() is ready
    evidence = adapter.stationary_evidence()
    assert evidence['ready'] is ready
    if not ready:
        assert evidence['reason'] == 'wheel_speed'
        assert evidence['channel'] == '0x50a'


def test_real_arrival_gap_is_still_rejected(monkeypatch):
    adapter = receive_backlog(monkeypatch, gap=True)
    assert not adapter.stationary()
    assert adapter.stationary_evidence()['reason'] == 'frame_gap'


def test_between_segment_stop_can_keep_api_armed(monkeypatch):
    adapter = receive_backlog(monkeypatch, api=2)
    assert not adapter.stationary()
    assert adapter.stationary_evidence()['reason'] == 'api_mode'
    assert adapter.stationary(api_zero=False)


@pytest.mark.parametrize('seconds,nanoseconds', [(-1, 0), (999, 1_000_000_000), (1001, 0)])
def test_invalid_or_future_kernel_clock_is_not_fresh_feedback(seconds, nanoseconds):
    from mower_console.can_feedback import arrival_time
    ancillary = [(socket.SOL_SOCKET, 35, struct.pack('=qq', seconds, nanoseconds))]
    assert arrival_time(ancillary, 10., 1_000_000_000_000) is None


def test_truncated_timestamp_is_not_fresh_feedback():
    from mower_console.can_feedback import arrival_time
    assert arrival_time([(socket.SOL_SOCKET, 35, b'\0' * 8)], 10., 1_000_000_000_000) is None
