import signal
from types import SimpleNamespace
from mower_nav2 import field_workflow


def test_remaining_nodes_stopped_after_launch_leader_exits(monkeypatch):
    sent = []
    alive = True
    def killpg(pid, sig):
        nonlocal alive
        if not alive:
            raise ProcessLookupError()
        sent.append(sig)
        if sig == signal.SIGTERM:
            alive = False
    monkeypatch.setattr(field_workflow.os, 'killpg', killpg)
    child = SimpleNamespace(pid=42, wait=lambda timeout: 0)
    field_workflow.stop_processes([child])
    assert signal.SIGTERM in sent
    assert not alive


def test_stubborn_owned_group_is_killed_with_bounded_wait(monkeypatch):
    sent = []
    alive = True
    def killpg(pid, sig):
        nonlocal alive
        if not alive:
            raise ProcessLookupError()
        sent.append(sig)
        if sig == signal.SIGKILL:
            alive = False
    ticks = iter(range(100))
    monkeypatch.setattr(field_workflow.os, 'killpg', killpg)
    monkeypatch.setattr(field_workflow.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(field_workflow.time, 'sleep', lambda duration: None)
    field_workflow.stop_processes([SimpleNamespace(pid=42, wait=lambda timeout: 0)])
    assert signal.SIGKILL in sent
    assert not alive
