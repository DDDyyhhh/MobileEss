"""Planning may spend time checking a route while state callbacks keep running."""
import asyncio
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from mower_console.ros_adapter import RosAdapter


@pytest.mark.asyncio
async def test_route_checks_leave_the_event_loop_available(monkeypatch):
    action_module = ModuleType('nav2_msgs.action')
    action_module.ComputePathToPose = SimpleNamespace(Goal=SimpleNamespace)
    monkeypatch.setitem(sys.modules, 'nav2_msgs', ModuleType('nav2_msgs'))
    monkeypatch.setitem(sys.modules, 'nav2_msgs.action', action_module)
    entered = [threading.Event(), threading.Event()]
    release = [threading.Event(), threading.Event()]

    class Grid:
        calls = 0

        def validate(self, _):
            index = self.calls
            self.calls += 1
            entered[index].set()
            if not release[index].wait(.3):
                raise RuntimeError('route checking prevented state work from running')

    async def action(*_):
        await asyncio.sleep(0)
        return SimpleNamespace(path=SimpleNamespace(poses=[
            SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0),
                                                 orientation=None)) for x in (0., 1.)]))

    adapter = SimpleNamespace(assets=SimpleNamespace(grid=Grid()),
                              snapshot=lambda: {'pose': [0., 0., 0.]},
                              stamped_pose=lambda p: p, yaw=lambda _: 0.,
                              action=action, compute=None, reverse_compute=None)

    async def state_work():
        for waiting, resume in zip(entered, release):
            while not waiting.is_set():
                await asyncio.sleep(.001)
            resume.set()

    work = asyncio.create_task(state_work())
    try:
        poses = await RosAdapter.plan(adapter, [1., 0.])
        assert poses == [[0., 0., 0.], [1., 0., 0.]]
        await work
    finally:
        work.cancel()
        for event in release:
            event.set()
        await asyncio.gather(work, return_exceptions=True)
