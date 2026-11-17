"""Offline ROS action futures: cancellation must confirm a terminal result."""
import asyncio
from types import SimpleNamespace

import pytest

from mower_console.ros_adapter import RosAdapter


def action_adapter(cancel_code=0):
    loop = asyncio.get_running_loop()
    result = loop.create_future()
    response = loop.create_future()
    response.set_result(SimpleNamespace(return_code=cancel_code))
    cancellations = []

    def cancel():
        cancellations.append(True)
        return response

    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                             cancel_goal_async=cancel)
    sent = loop.create_future()
    sent.set_result(handle)
    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=lambda _: sent)
    adapter = RosAdapter.__new__(RosAdapter)
    adapter.action_handle = None
    adapter.pending_goal = None
    return adapter, client, handle, result, cancellations


@pytest.mark.asyncio
@pytest.mark.parametrize('cancel_code', [0, 3])
async def test_cancel_waits_for_terminal_result(cancel_code):
    adapter, client, handle, result, cancellations = action_adapter(cancel_code)
    job = asyncio.create_task(adapter.action(client, object(), 10.))
    await asyncio.sleep(0)
    job.cancel()
    try:
        await asyncio.sleep(.04)
        assert cancellations and not job.done()
        assert adapter.action_handle is handle
    finally:
        result.set_result(SimpleNamespace(status=5, result=None))
        await asyncio.gather(job, return_exceptions=True)
    assert job.cancelled() and adapter.action_handle is None


@pytest.mark.asyncio
@pytest.mark.parametrize('cancel_code', [1, 2])
async def test_unconfirmed_cancel_blocks_new_goals_and_arming(cancel_code):
    adapter, client, handle, _, cancellations = action_adapter(cancel_code)
    job = asyncio.create_task(adapter.action(client, object(), 10.))
    await asyncio.sleep(0)
    job.cancel()
    with pytest.raises(RuntimeError, match='取消'):
        await job
    assert adapter.action_handle is handle and len(cancellations) == 1
    with pytest.raises(ValueError, match='尚未确认结束'):
        await adapter.action(client, object(), 10.)
    with pytest.raises(ValueError, match='尚未确认结束'):
        await adapter.arm()


@pytest.mark.asyncio
@pytest.mark.parametrize('cancel_code', [0, 1])
async def test_late_goal_acceptance_must_be_cancelled_and_confirmed(cancel_code):
    adapter, client, handle, result, _ = action_adapter(cancel_code)
    sent = asyncio.get_running_loop().create_future()
    client.send_goal_async = lambda _: sent
    job = asyncio.create_task(adapter.action(client, object(), 10.))
    await asyncio.sleep(0)
    job.cancel()
    await asyncio.sleep(0)
    assert not job.done() and adapter.pending_goal is sent
    with pytest.raises(ValueError, match='尚未确认结束'):
        await adapter.action(client, object(), 10.)
    with pytest.raises(ValueError, match='尚未确认结束'):
        await adapter.arm()
    sent.set_result(handle)
    result.set_result(SimpleNamespace(status=5, result=None))
    with pytest.raises(asyncio.CancelledError if cancel_code == 0 else RuntimeError):
        await job
    assert adapter.pending_goal is None
    assert adapter.action_handle is (None if cancel_code == 0 else handle)


@pytest.mark.asyncio
async def test_unknown_result_is_not_a_confirmed_stop():
    adapter, client, handle, result, _ = action_adapter()
    result.set_result(SimpleNamespace(status=0, result=None))
    with pytest.raises(RuntimeError, match='终态'):
        await adapter.action(client, object(), 10.)
    assert adapter.action_handle is handle


@pytest.mark.asyncio
async def test_cancel_failure_is_not_retried_as_an_obstacle():
    from test_console import CONFIRM, make_console

    ros, client, handle, _, _ = action_adapter(cancel_code=1)
    console, adapter, _ = make_console()
    adapter.clear = False

    async def fail_segment(_):
        await ros.action(client, object(), .01)

    adapter.execute_segment = fail_segment
    plan = await console.preview('tab1', [1., 0.])
    await console.start('tab1', plan['id'], CONFIRM)
    await asyncio.wait_for(console.job, 1.)
    assert console.task['state'] == 'BLOCKED' and '取消未确认' in console.task['reason']
    assert len(adapter.plans) == 1 and adapter.permits[-1] is False
    assert ros.action_handle is handle


@pytest.mark.asyncio
@pytest.mark.parametrize('status', [4, 6])
async def test_terminal_action_does_not_need_cancellation(status):
    adapter, client, _, result, cancellations = action_adapter()
    result.set_result(SimpleNamespace(status=status, result='finished'))
    if status == 4:
        assert await adapter.action(client, object(), 10.) == 'finished'
    else:
        with pytest.raises(ValueError, match='执行失败'):
            await adapter.action(client, object(), 10.)
    assert not cancellations and adapter.action_handle is None


@pytest.mark.asyncio
async def test_stop_cancel_failure_still_closes_motion_process(monkeypatch):
    adapter, _, handle, _, _ = action_adapter(cancel_code=1)
    adapter.action_handle = handle
    adapter.motion_process = SimpleNamespace(pid=12345, poll=lambda: None, wait=lambda _: 0)
    permits, signals = [], []
    adapter.permit = permits.append
    monkeypatch.setattr('mower_console.ros_adapter.os.killpg', lambda *args: signals.append(args))
    with pytest.raises(RuntimeError, match='取消'):
        await adapter.stop()
    assert permits == [False] and signals and adapter.motion_process is None
    assert adapter.action_handle is handle
