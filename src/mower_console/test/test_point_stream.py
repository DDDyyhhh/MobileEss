"""Display backpressure must not interrupt state or controller heartbeats."""
import asyncio
from types import SimpleNamespace

import pytest
from aiohttp import CookieJar, WSMsgType, web
from aiohttp.test_utils import TestClient, TestServer

from mower_console.server import create_app
from test_console import make_console
from test_http import receive_type


def app_with_points(tmp_path):
    console, adapter, _ = make_console()
    console.owner = None
    adapter.points = lambda: {'cloud': (1, [[1., 2., 3.]] * 5000),
                              'obstacles': (1, [[2., 1., 0.]])}
    assets = SimpleNamespace(description={'revision': 'test'})
    return console, create_app(console, assets, '123456', tmp_path)


@pytest.mark.asyncio
async def test_slow_point_write_does_not_delay_state(tmp_path, monkeypatch):
    _, app = app_with_points(tmp_path)
    send_bytes = web.WebSocketResponse.send_bytes

    async def slow_points(ws, data, *args, **kwargs):
        await asyncio.sleep(.4)
        await send_bytes(ws, data, *args, **kwargs)

    monkeypatch.setattr(web.WebSocketResponse, 'send_bytes', slow_points)
    async with TestClient(TestServer(app), cookie_jar=CookieJar(unsafe=True)) as client:
        await client.post('/api/pair', json={'code': '123456'})
        async with client.ws_connect('/api/events') as state:
            await receive_type(state, 'state')
            frame = await asyncio.wait_for(state.receive(), .35)
            assert frame.type == WSMsgType.TEXT
            assert frame.json()['type'] == 'state'


@pytest.mark.asyncio
async def test_points_wait_for_browser_demand_and_only_send_selected_layers(tmp_path):
    _, app = app_with_points(tmp_path)
    async with TestClient(TestServer(app), cookie_jar=CookieJar(unsafe=True)) as client:
        await client.post('/api/pair', json={'code': '123456'})
        async with client.ws_connect('/api/points?cloud=0') as points:
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(points.receive(), .1)
            await points.send_str('next')
            frame = await points.receive()
            assert frame.type == WSMsgType.BINARY and frame.data[0] == 2
            assert await points.receive_json() == {'type': 'ready'}
            await points.send_str('next')
            assert await points.receive_json() == {'type': 'ready'}


@pytest.mark.asyncio
async def test_stalled_point_connection_keeps_state_and_ownership(tmp_path, monkeypatch):
    console, app = app_with_points(tmp_path)
    blocked = asyncio.Event()

    async def stall_points(*args, **kwargs):
        blocked.set()
        await asyncio.Event().wait()

    monkeypatch.setattr(web.WebSocketResponse, 'send_bytes', stall_points)
    async with TestClient(TestServer(app), cookie_jar=CookieJar(unsafe=True)) as client:
        await client.post('/api/pair', json={'code': '123456'})
        async with client.ws_connect('/api/events') as state:
            await receive_type(state, 'state')
            await state.send_json({'type': 'claim'})
            owner = await receive_type(state, 'ownership')
            async with client.ws_connect('/api/points?cloud=1') as points:
                await points.send_str('next')
                await asyncio.wait_for(blocked.wait(), .3)
                for _ in range(4):
                    await state.send_json({'type': 'heartbeat'})
                    frame = await asyncio.wait_for(state.receive(), .35)
                    assert frame.type == WSMsgType.TEXT
                    assert frame.json()['type'] == 'state'
                console.require_owner(owner['controller'])


@pytest.mark.asyncio
async def test_point_socket_requires_pairing_and_same_origin(tmp_path):
    _, app = app_with_points(tmp_path)
    async with TestClient(TestServer(app), cookie_jar=CookieJar(unsafe=True)) as client:
        assert (await client.get('/api/points')).status == 401
        await client.post('/api/pair', json={'code': '123456'})
        assert (await client.get('/api/points?cloud=invalid')).status == 400
        assert (await client.get('/api/points?cloud=1', headers={'Origin': 'https://invalid.example'})).status == 403
