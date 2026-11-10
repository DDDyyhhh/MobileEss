import asyncio

import pytest
from aiohttp import CookieJar
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from mower_console.server import create_app
from test_console import make_console


async def receive_type(ws, kind):
    while True:
        frame=await asyncio.wait_for(ws.receive_json(),1)
        if frame.get('type')==kind:
            return frame


@pytest.mark.asyncio
async def test_cancelled_old_stream_does_not_stop_new_page_state(tmp_path, monkeypatch):
    c,_,_=make_console()
    assets=type('Assets',(),{'description':{'revision':'test'}})()
    original=web.WebSocketResponse.send_json
    cancel_next=False

    async def send(ws, data, *args, **kwargs):
        nonlocal cancel_next
        if cancel_next and data.get('type')=='state':
            cancel_next=False
            raise asyncio.CancelledError()
        return await original(ws,data,*args,**kwargs)

    monkeypatch.setattr(web.WebSocketResponse,'send_json',send)
    async with TestClient(TestServer(create_app(c,assets,'123456',tmp_path)),cookie_jar=CookieJar(unsafe=True)) as client:
        assert (await client.post('/api/pair',json={'code':'123456'})).status==200
        old=await client.ws_connect('/api/events')
        assert (await asyncio.wait_for(old.receive_json(),1))['type']=='state'
        cancel_next=True
        await asyncio.sleep(.3)
        refreshed=await client.ws_connect('/api/events')
        frame=await asyncio.wait_for(refreshed.receive_json(),1)
        assert frame['type']=='state' and frame['state']['localized']
        await old.close();await refreshed.close()


@pytest.mark.asyncio
async def test_installed_symlink_assets_are_served_without_opening_other_paths(tmp_path):
    c,_,_=make_console()
    source=tmp_path/'source';source.mkdir()
    (source/'app.js').write_text('console.log("installed")')
    static=tmp_path/'installed';(static/'assets').mkdir(parents=True)
    (static/'index.html').write_text('<script src="/assets/app.js"></script>')
    (static/'assets/app.js').symlink_to(source/'app.js')
    assets=type('Assets',(),{'description':{'revision':'test'}})()
    async with TestClient(TestServer(create_app(c,assets,'123456',static))) as client:
        response=await client.get('/assets/app.js')
        assert response.status==200
        assert await response.text()=='console.log("installed")'
        assert (await client.get('/assets/not-installed.js')).status==404
        assert (await client.get('/assets/%2E%2E%2Fsource%2Fapp.js')).status==404


@pytest.mark.asyncio
async def test_pairing_origin_and_websocket_controller_boundary(tmp_path):
    c,a,_=make_console();c.owner=None
    assets=type('Assets',(),{'description':{'revision':'test'}})()
    async with TestClient(TestServer(create_app(c,assets,'123456',tmp_path)),cookie_jar=CookieJar(unsafe=True)) as client:
        assert (await client.get('/api/state')).status==401
        assert (await client.post('/api/pair',json={'code':'123456'},headers={'Origin':'https://attacker.invalid'})).status==403
        assert (await client.post('/api/pair',json={'code':'123456'})).status==200
        ws=await client.ws_connect('/api/events');await ws.send_json({'type':'claim'})
        owned=await receive_type(ws,'ownership');assert owned['owner']
        another=await client.ws_connect('/api/events');await another.send_json({'type':'claim'})
        assert (await receive_type(another,'error'))['type']=='error'
        assert (await client.post('/api/plans',json={'goal':[1,0]})).status==409
        response=await client.post('/api/plans',json={'goal':[1,0]},headers={'X-Mower-Control':owned['controller']})
        assert response.status==200
        await ws.close();await another.close()


@pytest.mark.asyncio
async def test_page_refresh_always_gets_current_first_state(tmp_path):
    c,a,_=make_console()
    assets=type('Assets',(),{'description':{'revision':'test'}})()
    async with TestClient(TestServer(create_app(c,assets,'123456',tmp_path)),cookie_jar=CookieJar(unsafe=True)) as client:
        assert (await client.post('/api/pair',json={'code':'123456'})).status==200
        for i in range(8):
            a.data['pose']=[float(i),0.,0.]
            ws=await client.ws_connect('/api/events')
            frame=await receive_type(ws,'state')
            assert frame['state']['pose'][0]==i
            await ws.close()


@pytest.mark.asyncio
async def test_slow_observer_does_not_block_another_page(tmp_path,monkeypatch):
    c,_,_=make_console()
    assets=type('Assets',(),{'description':{'revision':'test'}})()
    original=web.WebSocketResponse.send_json
    old_peer=None
    blocked=False

    async def send(ws,data,*args,**kwargs):
        nonlocal old_peer
        if old_peer is None:
            old_peer=ws
        if blocked and ws is old_peer:
            await asyncio.Event().wait()
        return await original(ws,data,*args,**kwargs)

    monkeypatch.setattr(web.WebSocketResponse,'send_json',send)
    async with TestClient(TestServer(create_app(c,assets,'123456',tmp_path)),cookie_jar=CookieJar(unsafe=True)) as client:
        assert (await client.post('/api/pair',json={'code':'123456'})).status==200
        old=await client.ws_connect('/api/events')
        await receive_type(old,'state')
        blocked=True
        refreshed=await client.ws_connect('/api/events')
        assert (await receive_type(refreshed,'state'))['state']['localized']
        await refreshed.close();await old.close()
