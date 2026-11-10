"""Same-origin authenticated local HTTP API. ROS is imported only on board."""
from __future__ import annotations

import argparse
import asyncio
import hmac
import json
import logging
import os
from pathlib import Path
import secrets
import time

from aiohttp import web
import yaml

from .assets import MapAssets
from .domain import Console, Refused

logger = logging.getLogger(__name__)


def create_app(console, assets, pairing_code, static_dir):
    sessions, peers, controllers, streams = {}, set(), {}, set()

    @web.middleware
    async def boundary(request, handler):
        try:
            if request.method != 'GET':
                origin = request.headers.get('Origin')
                if origin and origin != f'{request.scheme}://{request.host}':
                    raise web.HTTPForbidden(text='跨站操作被拒绝')
            if request.path.startswith('/api/') and request.path != '/api/pair':
                session = request.cookies.get('mower_session')
                if session not in sessions:
                    raise web.HTTPUnauthorized(text='请先配对')
                controller = request.headers.get('X-Mower-Control')
                request['session'] = controller if controllers.get(controller) == session else session
                request['pair_session'] = session
            response = await handler(request)
            response.headers['X-Content-Type-Options'] = 'nosniff'
            response.headers['Referrer-Policy'] = 'same-origin'
            if request.path.startswith('/api/'):
                response.headers['Cache-Control'] = 'no-store'
            return response
        except (Refused, ValueError, KeyError, TypeError) as exc:
            return web.json_response({'error': str(exc)}, status=409)
        except asyncio.TimeoutError:
            return web.json_response({'error': '操作超时，未启动运动'}, status=504)

    app = web.Application(middlewares=[boundary], client_max_size=16384)

    async def pair(request):
        # Bound PIN attempts per remote address, not per generated session.
        now = time.monotonic()
        key = request.remote or 'local'
        attempts = app['attempts'].setdefault(key, [])
        attempts[:] = [t for t in attempts if now-t < 60]
        if len(attempts) >= 5:
            raise web.HTTPTooManyRequests(text='请一分钟后重试')
        attempts.append(now)
        body = await request.json()
        if not hmac.compare_digest(str(body.get('code', '')), pairing_code):
            raise web.HTTPForbidden(text='配对码不正确')
        sid = secrets.token_urlsafe(32)
        sessions[sid] = now
        response = web.json_response({'paired': True})
        response.set_cookie('mower_session', sid, httponly=True, samesite='Strict',
                            secure=request.secure, path='/')
        return response

    async def state(request):
        return web.json_response(console.state())

    async def map_info(request):
        return web.json_response(assets.description)

    async def points(request):
        try:
            data = await asyncio.to_thread(assets.points)
        except (OSError, ValueError) as exc:
            return web.json_response({'error': str(exc)}, status=503)
        return web.Response(body=data, content_type='application/octet-stream')

    async def plans(request):
        body = await request.json()
        return web.json_response(await console.preview(request['session'], body['goal']))

    async def tasks(request):
        body = await request.json()
        return web.json_response(await console.start(request['session'], body['plan_id'], body.get('confirmations', {})))

    async def stop(request):
        # Every paired observer may stop; only the owner may start.
        if console.task and request.match_info['id'] != console.task['id']:
            raise Refused('任务编号不匹配')
        return web.json_response(await console.stop())

    async def relocalize(request):
        body = await request.json()
        await console.relocalize(request['session'], body.get('seed'))
        return web.json_response({'accepted': True})

    async def events(request):
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden()
        ws = web.WebSocketResponse(heartbeat=5, max_msg_size=4096)
        await ws.prepare(request)
        peers.add(ws)
        sender = asyncio.create_task(publish(ws))
        streams.add(sender)
        sender.add_done_callback(streams.discard)
        controller = secrets.token_urlsafe(32)
        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                    if data.get('type') == 'claim':
                        console.claim(controller)
                        controllers[controller] = request['pair_session']
                        await ws.send_json({'type': 'ownership', 'owner': True, 'controller': controller})
                    elif data.get('type') == 'heartbeat':
                        console.beat(controller)
                    else:
                        raise Refused('未知操作')
                except (ValueError, Refused) as exc:
                    await ws.send_json({'type': 'error', 'error': str(exc)})
        finally:
            peers.discard(ws)
            controllers.pop(controller, None)
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
        return ws

    async def publish(peer):
        # One page owns one sender. A cancelled/slow old connection cannot
        # terminate or block the state stream of a refreshed page.
        last_cloud = -1.
        try:
            while not peer.closed:
                payload = {'type': 'state', 'state': console.state()}
                now = time.monotonic()
                if now-last_cloud < .5:
                    payload['state'].pop('cloud', None)
                else:
                    last_cloud = now
                await asyncio.wait_for(peer.send_json(payload), .5)
                await asyncio.sleep(.2)
        except (ConnectionError, asyncio.TimeoutError):
            logger.info('Closing an unresponsive console state connection')
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception('Console state connection failed')
        finally:
            peers.discard(peer)
            if not peer.closed:
                try:
                    await asyncio.wait_for(peer.close(), 1.)
                except (Exception, asyncio.CancelledError):
                    pass

    async def shutdown(app):
        await console.stop()
        pending = list(streams)
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)

    async def cleanup(app):
        console.adapter.close()

    app['attempts'] = {}
    app.router.add_post('/api/pair', pair)
    app.router.add_get('/api/state', state)
    app.router.add_get('/api/map', map_info)
    app.router.add_get('/api/map/points', points)
    app.router.add_get('/api/events', events)
    app.router.add_post('/api/plans', plans)
    app.router.add_post('/api/tasks', tasks)
    app.router.add_post('/api/tasks/{id}/stop', stop)
    app.router.add_post('/api/localization/relocalize', relocalize)
    static = Path(static_dir).resolve()
    async def index(request):
        return web.FileResponse(static/'index.html')
    app.router.add_get('/', index)
    if (static/'assets').is_dir():
        # colcon --symlink-install points installed assets into the source tree.
        # Serve a fixed filename allowlist rather than enabling directory-wide
        # symlink following on the board's older aiohttp release.
        installed_assets = {p.name: p.resolve() for p in (static/'assets').iterdir() if p.is_file()}
        async def asset(request):
            path = installed_assets.get(request.match_info['name'])
            if path is None:
                raise web.HTTPNotFound()
            return web.FileResponse(path)
        app.router.add_get('/assets/{name}', asset)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--demo', action='store_true', help='Offline display only; never opens CAN/ROS')
    parser.add_argument('--observe-only', action='store_true', help='Hard-disable task execution regardless of config')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--static', type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.observe_only:
        config['motion_enabled'] = False
        config['accepted_capabilities'] = dict(forward=False, reverse=False, rotate=False)
    assets = MapAssets(config['map_directory'])
    if args.demo:
        from .demo import DemoAdapter
        adapter = DemoAdapter(assets)
        config['motion_enabled'] = False
    else:
        from .ros_adapter import RosAdapter
        adapter = RosAdapter(assets, config)
    credential = Path(config.get('pairing_file', str(Path.home()/'.config/mower_console/pairing-code')))
    credential.parent.mkdir(parents=True, exist_ok=True)
    if not credential.exists():
        fd = os.open(credential, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(f'{secrets.randbelow(1000000):06d}\n')
    code = credential.read_text().strip()
    static = args.static
    if static is None:
        from ament_index_python.packages import get_package_share_directory
        static = Path(get_package_share_directory('mower_console'))/'web/dist'
    console = Console(adapter, config)
    web.run_app(create_app(console, assets, code, static), host=args.host, port=args.port,
                access_log=None, print=lambda msg: print(msg, flush=True))


if __name__ == '__main__':
    main()
