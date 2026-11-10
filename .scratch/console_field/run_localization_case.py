#!/usr/bin/env python3
"""Run exactly one operator-prepared, observation-only field localization case.

Execute on RK3588 after sourcing ROS and the console UDP DDS profile.
Records real topics and receive-only CAN; never starts a motion task.
"""
import argparse
import asyncio
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import socket
import struct
import subprocess
import threading
import time

import aiohttp


def brief(state):
    keys = ('phase', 'localized', 'localization', 'generation', 'pose', 'healthy',
            'fault', 'stationary', 'api_mode', 'wheel_mm_s', 'motion_enabled',
            'map_revision', 'capabilities', 'task', 'boot_id', 'stationary_evidence',
            'can_receive_lag_s', 'can_max_receive_lag_s', 'can_timestamp_errors')
    return {key: state.get(key) for key in keys}


async def run(args):
    root = Path('/home/cat/MobileEss_data')
    prefix = root / ('console-field-' + args.case + '-' +
                     datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    records, frames = [], []
    finished = threading.Event()
    errors = []
    result = dict(case=args.case, kind='real_live_stationary_full_map_search',
                  utc=datetime.now(timezone.utc).isoformat(),
                  operator_setup_confirmation=args.setup_confirmation,
                  operator_pose_confirmation='pending', bag=str(prefix) + '-bag',
                  started_search=False)

    def can_observer():
        try:
            from mower_console.can_feedback import TIMESTAMPNS, arrival_time
            with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                               b''.join(struct.pack('=II', cid, mask) for cid in (5, 0x507, 0x50A)))
                bus.setsockopt(socket.SOL_SOCKET, TIMESTAMPNS, 1)
                bus.bind(('can0',))
                bus.settimeout(.1)
                while not finished.is_set():
                    try:
                        raw, ancillary, flags, _ = bus.recvmsg(16, socket.CMSG_SPACE(16))
                    except socket.timeout:
                        continue
                    cid, dlc, data = struct.unpack('=IB3x8s', raw)
                    if not cid & 0xE0000000 and dlc == 8 and cid in (5, 0x507, 0x50A):
                        before = time.monotonic()
                        wall = time.time_ns()
                        after = time.monotonic()
                        timestamp = arrival_time(ancillary, (before+after)/2., wall)
                        if timestamp is None or flags & socket.MSG_CTRUNC or after-before > .01:
                            raise RuntimeError('Independent CAN timestamp unavailable')
                        frames.append(dict(t=timestamp, received_t=after, id=cid, data=data.hex()))
        except Exception as exc:
            errors.append(str(exc))

    bag, thread, heartbeat, receiver = None, None, None, None
    log = prefix.with_suffix('.log').open('x')
    try:
        async with aiohttp.ClientSession(cookie_jar=aiohttp.CookieJar(unsafe=True)) as session:
            base = 'http://127.0.0.1:8080'
            pin = (Path.home() / '.config/mower_console/pairing-code').read_text().strip()
            async with session.post(base + '/api/pair', json={'code': pin}) as response:
                response.raise_for_status()

            async def state():
                async with session.get(base + '/api/state', timeout=aiohttp.ClientTimeout(total=3)) as response:
                    response.raise_for_status()
                    current = await response.json()
                records.append(dict(t=time.monotonic(), **brief(current)))
                if errors:
                    raise RuntimeError('CAN observation failed: ' + errors[0])
                if any(frame['id'] == 5 for frame in frames):
                    raise RuntimeError('An external CAN control sender was observed')
                if current['motion_enabled'] or any(current['capabilities'].values()):
                    raise RuntimeError('Observation-only configuration required')
                if current['api_mode'] != 0 or current['task'] is not None:
                    raise RuntimeError('API0 and no task required')
                if not current['stationary'] or not current['wheel_mm_s'] or any(abs(v) > 10 for v in current['wheel_mm_s']):
                    raise RuntimeError('Fresh stationary feedback required')
                return current

            initial = await state()
            result['before'] = brief(initial)
            async with session.ws_connect(base + '/api/events') as ws:
                await ws.send_json({'type': 'claim'})
                controller = None
                until = time.monotonic() + 5
                while time.monotonic() < until:
                    message = await ws.receive_json(timeout=2)
                    if message.get('type') == 'error':
                        raise RuntimeError(message['error'])
                    if message.get('type') == 'ownership':
                        controller = message['controller']
                        break
                if not controller:
                    raise RuntimeError('Could not obtain temporary localization control')

                async def beats():
                    while True:
                        await ws.send_json({'type': 'heartbeat'})
                        await asyncio.sleep(.25)

                async def receive():
                    async for message in ws:
                        if message.type == aiohttp.WSMsgType.TEXT:
                            payload = json.loads(message.data)
                            if payload.get('type') == 'error':
                                raise RuntimeError(payload.get('error'))
                    raise RuntimeError('Localization control connection closed')

                heartbeat = asyncio.create_task(beats())
                receiver = asyncio.create_task(receive())
                thread = threading.Thread(target=can_observer)
                thread.start()
                topics = ['/cloud_registered', '/odometry/filtered',
                          '/fixed_map_localization/diagnostics',
                          '/fixed_map_localization/odometry', '/tf', '/tf_static',
                          '/mower_console/stationary']
                bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(prefix) + '-bag', *topics],
                                       stdout=log, stderr=log, start_new_session=True)
                await asyncio.sleep(3)
                if bag.poll() is not None:
                    raise RuntimeError('Passive recorder failed to start')
                current = await state()
                result['previous_generation'] = current['generation']
                expected = current['generation'] + 1
                started = time.monotonic()
                async with session.post(base + '/api/localization/relocalize', json={},
                                        headers={'X-Mower-Control': controller}) as response:
                    body = await response.json()
                    if response.status != 200:
                        raise RuntimeError(str(body))
                result['started_search'] = True
                print(json.dumps(dict(event='search_started', case=args.case,
                                      previous_generation=current['generation'], bag=result['bag'])), flush=True)
                accepted, invalidated = None, False
                while time.monotonic() - started < 128:
                    for task in (heartbeat, receiver):
                        if task.done():
                            task.result()
                            raise RuntimeError('Control monitoring stopped')
                    if bag.poll() is not None:
                        raise RuntimeError('Passive recorder exited during search')
                    current = await state()
                    if current['boot_id'] != initial['boot_id']:
                        raise RuntimeError('Console service restarted')
                    if current['generation'] > expected:
                        raise RuntimeError('Search generation changed unexpectedly; no automatic retry')
                    if not current['localized']:
                        invalidated = True
                    if current['generation'] == expected and current['localized'] and current['healthy']:
                        if accepted is None:
                            accepted = time.monotonic()
                            result.update(first_ready_s=accepted-started,
                                          accepted_generation=expected, after=brief(current))
                            print(json.dumps(dict(event='accepted', case=args.case,
                                                  seconds=result['first_ready_s'], pose=current['pose'])), flush=True)
                        if time.monotonic() - accepted >= 10:
                            result['stable_10s'] = True
                            break
                    elif accepted is not None:
                        raise RuntimeError('Localization lost during post-search stability window')
                    if current['localization'] == 'startup_timeout':
                        raise RuntimeError('Full-map search timed out')
                    await asyncio.sleep(.2)
                result['observed_invalidated'] = invalidated
                if accepted is None or not result.get('stable_10s') or not invalidated:
                    raise RuntimeError('Search or invalidation/stability check incomplete')
                result.update(software_result='PASS', field_pose_result='pending_operator_confirmation')
    except BaseException as exc:
        result.update(software_result='ERROR', error=str(exc))
        raise
    finally:
        for task in (heartbeat, receiver):
            if task:
                task.cancel()
        await asyncio.gather(*[t for t in (heartbeat, receiver) if t], return_exceptions=True)
        finished.set()
        if thread:
            thread.join(timeout=2)
        if bag and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                await asyncio.to_thread(bag.wait, 10)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGTERM)
                await asyncio.to_thread(bag.wait, 3)
        result.update(recorder_exit_code=bag.returncode if bag else None,
                      states=records, can_frames=frames,
                      control_frames=sum(f['id'] == 5 for f in frames))
        output = prefix.with_suffix('.json')
        output.write_text(json.dumps(result, indent=2, ensure_ascii=False))
        (root / ('console-field-' + args.case + '-latest.json')).write_text(json.dumps(
            dict(result=str(output), bag=result['bag'], log=str(prefix.with_suffix('.log'))), indent=2))
        log.close()
        print(json.dumps({k: v for k, v in result.items() if k not in ('states', 'can_frames')},
                         indent=2, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', required=True)
    parser.add_argument('--setup-confirmation', required=True)
    args = parser.parse_args()
    if platform.machine() not in ('aarch64', 'arm64'):
        parser.error('Run on RK3588 only')
    if not re.fullmatch(r'[A-E][1-4]', args.case):
        parser.error('Case must be A1 through E4')
    asyncio.run(run(args))
