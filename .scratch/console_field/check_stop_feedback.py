#!/usr/bin/env python3
"""RK3588-only receive-only CAN/backend comparison; no search or motion calls."""
import platform
if platform.machine() not in ("aarch64", "arm64"):
    raise SystemExit("Run this passive integration check on RK3588")
import asyncio,json,time,socket,struct,threading
from pathlib import Path
from datetime import datetime,timezone
from collections import deque
from unittest.mock import patch
import aiohttp
from mower_console.ros_adapter import RosAdapter
rows=[];frames=[];done=threading.Event()
def receiver():
    with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as bus:
        bus.bind(('can0',));bus.settimeout(.1)
        while not done.is_set():
            try:raw=bus.recv(16)
            except socket.timeout:continue
            cid,dlc,data=struct.unpack('=IB3x8s',raw)
            if not cid&0xE0000000 and dlc==8 and cid in (5,0x507,0x50a):
                frames.append((time.monotonic(),cid,bytes(data)))
async def main():
    thread=threading.Thread(target=receiver);thread.start()
    try:
        async with aiohttp.ClientSession(cookie_jar=aiohttp.CookieJar(unsafe=True)) as session:
            pin=(Path.home()/'.config/mower_console/pairing-code').read_text().strip()
            base='http://127.0.0.1:8080'
            async with session.post(base+'/api/pair',json={'code':pin}) as r:r.raise_for_status()
            deadline=time.monotonic()+60
            while time.monotonic()<deadline:
                begin=time.monotonic()
                async with session.get(base+'/api/state') as r:r.raise_for_status();s=await r.json()
                rows.append({'t':time.monotonic(),'begin':begin,**{k:s.get(k) for k in ('stationary','healthy','fault','localized','localization','generation','pose','api_mode','wheel_mm_s','motion_enabled','task','boot_id')}})
                await asyncio.sleep(.05)
    finally:
        done.set();thread.join(timeout=2)
    adapter=RosAdapter.__new__(RosAdapter);adapter.lock=threading.RLock()
    mismatches=[]
    for row in rows:
        adapter.events=deque((t,c,d) for t,c,d in frames if t<=row['t'])
        with patch('mower_console.ros_adapter.time.monotonic',return_value=row['t']):
            independent=adapter.stationary()
        row['independent_stationary']=independent
        if independent and not row['stationary']:mismatches.append(row)
    result={'utc':datetime.now(timezone.utc).isoformat(),'samples':rows,'frames':[dict(t=t,id=c,data=d.hex()) for t,c,d in frames],'summary':{'samples':len(rows),'backend_not_stationary':sum(not r['stationary'] for r in rows),'backend_unhealthy':sum(not r['healthy'] for r in rows),'independent_vs_backend_mismatches':mismatches,'control_frames':sum(c==5 for _,c,_ in frames),'max_http_s':max(r['t']-r['begin'] for r in rows),'final':rows[-1]}}
    Path('/home/cat/MobileEss_data/console-A3-stop-observer-after-20261021.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result['summary'],indent=2))
    assert not mismatches,'backend stationary disagreed with independent CAN'
    assert all(not r['motion_enabled'] and r['api_mode']==0 and r['task'] is None for r in rows)
    assert result['summary']['control_frames']==0
asyncio.run(main())

