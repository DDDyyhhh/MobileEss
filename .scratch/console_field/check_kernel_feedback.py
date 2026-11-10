#!/usr/bin/env python3
"""RK3588-only passive comparison under four observer connections."""
import platform
if platform.machine() not in ("aarch64","arm64"):
    raise SystemExit("Run on RK3588 only")
import asyncio,json,time,socket,struct,threading
from pathlib import Path
from datetime import datetime,timezone
import aiohttp
from mower_console.can_feedback import TIMESTAMPNS,arrival_time,stationary_evidence
frames=[];samples=[];done=threading.Event();rx_errors=[];peer_counts=[0]*4
def receive():
    try:
        with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as bus:
            mask=socket.CAN_SFF_MASK|socket.CAN_EFF_FLAG|socket.CAN_RTR_FLAG
            bus.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,b''.join(struct.pack('=II',cid,mask) for cid in (5,0x507,0x50a)))
            bus.setsockopt(socket.SOL_SOCKET,TIMESTAMPNS,1);bus.bind(('can0',));bus.settimeout(.1)
            while not done.is_set():
                try:raw,ancillary,flags,_=bus.recvmsg(16,socket.CMSG_SPACE(16))
                except socket.timeout:continue
                before=time.monotonic();wall=time.time_ns();after=time.monotonic()
                stamp=arrival_time(ancillary,(before+after)/2,wall)
                if stamp is None or flags&socket.MSG_CTRUNC or after-before>.01:
                    rx_errors.append('invalid timestamp');continue
                cid,dlc,data=struct.unpack('=IB3x8s',raw)
                if dlc==8:frames.append((stamp,cid,bytes(data),after))
    except Exception as exc:rx_errors.append(str(exc))
async def main():
    thread=threading.Thread(target=receive);thread.start();peers=[];readers=[]
    try:
        async with aiohttp.ClientSession(cookie_jar=aiohttp.CookieJar(unsafe=True)) as session:
            base='http://127.0.0.1:8080';pin=(Path.home()/'.config/mower_console/pairing-code').read_text().strip()
            async with session.post(base+'/api/pair',json={'code':pin}) as r:r.raise_for_status()
            async def consume(ws,index):
                async for msg in ws:
                    if msg.type==aiohttp.WSMsgType.TEXT:peer_counts[index]+=json.loads(msg.data).get('type')=='state'
            for index in range(4):
                ws=await session.ws_connect(base+'/api/events');peers.append(ws);readers.append(asyncio.create_task(consume(ws,index)))
            deadline=time.monotonic()+90
            while time.monotonic()<deadline:
                begin=time.monotonic()
                async with session.get(base+'/api/state') as r:r.raise_for_status();state=await r.json()
                fields=('stationary','stationary_evidence','can_receive_lag_s','can_max_receive_lag_s','can_timestamp_errors','healthy','fault','localized','localization','generation','api_mode','wheel_mm_s','motion_enabled','task','pose','boot_id')
                samples.append({'t':time.monotonic(),'request_s':time.monotonic()-begin,**{k:state.get(k) for k in fields}})
                await asyncio.sleep(.05)
            for task in readers:task.cancel()
            await asyncio.gather(*readers,return_exceptions=True)
            for ws in peers:await ws.close()
    finally:done.set();thread.join(timeout=2)
    disagreements=[];compared=0;reasons={}
    for sample in samples:
        evidence=sample['stationary_evidence'];at=evidence['evaluated_at']
        if not frames or at<frames[0][0]+1.5:continue
        compared+=1
        wire=stationary_evidence([(t,c,d) for t,c,d,_ in frames if t<=at],at)
        if not sample['stationary']:reasons[evidence['reason']]=reasons.get(evidence['reason'],0)+1
        if wire['ready']!=sample['stationary']:disagreements.append({'sample':sample,'independent':wire})
    speeds=[struct.unpack('>hhhh',d) for _,c,d,_ in frames if c==0x50a]
    summary={'samples':len(samples),'compared_after_warmup':compared,'backend_independent_disagreements':disagreements,'backend_rejection_reasons':reasons,'independent_rx_errors':rx_errors,'peer_frames':peer_counts,'control_frames':sum(c==5 for _,c,_,_ in frames),'wheel_max_abs_mm_s':max(abs(v) for row in speeds for v in row),'max_http_s':max(s['request_s'] for s in samples),'last':samples[-1]}
    output=Path('/home/cat/MobileEss_data/console-kernel-time-observer-20261021.json')
    output.write_text(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'summary':summary,'samples':samples,'can_frames':[{'t':t,'id':c,'data':d.hex(),'received_t':received} for t,c,d,received in frames]},indent=2))
    print(json.dumps(summary,indent=2))
    assert not disagreements and not rx_errors
    assert all(n>100 for n in peer_counts)
    assert all(s['can_timestamp_errors']==0 and not s['motion_enabled'] and s['api_mode']==0 and s['task'] is None for s in samples)
    assert summary['control_frames']==0
asyncio.run(main())

