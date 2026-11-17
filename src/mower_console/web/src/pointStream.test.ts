import {afterEach,beforeEach,expect,test,vi} from 'vitest';
import {decodePoints,startPointStream} from './pointStream';
import {startStateStream} from './stateStream';

class Socket {
  static OPEN=1;
  static instances:Socket[]=[];
  readyState=0;
  sent:string[]=[];
  binaryType='blob';
  onopen:(()=>void)|null=null;
  onclose:(()=>void)|null=null;
  onmessage:((event:{data:string|ArrayBuffer})=>void)|null=null;
  constructor(readonly url:string){Socket.instances.push(this);}
  open(){this.readyState=1;this.onopen?.();}
  receive(data:string|ArrayBuffer){this.onmessage?.({data});}
  send(data:string){this.sent.push(data);}
  close(){if(this.readyState===3)return;this.readyState=3;this.onclose?.();}
}

const cleanup:Array<()=>void>=[];
function start(){
  const onPoints=vi.fn(),onReset=vi.fn();
  cleanup.push(startPointStream({url:'ws://console/api/points?cloud=1',onPoints,onReset}));
  return {socket:Socket.instances.at(-1)!,onPoints,onReset};
}
function frame(kind:number,values:number[],count=values.length/3){
  const buffer=new ArrayBuffer(8+values.length*4),view=new DataView(buffer);
  view.setUint8(0,kind);view.setUint32(4,count,true);
  values.forEach((v,i)=>view.setFloat32(i*4+8,v,true));
  return buffer;
}
beforeEach(()=>{
  vi.useFakeTimers({toFake:['setTimeout','clearTimeout','setInterval','clearInterval','performance']});
  Socket.instances=[];vi.stubGlobal('WebSocket',Socket);
});
afterEach(()=>{cleanup.splice(0).forEach(stop=>stop());vi.unstubAllGlobals();vi.useRealTimers();});

test('validates network frames before exposing point arrays',()=>{
  const decoded=decodePoints(frame(1,[1,2,3,4,5,6]));
  expect(decoded?.kind).toBe('cloud');expect(Array.from(decoded!.xyz)).toEqual([1,2,3,4,5,6]);
  expect(decodePoints(frame(2,[]))?.kind).toBe('obstacles');
  expect(decodePoints(frame(3,[1,2,3]))).toBeNull();
  expect(decodePoints(frame(1,[1,2,3],2))).toBeNull();
  expect(decodePoints(new ArrayBuffer(4))).toBeNull();
});

test('waits for a full batch before requesting the next display update',async()=>{
  const {socket,onPoints}=start();socket.open();
  expect(socket.sent).toEqual(['next']);
  socket.receive(frame(2,[1,2,0]));await vi.advanceTimersByTimeAsync(500);
  expect(socket.sent).toEqual(['next']);
  expect(onPoints).toHaveBeenCalledWith('obstacles',new Float32Array([1,2,0]));
  socket.receive('{"type":"ready"}');await vi.advanceTimersByTimeAsync(200);
  expect(socket.sent).toEqual(['next','next']);
});

test('a stalled display stream clears points but leaves state ownership and heartbeats intact',async()=>{
  const onOwnership=vi.fn(),onConnected=vi.fn();
  cleanup.push(startStateStream({url:'ws://console/api/events',readState:async()=>({}),
    onState:vi.fn(),onConnected,onOwnership,onSocket:vi.fn(),onAuthRequired:vi.fn(),onError:vi.fn()}));
  const control=Socket.instances[0];control.open();
  control.receive('{"type":"ownership","owner":true,"controller":"test"}');
  const {socket,onReset}=start();socket.open();
  for(let i=0;i<5;i++){
    control.receive('{"type":"state","state":{}}');
    await vi.advanceTimersByTimeAsync(250);
  }
  expect(onReset).toHaveBeenCalledOnce();expect(socket.readyState).toBe(3);
  expect(control.readyState).toBe(1);expect(control.sent).toHaveLength(5);
  expect(onOwnership).toHaveBeenLastCalledWith(true,'test');
  expect(onConnected).toHaveBeenLastCalledWith(true);
});

test('reconnect rejects late old batches and cleanup leaves no polling',async()=>{
  const {socket,onPoints}=start();socket.open();
  await vi.advanceTimersByTimeAsync(2500);
  const next=Socket.instances.at(-1)!;expect(next).not.toBe(socket);
  socket.receive(frame(1,[9,9,9]));expect(onPoints).not.toHaveBeenCalled();
  next.open();next.receive('{"type":"ready"}');
  cleanup.pop()!();await vi.advanceTimersByTimeAsync(5000);
  expect(next.sent).toEqual(['next']);expect(next.readyState).toBe(3);
  expect(Socket.instances).toHaveLength(2);
});
