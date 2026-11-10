import {afterEach,beforeEach,describe,expect,it,vi} from 'vitest';
import {startStateStream} from './stateStream';

class Socket {
  static OPEN=1;
  static instances:Socket[]=[];
  readyState=0;
  sent:string[]=[];
  onopen:(()=>void)|null=null;
  onclose:(()=>void)|null=null;
  onmessage:((event:{data:string})=>void)|null=null;
  constructor(readonly url:string){Socket.instances.push(this)}
  open(){this.readyState=1;this.onopen?.()}
  receive(data:unknown){this.onmessage?.({data:JSON.stringify(data)})}
  send(data:string){this.sent.push(data)}
  close(){if(this.readyState===3)return;this.readyState=3;this.onclose?.()}
}

const cleanup:Array<()=>void>=[];
function start(readState=vi.fn(async()=>({localized:true}))) {
  const options={url:'ws://console/api/events',readState,onState:vi.fn(),onConnected:vi.fn(),
    onOwnership:vi.fn(),onSocket:vi.fn(),onAuthRequired:vi.fn(),onError:vi.fn()};
  const stop=startStateStream(options);cleanup.push(stop);
  return {options,stop,socket:Socket.instances.at(-1)!};
}

beforeEach(()=>{
  vi.useFakeTimers({toFake:['setTimeout','clearTimeout','setInterval','clearInterval','performance']});
  Socket.instances=[];vi.stubGlobal('WebSocket',Socket);
});
afterEach(()=>{cleanup.splice(0).forEach(stop=>stop());vi.unstubAllGlobals();vi.useRealTimers()});

describe('state connection after refresh',()=>{
  it('does not overwrite a live frame with a slower bootstrap response',async()=>{
    let respond:(value:{localized:boolean})=>void=()=>{};
    const read=vi.fn(()=>new Promise<{localized:boolean}>(resolve=>{respond=resolve}));
    const {socket,options}=start(read);socket.open();
    socket.receive({type:'state',state:{localized:false}});
    respond({localized:true});await Promise.resolve();
    expect(options.onState).toHaveBeenLastCalledWith({localized:false});
  });

  it('hydrates from HTTP before the first stream frame and marks live only on a frame',async()=>{
    const {socket,options}=start();socket.open();await Promise.resolve();
    expect(options.onState).toHaveBeenCalledWith({localized:true});
    expect(options.onConnected).not.toHaveBeenCalledWith(true);
    socket.receive({type:'state',state:{localized:true}});
    expect(options.onConnected).toHaveBeenLastCalledWith(true);
    expect(socket.sent).toEqual([]);
  });

  it('reconnects a socket that opens without ever delivering state',async()=>{
    const {socket,options}=start();socket.open();await Promise.resolve();
    await vi.advanceTimersByTimeAsync(1000);
    expect(socket.readyState).toBe(3);
    expect(options.onOwnership).toHaveBeenLastCalledWith(false,'');
    await vi.advanceTimersByTimeAsync(1500);
    expect(Socket.instances).toHaveLength(2);
  });

  it('revokes ownership and stops heartbeats when state delivery stalls',async()=>{
    const {socket,options}=start();socket.open();
    socket.receive({type:'state',state:{localized:true}});
    socket.receive({type:'ownership',owner:true,controller:'test-controller'});
    await vi.advanceTimersByTimeAsync(250);
    expect(socket.sent).toEqual([JSON.stringify({type:'heartbeat'})]);
    await vi.advanceTimersByTimeAsync(750);
    expect(socket.readyState).toBe(3);
    expect(options.onOwnership).toHaveBeenLastCalledWith(false,'');
    const count=socket.sent.length;
    await vi.advanceTimersByTimeAsync(1500);
    expect(socket.sent).toHaveLength(count);
    const next=Socket.instances.at(-1)!;next.open();next.receive({type:'state',state:{localized:true}});
    expect(next.sent).toEqual([]); // A reconnect never claims or restarts a task.
  });

  it('requests pairing after service restart and leaves no reconnect loop',async()=>{
    const {socket,options}=start(vi.fn(async()=>{throw Error('请先配对')}));
    socket.open();await vi.advanceTimersByTimeAsync(0);
    expect(options.onAuthRequired).toHaveBeenCalledOnce();
    expect(socket.readyState).toBe(3);
    await vi.advanceTimersByTimeAsync(5000);
    expect(Socket.instances).toHaveLength(1);
  });

  it('refresh cleanup closes the old socket without leaving retry timers',async()=>{
    const first=start();first.socket.open();first.stop();
    const next=start();next.socket.open();next.socket.receive({type:'state',state:{localized:true}});
    await vi.advanceTimersByTimeAsync(500);
    expect(first.socket.readyState).toBe(3);
    expect(Socket.instances).toHaveLength(2);
    expect(next.options.onState).toHaveBeenCalledWith({localized:true});
  });
});
