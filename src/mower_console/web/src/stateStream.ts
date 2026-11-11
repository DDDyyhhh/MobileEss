/** Keep display data and control ownership separate across refresh/reconnect. */
export type PointKind = 'cloud'|'obstacles';

/** Binary frame from the board: u8 kind, 3 pad bytes, u32 count, float32 xyz. */
export function decodePoints(buffer: ArrayBuffer): {kind: PointKind; xyz: Float32Array}|null {
  if(buffer.byteLength<8)return null;
  const view=new DataView(buffer),code=view.getUint8(0),count=view.getUint32(4,true);
  const kind=code===1?'cloud':code===2?'obstacles':null;
  if(!kind||buffer.byteLength!==8+count*12)return null;
  return {kind,xyz:new Float32Array(buffer,8,count*3)};
}

type StateStreamOptions<T> = {
  url: string;
  readState: () => Promise<T>;
  onState: (state: T) => void;
  onConnected: (connected: boolean) => void;
  onOwnership: (owner: boolean, controller: string) => void;
  onSocket: (socket: WebSocket|null) => void;
  onAuthRequired: () => void;
  onError: (message: string) => void;
  onPoints?: (kind: PointKind, xyz: Float32Array) => void;
};

export function startStateStream<T>(options: StateStreamOptions<T>) {
  let stopped=false, current:WebSocket|null=null, owns=false;
  let retry:ReturnType<typeof setTimeout>|undefined;
  let watchdog:ReturnType<typeof setInterval>|undefined;
  const revoke=()=>{owns=false;options.onOwnership(false,'');};

  const stop=()=>{
    stopped=true;clearTimeout(retry);clearInterval(watchdog);
    revoke();options.onConnected(false);options.onSocket(null);
    const socket=current;current=null;socket?.close();
  };
  const snapshot=(expected:WebSocket|null,accept=()=>true)=>{
    options.readState().then(state=>{
      if(!stopped&&current===expected&&accept())options.onState(state);
    }).catch(error=>{
      if(!stopped&&current===expected&&String(error.message).includes('配对')){
        stop();options.onAuthRequired();
      }
    });
  };

  const connect=()=>{
    if(stopped)return;
    const socket=new WebSocket(options.url);current=socket;socket.binaryType='arraybuffer';
    let lastState:number|null=null;
    const opened=performance.now();
    revoke();options.onConnected(false);options.onSocket(socket);
    watchdog=setInterval(()=>{
      if(stopped||current!==socket)return;
      if(performance.now()-(lastState??opened)>=1000){
        revoke();options.onConnected(false);socket.close();return;
      }
      if(lastState!==null&&owns&&socket.readyState===WebSocket.OPEN)
        socket.send(JSON.stringify({type:'heartbeat'}));
    },250);
    socket.onopen=()=>{if(!stopped&&current===socket)snapshot(socket,()=>lastState===null);};
    socket.onmessage=event=>{
      if(stopped||current!==socket)return;
      // Point frames are display data only; they never keep the stream alive.
      if(event.data instanceof ArrayBuffer){
        const points=decodePoints(event.data);
        if(points)options.onPoints?.(points.kind,points.xyz);
        return;
      }
      let message;
      try{message=JSON.parse(event.data);}catch{socket.close();return;}
      if(message.type==='state'&&message.state){
        lastState=performance.now();options.onState(message.state);options.onConnected(true);
      }else if(message.type==='ownership'){
        owns=message.owner===true;options.onOwnership(owns,owns?message.controller:'');
      }else if(message.type==='error'){
        if(String(message.error).includes('没有控制权'))revoke();
        else options.onError(String(message.error));
      }
    };
    socket.onclose=()=>{
      if(stopped||current!==socket)return;
      clearInterval(watchdog);current=null;revoke();
      options.onConnected(false);options.onSocket(null);
      snapshot(null);retry=setTimeout(connect,1500);
    };
  };
  connect();
  return stop;
}
