/** Keep display data and control ownership separate across refresh/reconnect. */
type StateStreamOptions<T> = {
  url: string;
  readState: () => Promise<T>;
  onState: (state: T) => void;
  onConnected: (connected: boolean) => void;
  onOwnership: (owner: boolean, controller: string) => void;
  onSocket: (socket: WebSocket|null) => void;
  onAuthRequired: () => void;
  onError: (message: string) => void;
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
    const socket=new WebSocket(options.url);current=socket;
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
