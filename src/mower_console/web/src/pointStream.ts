export type PointKind = 'cloud'|'obstacles';

/** Binary frame: u8 kind, 3 pad bytes, u32 count, then float32 xyz. */
export function decodePoints(buffer:ArrayBuffer):{kind:PointKind;xyz:Float32Array}|null {
  if(buffer.byteLength<8)return null;
  const view=new DataView(buffer),code=view.getUint8(0),count=view.getUint32(4,true);
  const kind=code===1?'cloud':code===2?'obstacles':null;
  if(!kind||buffer.byteLength!==8+count*12)return null;
  return {kind,xyz:new Float32Array(buffer,8,count*3)};
}

type Options={url:string;onPoints:(kind:PointKind,xyz:Float32Array)=>void;onReset:()=>void};

/** Pull one display batch at a time, on a socket that never carries control. */
export function startPointStream({url,onPoints,onReset}:Options) {
  let stopped=false,current:WebSocket|null=null;
  let next:ReturnType<typeof setTimeout>|undefined,deadline:ReturnType<typeof setTimeout>|undefined;
  const connect=()=>{
    if(stopped)return;
    const socket=new WebSocket(url);current=socket;socket.binaryType='arraybuffer';
    const lost=()=>{
      if(stopped||current!==socket)return;
      current=null;clearTimeout(next);clearTimeout(deadline);onReset();socket.close();
      next=setTimeout(connect,1500);
    };
    const request=()=>{
      if(stopped||current!==socket)return;
      socket.send('next');deadline=setTimeout(lost,1000);
    };
    socket.onopen=request;
    socket.onclose=lost;
    socket.onmessage=event=>{
      if(stopped||current!==socket)return;
      if(event.data instanceof ArrayBuffer){
        const points=decodePoints(event.data);
        if(!points){lost();return;}
        onPoints(points.kind,points.xyz);
      }else{
        let message;
        try{message=JSON.parse(event.data);}catch{lost();return;}
        if(message?.type!=='ready'){lost();return;}
        clearTimeout(deadline);next=setTimeout(request,200);
      }
    };
  };
  connect();
  return()=>{
    stopped=true;clearTimeout(next);clearTimeout(deadline);
    current?.close();current=null;onReset();
  };
}
