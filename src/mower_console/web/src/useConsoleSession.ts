import {useCallback,useEffect,useRef,useState} from 'react';
import {api,Clouds,MapData,setController,State} from './types';
import {startStateStream} from './stateStream';
import {startPointStream} from './pointStream';

const emptyClouds:Clouds={cloud:new Float32Array(0),obstacles:new Float32Array(0)};
export function useConsoleSession(showCloud:boolean) {
  const [auth,setAuth]=useState<'checking'|'required'|'paired'>('checking');
  const [state,setState]=useState<State|null>(null),[map,setMap]=useState<MapData|null>(null);
  const [connected,setConnected]=useState(false),[owner,setOwner]=useState(false);
  const [clouds,setClouds]=useState(emptyClouds),[error,setError]=useState('');
  const [mapError,setMapError]=useState(''),[mapAttempt,setMapAttempt]=useState(0);
  const [pending,setPending]=useState(''),[stopping,setStopping]=useState(false);
  const socket=useRef<WebSocket|null>(null),requestLock=useRef(false),stopLock=useRef(false);
  const boot=useRef<string|undefined>(undefined);
  useEffect(()=>{
    let alive=true;
    api('state').then(s=>{if(alive){setState(s);setAuth('paired');}}).catch(()=>{if(alive)setAuth('required');});
    return()=>{alive=false;};
  },[]);
  useEffect(()=>{
    if(auth!=='paired') return;
    let alive=true;
    setMapError('');
    api('map').then(m=>{if(alive)setMap(m);}).catch(e=>{if(alive)setMapError(e.message);});
    return()=>{alive=false;};
  },[auth,mapAttempt]);
  useEffect(()=>{
    if(auth!=='paired') return;
    return startStateStream<State>({
      url:`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/events`,
      readState:()=>api('state'),
      onState:s=>{
        if(boot.current&&s.boot_id&&boot.current!==s.boot_id){
          setController('');setOwner(false);setClouds(emptyClouds);setMapAttempt(n=>n+1);
        }
        boot.current=s.boot_id;setState(s);
      },
      onConnected:value=>{setConnected(value);if(!value)setClouds(emptyClouds);},onSocket:s=>{socket.current=s;},
      onOwnership:(owns,token)=>{setOwner(owns);setController(token);},
      onAuthRequired:()=>{setAuth('required');setState(null);setClouds(emptyClouds);setMap(null);},
      onError:setError,
    });
  },[auth]);
  useEffect(()=>{
    if(auth!=='paired'||!connected)return;
    return startPointStream({
      url:`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/points?cloud=${showCloud?1:0}`,
      onPoints:(kind,xyz)=>setClouds(c=>({...c,[kind]:xyz})),
      onReset:()=>setClouds(emptyClouds),
    });
  },[auth,connected,showCloud]);
  const perform=useCallback(async(name:string,action:()=>Promise<unknown>)=>{
    if(requestLock.current)return false;
    requestLock.current=true;setPending(name);setError('');
    try{await action();return true;}catch(e){setError(e instanceof Error?e.message:'请求失败，请重试');return false;}
    finally{requestLock.current=false;setPending('');}
  },[]);
  const pair=useCallback((code:string)=>perform('pair',async()=>{await api('pair',{code});setAuth('paired');}),[perform]);
  const stop=useCallback(async()=>{
    if(!connected||!state?.task||stopLock.current)return;
    stopLock.current=true;setStopping(true);setError('');
    try{await api(`tasks/${state.task.id}/stop`,{});}catch(e){setError(e instanceof Error?e.message:'停止请求失败，请使用现场停止');}
    finally{stopLock.current=false;setStopping(false);}
  },[connected,state?.task?.id]);
  const claim=()=>{if(socket.current?.readyState===WebSocket.OPEN){setError('');socket.current.send(JSON.stringify({type:'claim'}));}};
  return {auth,state,map,connected,owner,clouds,error,setError,mapError,retryMap:()=>setMapAttempt(n=>n+1),pending,stopping,perform,pair,stop,claim};
}
