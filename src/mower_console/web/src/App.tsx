import {useEffect,useState} from 'react';
import {api,Goal,goalArray,goalFromArray,sameGoal} from './types';
import {actionAvailability} from './consoleModel';
import {useConsoleSession} from './useConsoleSession';
import {MapWorkspace} from './MapWorkspace';
import {TaskPanel} from './TaskPanel';
import {PairDialog,StartDialog,VehicleDrawer} from './Dialogs';
import {Button,Icon,IconButton} from './ui';

export function App() {
  const [view,setView]=useState<'2d'|'3d'>('2d');
  const session=useConsoleSession(view==='3d'),{state,connected,owner,pending}=session;
  const [goal,setGoal]=useState<Goal|null|undefined>(undefined);
  const [details,setDetails]=useState(false),[hintMode,setHintMode]=useState(false),[seed,setSeed]=useState(['','','0']);
  const [confirmation,setConfirmation]=useState<string|null>(null);
  const shownGoal=goal===undefined?goalFromArray(state?.plan?.goal):goal;
  const availability=actionAvailability({state,connected,owner,pending:!!pending,goal:shownGoal});
  const confirmationValid=actionAvailability({state,connected,owner,pending:false,goal:shownGoal}).canStart;
  const confirmationKey=JSON.stringify([state?.boot_id,state?.plan?.id,state?.generation,shownGoal]);
  useEffect(()=>{setGoal(undefined);setConfirmation(null);setHintMode(false);},[state?.boot_id,session.map?.revision]);
  useEffect(()=>{if(!confirmationValid||confirmation!==confirmationKey)setConfirmation(null);},[confirmationValid,confirmationKey,confirmation]);
  const setHeading=(yaw:number|null)=>{if(!availability.active&&shownGoal)setGoal({...shownGoal,yaw});};
  const select=(xy:number[])=>{
    if(availability.active)return;
    if(hintMode){setSeed([xy[0].toFixed(2),xy[1].toFixed(2),seed[2]]);setHintMode(false);setDetails(true);}
    else setGoal(g=>({x:xy[0],y:xy[1],yaw:g?.yaw??null}));
  };
  const start=async(checks:boolean[])=>{
    if(!availability.canStart||confirmation!==confirmationKey||!state?.plan||!sameGoal(state.plan.goal,shownGoal))return;
    const ok=await session.perform('start',()=>api('tasks',{plan_id:state.plan!.id,confirmations:{route_clear:checks[0],stop_ready:checks[1],actuators_isolated:checks[2]}}));
    if(ok)setConfirmation(null);
  };
  const overlay=session.auth==='required'||details||confirmation!==null;
  return <>
    <div className="app-shell" inert={overlay}>
      <header className="global-nav"><a className="brand" href="/" aria-label="MobileEss 控制台"><svg width="22" height="22" viewBox="0 0 24 24" fill="none" aria-hidden="true"><path d="M3 18V6l9 8 9-8v12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/></svg><span>MobileEss</span></a><span className="nav-caption">导航控制台</span><div className="connection-state"><i className={`status-dot ${connected?'connected':''}`}/><span>{connected?(state?.demo?'离线演示已连接':'板端已连接'):'正在连接板端'}</span></div></header>
      <nav className="context-bar" aria-label="控制台操作"><div className="context-title"><strong>导航</strong><span className="mode-chip"><Icon name={state?.motion_enabled?'route':'lock'} size={13}/>{state?.demo?'离线预览':state?.motion_enabled?'运动已启用':'观测模式'}</span></div><div className="context-actions"><Button variant="quiet" aria-label="车辆详情" onClick={()=>setDetails(true)}><Icon name="info" size={17}/><span>车辆详情</span></Button>{owner?<span className="ownership-chip"><Icon name="check" size={15}/>已取得控制权</span>:<Button disabled={!connected||session.auth!=='paired'} onClick={session.claim}>取得控制权<Icon name="arrow" size={16}/></Button>}</div></nav>
      <main className="workspace"><MapWorkspace map={session.map} state={state} connected={connected} goal={shownGoal} clouds={session.clouds} view={view} hintMode={hintMode} mapError={session.mapError} onRetry={session.retryMap} onView={v=>{setView(v);setHintMode(false);}} onSelect={select} onHeading={setHeading} onCancelHint={()=>setHintMode(false)}/>
        <TaskPanel state={state} connected={connected} owner={owner} goal={shownGoal} pending={pending} stopping={session.stopping} availability={availability} onHeading={setHeading} onClear={()=>setGoal(null)} onDetails={()=>setDetails(true)} onPreview={()=>{if(availability.canPreview&&shownGoal)void session.perform('preview',()=>api('plans',{goal:goalArray(shownGoal)}));}} onStart={()=>{if(availability.canStart)setConfirmation(confirmationKey);}} onStop={session.stop}/>
      </main>
    </div>
    {session.auth==='checking'&&<div className="initial-status" role="status"><span className="loader"/>正在连接控制台…</div>}
    {session.auth==='required'&&<PairDialog busy={pending==='pair'} error={session.error} onPair={session.pair}/>}
    {details&&session.auth!=='required'&&<VehicleDrawer state={state} connected={connected} owner={owner} busy={!!pending} canRelocalize={availability.canRelocalize} seed={seed} onSeed={setSeed} onClose={()=>setDetails(false)} onHint={()=>{setView('2d');setHintMode(true);setDetails(false);}} onRelocalize={hint=>{if(availability.canRelocalize)void session.perform('localize',()=>api('localization/relocalize',hint?{seed:hint}:{}));}}/>}
    {confirmation!==null&&session.auth==='paired'&&<StartDialog key={confirmation} busy={pending==='start'} canStart={availability.canStart} onClose={()=>setConfirmation(null)} onStart={start}/>}
    {session.error&&session.auth!=='required'&&<div className="toast" role="alert"><Icon name="alert" size={20}/><span>{session.error}</span><IconButton icon="close" label="关闭提示" onClick={()=>session.setError('')}/></div>}
  </>;
}
