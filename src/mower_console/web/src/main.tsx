import React,{useEffect,useRef,useState} from 'react';
import {createRoot} from 'react-dom/client';
import {MapCanvas} from './MapCanvas';
import {api,kinds,MapData,phases,State,setController} from './types';
import {startStateStream} from './stateStream';
import './style.css';
const ThreeView=React.lazy(()=>import('./ThreeView').then(m=>({default:m.ThreeView})));

function App(){
  const [paired,setPaired]=useState(false),[code,setCode]=useState(''),[error,setError]=useState('');
  const [map,setMap]=useState<MapData|null>(null),[state,setState]=useState<State|null>(null);
  const [connected,setConnected]=useState(false),[owner,setOwner]=useState(false),[goal,setGoal]=useState<number[]|null>(null),[view,setView]=useState('2d');
  const [busy,setBusy]=useState(false),[confirm,setConfirm]=useState(false),[checks,setChecks]=useState([false,false,false]);
  const [hintMode,setHintMode]=useState(false),[seed,setSeed]=useState(['0','0','0']);
  const ws=useRef<WebSocket|null>(null);
  useEffect(()=>{api('state').then(s=>{setState(s);setPaired(true)}).catch(()=>{});},[]);
  useEffect(()=>{if(!paired)return;
    api('map').then(setMap).catch(e=>setError(e.message));
    return startStateStream<State>({
      url:`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/events`,
      readState:()=>api('state'),
      onState:s=>setState(previous=>({...s,cloud:s.cloud??previous?.cloud??[]})),
      onConnected:setConnected,onSocket:socket=>{ws.current=socket},
      onOwnership:(value,controller)=>{setOwner(value);setController(controller)},
      onAuthRequired:()=>{setPaired(false);setState(null)},onError:setError,
    });
  },[paired]);
  useEffect(()=>{setChecks([false,false,false]);setConfirm(false);},[state?.plan?.id]);
  const operation=async(action:()=>Promise<unknown>)=>{setBusy(true);setError('');try{await action();}catch(e){setError((e as Error).message)}finally{setBusy(false)}};
  const active=state?.task&&['RUNNING','STOPPING'].includes(state.task.state);
  const select=(xy:number[])=>{if(active)return;if(hintMode){setSeed([xy[0].toFixed(2),xy[1].toFixed(2),seed[2]]);setHintMode(false);}else setGoal(xy);};
  return <div className="app"><header><div className="brand"><span className="brand-icon">M</span><div>MobileEss<small>自主导航控制台</small></div></div><nav><span className="nav-current">导航</span><span className="site-name">停车场 / 固定地图</span></nav><div className="connection"><i className={connected?'on':''}/>{connected?(state?.demo?'演示已连接':'板端已连接'):'未连接'}<span>{state?.demo?'离线演示':owner?'当前控制席':'观察席'}</span></div></header>
    <main><section className="workspace"><div className="map-toolbar"><div><span className="eyebrow">现场地图</span><h1>停车场目标导航</h1></div><div className="view-tabs"><button className={view==='2d'?'selected':''} onClick={()=>setView('2d')}>二维导航</button><button className={view==='3d'?'selected':''} onClick={()=>setView('3d')}>三维观察</button></div></div>
      <div className="map-wrap">{map?(view==='2d'?<MapCanvas map={map} state={state} goal={goal} onSelect={select}/>:<React.Suspense fallback={<div className="empty-map">正在加载三维视图…</div>}><ThreeView map={map} state={state}/></React.Suspense>):<div className="empty-map"><span>⌖</span><p>等待地图连接</p><small>配对后显示停车场地图与车辆状态</small></div>}
        {state?.demo&&<div className="demo-badge">离线演示 · 位姿为示例 · 无车辆控制</div>}
        {hintMode&&<div className="map-prompt">请在二维地图点击大致车位，然后填写车头朝向</div>}
        <div className="legend"><span><i className="forward"/>前进</span><span><i className="reverse"/>倒车</span><span><i className="rotate"/>原地转向</span><span><i className="obstacle"/>障碍</span></div></div>
      <footer className="telemetry-strip"><div><small>定位</small><strong>{state?.localized?'已匹配':'等待匹配'}</strong></div><div><small>车位 / m</small><strong>{state?.pose?`${state.pose[0].toFixed(2)}, ${state.pose[1].toFixed(2)}`:'—'}</strong></div><div><small>车头朝向</small><strong>{state?.pose?`${(state.pose[2]*180/Math.PI).toFixed(1)}°`:'—'}</strong></div><div><small>实际车速 · 0x50A</small><strong>{state?.feedback_fresh&&!state.demo?`${state.actual_v.toFixed(2)} m/s`:'—'}</strong></div><div><small>API 状态</small><strong>{state?.api_mode??'—'}</strong></div></footer>
    </section><aside><div className="section-heading"><span className="eyebrow">本次任务</span><span className="task-counter">SINGLE GOAL</span></div><div className="status-card"><span className={'status-dot '+(state?.localized?'ready':'')}/><div><h2>{state?phases[state.phase]||state.phase:'等待连接'}</h2><p>{state?.reason||state?.fault||(state?.localized?'在地图点选一个目标位置':'等待地图定位与新鲜传感器数据')}</p></div></div>
      {!owner&&paired&&<button className="secondary wide" disabled={!connected} onClick={()=>ws.current?.send(JSON.stringify({type:'claim'}))}>取得控制权</button>}
      <div className="step-title"><b>01</b><h3>选择目标</h3></div><p className="muted">点击二维地图设置位置，到达朝向由系统规划。</p><div className="coordinates"><div><small>X / m</small><strong>{goal?.[0].toFixed(2)??'—'}</strong></div><div><small>Y / m</small><strong>{goal?.[1].toFixed(2)??'—'}</strong></div></div>
      <button className="secondary wide" disabled={!owner||!goal||!!active||busy||!state?.localized} onClick={()=>operation(()=>api('plans',{goal}))}>{busy?'正在检查…':'生成路线预览'}</button>
      <div className="step-title"><b>02</b><h3>检查路线</h3></div>{state?.plan?<div className="plan-summary"><div><span>总路线</span><strong>{state.plan.length_m.toFixed(2)} <small>m</small></strong></div><div><span>运动分段</span><strong>{state.plan.segments.length} <small>段</small></strong></div><ol>{state.plan.segments.map((s,i)=><li key={i} className={state.task?.segment===i&&active?'current':''}><i className={s.kind}/>{kinds[s.kind]}<span>{s.kind==='rotate'?`${(s.turn*180/Math.PI).toFixed(0)}°`:`${s.length_m.toFixed(2)} m`}</span></li>)}</ol></div>:<div className="preview-empty">路线生成后会显示前进、倒车与转向段</div>}
      <div className="limits"><span>前进 ≤ 0.3 m/s</span><span>倒车 ≤ 0.2 m/s</span><span>连续倒车 ≤ 1 m</span></div>
      <div className="action-area">{!state?.motion_enabled&&<p className="notice">观测模式 · 运动入口待实车验收启用</p>}<button className="primary wide" disabled={!owner||!state?.plan||!!active||busy||!state.motion_enabled||!!(goal&&state.plan.goal.some((v,i)=>v!==goal[i]))} onClick={()=>setConfirm(true)}>确认路线并开始 <span>→</span></button><button className="stop wide" disabled={!active||busy} onClick={()=>operation(()=>api(`tasks/${state!.task!.id}/stop`,{}))}>■ 停止本次任务</button></div>
      <details className="diagnostics"><summary>定位与车辆详情</summary><p>{state?.localization||'尚无定位诊断'}</p><p>实际四轮 mm/s：{state?.wheel_mm_s.join(' / ')||'—'}</p><p>0x507 命令回显：{state?.command_echo?.join(' / ')??'—'}</p><button className="secondary wide" disabled={!owner||!!active||busy||!state?.stationary} onClick={()=>operation(()=>api('localization/relocalize',{}))}>重新全图定位 · 最多120秒</button><h4>人工粗定位辅助</h4><button className="text-button" disabled={!!active} onClick={()=>{setView('2d');setHintMode(true)}}>在地图标出大致车位</button><div className="seed-inputs">{['X / m','Y / m','朝向 / °'].map((label,i)=><label key={label}>{label}<input type="number" step="0.1" value={seed[i]} onChange={e=>setSeed(seed.map((v,j)=>j===i?e.target.value:v))}/></label>)}</div><button className="secondary wide" disabled={!owner||!!active||busy||!state?.stationary} onClick={()=>operation(()=>api('localization/relocalize',{seed:[Number(seed[0]),Number(seed[1]),Number(seed[2])*Math.PI/180]}))}>提交提示并重新匹配</button></details>
    </aside></main>{error&&<div className="error-toast" role="alert"><span>{error}</span><button onClick={()=>setError('')}>×</button></div>}
    {!paired&&<div className="modal-shade"><form className="modal" onSubmit={e=>{e.preventDefault();operation(async()=>{await api('pair',{code});setPaired(true)})}}><span className="eyebrow">连接车辆</span><h2>配对导航控制台</h2><p>请输入板端生成的配对码。配对后可查看地图并取得控制权。</p><label>配对码<input autoFocus autoComplete="off" value={code} onChange={e=>setCode(e.target.value)}/></label><button className="primary wide" disabled={busy}>连接控制台 →</button></form></div>}
    {confirm&&<div className="modal-shade"><div className="modal"><span className="eyebrow">本轮现场确认</span><h2>准备执行这条路线</h2><p>本次确认只用于当前预览。请核对地图定位与整段通行范围。</p>{['整条路线及停车余量净空，定位与现场贴合','CH5与独立停止已检查，现场监护就绪','刀盘及其他作业机构已隔离'].map((label,i)=><label className="check" key={label}><input type="checkbox" checked={checks[i]} onChange={e=>setChecks(checks.map((v,j)=>i===j?e.target.checked:v))}/>{label}</label>)}<button className="primary wide" disabled={!checks.every(Boolean)||busy} onClick={()=>operation(async()=>{await api('tasks',{plan_id:state?.plan?.id,confirmations:{route_clear:checks[0],stop_ready:checks[1],actuators_isolated:checks[2]}});setConfirm(false);})}>开始本次任务 →</button><button className="text-button" onClick={()=>setConfirm(false)}>返回检查路线</button></div></div>}
  </div>;
}
createRoot(document.getElementById('root')!).render(<React.StrictMode><App/></React.StrictMode>);
