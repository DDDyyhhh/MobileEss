import {useState} from 'react';
import {Goal,State,kinds} from './types';
import {actionAvailability,motionKinds,readableReason,statusCopy} from './consoleModel';
import {Button,Icon} from './ui';

type Props={state:State|null;connected:boolean;owner:boolean;goal:Goal|null;pending:string;stopping:boolean;
  availability:ReturnType<typeof actionAvailability>;onHeading:(yaw:number|null)=>void;onClear:()=>void;
  onPreview:()=>void;onStart:()=>void;onStop:()=>void;onDetails:()=>void};
export function TaskPanel(p:Props) {
  const [expanded,setExpanded]=useState(false);
  const {state,goal,availability:a}=p,copy=statusCopy(state,p.connected);
  const task=state?.task,route=a.active&&task?.route?task.route:state?.plan?.segments;
  const length=route?.reduce((sum,s)=>sum+s.length_m,0);
  return <aside className="task-panel" data-expanded={expanded} aria-label="导航任务">
    <button className="mobile-task-toggle" onClick={()=>setExpanded(v=>!v)} aria-expanded={expanded} aria-controls="task-content"><span><i className={`status-dot ${copy.tone}`}/>{copy.title}</span><span>{expanded?'收起任务':'展开任务'}<Icon name="chevron" size={16}/></span></button>
    <div className="task-scroll" id="task-content">
      <section className={`task-status tone-${copy.tone}`} aria-live="polite">
        <div className="eyebrow"><span className="status-dot"/>导航任务</div>
        <h2>{copy.title}</h2><p>{copy.detail}</p>
        {a.active&&task?.substate!=='DRIVING'&&<div className="wait-metrics"><span>已等待 <strong>{task?.waiting_s??0}</strong> 秒</span><span>已绕行 <strong>{task?.replans??0}</strong> 次</span></div>}
        {task?.replan_error&&<p className="inline-warning">上次绕行未采用：{readableReason(task.replan_error)}</p>}
        {!state?.localized&&<button className="text-button" onClick={p.onDetails}>查看定位详情<Icon name="chevron" size={16}/></button>}
      </section>
      <section className="task-section">
        <div className="section-label"><span className="step-index">01</span><h3>目的地</h3>{goal&&!a.active&&<button className="text-button" onClick={p.onClear}>清除</button>}</div>
        {goal?<><div className="coordinate-pair"><div><span>X 坐标</span><strong>{goal.x.toFixed(2)}<small>m</small></strong></div><div><span>Y 坐标</span><strong>{goal.y.toFixed(2)}<small>m</small></strong></div></div>
          <div className="heading-control"><label className="check-row"><input type="checkbox" checked={goal.yaw===null} disabled={a.active} onChange={e=>p.onHeading(e.target.checked?null:state?.pose?Math.atan2(goal.y-state.pose[1],goal.x-state.pose[0]):0)}/>自动选择到达朝向</label>
          {goal.yaw!==null&&<label className="input-label heading-input">到达朝向（°）<input type="number" step="5" value={Math.round(goal.yaw*180/Math.PI)} disabled={a.active} onChange={e=>{if(e.target.value!==''&&Number.isFinite(Number(e.target.value)))p.onHeading(Number(e.target.value)*Math.PI/180);}}/></label>}</div>
        </>:<div className="destination-empty"><span className="empty-icon"><Icon name="pin" size={22}/></span><div><strong>想去哪里？</strong><p>在地图上轻点选择目的地。</p></div></div>}
        <Button variant="secondary" className="full-width" disabled={!a.canPreview} aria-describedby="preview-reason" onClick={p.onPreview}><Icon name="route" size={18}/>{p.pending==='preview'||state?.phase==='PLANNING'?'正在生成路线…':'预览路线'}</Button>
        <p id="preview-reason" className="helper">{a.previewReason||(goal?.yaw===null?'可从二维目标标记拖出箭头，设置到达朝向。':'预览将包含指定的到达朝向。')}</p>
      </section>
      <section className="task-section route-section">
        <div className="section-label"><span className="step-index">02</span><h3>{a.active&&task?.replans?'当前绕行路线':'路线预览'}</h3>{route&&<span className="mini-label">{route.length} 段</span>}</div>
        {route&&state?.plan?<><div className="route-distance"><strong>{length?.toFixed(1)}</strong><span>米</span></div>
          <ol className="route-segments">{route.map((s,i)=><li key={i} className={a.active&&task?.segment===i?'current':''}><span className={`route-marker ${s.kind}`}><Icon name={s.kind==='rotate'?'rotate':'arrow'} size={15}/></span><span>{kinds[s.kind]}</span><strong>{s.kind==='rotate'?`${Math.round(s.turn*180/Math.PI)}°`:`${s.length_m.toFixed(1)} m`}</strong></li>)}</ol>
          {a.stale&&<p className="inline-warning">目标或朝向已改变，请重新生成预览。</p>}
          {!a.active&&<p className="helper">预览生成后 120 秒有效；条件变化时需重新预览。</p>}
        </>:<p className="route-empty">选好目的地后，先看看如何到达。</p>}
      </section>
      <section className="capability-summary"><div className="section-label"><h3>运动能力</h3><button className="text-button" onClick={p.onDetails}>详情<Icon name="chevron" size={14}/></button></div>
        <div className="capability-list">{motionKinds.map(kind=><span key={kind} className={state?.capabilities?.[kind]?'accepted':''}><Icon name={state?.capabilities?.[kind]?'check':'lock'} size={13}/>{kinds[kind]}<em>{state?.capabilities?.[kind]?'已验收':'未验收'}</em></span>)}</div>
      </section>
    </div>
    <div className="task-actions">
      <p className="action-explanation" id="start-reason"><Icon name={state?.motion_enabled?'info':'lock'} size={15}/><span>{a.active?'任务执行中，可随时请求停止。':a.startReason||'路线已就绪，开始前请完成现场确认。'}</span></p>
      <div className="action-buttons"><Button disabled={!a.canStart} aria-describedby="start-reason" onClick={p.onStart}>确认并开始<Icon name="arrow" size={18}/></Button><Button variant="stop" disabled={!a.canStop||p.stopping} onClick={p.onStop}><Icon name="stop" size={17}/>{p.stopping?'请求中':'停止'}</Button></div>
    </div>
  </aside>;
}
