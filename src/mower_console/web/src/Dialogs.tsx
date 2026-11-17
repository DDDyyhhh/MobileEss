import {useState} from 'react';
import {State} from './types';
import {readableReason} from './consoleModel';
import {Button,Icon,Modal} from './ui';

export function PairDialog({busy,error,onPair}:{busy:boolean;error:string;onPair:(code:string)=>void}) {
  const [code,setCode]=useState('');
  return <Modal title="连接你的 MobileEss"><div className="pair-symbol"><Icon name="wifi" size={36}/></div><p className="modal-intro">与车辆配对，查看现场地图与实时状态。</p>
    <form onSubmit={e=>{e.preventDefault();if(/^\d{6}$/.test(code)&&!busy)onPair(code);}}>
      <label className="input-label">6 位配对码<input data-autofocus className="pair-input" inputMode="numeric" autoComplete="off" maxLength={6} value={code} placeholder="输入配对码" onChange={e=>setCode(e.target.value.replace(/\D/g,''))}/></label>
      <p className="helper">使用板端本地生成的配对码。配对后先进入观察席。</p>
      {error&&<p className="form-error" role="alert">{error}</p>}
      <Button type="submit" className="full-width" disabled={code.length!==6||busy}>{busy?'正在连接…':'连接控制台'}<Icon name="arrow" size={18}/></Button>
    </form><div className="modal-footnote"><Icon name="lock" size={14}/>局域网连接 · 运动默认关闭</div></Modal>;
}
const checksText=['整条路线及停车余量净空，定位与现场贴合','CH5 与独立停止已检查，现场监护就绪','刀盘及其他作业机构已隔离'];
export function StartDialog({busy,canStart,onClose,onStart}:{busy:boolean;canStart:boolean;onClose:()=>void;onStart:(checks:boolean[])=>void}) {
  const [checks,setChecks]=useState([false,false,false]);
  return <Modal title="确认这一次出发" onClose={onClose}><p className="modal-intro">以下确认只适用于当前预览路线。</p>
    <div className="confirmation-list">{checksText.map((label,i)=><label className="check-row" key={label}><input type="checkbox" checked={checks[i]} onChange={e=>setChecks(v=>v.map((c,j)=>j===i?e.target.checked:c))}/><span>{label}</span></label>)}</div>
    <Button className="full-width" disabled={!canStart||!checks.every(Boolean)||busy} onClick={()=>onStart(checks)}>{busy?'正在提交…':'开始本次任务'}<Icon name="arrow" size={18}/></Button>
    <Button variant="quiet" onClick={onClose}>返回检查路线</Button></Modal>;
}
type VehicleProps={state:State|null;connected:boolean;owner:boolean;busy:boolean;canRelocalize:boolean;seed:string[];
  onSeed:(seed:string[])=>void;onHint:()=>void;onRelocalize:(seed?:number[])=>void;onClose:()=>void};
export function VehicleDrawer(p:VehicleProps) {
  const {state:s}=p,fresh=p.connected&&s?.feedback_fresh&&!s.demo;
  const validSeed=p.seed.every(v=>v.trim()!==''&&Number.isFinite(Number(v)));
  return <Modal title="定位与车辆" drawer onClose={p.onClose}>
    <section className="detail-section"><div className="detail-title"><Icon name="target"/><h3>定位状态</h3></div><p>{p.connected?readableReason(s?.localization)||'等待定位数据':'连接中断，数据暂停更新。'}</p>
      <dl className="detail-list"><div><dt>位置</dt><dd>{p.connected&&s?.pose?`${s.pose[0].toFixed(2)}, ${s.pose[1].toFixed(2)} m`:'未确认'}</dd></div><div><dt>车头朝向</dt><dd>{p.connected&&s?.pose?`${(s.pose[2]*180/Math.PI).toFixed(1)}°`:'—'}</dd></div><div><dt>停稳反馈</dt><dd>{p.connected&&s?.stationary?'已确认':'未确认'}</dd></div></dl>
      <Button variant="secondary" className="full-width" disabled={!p.canRelocalize} onClick={()=>p.onRelocalize()}>重新全图定位<Icon name="rotate" size={17}/></Button>
      <p className="helper">最长搜索 120 秒。需取得控制权，并保持 API 0 和车辆停稳。</p>
    </section>
    <section className="detail-section"><h3>提供大致车位</h3><p className="helper">在地图上标出车辆实际位置，再填写车头朝向。</p><Button variant="secondary" disabled={!p.canRelocalize} className="full-width" onClick={p.onHint}><Icon name="pin" size={17}/>在二维地图上标记</Button>
      <div className="seed-fields">{['X（m）','Y（m）','朝向（°）'].map((label,i)=><label className="input-label" key={label}>{label}<input type="number" step="0.1" value={p.seed[i]} onChange={e=>p.onSeed(p.seed.map((v,j)=>i===j?e.target.value:v))}/></label>)}</div>
      <Button className="full-width" disabled={!p.canRelocalize||!validSeed} onClick={()=>p.onRelocalize([Number(p.seed[0]),Number(p.seed[1]),Number(p.seed[2])*Math.PI/180])}>按提示重新匹配</Button>
    </section>
    <section className="detail-section"><div className="detail-title"><Icon name="info"/><h3>车辆反馈</h3></div>
      <dl className="detail-list"><div><dt>实际车速</dt><dd>{fresh?`${s!.actual_v.toFixed(2)} m/s`:'暂无有效反馈'}</dd></div><div><dt>四轮实际速度</dt><dd>{fresh?s!.wheel_mm_s.join(' / ')+' mm/s':'暂无有效反馈'}</dd></div><div><dt>API 模式</dt><dd>{p.connected?s?.api_mode??'未知':'未知'}</dd></div><div><dt>速度命令回显</dt><dd>{p.connected&&s?.command_echo?s.command_echo.join(' / '):'暂无反馈'}</dd></div></dl>
      <p className="helper">实际速度来自 0x50A；0x507 仅为命令回显。</p>
    </section>
    <details className="raw-details"><summary>技术诊断</summary><dl className="detail-list"><div><dt>定位原始状态</dt><dd>{s?.localization||'—'}</dd></div><div><dt>数据状态</dt><dd>{readableReason(s?.fault)||'无已报告异常'}</dd></div><div><dt>门控</dt><dd>{s?.gate_reason||'—'}</dd></div><div><dt>任务</dt><dd>{s?.task?.id||'无任务'}</dd></div></dl></details>
  </Modal>;
}
