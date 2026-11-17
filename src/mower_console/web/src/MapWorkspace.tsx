import {Component,lazy,ReactNode,Suspense,useState} from 'react';
import {MapCanvas} from './MapCanvas';
import {Clouds,Goal,MapData,State} from './types';
import {defaultLayers,Layers,ViewCommand} from './mapTheme';
import {Button,Icon,IconButton} from './ui';
const ThreeView=lazy(()=>import('./ThreeView').then(m=>({default:m.ThreeView})));
class ViewBoundary extends Component<{children:ReactNode;onReturn:()=>void},{failed:boolean}> {
  state={failed:false};
  static getDerivedStateFromError(){return {failed:true};}
  render(){return this.state.failed?<div className="map-empty"><Icon name="cube" size={32}/><h3>三维视图暂不可用</h3><Button variant="secondary" onClick={this.props.onReturn}>返回二维地图</Button></div>:this.props.children;}
}
const emptyPoints=new Float32Array(0);
type Props={map:MapData|null;state:State|null;connected:boolean;goal:Goal|null;clouds:Clouds;view:'2d'|'3d';hintMode:boolean;
  mapError:string;onRetry:()=>void;onView:(v:'2d'|'3d')=>void;onSelect:(xy:number[])=>void;onHeading:(yaw:number)=>void;onCancelHint:()=>void};
export function MapWorkspace(p:Props) {
  const [layers,setLayers]=useState<Layers>(defaultLayers),[showLayers,setShowLayers]=useState(false);
  const [command,setCommand]=useState<ViewCommand>({kind:'fit',serial:0});
  const run=(kind:ViewCommand['kind'])=>setCommand(c=>({kind,serial:c.serial+1}));
  const visibleState=p.connected?p.state:p.state?{...p.state,pose:null,localized:false}:null;
  const pose=p.connected&&p.state?.localized?p.state.pose:null;
  const velocity=p.connected&&p.state?.feedback_fresh&&!p.state.demo?p.state.actual_v:null;
  const layerNames:Record<keyof Layers,string>={route:'规划路线',trajectory:'已行驶轨迹',obstacles:'实时障碍',cloud:'实时点云',staticCloud:'静态点云'};
  return <section className="map-workspace" aria-label="地图工作区">
    <div className="map-heading"><div><p className="eyebrow">现场地图</p><h1>每一步，清晰可见。</h1></div><span className="map-edition"><Icon name="map" size={15}/>固定地图</span></div>
    <div className={`map-surface ${p.view==='3d'?'is-3d':''}`}>
      {p.map?(p.view==='2d'?<MapCanvas map={p.map} state={visibleState} goal={p.goal} obstacles={p.connected?p.clouds.obstacles:emptyPoints} layers={layers} command={command} onSelect={p.onSelect} onHeading={p.onHeading}/>
        :<ViewBoundary onReturn={()=>p.onView('2d')}><Suspense fallback={<div className="map-empty"><span className="loader"/><p>正在准备三维视图</p></div>}><ThreeView map={p.map} state={visibleState} goal={p.goal} obstacles={p.connected?p.clouds.obstacles:emptyPoints} cloud={p.connected?p.clouds.cloud:emptyPoints} layers={layers} command={command} onSelect={p.onSelect} onReturn={()=>p.onView('2d')}/></Suspense></ViewBoundary>)
        :<div className="map-empty"><div className="map-empty-symbol"><Icon name="map" size={38}/></div><h3>{p.mapError?'地图加载失败':'你的现场，即将呈现。'}</h3><p>{p.mapError||'连接车辆，查看地图与实时位置。'}</p>{p.mapError&&<Button variant="secondary" onClick={p.onRetry}>重试加载地图</Button>}</div>}
      <div className="map-top-controls"><div className="view-switch" aria-label="地图视图"><button aria-pressed={p.view==='2d'} onClick={()=>p.onView('2d')}><Icon name="map" size={17}/>二维</button><button aria-pressed={p.view==='3d'} onClick={()=>p.onView('3d')}><Icon name="cube" size={17}/>三维</button></div>
        <div className="layers-control"><IconButton icon="layers" label="地图图层" disabled={!p.map} aria-expanded={showLayers} onClick={()=>setShowLayers(v=>!v)}/>{showLayers&&<div className="layers-popover"><div className="popover-title">地图图层<IconButton icon="close" label="关闭图层" onClick={()=>setShowLayers(false)}/></div>{(Object.keys(layerNames) as (keyof Layers)[]).filter(k=>p.view==='3d'||!['cloud','staticCloud'].includes(k)).map(k=><label className="check-row" key={k}><input type="checkbox" checked={layers[k]} onChange={e=>setLayers(v=>({...v,[k]:e.target.checked}))}/>{layerNames[k]}</label>)}</div>}</div>
      </div>
      {(p.state?.demo||!p.connected&&!!p.state||p.hintMode)&&<div className="map-notice">{p.hintMode?<><Icon name="pin" size={17}/>标出车辆当前的大致位置<button onClick={p.onCancelHint}>取消</button></>:p.state?.demo?'离线演示 · 示例位姿，不控制车辆':'连接已中断 · 实时数据暂停'}</div>}
      <div className="map-navigation"><div className="map-zoom"><IconButton icon="plus" label="放大地图" disabled={!p.map} onClick={()=>run('in')}/><IconButton icon="minus" label="缩小地图" disabled={!p.map} onClick={()=>run('out')}/></div><IconButton icon="target" label="回到车辆" disabled={!pose} onClick={()=>run('vehicle')}/><IconButton icon="expand" label="显示全图" disabled={!p.map} onClick={()=>run('fit')}/></div>
      <div className="map-bottom"><span className="map-instruction">{p.view==='2d'?'点选目标 · 拖动平移':'点选地面 · 拖动旋转'}<span className="desktop-only"> · 滚轮缩放</span></span><div className="map-legend"><span><i className="legend-route"/>路线</span><span><i className="legend-obstacle"/>障碍</span><span><i className="legend-vehicle"/>车辆</span></div></div>
    </div>
    <dl className="telemetry-strip"><div><dt>地图定位</dt><dd>{!p.connected?'等待连接':p.state?.demo?'示例位姿':p.state?.localized?'已匹配':'等待匹配'}</dd></div><div><dt>实际车速</dt><dd>{velocity===null?'—':velocity.toFixed(2)}<small>m/s</small></dd></div><div><dt>车头朝向</dt><dd>{pose?`${(pose[2]*180/Math.PI).toFixed(1)}°`:'—'}</dd></div><div><dt>当前位置 · m</dt><dd className="position-value">{pose?`${pose[0].toFixed(2)}, ${pose[1].toFixed(2)}`:'—'}</dd></div></dl>
  </section>;
}
