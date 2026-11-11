import {useEffect, useRef, useState} from 'react';
import {Goal, MapData, Segment, State, headingFromDrag} from './types';

export function mapCenter(map: MapData): [number,number] {
  const [x,y,a]=map.origin, w=map.width*map.resolution/2, h=map.height*map.resolution/2;
  return [x+Math.cos(a)*w-Math.sin(a)*h,y+Math.sin(a)*w+Math.cos(a)*h];
}
export function screenToWorld(x:number,y:number,w:number,h:number,center:number[],scale:number):[number,number] {
  return [center[0]+(x-w/2)/scale,center[1]-(y-h/2)/scale];
}
export const segmentColor=(kind:Segment['kind'])=>kind==='reverse'?'#f3b565':kind==='rotate'?'#bf9dff':'#68d6be';

type Props={map:MapData;state:State|null;goal:Goal|null;obstacles:Float32Array;
  onSelect:(xy:number[])=>void;onHeading:(yaw:number)=>void};

export function MapCanvas({map,state,goal,obstacles,onSelect,onHeading}:Props) {
  const canvas=useRef<HTMLCanvasElement>(null);
  const [view,setView]=useState({center:mapCenter(map),scale:28});
  const [size,setSize]=useState([800,700]);
  // 'pan' drags the view; 'heading' starts on the goal marker and turns it.
  const drag=useRef<{x:number;y:number;cx:number;cy:number;moved:boolean;mode:'pan'|'heading'}|null>(null);
  const image=useRef<HTMLCanvasElement|null>(null);
  useEffect(()=>{
    const el=canvas.current!;
    const ro=new ResizeObserver(entries=>{const r=entries[0].contentRect;setSize([r.width,r.height]);});
    ro.observe(el);return()=>ro.disconnect();
  },[]);
  useEffect(()=>{
    const img=document.createElement('canvas');img.width=map.width;img.height=map.height;
    const ctx=img.getContext('2d')!, pixels=ctx.createImageData(map.width,map.height);
    map.data.forEach((v,i)=>{const p=((map.height-1-Math.floor(i/map.width))*map.width+i%map.width)*4;
      const c=v<0?[22,31,42]:v>=65?[110,128,136]:[44,62,67];
      pixels.data.set([...c,255],p);});ctx.putImageData(pixels,0,0);image.current=img;
    setView({center:mapCenter(map),scale:Math.min(size[0]/(map.width*map.resolution),size[1]/(map.height*map.resolution))*.85});
  },[map]);
  useEffect(()=>{
    const el=canvas.current!,ctx=el.getContext('2d')!,dpr=window.devicePixelRatio||1;
    const [w,h]=size;el.width=w*dpr;el.height=h*dpr;
    ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#101923';ctx.fillRect(0,0,w,h);
    ctx.setTransform(dpr*view.scale,0,0,-dpr*view.scale,dpr*(w/2-view.center[0]*view.scale),dpr*(h/2+view.center[1]*view.scale));
    ctx.lineWidth=1/view.scale;ctx.strokeStyle='#23313e';
    for(let x=Math.floor(view.center[0]-w/view.scale);x<view.center[0]+w/view.scale;x++){ctx.beginPath();ctx.moveTo(x,view.center[1]-h/view.scale);ctx.lineTo(x,view.center[1]+h/view.scale);ctx.stroke();}
    for(let y=Math.floor(view.center[1]-h/view.scale);y<view.center[1]+h/view.scale;y++){ctx.beginPath();ctx.moveTo(view.center[0]-w/view.scale,y);ctx.lineTo(view.center[0]+w/view.scale,y);ctx.stroke();}
    ctx.save();ctx.translate(map.origin[0],map.origin[1]);ctx.rotate(map.origin[2]);ctx.scale(map.resolution,-map.resolution);ctx.imageSmoothingEnabled=false;if(image.current)ctx.drawImage(image.current,0,-map.height);ctx.restore();
    const path=(points:number[][],color:string,width:number,dash:number[]=[])=>{if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineCap='round';ctx.lineJoin='round';ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);};
    const drawRoute=(segments:Segment[])=>segments.forEach(s=>{
      const color=segmentColor(s.kind);
      path(s.poses,color+'18',1.96);path(s.poses,color,.08);
      if(s.kind==='rotate'){ctx.beginPath();ctx.arc(s.poses[0][0],s.poses[0][1],.38,0,Math.PI*2);ctx.strokeStyle=color;ctx.lineWidth=.055;ctx.stroke();}
    });
    if(state?.trajectory)path(state.trajectory,'#70b9b780',.055);
    // After a detour the live route replaces the reviewed preview, which stays dashed.
    const route=state?.task?.route;
    if(route&&state?.task?.replans){state?.plan?.segments.forEach(s=>path(s.poses,'#9fb3b9',.05,[.25,.2]));}
    drawRoute(route??state?.plan?.segments??[]);
    ctx.fillStyle='#ef977b';for(let i=0;i+2<obstacles.length;i+=3)ctx.fillRect(obstacles[i]-.055,obstacles[i+1]-.055,.11,.11);
    if(goal){
      const {x,y,yaw}=goal;
      ctx.beginPath();ctx.arc(x,y,.20,0,Math.PI*2);ctx.strokeStyle='#e9d9b9';ctx.lineWidth=.055;ctx.stroke();
      ctx.beginPath();ctx.moveTo(x-.34,y);ctx.lineTo(x+.34,y);ctx.moveTo(x,y-.34);ctx.lineTo(x,y+.34);ctx.stroke();
      if(yaw!==null){
        const tx=x+Math.cos(yaw)*.8,ty=y+Math.sin(yaw)*.8;
        ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.strokeStyle='#f1d48c';ctx.lineWidth=.07;ctx.stroke();
        ctx.beginPath();ctx.moveTo(tx,ty);ctx.lineTo(tx-Math.cos(yaw-.45)*.25,ty-Math.sin(yaw-.45)*.25);ctx.lineTo(tx-Math.cos(yaw+.45)*.25,ty-Math.sin(yaw+.45)*.25);ctx.closePath();ctx.fillStyle='#f1d48c';ctx.fill();
      }
    }
    if(state?.pose){const [x,y,yaw]=state.pose;ctx.save();ctx.translate(x,y);ctx.rotate(yaw);ctx.shadowColor='#6de0c8';ctx.shadowBlur=12;ctx.fillStyle=state.localized?'#75d6c1':'#b6a47a';ctx.fillRect(-.543,-.384,1.086,.768);ctx.shadowBlur=0;ctx.fillStyle='#143b38';ctx.beginPath();ctx.moveTo(.42,0);ctx.lineTo(-.12,.23);ctx.lineTo(-.12,-.23);ctx.closePath();ctx.fill();ctx.restore();}
  },[map,state,goal,obstacles,view,size]);
  const world=(e:React.PointerEvent)=>{const r=canvas.current!.getBoundingClientRect();return screenToWorld(e.clientX-r.left,e.clientY-r.top,r.width,r.height,view.center,view.scale);};
  return <><canvas ref={canvas} className="map-canvas" aria-label="二维导航地图：点击选目标，从目标拖出设定到达朝向，拖动平移，滚轮缩放"
    onPointerDown={e=>{canvas.current!.setPointerCapture(e.pointerId);
      const p=world(e),onGoal=!!goal&&Math.hypot(p[0]-goal.x,p[1]-goal.y)<=Math.max(.6,14/view.scale);
      drag.current={x:e.clientX,y:e.clientY,cx:view.center[0],cy:view.center[1],moved:false,mode:onGoal?'heading':'pan'};}}
    onPointerMove={e=>{const d=drag.current;if(!d)return;if(Math.hypot(e.clientX-d.x,e.clientY-d.y)>4)d.moved=true;if(!d.moved)return;
      if(d.mode==='heading'){const yaw=goal&&headingFromDrag(goal,world(e));if(yaw!==null&&yaw!==undefined)onHeading(yaw);}
      else setView(v=>({...v,center:[d.cx-(e.clientX-d.x)/v.scale,d.cy+(e.clientY-d.y)/v.scale]}));}}
    onPointerUp={e=>{const d=drag.current;drag.current=null;if(d&&!d.moved)onSelect(world(e));}}
    onPointerCancel={()=>{drag.current=null;}}
    onWheel={e=>{e.preventDefault();setView(v=>({...v,scale:Math.max(8,Math.min(180,v.scale*Math.exp(-e.deltaY*.001)))}));}}/>
    <button className="fit-map" onClick={()=>setView({center:mapCenter(map),scale:Math.min(size[0]/(map.width*map.resolution),size[1]/(map.height*map.resolution))*.85})}>↗ 全图</button>
    <div className="map-coordinate">MAP · {map.resolution.toFixed(2)} m / 格 <span>↑ +Y　→ +X</span></div></>;
}
