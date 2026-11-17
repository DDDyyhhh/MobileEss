import {useEffect,useRef,useState} from 'react';
import {Goal,MapData,Segment,State,headingFromDrag} from './types';
import {Layers,mapTheme as t,ViewCommand} from './mapTheme';

export function mapCenter(map:MapData):[number,number] {
  const [x,y,a]=map.origin,w=map.width*map.resolution/2,h=map.height*map.resolution/2;
  return [x+Math.cos(a)*w-Math.sin(a)*h,y+Math.sin(a)*w+Math.cos(a)*h];
}
export function screenToWorld(x:number,y:number,w:number,h:number,center:number[],scale:number):[number,number] {
  return [center[0]+(x-w/2)/scale,center[1]-(y-h/2)/scale];
}
export const segmentColor=(kind:Segment['kind'])=>kind==='reverse'?t.reverse:kind==='rotate'?t.rotate:t.blue;
type Props={map:MapData;state:State|null;goal:Goal|null;obstacles:Float32Array;layers:Layers;command:ViewCommand;onSelect:(xy:number[])=>void;onHeading:(yaw:number)=>void};
export function MapCanvas({map,state,goal,obstacles,layers,command,onSelect,onHeading}:Props) {
  const canvas=useRef<HTMLCanvasElement>(null),image=useRef<HTMLCanvasElement|null>(null);
  const [view,setView]=useState({center:mapCenter(map),scale:20}),[size,setSize]=useState([800,600]);
  const viewRef=useRef(view);viewRef.current=view;
  const pointers=useRef(new Map<number,{x:number;y:number}>());
  const drag=useRef<{x:number;y:number;cx:number;cy:number;moved:boolean;mode:'pan'|'heading'}|null>(null);
  const pinch=useRef<{distance:number;scale:number}|null>(null),touched=useRef(false);
  const fit=()=>({center:mapCenter(map),scale:Math.min(size[0]/(map.width*map.resolution),size[1]/(map.height*map.resolution))*.82});
  const actions=useRef({fit,pose:state?.pose});actions.current={fit,pose:state?.pose};
  useEffect(()=>{const ro=new ResizeObserver(entries=>{const r=entries[0].contentRect;setSize([r.width,r.height]);});ro.observe(canvas.current!);return()=>ro.disconnect();},[]);
  useEffect(()=>{
    const img=document.createElement('canvas');img.width=map.width;img.height=map.height;
    const ctx=img.getContext('2d')!,pixels=ctx.createImageData(map.width,map.height);
    const rgb=(hex:string)=>[1,3,5].map(i=>parseInt(hex.slice(i,i+2),16));
    const free=rgb(t.free),unknown=rgb(t.unknown),occupied=rgb(t.occupied);
    map.data.forEach((v,i)=>{const p=((map.height-1-Math.floor(i/map.width))*map.width+i%map.width)*4;pixels.data.set([...(v<0?unknown:v>=65?occupied:free),255],p);});
    ctx.putImageData(pixels,0,0);image.current=img;touched.current=false;
  },[map]);
  useEffect(()=>{if(!touched.current)setView(actions.current.fit());},[map,size]);
  useEffect(()=>{
    touched.current=command.kind!=='fit';
    if(command.kind==='fit')setView(actions.current.fit());
    else if(command.kind==='vehicle'&&actions.current.pose)setView(v=>({...v,center:actions.current.pose!.slice(0,2) as [number,number]}));
    else if(command.kind==='in'||command.kind==='out')setView(v=>({...v,scale:Math.max(.5,Math.min(250,v.scale*(command.kind==='in'?1.3:1/1.3)))}));
  },[command.serial]);
  useEffect(()=>{
    const el=canvas.current!;
    const wheel=(e:WheelEvent)=>{
      e.preventDefault();touched.current=true;
      const r=el.getBoundingClientRect(),v=viewRef.current,x=e.clientX-r.left,y=e.clientY-r.top;
      const world=screenToWorld(x,y,r.width,r.height,v.center,v.scale),scale=Math.max(.5,Math.min(250,v.scale*Math.exp(-e.deltaY*.001)));
      setView({scale,center:[world[0]-(x-r.width/2)/scale,world[1]+(y-r.height/2)/scale]});
    };
    el.addEventListener('wheel',wheel,{passive:false});return()=>el.removeEventListener('wheel',wheel);
  },[]);
  useEffect(()=>{
    const el=canvas.current!,ctx=el.getContext('2d')!,dpr=Math.min(window.devicePixelRatio||1,2),[w,h]=size;
    el.width=w*dpr;el.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle=t.canvas;ctx.fillRect(0,0,w,h);
    ctx.setTransform(dpr*view.scale,0,0,-dpr*view.scale,dpr*(w/2-view.center[0]*view.scale),dpr*(h/2+view.center[1]*view.scale));
    ctx.lineWidth=1/view.scale;ctx.strokeStyle=t.grid;
    const step=view.scale<6?5:1;
    for(let x=Math.floor((view.center[0]-w/view.scale)/step)*step;x<view.center[0]+w/view.scale;x+=step){ctx.beginPath();ctx.moveTo(x,view.center[1]-h/view.scale);ctx.lineTo(x,view.center[1]+h/view.scale);ctx.stroke();}
    for(let y=Math.floor((view.center[1]-h/view.scale)/step)*step;y<view.center[1]+h/view.scale;y+=step){ctx.beginPath();ctx.moveTo(view.center[0]-w/view.scale,y);ctx.lineTo(view.center[0]+w/view.scale,y);ctx.stroke();}
    ctx.save();ctx.translate(map.origin[0],map.origin[1]);ctx.rotate(map.origin[2]);ctx.scale(map.resolution,-map.resolution);ctx.imageSmoothingEnabled=false;if(image.current)ctx.drawImage(image.current,0,-map.height);ctx.restore();
    const path=(points:number[][],color:string,width:number,dash:number[]=[])=>{if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineCap='round';ctx.lineJoin='round';ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);};
    if(layers.trajectory&&state?.trajectory)path(state.trajectory,t.history,.05,[.12,.1]);
    if(layers.route){
      const route=state?.task?.route;
      if(route&&state?.task?.replans)state.plan?.segments.forEach(s=>path(s.poses,t.muted,.06,[.25,.2]));
      (route??state?.plan?.segments??[]).forEach(s=>{
        if(!s.poses.length)return;
        path(s.poses,t.clearance,1.96);path(s.poses,segmentColor(s.kind),Math.max(.065,2/view.scale),s.kind==='reverse'?[.18,.14]:[]);
        if(s.kind==='rotate'){ctx.beginPath();ctx.arc(s.poses[0][0],s.poses[0][1],.38,0,Math.PI*1.75);ctx.strokeStyle=t.rotate;ctx.lineWidth=.05;ctx.stroke();}
      });
    }
    if(layers.obstacles){ctx.fillStyle=t.obstacle;for(let i=0;i+2<obstacles.length;i+=3)ctx.fillRect(obstacles[i]-.055,obstacles[i+1]-.055,.11,.11);}
    if(goal){
      const {x,y,yaw}=goal,r=Math.max(.20,6/view.scale);
      ctx.beginPath();ctx.arc(x,y,r*1.9,0,Math.PI*2);ctx.fillStyle=t.blue+'14';ctx.fill();
      ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=t.free;ctx.fill();ctx.strokeStyle=t.blue;ctx.lineWidth=Math.max(.06,2/view.scale);ctx.stroke();
      ctx.beginPath();ctx.arc(x,y,r*.35,0,Math.PI*2);ctx.fillStyle=t.blue;ctx.fill();
      if(yaw!==null){const len=Math.max(.8,26/view.scale),tx=x+Math.cos(yaw)*len,ty=y+Math.sin(yaw)*len;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.strokeStyle=t.blue;ctx.stroke();ctx.beginPath();ctx.moveTo(tx,ty);ctx.lineTo(tx-Math.cos(yaw-.45)*len*.3,ty-Math.sin(yaw-.45)*len*.3);ctx.lineTo(tx-Math.cos(yaw+.45)*len*.3,ty-Math.sin(yaw+.45)*len*.3);ctx.closePath();ctx.fill();}
    }
    if(state?.pose&&state.localized){const [x,y,yaw]=state.pose;ctx.save();ctx.translate(x,y);ctx.rotate(yaw);ctx.fillStyle=t.blue;ctx.fillRect(-.543,-.384,1.086,.768);ctx.strokeStyle=t.free;ctx.lineWidth=.055;ctx.strokeRect(-.543,-.384,1.086,.768);ctx.beginPath();ctx.moveTo(.36,0);ctx.lineTo(-.16,.21);ctx.lineTo(-.16,-.21);ctx.closePath();ctx.fillStyle=t.free;ctx.fill();ctx.restore();}
  },[map,state,goal,obstacles,layers,view,size]);
  const world=(e:React.PointerEvent)=>{const r=canvas.current!.getBoundingClientRect();return screenToWorld(e.clientX-r.left,e.clientY-r.top,r.width,r.height,view.center,view.scale);};
  const scaleMetres=[.1,.2,.5,1,2,5,10,20,50,100].find(v=>v*view.scale>=40)??100;
  return <><canvas ref={canvas} className="map-canvas" aria-label="二维导航地图：点击选目标，从目标拖出设定朝向，拖动平移，双指或滚轮缩放"
    onPointerDown={e=>{if(e.button!==0)return;canvas.current!.setPointerCapture(e.pointerId);pointers.current.set(e.pointerId,{x:e.clientX,y:e.clientY});
      if(pointers.current.size===2){const [a,b]=[...pointers.current.values()];pinch.current={distance:Math.hypot(a.x-b.x,a.y-b.y),scale:view.scale};drag.current=null;return;}
      const p=world(e),onGoal=!!goal&&Math.hypot(p[0]-goal.x,p[1]-goal.y)<=Math.max(.5,14/view.scale);
      drag.current={x:e.clientX,y:e.clientY,cx:view.center[0],cy:view.center[1],moved:false,mode:onGoal?'heading':'pan'};}}
    onPointerMove={e=>{if(!pointers.current.has(e.pointerId))return;pointers.current.set(e.pointerId,{x:e.clientX,y:e.clientY});
      if(pinch.current&&pointers.current.size>=2){const [a,b]=[...pointers.current.values()];touched.current=true;const scale=pinch.current.scale*Math.hypot(a.x-b.x,a.y-b.y)/Math.max(1,pinch.current.distance);setView(v=>({...v,scale:Math.max(.5,Math.min(250,scale))}));return;}
      const d=drag.current;if(!d)return;if(Math.hypot(e.clientX-d.x,e.clientY-d.y)>4)d.moved=true;if(!d.moved)return;touched.current=true;
      if(d.mode==='heading'){const yaw=goal&&headingFromDrag(goal,world(e));if(yaw!==null&&yaw!==undefined)onHeading(yaw);}
      else setView(v=>({...v,center:[d.cx-(e.clientX-d.x)/v.scale,d.cy+(e.clientY-d.y)/v.scale]}));}}
    onPointerUp={e=>{pointers.current.delete(e.pointerId);const d=drag.current;drag.current=null;if(!pinch.current&&d&&!d.moved)onSelect(world(e));if(!pointers.current.size)pinch.current=null;}}
    onPointerCancel={()=>{pointers.current.clear();pinch.current=null;drag.current=null;}}/>
    <div className="map-coordinate"><i className="scale-line" style={{width:scaleMetres*view.scale}}/>{scaleMetres} m · +Y ↑</div></>;
}
