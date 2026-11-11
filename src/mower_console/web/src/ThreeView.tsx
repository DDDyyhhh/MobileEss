import {useEffect,useMemo,useRef,useState} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {Goal,MapData,Segment,State} from './types';
import {mapCenter} from './MapCanvas';

let staticMap:{revision:string;points:Promise<ArrayBuffer>}|null=null;
function mapPoints(revision:string) {
  if(!staticMap||staticMap.revision!==revision){
    const points=fetch('/api/map/points').then(r=>{if(!r.ok)throw Error('PCD不可用');return r.arrayBuffer()});
    staticMap={revision,points};
    points.catch(()=>{if(staticMap?.points===points)staticMap=null;});
  }
  return staticMap.points;
}

/** Overwrite a dynamic geometry in place; grow its buffer only when it is too small. */
export function writePositions(geometry:THREE.BufferGeometry,xyz:ArrayLike<number>) {
  let attr=geometry.getAttribute('position') as THREE.BufferAttribute|undefined;
  if(!attr||attr.array.length<xyz.length){
    let capacity=Math.max(192,attr?.array.length??0);while(capacity<xyz.length)capacity*=2;
    attr=new THREE.BufferAttribute(new Float32Array(capacity),3);attr.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('position',attr);
  }
  (attr.array as Float32Array).set(xyz);attr.needsUpdate=true;
  geometry.setDrawRange(0,Math.floor(xyz.length/3));
}
const flat=(points:number[][],z:number)=>points.flatMap(p=>[p[0],p[1],z]);
const colors={forward:0x68d6be,reverse:0xf3b565,rotate:0xbf9dff};

type Overlay={vehicle:THREE.Mesh;heading:THREE.ArrowHelper;trajectory:THREE.Line;obstacles:THREE.Points;
  cloud:THREE.Points;route:THREE.Group;goal:THREE.Group;goalArrow:THREE.ArrowHelper};
type Props={map:MapData;state:State|null;goal:Goal|null;obstacles:Float32Array;cloud:Float32Array;onSelect:(xy:number[])=>void};

export function ThreeView({map,state,goal,obstacles,cloud,onSelect}:Props) {
  const host=useRef<HTMLDivElement>(null),overlay=useRef<Overlay|null>(null);
  const select=useRef(onSelect);select.current=onSelect;
  // Bumped when a scene is (re)built so every overlay effect below repaints it.
  const [sceneVersion,setSceneVersion]=useState(0);
  useEffect(()=>{
    const el=host.current!,scene=new THREE.Scene();scene.background=new THREE.Color('#101923');
    const [cx,cy]=mapCenter(map);const camera=new THREE.PerspectiveCamera(50,1,.05,300);camera.up.set(0,0,1);camera.position.set(cx+11,cy-15,18);
    let renderer:THREE.WebGLRenderer;
    try{renderer=new THREE.WebGLRenderer({antialias:true});}catch{el.dataset.error='当前浏览器无法开启三维渲染，请返回二维导航';return;}
    renderer.setPixelRatio(Math.min(devicePixelRatio,1.25));el.appendChild(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(cx,cy,0);controls.enableDamping=true;
    const grid=new THREE.GridHelper(40,40,0x3c535e,0x243440);grid.rotation.x=Math.PI/2;grid.position.set(cx,cy,-.1);scene.add(grid);
    const dynamic=<T extends THREE.Object3D>(o:T)=>{o.frustumCulled=false;scene.add(o);return o;};
    const vehicle=dynamic(new THREE.Mesh(new THREE.BoxGeometry(1.086,.768,.36),new THREE.MeshBasicMaterial({color:0x75d6c1})));
    const heading=dynamic(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),1,0xe4ffed,.2,.14));
    const goalGroup=dynamic(new THREE.Group());
    const ring=new THREE.Mesh(new THREE.RingGeometry(.16,.24,40),new THREE.MeshBasicMaterial({color:0xe9d9b9,side:THREE.DoubleSide}));ring.position.z=.12;goalGroup.add(ring);
    const goalArrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,.14),.8,0xf1d48c,.25,.16);goalGroup.add(goalArrow);
    overlay.current={vehicle,heading,goalArrow,goal:goalGroup,route:dynamic(new THREE.Group()),
      trajectory:dynamic(new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x70b9b7}))),
      obstacles:dynamic(new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0xef977b,size:.12}))),
      cloud:dynamic(new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0xe9b97c,size:.08})))};
    vehicle.visible=heading.visible=goalGroup.visible=false;
    let dead=false,staticCloud:THREE.Points|null=null;
    mapPoints(map.revision).then(buffer=>{if(dead)return;const xyz=new Float32Array(buffer),geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));const colors=new Float32Array(xyz.length);for(let i=0;i<xyz.length;i+=3){const h=Math.max(0,Math.min(1,(xyz[i+2]+.3)/3));const c=new THREE.Color().setHSL(.47-h*.10,.28,.35+h*.18);colors.set([c.r,c.g,c.b],i);}geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));staticCloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:.055,vertexColors:true}));scene.add(staticCloud);}).catch(e=>{if(!dead)el.dataset.error=e.message;});
    // A click (not an orbit drag) picks the nearest static map point under the
    // cursor, falling back to the z=0 plane, and selects its map x/y as the goal.
    const raycaster=new THREE.Raycaster();raycaster.params.Points={threshold:.15};
    let down:{x:number;y:number}|null=null;
    const pointerDown=(e:PointerEvent)=>{down=e.button===0?{x:e.clientX,y:e.clientY}:null;};
    const pointerUp=(e:PointerEvent)=>{
      if(!down||Math.hypot(e.clientX-down.x,e.clientY-down.y)>4){down=null;return;}down=null;
      const r=renderer.domElement.getBoundingClientRect();
      raycaster.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);
      const hit=staticCloud?raycaster.intersectObject(staticCloud)[0]?.point:undefined;
      const point=hit??raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,0,1),0),new THREE.Vector3());
      if(point)select.current([point.x,point.y]);
    };
    renderer.domElement.addEventListener('pointerdown',pointerDown);renderer.domElement.addEventListener('pointerup',pointerUp);
    const ro=new ResizeObserver(()=>{camera.aspect=el.clientWidth/el.clientHeight;camera.updateProjectionMatrix();renderer.setSize(el.clientWidth,el.clientHeight);});ro.observe(el);
    setSceneVersion(v=>v+1);
    let frame=0,lastFrame=0;const animate=(now:number)=>{if(now-lastFrame>=50&&document.visibilityState==='visible'){controls.update();renderer.render(scene,camera);lastFrame=now;}frame=requestAnimationFrame(animate);};frame=requestAnimationFrame(animate);
    return()=>{dead=true;cancelAnimationFrame(frame);ro.disconnect();controls.dispose();
      renderer.domElement.removeEventListener('pointerdown',pointerDown);renderer.domElement.removeEventListener('pointerup',pointerUp);
      scene.traverse(o=>{const m=o as THREE.Mesh;m.geometry?.dispose();const a=m.material;if(Array.isArray(a))a.forEach(v=>v.dispose());else a?.dispose();});renderer.dispose();el.replaceChildren();overlay.current=null;};
  },[map]);

  const pose=state?.pose;
  useEffect(()=>{const o=overlay.current;if(!o)return;
    o.vehicle.visible=o.heading.visible=!!pose;if(!pose)return;
    o.vehicle.position.set(pose[0],pose[1],.18);o.vehicle.rotation.z=pose[2];
    o.heading.position.set(pose[0],pose[1],.45);o.heading.setDirection(new THREE.Vector3(Math.cos(pose[2]),Math.sin(pose[2]),0));
  },[sceneVersion,pose?.[0],pose?.[1],pose?.[2]]);
  useEffect(()=>{const o=overlay.current;if(o)writePositions(o.trajectory.geometry,flat(state?.trajectory??[],.06));},[sceneVersion,state?.trajectory]);
  useEffect(()=>{const o=overlay.current;if(o)writePositions(o.obstacles.geometry,obstacles);},[sceneVersion,obstacles]);
  useEffect(()=>{const o=overlay.current;if(o)writePositions(o.cloud.geometry,cloud);},[sceneVersion,cloud]);
  useEffect(()=>{const o=overlay.current;if(!o)return;
    o.goal.visible=!!goal;if(!goal)return;
    o.goal.position.set(goal.x,goal.y,0);o.goalArrow.visible=goal.yaw!==null;
    if(goal.yaw!==null)o.goalArrow.setDirection(new THREE.Vector3(Math.cos(goal.yaw),Math.sin(goal.yaw),0));
  },[sceneVersion,goal?.x,goal?.y,goal?.yaw]);
  // Rebuild route lines only when the route itself changes (preview, resume or detour).
  const segments:Segment[]=state?.task?.route??state?.plan?.segments??[];
  const routeKey=useMemo(()=>segments.map(s=>`${s.kind}:${s.poses.length}:${s.poses[0]}:${s.poses.at(-1)}`).join('|'),[segments]);
  useEffect(()=>{const o=overlay.current;if(!o)return;
    o.route.children.slice().forEach(c=>{o.route.remove(c);const m=c as THREE.Mesh;m.geometry.dispose();(m.material as THREE.Material).dispose();});
    segments.forEach(s=>{const color=colors[s.kind];
      if(s.kind==='rotate'){const ring=new THREE.Mesh(new THREE.RingGeometry(.34,.39,40),new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide}));ring.position.set(s.poses[0][0],s.poses[0][1],.12);o.route.add(ring);}
      else{const geo=new THREE.BufferGeometry().setFromPoints(s.poses.map(p=>new THREE.Vector3(p[0],p[1],.10)));o.route.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color})));}
    });
  },[sceneVersion,routeKey]);
  return <div className="three-view" ref={host} aria-label="三维地图观察视图"><div className="three-note">三维观察 · 拖动旋转 / 滚轮缩放 · 单击地面设目标</div></div>;
}
