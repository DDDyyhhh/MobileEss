import {useEffect,useMemo,useRef,useState} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {Goal,MapData,Segment,State} from './types';
import {mapCenter} from './MapCanvas';
import {Layers,mapTheme as t,ViewCommand} from './mapTheme';
import {Button} from './ui';

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
const colors={forward:t.live,reverse:'#92abc4',rotate:t.onDark};

type Overlay={vehicle:THREE.Mesh;heading:THREE.ArrowHelper;trajectory:THREE.Line;obstacles:THREE.Points;
  cloud:THREE.Points;route:THREE.Group;goal:THREE.Group;goalArrow:THREE.ArrowHelper};
type Props={map:MapData;state:State|null;goal:Goal|null;obstacles:Float32Array;cloud:Float32Array;layers:Layers;command:ViewCommand;onSelect:(xy:number[])=>void;onReturn:()=>void};

export function ThreeView({map,state,goal,obstacles,cloud,layers,command,onSelect,onReturn}:Props) {
  const host=useRef<HTMLDivElement>(null),overlay=useRef<Overlay|null>(null);
  const select=useRef(onSelect);select.current=onSelect;
  const [error,setError]=useState('');
  const latestLayers=useRef(layers);latestLayers.current=layers;
  const sceneHandle=useRef<{camera:THREE.PerspectiveCamera;controls:OrbitControls;staticCloud:THREE.Points|null}|null>(null);
  const poseRef=useRef(state?.pose);poseRef.current=state?.pose;
  // Bumped when a scene is (re)built so every overlay effect below repaints it.
  const [sceneVersion,setSceneVersion]=useState(0);
  useEffect(()=>{
    const el=host.current!,scene=new THREE.Scene();scene.background=new THREE.Color(t.dark);setError('');
    const [cx,cy]=mapCenter(map);const camera=new THREE.PerspectiveCamera(50,1,.05,300);camera.up.set(0,0,1);camera.position.set(cx+11,cy-15,18);
    let renderer:THREE.WebGLRenderer;
    try{renderer=new THREE.WebGLRenderer({antialias:true});}catch{setError('当前浏览器无法开启三维渲染，可以继续使用二维地图。');return;}
    renderer.setPixelRatio(Math.min(devicePixelRatio,1.25));el.appendChild(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(cx,cy,0);controls.enableDamping=true;controls.minDistance=1;controls.maxDistance=150;
    sceneHandle.current={camera,controls,staticCloud:null};
    const grid=new THREE.GridHelper(100,100,t.darkGrid,t.darkGrid);grid.rotation.x=Math.PI/2;grid.position.set(cx,cy,-.1);scene.add(grid);
    const dynamic=<T extends THREE.Object3D>(o:T)=>{o.frustumCulled=false;scene.add(o);return o;};
    const vehicle=dynamic(new THREE.Mesh(new THREE.BoxGeometry(1.086,.768,.36),new THREE.MeshBasicMaterial({color:t.live})));
    const heading=dynamic(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),1,t.onDark,.2,.14));
    const goalGroup=dynamic(new THREE.Group());
    const ring=new THREE.Mesh(new THREE.RingGeometry(.16,.24,40),new THREE.MeshBasicMaterial({color:t.onDark,side:THREE.DoubleSide}));ring.position.z=.12;goalGroup.add(ring);
    const goalArrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,.14),.8,t.live,.25,.16);goalGroup.add(goalArrow);
    overlay.current={vehicle,heading,goalArrow,goal:goalGroup,route:dynamic(new THREE.Group()),
      trajectory:dynamic(new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:t.history}))),
      obstacles:dynamic(new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:t.obstacle,size:.12}))),
      cloud:dynamic(new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:t.live,size:.08})))};
    vehicle.visible=heading.visible=goalGroup.visible=false;
    let dead=false,staticCloud:THREE.Points|null=null;
    mapPoints(map.revision).then(buffer=>{if(dead)return;const xyz=new Float32Array(buffer),geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));const colors=new Float32Array(xyz.length);for(let i=0;i<xyz.length;i+=3){const h=Math.max(0,Math.min(1,(xyz[i+2]+.3)/3));const c=new THREE.Color().setHSL(.60,.04,.38+h*.35);colors.set([c.r,c.g,c.b],i);}geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));staticCloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:.055,vertexColors:true}));staticCloud.visible=latestLayers.current.staticCloud;scene.add(staticCloud);if(sceneHandle.current)sceneHandle.current.staticCloud=staticCloud;}).catch(e=>{if(!dead)setError(e.message);});
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
      scene.traverse(o=>{const m=o as THREE.Mesh;m.geometry?.dispose();const a=m.material;if(Array.isArray(a))a.forEach(v=>v.dispose());else a?.dispose();});renderer.dispose();renderer.domElement.remove();overlay.current=null;sceneHandle.current=null;};
  },[map]);

  useEffect(()=>{
    const h=sceneHandle.current;if(!h)return;
    if(command.kind==='fit'){const [x,y]=mapCenter(map);h.controls.target.set(x,y,0);h.camera.position.set(x+11,y-15,18);}
    else if(command.kind==='vehicle'&&poseRef.current){const p=poseRef.current;const offset=h.camera.position.clone().sub(h.controls.target);h.controls.target.set(p[0],p[1],0);h.camera.position.copy(h.controls.target).add(offset);}
    else if(command.kind==='in'||command.kind==='out'){const offset=h.camera.position.clone().sub(h.controls.target).multiplyScalar(command.kind==='in'?.77:1.3);offset.clampLength(1,150);h.camera.position.copy(h.controls.target).add(offset);}
    h.controls.update();
  },[command.serial,sceneVersion]);
  useEffect(()=>{const o=overlay.current;if(!o)return;o.trajectory.visible=layers.trajectory;o.route.visible=layers.route;o.cloud.visible=layers.cloud;o.obstacles.visible=layers.obstacles;const s=sceneHandle.current?.staticCloud;if(s)s.visible=layers.staticCloud;},[layers,sceneVersion]);
  const pose=state?.localized?state.pose:null;
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
  const original=state?.task?.replans?state.plan?.segments:undefined;
  const routeKey=useMemo(()=>JSON.stringify([segments,original]),[segments,original]);
  useEffect(()=>{const o=overlay.current;if(!o)return;
    o.route.children.slice().forEach(c=>{o.route.remove(c);const m=c as THREE.Mesh;m.geometry.dispose();(m.material as THREE.Material).dispose();});
    original?.forEach(s=>{const geo=new THREE.BufferGeometry().setFromPoints(s.poses.map(p=>new THREE.Vector3(p[0],p[1],.08)));const line=new THREE.Line(geo,new THREE.LineDashedMaterial({color:t.muted,dashSize:.25,gapSize:.2}));line.computeLineDistances();o.route.add(line);});
    segments.forEach(s=>{if(!s.poses.length)return;const color=colors[s.kind];
      if(s.kind==='rotate'){const ring=new THREE.Mesh(new THREE.RingGeometry(.34,.39,40),new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide}));ring.position.set(s.poses[0][0],s.poses[0][1],.12);o.route.add(ring);}
      else{const geo=new THREE.BufferGeometry().setFromPoints(s.poses.map(p=>new THREE.Vector3(p[0],p[1],.10)));o.route.add(new THREE.Line(geo,s.kind==='reverse'?new THREE.LineDashedMaterial({color,dashSize:.18,gapSize:.14}):new THREE.LineBasicMaterial({color})).computeLineDistances());}
    });
  },[sceneVersion,routeKey]);
  return <div className="three-view" ref={host} aria-label="三维地图观察视图">{error&&<div className="three-error" role="alert"><p>{error}</p><Button variant="secondary" onClick={onReturn}>返回二维地图</Button></div>}</div>;
}
