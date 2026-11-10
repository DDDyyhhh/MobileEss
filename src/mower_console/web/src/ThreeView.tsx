import {useEffect,useRef} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {MapData,State} from './types';
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

export function ThreeView({map,state}:{map:MapData;state:State|null}) {
  const host=useRef<HTMLDivElement>(null),overlay=useRef<THREE.Group|null>(null);
  useEffect(()=>{
    const el=host.current!,scene=new THREE.Scene();scene.background=new THREE.Color('#101923');
    const [cx,cy]=mapCenter(map);const camera=new THREE.PerspectiveCamera(50,1,.05,300);camera.up.set(0,0,1);camera.position.set(cx+11,cy-15,18);
    let renderer:THREE.WebGLRenderer;
    try{renderer=new THREE.WebGLRenderer({antialias:true});}catch{el.dataset.error='当前浏览器无法开启三维渲染，请返回二维导航';return;}
    renderer.setPixelRatio(Math.min(devicePixelRatio,1.25));el.appendChild(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(cx,cy,0);controls.enableDamping=true;
    const grid=new THREE.GridHelper(40,40,0x3c535e,0x243440);grid.rotation.x=Math.PI/2;grid.position.set(cx,cy,-.1);scene.add(grid);
    const group=new THREE.Group();scene.add(group);overlay.current=group;
    let dead=false;
    mapPoints(map.revision).then(buffer=>{if(dead)return;const xyz=new Float32Array(buffer),geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));const colors=new Float32Array(xyz.length);for(let i=0;i<xyz.length;i+=3){const h=Math.max(0,Math.min(1,(xyz[i+2]+.3)/3));const c=new THREE.Color().setHSL(.47-h*.10,.28,.35+h*.18);colors.set([c.r,c.g,c.b],i);}geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));scene.add(new THREE.Points(geometry,new THREE.PointsMaterial({size:.055,vertexColors:true})));}).catch(e=>{if(!dead)el.dataset.error=e.message;});
    const ro=new ResizeObserver(()=>{camera.aspect=el.clientWidth/el.clientHeight;camera.updateProjectionMatrix();renderer.setSize(el.clientWidth,el.clientHeight);});ro.observe(el);
    let frame=0,lastFrame=0;const animate=(now:number)=>{if(now-lastFrame>=50&&document.visibilityState==='visible'){controls.update();renderer.render(scene,camera);lastFrame=now;}frame=requestAnimationFrame(animate);};frame=requestAnimationFrame(animate);
    return()=>{dead=true;cancelAnimationFrame(frame);ro.disconnect();controls.dispose();scene.traverse(o=>{const m=o as THREE.Mesh;m.geometry?.dispose();const a=m.material;if(Array.isArray(a))a.forEach(v=>v.dispose());else a?.dispose();});renderer.dispose();el.replaceChildren();overlay.current=null;};
  },[map]);
  useEffect(()=>{
    const g=overlay.current;if(!g)return;g.children.slice().forEach(o=>{g.remove(o);o.traverse(child=>{const m=child as THREE.Mesh;m.geometry?.dispose();if(Array.isArray(m.material))m.material.forEach(v=>v.dispose());else m.material?.dispose();});});
    if(state?.pose){const vehicle=new THREE.Mesh(new THREE.BoxGeometry(1.086,.768,.36),new THREE.MeshBasicMaterial({color:0x75d6c1}));vehicle.position.set(state.pose[0],state.pose[1],.18);vehicle.rotation.z=state.pose[2];g.add(vehicle);const arrow=new THREE.ArrowHelper(new THREE.Vector3(Math.cos(state.pose[2]),Math.sin(state.pose[2]),0),new THREE.Vector3(state.pose[0],state.pose[1],.45),1,0xe4ffed,.2,.14);g.add(arrow);}
    state?.plan?.segments.forEach(s=>{const color=s.kind==='reverse'?0xf3b565:s.kind==='rotate'?0xbf9dff:0x68d6be;
      if(s.kind==='rotate'){const ring=new THREE.Mesh(new THREE.RingGeometry(.34,.39,40),new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide}));ring.position.set(s.poses[0][0],s.poses[0][1],.12);g.add(ring);}
      else{const geo=new THREE.BufferGeometry().setFromPoints(s.poses.map(p=>new THREE.Vector3(p[0],p[1],.10)));g.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color})));}
    });
    if(state?.trajectory.length){const geo=new THREE.BufferGeometry().setFromPoints(state.trajectory.map(p=>new THREE.Vector3(p[0],p[1],.06)));g.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x70b9b7})));}
    if(state?.obstacles.length){const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(state.obstacles.flat(),3));g.add(new THREE.Points(geo,new THREE.PointsMaterial({color:0xef977b,size:.12})));}
    const points=state?.cloud||[];if(points.length){const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(points.flat(),3));g.add(new THREE.Points(geo,new THREE.PointsMaterial({color:0xe9b97c,size:.08})));}
  },[state]);
  return <div className="three-view" ref={host} aria-label="三维地图观察视图"><div className="three-note">三维观察 · 拖动旋转 / 滚轮缩放 · 目标请在二维地图设置</div></div>;
}
