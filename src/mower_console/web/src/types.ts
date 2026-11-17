export type Pose = [number, number, number];
export type Kind = 'forward'|'reverse'|'rotate';
export type Segment = {kind: Kind; poses: Pose[]; length_m: number; turn: number};
export type Plan = {id: string; goal: number[]; start: Pose; segments: Segment[]; length_m: number; consumed: boolean; generation?: number};
export type MapData = {width: number; height: number; resolution: number; origin: Pose; data: number[]; revision: string};
export type Task = {
  id: string; state: string; segment: number; stop_confirmed: boolean; reason?: string;
  route?: Segment[]; substate?: string; replans?: number; waiting_s?: number;
  obstacle_reason?: string; replan_error?: string;
};
export type State = {
  phase: string; reason: string; localized: boolean; localization: string; stationary: boolean;
  pose: Pose|null; api_mode: number|null; wheel_mm_s: number[]; actual_v: number;
  command_echo: number[]|null; healthy: boolean; fault: string; demo: boolean;
  plan: Plan|null; motion_enabled: boolean; capabilities?: Partial<Record<Kind, boolean>>;
  generation: number; trajectory: number[][]; task: Task|null;
  obstacle_blocked?: boolean; gate_reason?: string; feedback_fresh?: boolean;
  boot_id?: string; map_revision?: string;
};
/** A goal position; yaw null lets the planner choose the arrival heading. */
export type Goal = {x: number; y: number; yaw: number|null};
export type Clouds = {cloud: Float32Array; obstacles: Float32Array};

export const phases: Record<string,string> = {LOCALIZING:'定位中',READY:'观测就绪',PLANNING:'正在规划',PLANNED:'等待路线确认',RUNNING:'正在导航',STOPPING:'正在确认停车',SUCCEEDED:'已到达',BLOCKED:'已阻塞',CANCELED:'已取消',FAULT:'需要处理'};
export const substates: Record<string,string> = {DRIVING:'正在导航',WAITING_OBSTACLE:'等待障碍离开',REPLANNING:'正在规划绕行'};
export const kinds: Record<Kind,string> = {forward:'前进', reverse:'倒车', rotate:'原地转向'};

export function goalArray(goal: Goal) {
  return goal.yaw===null ? [goal.x, goal.y] : [goal.x, goal.y, goal.yaw];
}
export function goalFromArray(values: number[]|undefined|null): Goal|null {
  return values && values.length>=2 ? {x:values[0], y:values[1], yaw:values.length>2?values[2]:null} : null;
}
export function sameGoal(values: number[]|undefined, goal: Goal|null) {
  const a=goalArray(goal??{x:NaN,y:NaN,yaw:null});
  return !!values && values.length===a.length && values.every((v,i)=>v===a[i]);
}
/** Heading from the goal towards a dragged point, or null when too close to tell. */
export function headingFromDrag(goal: Goal, point: number[], minimum=.15) {
  const dx=point[0]-goal.x, dy=point[1]-goal.y;
  return Math.hypot(dx,dy)<minimum ? null : Math.atan2(dy,dx);
}
/** Motion kinds a route needs that the board config has not accepted. */
export function unacceptedKinds(segments: Segment[]|undefined, capabilities: State['capabilities']) {
  return [...new Set((segments??[]).map(s=>s.kind))].filter(k=>!capabilities?.[k]);
}

let controller = '';
export function setController(value: string) {controller=value}
export async function api(path: string, body?: unknown) {
  const r = await fetch('/api/'+path, {credentials:'same-origin', headers:{'X-Mower-Control':controller,...(body===undefined?{}:{'Content-Type':'application/json'})}, method:body===undefined?'GET':'POST', body:body===undefined?undefined:JSON.stringify(body)});
  const text = await r.text();
  let data; try {data=JSON.parse(text)} catch {data={error:text}}
  if (!r.ok) throw new Error(data.error || '请求失败');
  return data;
}
