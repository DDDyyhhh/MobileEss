export type Pose = [number, number, number];
export type Segment = {kind: 'forward'|'reverse'|'rotate'; poses: Pose[]; length_m: number; turn: number};
export type Plan = {id: string; goal: number[]; start: Pose; segments: Segment[]; length_m: number; consumed: boolean};
export type MapData = {width: number; height: number; resolution: number; origin: Pose; data: number[]; revision: string};
export type State = {
  phase: string; reason: string; localized: boolean; localization: string; stationary: boolean;
  pose: Pose|null; api_mode: number|null; wheel_mm_s: number[]; actual_v: number;
  command_echo: number[]|null; healthy: boolean; fault: string; demo: boolean;
  plan: Plan|null; motion_enabled: boolean; generation: number; trajectory: number[][];
  obstacles: number[][]; cloud: number[][]; task: {id:string;state:string;segment:number;stop_confirmed:boolean;reason?:string}|null;
  feedback_fresh?: boolean;
};
export const phases: Record<string,string> = {LOCALIZING:'定位中',READY:'观测就绪',PLANNING:'正在规划',PLANNED:'等待路线确认',RUNNING:'正在导航',STOPPING:'正在确认停车',SUCCEEDED:'已到达',BLOCKED:'已阻塞',CANCELED:'已取消',FAULT:'需要处理'};
export const kinds = {forward:'前进', reverse:'倒车', rotate:'原地转向'};
let controller = '';
export function setController(value: string) {controller=value}
export async function api(path: string, body?: unknown) {
  const r = await fetch('/api/'+path, {credentials:'same-origin', headers:{'X-Mower-Control':controller,...(body===undefined?{}:{'Content-Type':'application/json'})}, method:body===undefined?'GET':'POST', body:body===undefined?undefined:JSON.stringify(body)});
  const text = await r.text();
  let data; try {data=JSON.parse(text)} catch {data={error:text}}
  if (!r.ok) throw new Error(data.error || '请求失败');
  return data;
}
