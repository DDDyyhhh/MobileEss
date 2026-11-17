import {Goal, Kind, State, kinds, sameGoal, unacceptedKinds} from './types';

export const activeTask = (state:State|null) => !!state?.task && ['RUNNING','STOPPING'].includes(state.task.state);
const localizationMessages: Record<string,string> = {
  startup_vehicle_moved:'定位期间检测到位姿变化。请保持车辆静止，再重新定位。',
  startup_waiting_stationary:'等待新鲜的停稳反馈，确认车辆静止后开始定位。',
  startup_waiting_odometry:'等待有效里程计数据。',
  startup_waiting_map:'正在等待固定地图。',
  startup_searching:'正在已建地图中搜索车辆位置，请保持静止。',
  startup_confirming:'已找到候选位置，正在用后续点云确认。',
  startup_timeout:'定位搜索已超时。可以重试，或在地图上标出大致车位。',
  startup_insufficient_structure:'当前点云结构不足。请检查雷达视野和点云数据。',
  startup_search_rejected:'暂未找到可靠匹配。可以提供大致车位和朝向。',
  startup_ambiguous:'存在多个相似位置。请提供大致车位和朝向。',
  startup_candidates_ambiguous:'存在多个相似位置。请提供大致车位和朝向。',
  startup_unstable:'候选位置尚不稳定，正在继续确认。',
  startup_outside_free_map:'候选位置未通过地图检查，请核对车辆所在区域。',
  startup_invalidated:'定位条件发生变化，请静止后重试。',
  relocalization_requested:'已请求重新定位，等待新的可靠匹配。',
  awaiting_initial_alignment:'等待首次可靠定位。',
  heartbeat_stale:'定位数据已过期，请检查传感器与定位服务。',
  accepted:'定位已匹配。',
};
export function readableReason(value:string|undefined) {
  if (!value) return '';
  return localizationMessages[value] ?? value.replace(/status数据过期/g,'底盘状态已过期')
    .replace(/wheels数据过期/g,'轮速反馈已过期').replace(/odom数据过期/g,'里程计已过期')
    .replace(/costmap数据过期/g,'局部地图已过期').replace(/obstacles数据过期/g,'障碍点云已过期');
}
export function statusCopy(state:State|null, connected:boolean) {
  if (!connected) return {title:state?'连接已中断':'正在连接', detail:state?'正在重新连接板端。实时位置与操作将在连接恢复后更新。':'等待板端状态。', tone:'muted'};
  if (!state) return {title:'等待车辆状态',detail:'已连接，等待首帧数据。',tone:'muted'};
  const task=state.task;
  if (task?.state==='STOPPING') return {title:'正在确认停车',detail:'停止请求已发送，等待车辆停稳反馈。',tone:'warning'};
  if (task?.state==='RUNNING') {
    if(task.substate==='WAITING_OBSTACLE') return {title:'等待障碍离开',detail:'车辆已暂停。通道恢复后继续，持续阻塞时尝试绕行。',tone:'warning'};
    if(task.substate==='REPLANNING') return {title:'正在规划绕行',detail:'正在寻找前往同一目标的可行路线。',tone:'blue'};
    return {title:'正在前往目标',detail:'实时跟踪当前路线，遇障自动等待。',tone:'blue'};
  }
  if (['FAULT','BLOCKED'].includes(state.phase)) return {title:state.phase==='BLOCKED'?'任务已阻塞':'需要处理',detail:readableReason(task?.reason||state.reason||state.fault),tone:'danger'};
  if(state.phase==='SUCCEEDED') return {title:'已到达目标',detail:task?.stop_confirmed?'本次任务完成，车辆已确认停稳。':'等待停车确认。',tone:'blue'};
  if(state.phase==='CANCELED') return {title:'任务已停止',detail:task?.stop_confirmed?'车辆已确认停稳，可重新选择目标。':'请核对停车反馈。',tone:'muted'};
  if (!state.localized) return {title:'正在定位',detail:readableReason(state.localization)||'等待可靠定位后显示车辆位置。',tone:'warning'};
  if(state.phase==='PLANNING') return {title:'正在生成路线',detail:'检查地图通行范围与目标朝向。',tone:'blue'};
  if(state.plan) return {title:'路线已就绪',detail:'请核对路线与现场，再决定下一步。',tone:'blue'};
  return {title:state.demo?'离线预览':state.healthy?'准备就绪':'等待数据恢复',detail:state.demo?'示例位姿，仅用于界面预览。':state.healthy?'在地图上选择目的地，预览到达路线。':readableReason(state.fault),tone:state.healthy||state.demo?'blue':'warning'};
}
export type ActionContext = {state:State|null; connected:boolean; owner:boolean; pending:boolean; goal:Goal|null};
export function actionAvailability({state,connected,owner,pending,goal}:ActionContext) {
  const active=activeTask(state);
  const missing=unacceptedKinds(state?.plan?.segments,state?.capabilities);
  let previewReason='';
  if(!connected) previewReason='等待连接恢复';
  else if(!owner) previewReason='请先取得控制权';
  else if(active) previewReason='当前任务结束后可重新规划';
  else if(pending||state?.phase==='PLANNING') previewReason='正在处理请求';
  else if(!state?.localized) previewReason='等待可靠定位';
  else if(!state.stationary) previewReason='等待车辆停稳反馈';
  else if(!state.healthy&&!state.demo) previewReason='等待传感器数据恢复';
  else if(!goal) previewReason='请先在地图上选择目标';
  let startReason=previewReason;
  if(!startReason) {
    if(!state?.motion_enabled||state.demo) startReason='观测模式，暂不启用运动';
    else if(!state.plan||state.plan.consumed) startReason='请生成新的路线预览';
    else if(!sameGoal(state.plan.goal,goal)) startReason='目标或朝向已改变，请重新预览';
    else if(state.plan.generation!==undefined&&state.plan.generation!==state.generation) startReason='定位已变化，请重新预览';
    else if(missing.length) startReason=`${missing.map(k=>kinds[k]).join('、')}尚未验收`;
  }
  return {active,missing,canPreview:!previewReason,previewReason,canStart:!startReason,startReason,
    canRelocalize:connected&&owner&&!active&&!pending&&!!state?.stationary&&!state.demo,
    canStop:connected&&active, stale:!!state?.plan&&!!goal&&!sameGoal(state.plan.goal,goal)};
}
export const motionKinds:Kind[]=['forward','rotate','reverse'];
