import {describe,expect,test} from 'vitest';
import {actionAvailability,readableReason,statusCopy} from './consoleModel';
import {Goal,State} from './types';

export const readyState=():State=>({phase:'PLANNED',reason:'',localized:true,localization:'accepted',stationary:true,
  pose:[0,0,0],api_mode:0,wheel_mm_s:[0,0,0,0],actual_v:0,command_echo:[0,0],healthy:true,fault:'',demo:false,
  plan:{id:'plan-a',goal:[4,2],start:[0,0,0],segments:[{kind:'forward',poses:[[0,0,0],[4,2,0]],length_m:4.5,turn:0}],length_m:4.5,consumed:false,generation:1},
  motion_enabled:true,capabilities:{forward:true,rotate:true,reverse:false},generation:1,trajectory:[],task:null,feedback_fresh:true,boot_id:'boot-a'});
const goal:Goal={x:4,y:2,yaw:null};
const context=()=>({state:readyState(),connected:true,owner:true,pending:false,goal});
describe('operator action availability',()=>{
  test('a healthy reviewed route is available; observation still allows preview but cannot start',()=>{
    const c=context();expect(actionAvailability(c).canStart).toBe(true);
    c.state.motion_enabled=false;const a=actionAvailability(c);expect(a.canPreview).toBe(true);expect(a.canStart).toBe(false);expect(a.startReason).toContain('观测模式');
  });
  test.each(['connection','ownership','localization','stationary','health','generation','consumed','heading','capability'])('%s loss prevents a start',reason=>{
    const c=context();
    switch(reason){case'connection':c.connected=false;break;case'ownership':c.owner=false;break;case'localization':c.state.localized=false;break;case'stationary':c.state.stationary=false;break;case'health':c.state.healthy=false;break;case'generation':c.state.generation=2;break;case'consumed':c.state.plan!.consumed=true;break;case'heading':c.goal={...goal,yaw:1};break;case'capability':c.state.capabilities={};}
    expect(actionAvailability(c).canStart).toBe(false);
  });
  test('a paired observer can stop while a different operation is pending',()=>{
    const c=context();c.owner=false;c.pending=true;c.state.task={id:'task-a',state:'RUNNING',segment:0,stop_confirmed:false};
    expect(actionAvailability(c).canStop).toBe(true);expect(actionAvailability(c).canStart).toBe(false);
  });
  test('missing goals and expired plans cannot be replayed',()=>{
    const c=context();c.state.plan=null;expect(actionAvailability(c).canStart).toBe(false);
    expect(actionAvailability({...context(),goal:null}).canPreview).toBe(false);
  });
});
test('disconnect takes priority over stale success and localization',()=>{
  const state=readyState();state.phase='SUCCEEDED';expect(statusCopy(state,false).title).toBe('连接已中断');
});
test('wait and detour have separate operator instructions',()=>{
  const state=readyState();state.task={id:'t',state:'RUNNING',segment:0,stop_confirmed:false,substate:'WAITING_OBSTACLE'};
  expect(statusCopy(state,true).title).toBe('等待障碍离开');state.task.substate='REPLANNING';expect(statusCopy(state,true).title).toBe('正在规划绕行');
});
test('localization movement is described as observed pose change; unknown errors remain visible',()=>{
  expect(readableReason('startup_vehicle_moved')).toContain('位姿变化');expect(readableReason('unrecognized_detail')).toBe('unrecognized_detail');
});
