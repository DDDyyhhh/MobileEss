// @vitest-environment jsdom
import React from 'react';
import {afterEach,beforeEach,expect,test,vi} from 'vitest';
import {cleanup,render,screen,waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {App} from './App';
import {PairDialog} from './Dialogs';
import {useConsoleSession} from './useConsoleSession';
import {api,State} from './types';

vi.mock('./useConsoleSession',()=>({useConsoleSession:vi.fn()}));
vi.mock('./types',async original=>({...await original<typeof import('./types')>(),api:vi.fn()}));
vi.mock('./MapWorkspace',()=>({MapWorkspace:({onSelect}:{onSelect:(xy:number[])=>void})=><button onClick={()=>onSelect([8,3])}>选择其他地图目标</button>}));
const state=():State=>({phase:'PLANNED',reason:'',localized:true,localization:'accepted',stationary:true,
  pose:[0,0,0],api_mode:0,wheel_mm_s:[0,0,0,0],actual_v:0,command_echo:[0,0],healthy:true,fault:'',demo:false,
  plan:{id:'plan-a',goal:[4,2],start:[0,0,0],segments:[{kind:'forward',poses:[[0,0,0],[4,2,0]],length_m:4.5,turn:0}],length_m:4.5,consumed:false,generation:1},
  motion_enabled:true,capabilities:{forward:true,rotate:true,reverse:false},generation:1,trajectory:[],task:null,feedback_fresh:true,boot_id:'boot-a'});
let session:ReturnType<typeof useConsoleSession>;
beforeEach(()=>{
  vi.clearAllMocks();session={auth:'paired',state:state(),map:null,connected:true,owner:true,clouds:{cloud:new Float32Array(),obstacles:new Float32Array()},error:'',setError:vi.fn(),mapError:'',retryMap:vi.fn(),pending:'',stopping:false,perform:vi.fn(async(_name,action)=>{await action();return true;}),pair:vi.fn(async()=>true),stop:vi.fn(async()=>{}),claim:vi.fn()};
  vi.mocked(useConsoleSession).mockImplementation(()=>session);vi.mocked(api).mockResolvedValue({});
});
afterEach(cleanup);
test('observation supports planning while start stays disabled with its reason',()=>{
  session.state!.motion_enabled=false;render(<App/>);
  expect((screen.getByRole('button',{name:'预览路线'}) as HTMLButtonElement).disabled).toBe(false);
  expect((screen.getByRole('button',{name:'确认并开始'}) as HTMLButtonElement).disabled).toBe(true);
  expect(screen.getByText('观测模式，暂不启用运动')).toBeTruthy();
});
test('clearing the destination cannot silently reuse the server preview goal',async()=>{
  render(<App/>);await userEvent.click(screen.getByRole('button',{name:'清除'}));
  expect(screen.getByText('想去哪里？')).toBeTruthy();expect((screen.getByRole('button',{name:'确认并开始'}) as HTMLButtonElement).disabled).toBe(true);
});
test('changing a goal requires a fresh route',async()=>{
  render(<App/>);await userEvent.click(screen.getByRole('button',{name:'选择其他地图目标'}));
  expect((screen.getByRole('button',{name:'确认并开始'}) as HTMLButtonElement).disabled).toBe(true);
  expect(screen.getByText('目标或朝向已改变，请重新生成预览。')).toBeTruthy();
});
test('a start sends exactly the reviewed plan and three explicit confirmations',async()=>{
  render(<App/>);await userEvent.click(screen.getByRole('button',{name:'确认并开始'}));
  const start=screen.getByRole('button',{name:'开始本次任务'}) as HTMLButtonElement;expect(start.disabled).toBe(true);
  for(const checkbox of Array.from(screen.getByRole('dialog').querySelectorAll('input[type=checkbox]')))await userEvent.click(checkbox);
  await userEvent.click(start);
  await waitFor(()=>expect(api).toHaveBeenCalledWith('tasks',{plan_id:'plan-a',confirmations:{route_clear:true,stop_ready:true,actuators_isolated:true}}));
});
test.each(['disconnect','ownership','new-preview','new-boot'])('%s invalidates an open start confirmation',async(reason)=>{
  const {rerender}=render(<App/>);await userEvent.click(screen.getByRole('button',{name:'确认并开始'}));expect(screen.getByRole('dialog')).toBeTruthy();
  if(reason==='disconnect')session.connected=false;
  if(reason==='ownership')session.owner=false;
  if(reason==='new-preview')session.state={...session.state!,plan:{...session.state!.plan!,id:'plan-b'}};
  if(reason==='new-boot')session.state={...session.state!,boot_id:'boot-b'};
  rerender(<App/>);await waitFor(()=>expect(screen.queryByRole('dialog')).toBeNull());expect(api).not.toHaveBeenCalled();
});
test('stop remains available to an observer during another pending request',async()=>{
  session.owner=false;session.pending='preview';session.state!.task={id:'task-a',state:'RUNNING',segment:0,stop_confirmed:false,substate:'WAITING_OBSTACLE',waiting_s:8,replans:1};
  render(<App/>);expect(screen.getAllByText('等待障碍离开').length).toBeGreaterThan(0);
  await userEvent.click(screen.getByRole('button',{name:'停止'}));expect(session.stop).toHaveBeenCalledOnce();
});
test('pairing shows failures and only submits a six-digit code',async()=>{
  const pair=vi.fn();render(<PairDialog busy={false} error="配对码不正确" onPair={pair}/>);
  const input=screen.getByLabelText('6 位配对码');await userEvent.type(input,'123');expect((screen.getByRole('button',{name:'连接控制台'}) as HTMLButtonElement).disabled).toBe(true);
  await userEvent.type(input,'456');await userEvent.click(screen.getByRole('button',{name:'连接控制台'}));expect(pair).toHaveBeenCalledWith('123456');expect(screen.getByRole('alert').textContent).toBe('配对码不正确');
});
test('details hide stale physical feedback and reject incomplete manual location',async()=>{
  session.state!.feedback_fresh=false;render(<App/>);await userEvent.click(screen.getByRole('button',{name:'车辆详情'}));
  expect(screen.getAllByText('暂无有效反馈')).toHaveLength(2);expect((screen.getByRole('button',{name:'按提示重新匹配'}) as HTMLButtonElement).disabled).toBe(true);
});
