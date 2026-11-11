import asyncio
import copy
import math
from types import SimpleNamespace

import pytest

from mower_console.domain import Console, Refused
from mower_console.geometry import Grid, split_path
from mower_console.visibility import rear_observed
from mcu_can_bridge.motion_permit import MotionPermit


class Adapter:
    def __init__(self):
        self.data=dict(localized=True,stationary=True,healthy=True,pose=[0.,0.,0.],
                       generation=1,map_revision='map1',actual_v=0.)
        self.armed=0
        self.permits=[]
        self.block=False
        self.change_on_validate=None
        self.stop_fails=False
        self.clear=True
        self.plans=[]
        self.next_plan=None
        self.executed=[]
        self.events=[]

    def snapshot(self): return copy.deepcopy(self.data)
    async def plan(self,goal,start=None):
        self.plans.append((list(goal),start))
        if self.next_plan is not None and len(self.plans)>1:
            return copy.deepcopy(self.next_plan)
        poses=[[0.,0.,0.],[float(goal[0]),float(goal[1]),0.]]
        if len(goal)==3: poses.append([float(goal[0]),float(goal[1]),float(goal[2])])
        return poses
    async def route_clear(self,poses,radius):
        return self.clear(poses) if callable(self.clear) else self.clear
    def audit_event(self,event): self.events.append(event)
    async def validate(self,segments):
        if self.change_on_validate: self.change_on_validate()
    async def arm(self): self.armed+=1
    def permit(self,value): self.permits.append(value)
    async def execute_segment(self,segment):
        self.executed.append(segment)
        if self.block: await asyncio.Event().wait()
        self.data['pose']=segment['poses'][-1]
    async def wait_stationary(self,require_api_zero=True): pass
    async def stop(self):
        if self.stop_fails: raise ValueError('反馈缺失')
    async def relocalize(self,seed): self.data['generation']+=1;self.data['localized']=False
    def close(self): pass


def make_console():
    now=[100.]
    a=Adapter()
    c=Console(a,dict(motion_enabled=True,accepted_capabilities=dict(forward=True,reverse=True,rotate=True)),clock=lambda:now[0])
    c.claim('tab1')
    return c,a,now


CONFIRM=dict(route_clear=True,stop_ready=True,actuators_isolated=True)


@pytest.mark.asyncio
async def test_request_retries_do_not_execute_twice_and_success_needs_stop():
    c,a,_=make_console();p=await c.preview('tab1',[1.,0.])
    first,second=await asyncio.gather(c.start('tab1',p['id'],CONFIRM),c.start('tab1',p['id'],CONFIRM))
    await c.job
    assert first['id']==second['id']
    assert a.armed==1 and c.task['state']=='SUCCEEDED' and c.task['stop_confirmed']
    assert (await c.start('tab1',p['id'],CONFIRM))['id']==first['id']
    assert c.state()['phase']=='SUCCEEDED'


@pytest.mark.asyncio
async def test_expired_preview_cannot_arm():
    c,a,now=make_console();p=await c.preview('tab1',[1.,0.]);now[0]+=121;c.beat('tab1')
    with pytest.raises(Refused,match='过期'): await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0


@pytest.mark.asyncio
@pytest.mark.parametrize('mutation',[
    lambda a:a.data.update(generation=2),lambda a:a.data.update(map_revision='map2'),
    lambda a:a.data.update(pose=[.11,0.,0.]),lambda a:a.data.update(pose=[0.,0.,.1]),
    lambda a:a.data.update(stationary=False),lambda a:a.data.update(localized=False)])
async def test_context_changes_reject_old_review(mutation):
    c,a,_=make_console();p=await c.preview('tab1',[1.,0.]);mutation(a)
    with pytest.raises(Refused): await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0


@pytest.mark.asyncio
async def test_context_is_checked_again_after_slow_validation():
    c,a,_=make_console();p=await c.preview('tab1',[1.,0.])
    a.change_on_validate=lambda:a.data.update(generation=2)
    with pytest.raises(Refused):await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0


@pytest.mark.asyncio
async def test_disconnected_browser_stops_and_does_not_resume():
    c,a,now=make_console();a.block=True;p=await c.preview('tab1',[1.,0.])
    await c.start('tab1',p['id'],CONFIRM);await asyncio.sleep(.02);now[0]+=1.01
    await asyncio.wait_for(c.job,1.)
    assert c.task['state']=='BLOCKED' and c.task['stop_confirmed'] and a.permits[-1] is False
    c.claim('tab2');await asyncio.sleep(.02)
    assert a.armed==1 and c.plan is None


@pytest.mark.asyncio
async def test_missing_stop_feedback_is_fault_not_success():
    c,a,_=make_console();a.stop_fails=True;p=await c.preview('tab1',[1.,0.])
    await c.start('tab1',p['id'],CONFIRM);await c.job
    assert c.task['state']=='FAULT' and not c.task['stop_confirmed']


@pytest.mark.asyncio
async def test_obstacle_latches_task_blocked_without_automatic_retry():
    c,a,_=make_console();a.block=True;p=await c.preview('tab1',[1.,0.])
    await c.start('tab1',p['id'],CONFIRM);await asyncio.sleep(.01)
    a.data.update(healthy=False,fault='障碍进入通行范围')
    await asyncio.wait_for(c.job,1.)
    a.data['healthy']=True
    assert c.task['state']=='BLOCKED' and c.plan is None and a.armed==1


@pytest.mark.asyncio
async def test_observation_config_cannot_be_armed_by_http_task():
    c,a,_=make_console();c.config['motion_enabled']=False;p=await c.preview('tab1',[1.,0.])
    with pytest.raises(Refused,match='观测模式'):await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0 and not any(a.permits)


@pytest.mark.asyncio
async def test_connection_guard_also_cancels_during_arming():
    c,a,now=make_console()
    async def slow_arm():await asyncio.Event().wait()
    a.arm=slow_arm;p=await c.preview('tab1',[1.,0.])
    await c.start('tab1',p['id'],CONFIRM);await asyncio.sleep(.01);now[0]+=1.1
    await asyncio.wait_for(c.job,1.)
    assert c.task['state']=='BLOCKED' and c.task['stop_confirmed']


@pytest.mark.asyncio
async def test_duplicate_stop_does_not_interrupt_stop_confirmation():
    c,a,_=make_console();a.block=True
    async def slow_stop():await asyncio.sleep(.04)
    a.stop=slow_stop;p=await c.preview('tab1',[1.,0.])
    await c.start('tab1',p['id'],CONFIRM);await asyncio.sleep(.01)
    await asyncio.gather(c.stop(),c.stop())
    assert c.task['stop_confirmed'] and c.task['state']=='CANCELED'


@pytest.mark.asyncio
async def test_audit_failure_cannot_leave_a_running_or_armed_task():
    c,a,_=make_console()
    def failed(*args):raise OSError('disk full')
    a.audit_start=failed;p=await c.preview('tab1',[1.,0.])
    with pytest.raises(Refused,match='未启动任务'):await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0 and c.task is None and not p['consumed']


@pytest.mark.asyncio
async def test_unaccepted_capability_and_incomplete_confirmation_never_arm():
    c,a,_=make_console();p=await c.preview('tab1',[1.,0.])
    with pytest.raises(Refused):await c.start('tab1',p['id'],{})
    c.config['accepted_capabilities']['forward']=False
    with pytest.raises(Refused,match='尚未验收'):await c.start('tab1',p['id'],CONFIRM)
    assert a.armed==0


def test_single_controller_does_not_evict_active_owner():
    c,_,now=make_console()
    with pytest.raises(Refused):c.claim('tab2')
    now[0]+=1.01;c.claim('tab2')
    with pytest.raises(Refused):c.beat('tab1')


def test_reverse_is_preserved_and_rotation_does_not_reset_reverse_budget():
    segments=split_path([[0,0,0],[-.4,0,0],[-.4,0,math.pi],[-.1,0,math.pi]])
    assert [s['kind'] for s in segments]==['reverse','rotate','reverse']
    with pytest.raises(ValueError,match='连续倒车'):
        split_path([[0,0,0],[-.7,0,0],[-.7,0,math.pi],[0,0,math.pi]])


def test_tight_curvature_and_lateral_motion_are_rejected():
    with pytest.raises(ValueError):split_path([[0,0,0],[.05,0,.1]])
    with pytest.raises(ValueError):split_path([[0,0,0],[0,.1,0]])


def test_filled_disk_finds_small_obstacle_inside_old_sampling_rings():
    grid=Grid(100,100,.1,(-5,-5,0),[0]*10000)
    grid.data[grid.index(.3,.3)]=100
    assert not grid.clear_disk(0,0,.7)
    assert not grid.clear_disk(4.8,0,.7)
    grid.data[grid.index(.3,.3)]=-1
    assert not grid.clear_disk(0,0,.7)


def test_rotated_map_origin_and_bounds():
    grid=Grid(10,20,.1,(2,3,math.pi/2),[0]*200)
    assert grid.index(1.95,3.05)==0
    assert grid.index(2.1,2.9) is None


def test_lease_is_latched_and_older_true_does_not_rearm():
    lease=MotionPermit(.25);assert not lease.valid(0)
    lease.update(True,1.);assert lease.valid(1.2)
    assert not lease.valid(1.26)
    lease.update(True,1.27);assert not lease.valid(1.27)
    other=MotionPermit();other.update(True,1);other.update(False,1.1);other.update(True,1.2)
    assert not other.valid(1.2)


def test_rear_missing_or_self_occluded_returns_do_not_clear():
    assert not rear_observed([])
    points=[]
    for sector in list(range(0,10))+list(range(62,72)):
        a=-math.pi+(sector+.5)*math.radians(5)
        points.append(SimpleNamespace(x=2*math.cos(a)-.17,y=2*math.sin(a),z=.2))
    assert rear_observed(points)
    points.append(SimpleNamespace(x=-.5,y=-.01,z=.2))
    assert not rear_observed(points)


async def wait_until(condition,timeout=2.):
    async def poll():
        while not condition(): await asyncio.sleep(.01)
    await asyncio.wait_for(poll(),timeout)


async def start_blocked(goal=(1.,0.)):
    c,a,now=make_console();a.block=True
    p=await c.preview('tab1',list(goal));await c.start('tab1',p['id'],CONFIRM)
    await wait_until(lambda:a.executed)
    a.clear=False
    a.data.update(obstacle_blocked=True,gate_reason='predicted footprint cost 254 >= 253')
    await wait_until(lambda:c.task.get('substate')=='WAITING_OBSTACLE')
    a.data['obstacle_blocked']=False
    return c,a,now


def advance(c,now,seconds):
    now[0]+=seconds;c.beat('tab1')


@pytest.mark.asyncio
async def test_obstacle_waits_then_continues_on_the_reviewed_route():
    c,a,now=await start_blocked()
    assert c.state()['phase']=='RUNNING' and c.task['obstacle_reason'].startswith('predicted')
    a.block=False;a.clear=True
    await asyncio.sleep(.3);advance(c,now,1.1)
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='SUCCEEDED' and c.task['replans']==0 and len(a.plans)==1
    assert len(a.executed)==2 and a.executed[1]['poses'][-1]==[1.,0.,0.]
    # The motion lease is renewed through the wait and revoked once at the end.
    assert a.permits.count(False)==1 and a.permits[-1] is False


@pytest.mark.asyncio
async def test_persistent_obstacle_gets_a_validated_detour_and_evidence():
    c,a,now=await start_blocked()
    a.next_plan=[[0.,0.,0.],[.5,0.,0.],[1.,0.,0.]]
    a.clear=lambda poses:len(a.plans)>1
    a.block=False;advance(c,now,10.1)
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='SUCCEEDED' and c.task['replans']==1
    assert a.plans[1][1]==[0.,0.,0.] and a.plans[1][0]==[1.,0.]
    assert c.task['route'][0]['poses'][1]==[.5,0.,0.]
    assert a.events and a.events[0]['kind']=='replan'


@pytest.mark.asyncio
async def test_no_detour_budget_blocks_and_stops():
    c,a,now=await start_blocked();c.config['max_replans']=0
    advance(c,now,10.1)
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='BLOCKED' and '绕行次数' in c.task['reason'] and c.task['stop_confirmed']


@pytest.mark.asyncio
async def test_detour_needing_unaccepted_reverse_is_refused_until_total_wait_ends():
    c,a,now=await start_blocked()
    c.config['accepted_capabilities']['reverse']=False
    a.next_plan=[[0.,0.,0.],[-.5,0.,0.]]
    advance(c,now,10.1)
    await wait_until(lambda:c.task['replan_error'])
    assert '倒车' in c.task['replan_error'] or 'reverse' in c.task['replan_error']
    assert c.task['replans']==0 and c.task['state']=='RUNNING'
    advance(c,now,51.)
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='BLOCKED' and '未清除' in c.task['reason']


@pytest.mark.asyncio
async def test_lost_controller_still_stops_during_an_obstacle_wait():
    c,a,now=await start_blocked()
    now[0]+=1.01
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='BLOCKED' and c.task['stop_confirmed'] and a.permits[-1] is False


@pytest.mark.asyncio
async def test_localization_fault_during_wait_is_not_treated_as_an_obstacle():
    c,a,_=await start_blocked()
    a.data.update(healthy=False,fault='定位未就绪')
    await asyncio.wait_for(c.job,2.)
    assert c.task['state']=='BLOCKED' and c.task['reason']=='定位未就绪'


@pytest.mark.asyncio
async def test_goal_heading_is_planned_and_checked_on_arrival():
    c,a,_=make_console()
    p=await c.preview('tab1',[.4,0.,math.pi/2])
    assert [s['kind'] for s in p['segments']]==['forward','rotate'] and p['goal'][2]==pytest.approx(math.pi/2)
    await c.start('tab1',p['id'],CONFIRM);await c.job
    assert c.task['state']=='SUCCEEDED'
    for bad in ([1.,0.,0.,0.],[1.,True],[1.,float('nan')]):
        with pytest.raises(Refused):await c.preview('tab1',bad)


def test_live_points_inside_the_gate_envelope_block_the_route():
    from mower_console.geometry import clear_of_points
    route=[[0.,0.,0.],[3.,0.,0.]]
    assert clear_of_points(route,[],1.1)
    assert clear_of_points(route,[[1.5,1.2,.3],[4.2,0.,.3]],1.1)
    assert not clear_of_points(route,[[1.5,1.,.3]],1.1)
    assert not clear_of_points(route,[[3.5,.3,.3]],1.1)


def test_resume_trims_interrupted_segments():
    from mower_console.domain import remaining_poses,resume_route
    route=split_path([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.],[2.,0.,math.pi/2]])
    assert [s['kind'] for s in route]==['forward','rotate']
    trimmed,index=resume_route(route,0,[1.02,0.,0.])
    assert index==0 and trimmed[0]['poses'][0]==[1.,0.,0.] and trimmed[0]['length_m']==pytest.approx(1.)
    assert remaining_poses(route,0,[1.02,0.,0.])[0]==[1.02,0.,0.]
    trimmed,index=resume_route(route,1,[2.,0.,math.pi/4])
    assert index==1 and trimmed[1]['turn']==pytest.approx(math.pi/4)
    assert resume_route(route,1,[2.,0.,math.pi/2])[1]==2
