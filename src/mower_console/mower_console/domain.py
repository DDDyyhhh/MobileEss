"""Single-use reviewed tasks, browser ownership and stop confirmation."""
from __future__ import annotations

import asyncio
import hashlib
import json
import math
import secrets
import time

from .geometry import angle, distance_to_path, finite_pose, length, split_path


class Refused(ValueError):
    pass


class Console:
    def __init__(self, adapter, config, clock=time.monotonic):
        self.adapter, self.config, self.clock = adapter, config, clock
        self.owner = None
        self.heartbeat = -math.inf
        self.plan = None
        self.task = None
        self.job = None
        self.lock = asyncio.Lock()
        self.phase = 'LOCALIZING'
        self.reason = ''
        self.boot_id = secrets.token_hex(16)
        self.history = []
        self.stop_requested = False
        self.abort_reason = None

    def claim(self, session):
        if self.owner not in (None, session) and self.clock()-self.heartbeat <= 1.:
            raise Refused('已有其他控制席；本页面只读')
        if self.task and self.task['state'] in ('RUNNING', 'STOPPING') and self.owner != session:
            raise Refused('上个任务尚未确认停车')
        self.owner = session
        self.heartbeat = self.clock()

    def beat(self, session):
        if self.owner != session:
            raise Refused('当前页面没有控制权')
        self.heartbeat = self.clock()

    def require_owner(self, session):
        if session != self.owner or self.clock()-self.heartbeat > 1.:
            raise Refused('控制连接已失效，请重新取得控制权')

    def state(self):
        snapshot = self.adapter.snapshot()
        if self.plan and not self.plan['consumed'] and self.clock() > self.plan['expires_at']:
            self.plan = None
            self.reason = '预览已过期，请重新规划'
        active = self.task and self.task['state'] in ('RUNNING', 'STOPPING')
        phase = self.phase if active or self.phase == 'PLANNING' or self.task else (
            'PLANNED' if self.plan else ('READY' if snapshot.get('localized') else 'LOCALIZING'))
        return dict(snapshot, phase=phase, reason=self.reason, task=self.task,
                    plan=self.plan, boot_id=self.boot_id,
                    motion_enabled=bool(self.config.get('motion_enabled', False)),
                    capabilities=self.config.get('accepted_capabilities', {}))

    def ensure_idle(self):
        if self.job and not self.job.done():
            raise Refused('任务或停车确认仍在进行')

    async def preview(self, session, goal):
        async with self.lock:
            self.require_owner(session)
            self.ensure_idle()
            if not isinstance(goal, list) or len(goal) != 2 or not all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in goal):
                raise Refused('目标必须是两个有限地图坐标')
            snapshot = self.adapter.snapshot()
            if not snapshot.get('localized') or not snapshot.get('stationary'):
                raise Refused('等待可靠定位和新鲜停稳反馈')
            self.task = None
            self.reason = ''
            self.plan = None
            self.phase = 'PLANNING'
            try:
                poses = await asyncio.wait_for(self.adapter.plan(goal), 12.)
                segments = split_path(poses)
                await self.adapter.validate(segments)
                after = self.adapter.snapshot()
                self.check_context(snapshot, after)
                identity = {'poses': poses, 'map_revision': snapshot['map_revision'],
                            'generation': snapshot['generation'], 'config': self.config,
                            'boot_id': self.boot_id}
                digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
                self.plan = dict(id=secrets.token_hex(16), digest=digest,
                                 created_at=self.clock(), expires_at=self.clock()+120.,
                                 start=snapshot['pose'], goal=goal, segments=segments,
                                 length_m=length(poses), map_revision=snapshot['map_revision'],
                                 generation=snapshot['generation'], consumed=False)
                self.reason = ''
                return self.plan
            finally:
                self.phase = 'READY'

    @staticmethod
    def check_context(before, after):
        if (not after.get('localized') or before['map_revision'] != after['map_revision'] or
                before['generation'] != after['generation'] or not after.get('stationary')):
            raise Refused('地图、定位或停车状态已变化，请重新预览')
        if (not finite_pose(after['pose']) or math.dist(before['pose'][:2], after['pose'][:2]) > .1 or
                abs(angle(before['pose'][2]-after['pose'][2])) > math.radians(5)):
            raise Refused('起始位姿已变化，请重新预览')

    async def start(self, session, plan_id, confirmations):
        async with self.lock:
            self.require_owner(session)
            # Retried HTTP requests are idempotent, including after completion.
            if self.task and self.task['plan_id'] == plan_id:
                return self.task
            self.ensure_idle()
            plan = self.plan
            if not plan or plan['id'] != plan_id or plan['consumed'] or self.clock() > plan['expires_at']:
                raise Refused('预览已过期或已使用，请重新规划')
            if not all(confirmations.get(k) is True for k in ('route_clear', 'stop_ready', 'actuators_isolated')):
                raise Refused('需要本轮全程净空、CH5及独立停止准备、机构隔离确认')
            if not self.config.get('motion_enabled', False):
                raise Refused('当前为观测模式：运动链尚待验收启用')
            caps = self.config.get('accepted_capabilities', {})
            for segment in plan['segments']:
                if not caps.get(segment['kind'], False):
                    raise Refused('该路线包含尚未验收的运动能力：'+segment['kind'])
            snapshot = self.adapter.snapshot()
            self.check_context(dict(map_revision=plan['map_revision'], generation=plan['generation'],
                                    pose=plan['start']), snapshot)
            await self.adapter.validate(plan['segments'])
            self.require_owner(session)
            self.check_context(dict(map_revision=plan['map_revision'], generation=plan['generation'],
                                    pose=plan['start']), self.adapter.snapshot())
            task = dict(id=secrets.token_hex(16), plan_id=plan_id, state='RUNNING',
                        segment=0, started_at=self.clock(), stop_confirmed=False)
            if hasattr(self.adapter, 'audit_start'):
                try:
                    self.adapter.audit_start(task, plan, confirmations)
                except OSError as exc:
                    raise Refused('无法记录运行证据，未启动任务：'+str(exc)) from exc
            plan['consumed'] = True
            self.task = task
            self.phase = 'RUNNING'
            self.stop_requested, self.abort_reason = False, None
            self.job = asyncio.create_task(self.execute(plan))
            return self.task

    async def execute(self, plan):
        outcome, reason = 'SUCCEEDED', '已到达目标并确认停稳'
        async def watch_connection():
            try:
                while True:
                    self.require_owner(self.owner)
                    if self.clock()-self.task['started_at'] > 600:
                        raise Refused('任务超过600秒')
                    await asyncio.sleep(.05)
            except Refused as exc:
                self.request_stop(str(exc), blocked=True)
        watcher = asyncio.create_task(watch_connection())
        try:
            await self.adapter.arm()
            travelled = reverse = 0.
            previous = self.adapter.snapshot()['pose']
            for index, segment in enumerate(plan['segments']):
                self.task['segment'] = index
                running = asyncio.create_task(self.adapter.execute_segment(segment))
                try:
                    while not running.done():
                        self.require_owner(self.owner)
                        snapshot = self.adapter.snapshot()
                        if self.clock()-self.task['started_at'] > 600:
                            raise Refused('任务超过600秒')
                        if not snapshot.get('healthy'):
                            raise Refused(snapshot.get('fault', '定位、感知或CAN反馈失效'))
                        if snapshot['generation'] != plan['generation'] or snapshot['map_revision'] != plan['map_revision']:
                            raise Refused('定位代次或地图变化')
                        pose = snapshot['pose']
                        step = math.dist(previous[:2], pose[:2])
                        if step > .5:
                            raise Refused('定位跳变')
                        travelled += step
                        if snapshot.get('actual_v', 0.) < -.01:
                            reverse += step
                        elif snapshot.get('actual_v', 0.) > .01:
                            reverse = 0.
                        if reverse > 1. or travelled > 42.:
                            raise Refused('实际倒车或行程超限')
                        if distance_to_path(segment['poses'], pose[:2]) > .2:
                            raise Refused('偏离预览路线超过20厘米')
                        previous = pose
                        self.adapter.permit(True)
                        await asyncio.sleep(.05)
                    await running
                    await self.adapter.wait_stationary(require_api_zero=False)
                finally:
                    if not running.done():
                        running.cancel()
                        await asyncio.gather(running, return_exceptions=True)
            if math.dist(self.adapter.snapshot()['pose'][:2], plan['goal']) > .25:
                raise Refused('未进入目标25厘米范围')
        except asyncio.CancelledError:
            outcome = 'BLOCKED' if self.abort_reason else 'CANCELED'
            reason = self.abort_reason or self.reason or '人工停止'
        except Exception as exc:
            outcome, reason = 'BLOCKED', str(exc)
        finally:
            watcher.cancel()
            await asyncio.gather(watcher, return_exceptions=True)
            self.phase = self.task['state'] = 'STOPPING'
            self.adapter.permit(False)
            try:
                await asyncio.wait_for(self.adapter.stop(), 10.)
                self.task['stop_confirmed'] = True
            except Exception as exc:
                outcome, reason = 'FAULT', '停车未确认：'+str(exc)
            self.task.update(state=outcome, reason=reason, ended_at=self.clock())
            self.phase, self.reason, self.plan = outcome, reason, None
            self.history.append(dict(self.task))
            if hasattr(self.adapter, 'audit_end'):
                try:
                    self.adapter.audit_end(self.task)
                except OSError as exc:
                    self.task.update(state='FAULT',reason='停车已确认，但写入运行证据失败：'+str(exc))
                    self.phase,self.reason='FAULT',self.task['reason']

    def request_stop(self, reason, blocked=False):
        self.adapter.permit(False)
        if self.job and not self.job.done() and not self.stop_requested:
            self.stop_requested = True
            self.reason = reason
            self.abort_reason = reason if blocked else None
            self.job.cancel()

    async def stop(self):
        self.request_stop('人工停止')
        if self.job and not self.job.done():
            await asyncio.shield(self.job)
        return self.task

    async def relocalize(self, session, seed=None):
        async with self.lock:
            self.require_owner(session)
            self.ensure_idle()
            if seed is not None and not finite_pose(seed):
                raise Refused('粗略位姿必须包含有限位置和朝向')
            if not self.adapter.snapshot().get('stationary'):
                raise Refused('尚未确认静止')
            self.plan = None
            self.task = None
            await self.adapter.relocalize(seed)
            self.phase = 'LOCALIZING'
