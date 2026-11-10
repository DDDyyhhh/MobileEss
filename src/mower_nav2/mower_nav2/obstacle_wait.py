"""Bound one ordinary-obstacle pause within an already reviewed straight task.

This policy never unlocks hardware, recovers a failed action, or changes a goal.
Callers must cancel and finish the active controller action before waiting, keep
zero commands alive, and separately enforce all localization/CAN/route guards.
"""
from dataclasses import dataclass
import math

from .reviewed_route import project


def goal_uuid(handle):
    """ROS Humble may expose uint8 numpy scalars; log portable JSON integers."""
    return [int(value) for value in handle.goal_id.uuid]


def finish_controller_pause(handle, finished, await_result):
    """Require both matching cancellation acknowledgement and terminal cancel.

    An aborted/succeeded follower is never interpreted as an ordinary pause.
    await_result must continue pumping zero commands and the safety guards.
    """
    response = await_result(handle.cancel_goal_async(), 3)
    matching = [list(item.goal_id.uuid) for item in response.goals_canceling]
    if goal_uuid(handle) not in matching:
        raise RuntimeError('Obstacle pause cancellation not acknowledged')
    if await_result(finished, 3).status != 5:
        raise RuntimeError('Obstacle pause requires confirmed cancelled follower')


@dataclass
class ObstacleWait:
    state: str = 'following'
    wait_started: float | None = None
    clear_since: float | None = None
    stationary_since: float | None = None
    previous_time: float | None = None
    resume_count: int = 0
    error: str = ''

    def update(self, now, *, blocked, stationary, healthy=True):
        """Return following/waiting/resume/fault; faults remain terminal.

        Fixed field scope: one pause, at most 30 s, at least 5 s observation,
        continuously clear for 2 s and wheels stationary for 1 s. A callback
        gap >300 ms or unhealthy evidence terminates the task rather than
        turning a sensor outage into a clear-space observation.
        """
        if self.state == 'fault':
            return self.state
        if (not math.isfinite(now) or not healthy or
                (self.previous_time is not None and
                 not 0 <= now - self.previous_time <= .3)):
            return self.fail('Wait evidence unhealthy or event loop stalled')
        self.previous_time = now
        # Remain pending until the task acknowledges the resume. A transient
        # event must not be lost while an asynchronous cancel result is pumped.
        if self.state == 'resume':
            if now - self.wait_started >= 30.:
                return self.fail('Obstacle wait expired')
            if blocked or not stationary:
                return self.fail('Clearance changed before resume acknowledgement')
            return self.state
        if self.state == 'following':
            if not blocked:
                return self.state
            if self.resume_count:
                return self.fail('Second obstacle episode outside single-pause scope')
            self.state = 'waiting'
            self.wait_started = now
        if now - self.wait_started >= 30.:
            return self.fail('Obstacle wait expired')
        self.clear_since = (self.clear_since if self.clear_since is not None else now) if not blocked else None
        self.stationary_since = (self.stationary_since if self.stationary_since is not None else now) if stationary else None
        if (self.clear_since is not None and self.stationary_since is not None and
                now - self.wait_started >= 5. and
                now - self.clear_since >= 2. and
                now - self.stationary_since >= 1.):
            self.state = 'resume'
            self.resume_count += 1
        return self.state

    def fail(self, error):
        self.error = error
        self.state = 'fault'
        return self.state

    def acknowledge_resume(self):
        if self.state != 'resume':
            raise ValueError('No pending obstacle resume')
        self.state = 'following'


def remaining_straight_path(path, pose):
    """Trim only passed samples; retain the exact original goal and heading."""
    distance, progress = project(path, pose[:2])
    if distance > .2:
        raise ValueError('Resume pose outside reviewed corridor')
    heading = path[0][2]
    if abs(math.atan2(math.sin(pose[2]-heading), math.cos(pose[2]-heading))) > .15:
        raise ValueError('Resume heading outside straight scope')
    offset = 0.
    remaining = [list(pose)]
    for a, b in zip(path, path[1:]):
        offset += math.dist(a[:2], b[:2])
        if offset > progress + .02:
            remaining.append(list(b))
    if len(remaining) < 2 or math.dist(remaining[0][:2], path[-1][:2]) < .3:
        raise ValueError('Insufficient remaining route for resume validation')
    return remaining
