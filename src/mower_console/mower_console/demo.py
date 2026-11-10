"""Explicit offline display adapter. It can NEVER arm or move a vehicle."""
import math


class DemoAdapter:
    def __init__(self, assets):
        self.assets = assets

    def snapshot(self):
        return dict(demo=True, localized=True, stationary=True, healthy=False,
                    fault='离线演示，没有连接车辆', pose=[1.48, .67, 0.],
                    generation=0, map_revision=self.assets.revision, actual_v=0.,
                    wheel_mm_s=[0, 0, 0, 0], api_mode=None, localization='离线示例位姿',
                    trajectory=[], obstacles=[], cloud=[], command_echo=None)

    async def plan(self, goal):
        start = self.snapshot()['pose']
        yaw = math.atan2(goal[1]-start[1], goal[0]-start[0])
        poses = [start, [start[0], start[1], yaw]]
        n = max(1, math.ceil(math.dist(start[:2], goal)/.05))
        poses += [[start[0]+(goal[0]-start[0])*i/n,
                   start[1]+(goal[1]-start[1])*i/n, yaw] for i in range(1, n+1)]
        return poses

    async def validate(self, segments):
        self.assets.grid.validate(segments)

    def permit(self, value):
        if value:
            raise RuntimeError('离线演示禁止运动')

    async def arm(self):
        raise RuntimeError('离线演示禁止运动')

    async def relocalize(self, seed):
        raise ValueError('离线演示不能定位车辆')

    async def stop(self):
        pass

    def close(self):
        pass
