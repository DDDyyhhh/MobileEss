from types import SimpleNamespace

from mower_nav2.collision_gate import CollisionGate


def make_gate(obstacle=None):
    gate = object.__new__(CollisionGate)
    gate._radius = 0.45
    gate._horizon = 2.0
    gate._dt = 0.05
    gate._collision_cost = 253
    metadata = SimpleNamespace(
        origin=SimpleNamespace(position=SimpleNamespace(x=-2.5, y=-2.5)),
        resolution=0.05,
        size_x=100,
        size_y=100)
    data = [0] * (metadata.size_x * metadata.size_y)
    if obstacle is not None:
        x, y, cost = obstacle
        mx = int((x - metadata.origin.position.x) / metadata.resolution)
        my = int((y - metadata.origin.position.y) / metadata.resolution)
        data[my * metadata.size_x + mx] = cost
    gate._costmap = SimpleNamespace(metadata=metadata, data=data)
    pose = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    gate._odom = SimpleNamespace(pose=SimpleNamespace(pose=pose))
    gate._last_reason = ''
    return gate


def forward_command():
    return SimpleNamespace(linear=SimpleNamespace(x=0.20),
                           angular=SimpleNamespace(z=0.0))


def test_gate_rejects_inscribed_cost_inside_predicted_footprint():
    gate = make_gate(obstacle=(0.50, 0.0, 253))
    assert gate._collision(forward_command()) is True
    assert 'cost 253' in gate._last_reason


def test_gate_allows_a_free_predicted_footprint():
    gate = make_gate()
    assert gate._collision(forward_command()) is False
    assert gate._last_reason == 'collision-free prediction'


def test_gate_rejects_trajectory_leaving_local_costmap():
    gate = make_gate()
    gate._odom.pose.pose.position.x = 2.40
    assert gate._collision(forward_command()) is True
    assert gate._last_reason == 'predicted footprint leaves local costmap'
