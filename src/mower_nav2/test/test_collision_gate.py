from types import SimpleNamespace

from mower_nav2.collision_gate import CollisionGate


def make_gate(obstacle=None):
    gate = object.__new__(CollisionGate)
    gate._radius = 0.45
    gate._horizon = 2.0
    gate._dt = 0.05
    gate._collision_cost = 253
    gate._filled = False
    gate._slip = 0.0
    gate._require_rear = False
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


def test_filled_profile_rejects_obstacle_between_old_rings():
    gate = make_gate(obstacle=(.10, .10, 254))
    gate._filled = True
    assert gate._collision(forward_command()) is True


def test_reverse_prediction_checks_rear_obstacle():
    gate = make_gate(obstacle=(-.65, .0, 254))
    gate._filled = True
    cmd = forward_command(); cmd.linear.x = -.20
    assert gate._collision(cmd) is True


def test_reverse_requires_recent_positive_rear_observation():
    gate = make_gate()
    gate._require_rear = True
    gate._rear_observed = False; gate._rear_time = 0.
    cmd = forward_command(); cmd.linear.x = -.20
    assert gate._collision(cmd) is True
    assert gate._last_reason.startswith('rear')


def test_rotation_checks_extra_slip_envelope():
    gate = make_gate(obstacle=(.6, .0, 254))
    gate._filled = True; gate._slip = .25
    cmd = forward_command(); cmd.linear.x = 0.; cmd.angular.z = .2
    assert gate._collision(cmd) is True


def test_fresh_costmap_cannot_hide_missing_obstacle_cloud(monkeypatch):
    from geometry_msgs.msg import Twist
    gate = make_gate()
    gate._command = Twist()
    gate._command.linear.x = 0.2
    gate._command_timeout = gate._costmap_timeout = gate._odom_timeout = 0.3
    gate._obstacle_timeout = 0.3
    gate._command_time = gate._costmap_time = gate._odom_time = 10.0
    gate._obstacle_time = 9.0
    gate._pub = SimpleNamespace(publish=lambda msg: published.append(msg))
    gate._diagnostic_pub = SimpleNamespace(publish=lambda msg: None)
    published = []
    monkeypatch.setattr('mower_nav2.collision_gate.time.monotonic', lambda: 10.0)
    # Diagnostics require a genuine ROS Time.
    from builtin_interfaces.msg import Time
    gate.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    gate._tick()
    assert published[-1].linear.x == 0.0
    assert 'obstacle cloud' in gate._last_reason
    gate._obstacle_time = 10.0
    gate._tick()
    assert published[-1].linear.x == 0.2
