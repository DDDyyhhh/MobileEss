"""ROS message-level checks run on the board; no SocketCAN output."""
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from mcu_can_bridge.cmd_vel_can_bridge import CmdVelCanBridge, CAN_FRAME_STRUCT


@pytest.mark.parametrize('requested,expected', [(-0.2, 0), (0.7, 2)])
def test_field_profile_rejects_reverse_and_caps_forward_without_unlock(monkeypatch, requested, expected):
    node = object.__new__(CmdVelCanBridge)
    node._lock = threading.Lock()
    node._remote_takeover = False
    node._last_status = node._last_command = node._last_localization_acceptance = 10.0
    node._status_timeout = node._timeout = node._localization_timeout = 0.3
    node._api_state = 2
    node._api_was_running = True
    node._allow_reverse = False
    node._require_permit = False
    node._max_reverse = 1.0
    node._max_linear = 0.2
    node._max_angular = 0.3
    node._command = Twist()
    node._command.linear.x = requested
    output = []
    node._socket = SimpleNamespace(send=output.append)
    monkeypatch.setattr('mcu_can_bridge.cmd_vel_can_bridge.time.monotonic', lambda: 10.0)
    node._send_control_frame()
    can_id, dlc, data = CAN_FRAME_STRUCT.unpack(output[0])
    assert can_id == 0x005 and dlc == 8
    assert data[0] == 2 and data[1] == expected
    assert data[4] == 0 and data[7] == 0  # blade off; no latch-clear key
