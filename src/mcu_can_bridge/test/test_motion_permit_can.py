"""Exercise actual encoded bridge output with an in-memory socket only."""
import struct
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Twist
from mcu_can_bridge.cmd_vel_can_bridge import CmdVelCanBridge
from mcu_can_bridge.motion_permit import MotionPermit


def bridge(monkeypatch):
    now=[10.]
    node=object.__new__(CmdVelCanBridge)
    node._lock=threading.Lock();node._remote_takeover=False
    node._last_status=node._last_command=node._last_localization_acceptance=10.
    node._status_timeout=node._timeout=node._localization_timeout=.5
    node._api_state=2;node._api_was_running=True;node._allow_reverse=True
    node._max_linear=.3;node._max_reverse=.2;node._max_angular=.2
    node._require_permit=True;node._permit=MotionPermit(.25);node._last_error=''
    node._command=Twist();frames=[];node._socket=SimpleNamespace(send=frames.append)
    monkeypatch.setattr('mcu_can_bridge.cmd_vel_can_bridge.time.monotonic',lambda:now[0])
    return node,frames,now


def test_unarmed_console_bridge_emits_no_can(monkeypatch):
    node,frames,_=bridge(monkeypatch);node._send_control_frame()
    assert not frames


def test_reverse_cap_and_ordinary_payload_without_unlock(monkeypatch):
    node,frames,_=bridge(monkeypatch);node._permit.update(True,10.)
    node._command.linear.x=-.3;node._command.angular.z=.3;node._send_control_frame()
    can_id,dlc,data=struct.unpack('=IB3x8s',frames[-1])
    assert can_id==5 and dlc==8 and data[0]==2
    assert int.from_bytes(data[1:2],'big',signed=True)==-2
    assert data[3]==2 and data[4]==0 and data[7]==0


def test_expired_lease_exits_api_and_late_true_cannot_restart(monkeypatch):
    node,frames,now=bridge(monkeypatch);node._permit.update(True,10.)
    now[0]=10.26;node._send_control_frame()
    assert struct.unpack('=IB3x8s',frames[-1])[2][0:5]==bytes(5)
    node._permit.update(True,10.27);now[0]=10.27;node._send_control_frame()
    assert struct.unpack('=IB3x8s',frames[-1])[2][0]==0
    node._api_state=0;count=len(frames);node._send_control_frame()
    assert len(frames)==count
