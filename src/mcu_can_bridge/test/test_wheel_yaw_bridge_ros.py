"""Run the real CAN receive handler with captured publishers; never opens CAN."""
import struct
import threading
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from mcu_can_bridge.mcu_can_bridge import McuCanBridge


def receiver(enabled=True):
 out=[]
 fake=SimpleNamespace(_can_id=0x50a,_track_width_m=.640,_yaw_model_enabled=enabled,
  _spin_scale=1.21,_moving_scale=.44,_lock=threading.Lock(),_malformed_frames=0,
  _dropped_frames=0,_valid_frames=0,_recovery_frames=1,_active=False,
  _raw_twist_pub='raw',_twist_pub='model',_last_estimate=None,
  _record_error=lambda err:out.append(('error',err)),
  _publish_twist=lambda v,w,pub:out.append((pub,v,w)))
 return fake,out


def frame(wheels):return struct.pack('=IB3x8s',0x50a,8,struct.pack('>hhhh',*wheels))


def test_real_receive_path_preserves_raw_yaw_and_publishes_fitted_yaw():
 fake,out=receiver();McuCanBridge._handle_frame(fake,frame((364,236,364,236)))
 assert out==[('raw',.3,pytest.approx(-.2)),('model',.3,pytest.approx(-.088))]
 assert fake._active and fake._last_estimate.regime=='right_forward_arc'


def test_disabled_fit_and_recovery_gate_preserve_existing_receive_contract():
 fake,out=receiver(False);fake._recovery_frames=3
 McuCanBridge._handle_frame(fake,frame((364,236,364,236)))
 McuCanBridge._handle_frame(fake,frame((364,236,364,236)));assert out==[]
 McuCanBridge._handle_frame(fake,frame((364,236,364,236)))
 assert out==[('raw',.3,pytest.approx(-.2)),('model',.3,pytest.approx(-.2))]


def test_malformed_receive_never_publishes_a_fabricated_zero():
 fake,out=receiver();McuCanBridge._handle_frame(fake,b'short')
 assert fake._malformed_frames==1 and not out
