import struct

from mcu_can_bridge.stationary_preflight import stationary_preflight


def trace():
    events=[]
    for i in range(61):
        t=i*.05
        events.extend([dict(t_s=t,can_id='0x507',payload='0000000000000000'),
                       dict(t_s=t,can_id='0x50a',payload='0000000000000000')])
    return events


def test_requires_a_full_continuous_stable_window():
    e=trace()
    assert stationary_preflight(e,3.)['ready']
    assert not stationary_preflight([x for x in e if x['t_s']>=2.1],3.)['ready']


def test_earlier_transient_can_settle_but_recent_transient_cannot_be_ignored():
    e=trace()
    e[21]['payload']=struct.pack('>hhhh',16,2,2,0).hex()
    assert stationary_preflight(e,3.)['ready']
    e[-9]['payload']=struct.pack('>hhhh',16,2,2,0).hex()
    assert not stationary_preflight(e,3.)['ready']


def test_does_not_allow_control_frames_or_api_takeover_to_age_out():
    e=trace();e[0]['can_id']='0x5'
    assert not stationary_preflight(e,3.)['ready']
    e=trace();e[0]['payload']='0000000002000000'
    assert not stationary_preflight(e,3.)['ready']


def test_stale_and_gapped_telemetry_cannot_be_a_stop():
    assert not stationary_preflight(trace(),3.3)['ready']
    for cid in ('0x507','0x50a'):
        e=[x for x in trace() if not (x['can_id']==cid and 2.2<x['t_s']<2.7)]
        assert not stationary_preflight(e,3.)['ready']


def test_rejects_invalid_payload_and_timestamps():
    for replacement in (dict(payload='00'),dict(t_s=float('nan')),dict(t_s=4.)):
        e=trace();e[-1].update(replacement)
        assert not stationary_preflight(e,3.)['ready']


def test_explicit_startup_tolerance_does_not_erase_continuity_or_ownership_checks():
    e=trace();e[-9]['payload']=struct.pack('>hhhh',16,2,2,0).hex()
    assert not stationary_preflight(e,3.)['ready']
    assert stationary_preflight(e,3.,50)['ready']
    e[-9]['payload']=struct.pack('>hhhh',51,2,2,0).hex()
    assert not stationary_preflight(e,3.,50)['ready']
    assert not stationary_preflight(trace(),3.3,50)['ready']
    assert not stationary_preflight(trace(),3.,51)['ready']
