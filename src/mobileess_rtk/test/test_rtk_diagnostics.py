from pathlib import Path


NODE = Path(__file__).parents[1] / 'mobileess_rtk' / 'rtk_node.py'


def test_ntrip_session_duration_is_defined_in_diagnostics_callback():
    text = NODE.read_text()
    diagnostics = text.split('    def _publish_diagnostics(self) -> None:', 1)[1]
    assert 'ntrip_session_sec = (' in diagnostics
    assert "'ntrip_session_sec': f'{ntrip_session_sec:.3f}'" in diagnostics


def test_ntrip_correction_stream_timeout_is_observable_and_reconnectable():
    text = NODE.read_text()
    assert "self.declare_parameter('ntrip_rtcm_timeout_sec', 3.0)" in text
    assert "raise ConnectionError('NTRIP correction stream timed out')" in text
    assert 'self._last_rtcm_time = time.monotonic()' in text
    assert "'ntrip_rtcm_age_sec': ntrip_rtcm_age_sec" in text
