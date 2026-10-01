import pytest

from usv_pwm_actuator.pwm_protocol import percent_to_compensated_pulse_us


def pulse(percent, limit=100.0, forward=1560, reverse=1440):
    return percent_to_compensated_pulse_us(percent, limit, forward, reverse)


def test_neutral_is_never_offset():
    assert pulse(0.0) == 1500


def test_forward_start_and_full_scale():
    assert pulse(0.001) == pytest.approx(1560, abs=1)
    assert pulse(100.0) == 2000


def test_reverse_start_and_full_scale():
    assert pulse(-0.001) == pytest.approx(1440, abs=1)
    assert pulse(-100.0) == 1000


def test_compensation_respects_global_limit():
    assert pulse(50.0, limit=50.0) == 1750
    assert pulse(-50.0, limit=50.0) == 1250
    assert pulse(1.0, limit=10.0) <= 1550
    assert pulse(-1.0, limit=10.0) >= 1450


def test_1500_start_preserves_original_linear_mapping():
    assert pulse(25.0, forward=1500, reverse=1500) == 1625
    assert pulse(-25.0, forward=1500, reverse=1500) == 1375
