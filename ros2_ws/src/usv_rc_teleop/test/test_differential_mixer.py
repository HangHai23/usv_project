import pytest

from usv_rc_teleop.differential_mixer import mix_differential


@pytest.mark.parametrize(
    "throttle,steering,expected",
    [
        (100.0, 0.0, (100.0, 100.0)),
        (100.0, 50.0, (100.0, 50.0)),
        (100.0, -100.0, (0.0, 100.0)),
        (-100.0, 50.0, (-50.0, -100.0)),
        (-100.0, -100.0, (-100.0, 0.0)),
        (0.0, 50.0, (50.0, -50.0)),
        (0.0, -100.0, (-100.0, 100.0)),
    ],
)
def test_constrained_mix(throttle, steering, expected):
    assert mix_differential(throttle, steering) == pytest.approx(expected)


def test_translation_never_reverses_one_motor():
    for throttle in (-100.0, -25.0, 25.0, 100.0):
        for steering in range(-100, 101):
            left, right = mix_differential(throttle, steering)
            assert left * throttle >= 0.0
            assert right * throttle >= 0.0
