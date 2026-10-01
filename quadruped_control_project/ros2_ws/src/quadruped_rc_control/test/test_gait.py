import pytest
from quadruped_rc_control.gait import leg_angles, throttle_fraction


def test_throttle():
    assert throttle_fraction(1000) == -1
    assert throttle_fraction(2000) == 1
    assert all(throttle_fraction(v) == 0 for v in (1480, 1500, 1520))
    assert throttle_fraction(1750) == -throttle_fraction(1250)


def test_cycle_pairing_and_bounds():
    for step in range(101):
        angles = leg_angles(step/100, [87]*4, [20]*4, [1, 1, 1, 1], [0, .5, .5, 0], .7)
        assert angles[0] == angles[3]
        assert angles[1] == angles[2]
        assert all(67 <= a <= 107 for a in angles)


def test_forward_reverse_stroke_and_recovery():
    args = ([90]*4, [30]*4, [1]*4, [0]*4, .7)
    assert leg_angles(.1, *args)[0] > leg_angles(.2, *args)[0]
    assert leg_angles(-.1, *args)[0] > leg_angles(-.2, *args)[0]
    assert leg_angles(.9, *args)[0] > leg_angles(.8, *args)[0]
    assert leg_angles(0, *args) == leg_angles(1, *args)
