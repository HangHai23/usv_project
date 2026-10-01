import math

import pytest

from usv_ball_follow.control_math import (
    desired_yaw_rate,
    mix_same_direction_or_pivot,
    pixel_center_to_bearing,
    scheduled_throttle,
    slew,
)


def test_pixel_center_to_bearing_center_and_edges():
    fov = math.radians(70.0)
    assert pixel_center_to_bearing(0.5, fov) == pytest.approx(0.0)
    assert pixel_center_to_bearing(0.0, fov) == pytest.approx(-fov / 2.0)
    assert pixel_center_to_bearing(1.0, fov) == pytest.approx(fov / 2.0)


def test_yaw_rate_deadband_and_limit():
    assert desired_yaw_rate(0.02, 0.03, 2.0, 0.7) == 0.0
    assert desired_yaw_rate(1.0, 0.03, 2.0, 0.7) == 0.7
    assert desired_yaw_rate(-1.0, 0.03, 2.0, 0.7) == -0.7


def test_forward_mix_never_reverses_inner_motor():
    left, right = mix_same_direction_or_pivot(10.0, 25.0)
    assert left == 35.0
    assert right == 0.0


def test_zero_throttle_allows_pivot():
    assert mix_same_direction_or_pivot(0.0, 12.0) == (12.0, -12.0)


def test_throttle_reduces_with_bearing_and_stops_near_target():
    assert scheduled_throttle(20.0, 0.0, 0.4, 0.01, 0.2) == 20.0
    assert scheduled_throttle(20.0, 0.4, 0.4, 0.01, 0.2) == 0.0
    assert scheduled_throttle(20.0, 0.0, 0.4, 0.2, 0.2) == 0.0


def test_slew_limit():
    assert slew(0.0, 20.0, 3.0) == 3.0
    assert slew(10.0, 0.0, 3.0) == 7.0
