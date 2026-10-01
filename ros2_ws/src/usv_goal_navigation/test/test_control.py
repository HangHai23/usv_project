import math

from usv_goal_navigation.control import (
    compute_forward_only_command,
    goal_center,
    predict_pose,
    wrap_angle,
)


def test_delay_prediction():
    x, y, heading = predict_pose(1.0, 2.0, 0.1, 2.0, -1.0, 0.4, 0.5)
    assert abs(x - 2.0) < 1e-9
    assert abs(y - 1.5) < 1e-9
    assert abs(heading - 0.3) < 1e-9


def test_goal_center_and_wrap():
    assert goal_center([[0.0, 4.0], [0.0, 8.0]]) == (0.0, 6.0)
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9


def test_forward_only_and_arrival():
    result = compute_forward_only_command(
        0, 0, 0, 10, 10, 0, 0.5, 12, 2, 0.6, 30, 20, 0.7
    )
    assert result[0] >= 0 and result[1] >= 0
    stopped = compute_forward_only_command(
        0, 0, 0, 0.1, 0.1, 0, 0.5, 12, 2, 0.6, 30, 20, 0.7
    )
    assert stopped[0] == stopped[1] == 0
