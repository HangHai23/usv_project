import json
import math
import pytest
from usv_goal_navigation.estimation import MapMemory, DelayedEstimator, propagate
from usv_goal_navigation.control import route_command
from usv_goal_navigation.analysis import estimate_delay
import numpy as np


def test_map_remembers_and_does_not_mix_coordinate_frames():
    m = MapMemory()
    obs = dict(coord='field1', field_valid=True, size_m=[20, 11],
               goals=[dict(side='left', ends_m=[[0, 4], [0, 7]])])
    assert m.ingest(obs)
    assert not m.ingest(dict(obs, goals=[], size_m=None))
    assert m.goals['left'] == [[0., 4.], [0., 7.]]
    assert m.snapshot()['boundary'][-1] == [0, 11.]
    with pytest.raises(ValueError):
        m.ingest(dict(obs, coord='field2'))


def test_replay_corrects_history_instead_of_treating_old_pose_as_current():
    e = DelayedEstimator(tau=1.)
    e.initialize([0, 0, 0], 0, 0, 0, 0)
    e.state[3] = 1.
    e.advance(1, 0, 1)
    result = e.correct([0.6, 0, 0], 0.5, 1., 1., max_innovation=2)
    assert result['accepted']
    assert e.state[0] == pytest.approx(1.1, abs=1e-6)
    before = e.state.copy()
    assert not e.correct([100, 0, 0], .8, 1, 1)['accepted']
    assert e.state == before


def test_positive_yaw_uses_right_motor_and_inertia_persists():
    from usv_goal_navigation.navigation_node import DEFAULTS
    left, right, _ = route_command([0,0,0,0], [0,0], [0,5], 0, DEFAULTS)
    assert 0 <= left < right <= DEFAULTS['maximum_command_percent']
    coast = propagate([0,0,0,1], 1, 0, 0, 1.5)
    assert coast[0] > 0 and 0 < coast[3] < 1


def test_straight_route_positive_yaw_is_braked_not_reinforced():
    from usv_goal_navigation.navigation_node import DEFAULTS
    assert DEFAULTS['imu_yaw_rate_reversed'] is False
    left,right,_ = route_command([0,0,0,.35],[0,0],[10,0],.25,DEFAULTS)
    assert left > right
    left,right,_ = route_command([0,0,0,.35],[0,0],[10,0],-.25,DEFAULTS)
    assert right > left


def test_turn_phase_waits_for_angular_speed_and_brakes_before_alignment():
    from usv_goal_navigation.control import TurnPhase
    from usv_goal_navigation.navigation_node import DEFAULTS
    phase=TurnPhase()
    assert phase.update(1.5,.3,DEFAULTS)
    assert phase.update(.1,.25,DEFAULTS)
    assert not phase.update(.1,.05,DEFAULTS)
    phase.update(1.5,.3,DEFAULTS)
    left,right,data=route_command([0,0,-.1,.1],[0,0],[10,0],.35,DEFAULTS,phase)
    assert data['motion_phase']=='aligning'
    assert data['desired_speed_m_s']==DEFAULTS['turn_speed_cap_m_s']
    assert data['desired_yaw_rate_rad_s']<0.
    assert left>right
    phase.reset()
    assert not phase.aligning


def test_delay_fit_recovers_injected_latency():
    t = np.arange(0, 30, .01)
    rate = .3*np.sin(t*1.1) + .1*np.sin(2.3*t)
    integral = np.r_[0., np.cumsum(np.diff(t)*(rate[1:]+rate[:-1])*.5)]
    vt = np.arange(3, 29, .1)
    heading = np.interp(vt-.64, t, integral)
    result = estimate_delay(t, rate, vt, heading)
    assert result['usable']
    assert result['estimated_total_delay_sec'] == pytest.approx(.64, abs=.021)
    assert not estimate_delay(t, rate*0, vt, heading)['usable']


def test_dynamic_gain_fit_and_log_export(tmp_path):
    from usv_goal_navigation.analysis import fit_gain, analyze
    from usv_goal_navigation.navigation_node import DEFAULTS
    t = np.arange(0, 20, .05)
    u = 50 + 40*np.sin(t)
    y = [0.]
    for value in u[:-1]:
        a = math.exp(-.05/1.5)
        y.append(a*y[-1]+(1-a)*(.003*value+.02))
    result = fit_gain(t, u, np.array(y), 1.5)
    assert result['usable']
    assert result['gain'] == pytest.approx(.003)
    (tmp_path/'parameters.json').write_text(json.dumps(DEFAULTS))
    (tmp_path/'telemetry.jsonl').write_text('{broken partial line\n')
    report = analyze(tmp_path)
    assert report['samples']['damaged_lines'] == 1
    assert not report['delay']['usable']
    assert (tmp_path/'candidate_parameters.yaml').is_file()


def test_node_dropout_duplicate_timeout_manual_and_reset(tmp_path, monkeypatch):
    import rclpy
    import usv_goal_navigation.navigation_node as module
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import Imu
    from usv_interfaces.msg import PwmOutputState
    monkeypatch.setitem(module.DEFAULTS, 'log_directory', str(tmp_path/'logs'))
    monkeypatch.setitem(module.DEFAULTS, 'map_cache_file', str(tmp_path/'map.json'))
    clock = [100.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    rclpy.init(args=[])
    node = module.GoalNavigationNode()
    try:
        report = dict(valid=True, reset_required=False, session='a', received_monotonic=100.,
            observation=dict(frame=1, processed_unix_ms=100000, coord='field1', field_valid=True,
                size_m=[20,11], goals=[dict(side='left', ends_m=[[0,4],[0,7]])]),
            own_boat=dict(xy_m=[10,5], heading_rad=math.pi))
        node.on_imu(Imu())
        positive_gyro=Imu()
        positive_gyro.angular_velocity.z=.25
        node.on_imu(positive_gyro)
        assert node.rate == pytest.approx(.25)
        node.on_imu(Imu())
        # Real ROS fixed-size arrays carry numpy.uint16/float32, not Python scalars.
        pwm = PwmOutputState(pulse_width_us=[1190, 1100], applied_percent=[1.5, 2.5])
        node.on_pwm(pwm)
        node.on_observation(String(data=json.dumps(report)))
        node.on_mode(Bool(data=True))
        node.update()
        assert node.start is not None
        node.rec.file.flush()
        events = [json.loads(line) for line in (node.rec.directory/'telemetry.jsonl').read_text().splitlines()]
        tick = next(row for row in reversed(events) if row['event'] == 'tick')
        assert tick['pulse_width_us'] == [1190, 1100]
        assert tick['applied_percent'] == [1.5, 2.5]
        recorded_pwm = next(row['message'] for row in events if row['event'] == 'pwm')
        assert recorded_pwm['pulse_width_us'] == [1190, 1100]
        original = node.last_visual
        for _ in range(25):
            clock[0] += .2
            node.on_imu(Imu())
            node.on_mode(Bool(data=True))
            node.on_observation(String(data='invalid JSON'))
            node.on_observation(String(data=json.dumps(report)))
            node.update()
        assert node.last_visual == original
        assert max(node.outputs) > 0
        assert node.map.goals['left']
        clock[0] = 110.1
        node.on_imu(Imu())
        node.on_mode(Bool(data=True))
        node.update()
        assert node.outputs == [0., 0.]
        node.on_mode(Bool(data=False))
        node.update()
        assert node.start is None and not node.running
        node.on_observation(String(data=json.dumps(dict(report, reset_required=True))))
        assert node.fault == 'coordinate_or_session_reset'
    finally:
        node.rec.close()
        node.destroy_node()
        rclpy.shutdown()
