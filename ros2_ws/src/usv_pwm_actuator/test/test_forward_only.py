from usv_pwm_actuator.pwm_actuator_node import PwmActuatorNode


def test_forward_only_mapping_and_stop():
    node=object.__new__(PwmActuatorNode)
    node._forward_only=True
    node._equal_pulse_increment=False
    node._output_limit_percent=100.
    node._unidirectional_start=(1000,1100)
    assert node._pulse_us(0,0)==1000
    assert node._pulse_us(0,-100)==1000
    assert node._pulse_us(0,50)==1500
    assert node._pulse_us(0,100)==2000
    assert node._pulse_us(1,0)==1000
    assert node._pulse_us(1,50)==1550
    node._output_limit_percent=50.
    assert node._pulse_us(1,50)==1500
    node._output_limit_percent=0.
    assert node._pulse_us(1,0)==1000


def test_different_start_thresholds_keep_equal_normalized_demand():
    node = object.__new__(PwmActuatorNode)
    node._forward_only = True
    node._equal_pulse_increment = True
    node._output_limit_percent = 65.0
    node._unidirectional_start = (1190, 1050)
    left_maximum = 1790.0
    right_maximum = 1650.0

    # A 10% stick demand becomes 6.5 applied percent after the 65% global
    # limit. Both channels must add the same number of microseconds above
    # their independently measured start thresholds.
    applied = 6.5
    pulses = [node._pulse_us(index, applied) for index in range(2)]
    fractions = [
        (pulses[index] - node._unidirectional_start[index])
        / ((left_maximum, right_maximum)[index] - node._unidirectional_start[index])
        for index in range(2)
    ]
    assert pulses == [1250, 1110]
    assert pulses[0] - 1190 == pulses[1] - 1050 == 60
    assert abs(fractions[0] - 0.10) < 0.002
    assert abs(fractions[1] - 0.10) < 0.002
    assert abs(fractions[0] - fractions[1]) < 0.002


def test_equal_increment_rejects_limit_that_exceeds_2000_us():
    node = object.__new__(PwmActuatorNode)
    node._forward_only = True
    node._equal_pulse_increment = True
    node._unidirectional_start = (1190, 1050)
    node._validate_unidirectional_limit(86.0)
    try:
        node._validate_unidirectional_limit(86.1)
    except ValueError:
        pass
    else:
        raise AssertionError("unsafe global limit was accepted")
