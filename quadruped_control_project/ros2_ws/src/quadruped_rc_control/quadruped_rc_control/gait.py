"""Single-axis leg cycle: slow support stroke, fast recovery stroke."""
import math


def throttle_fraction(pulse, deadband=20):
    delta = max(-500, min(500, pulse - 1500))
    if abs(delta) <= deadband:
        return 0.0
    return math.copysign((abs(delta) - deadband) / (500 - deadband), delta)


def leg_angles(phase, centers, amplitudes, directions, offsets, support_fraction):
    result = []
    for center, amplitude, direction, offset in zip(centers, amplitudes, directions, offsets):
        p = (phase + offset) % 1.0
        if p < support_fraction:
            stroke = 1.0 - 2.0 * p / support_fraction
        else:
            stroke = -1.0 + 2.0 * (p - support_fraction) / (1.0 - support_fraction)
        result.append(round(center + direction * amplitude * stroke))
    return result
