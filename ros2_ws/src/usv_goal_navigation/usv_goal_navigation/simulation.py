"""Offline motion/delay/dropout experiment. Never publishes ROS commands."""
import argparse
import csv
import json
import math
from pathlib import Path
from datetime import datetime
from .estimation import DelayedEstimator, propagate
from .control import route_command, TurnPhase
from .navigation_node import DEFAULTS


def simulate(side='left', seconds=120):
    c = dict(DEFAULTS)
    state = [16., 5.5, math.pi/2, 0.]
    start = state[:2]
    target = [0., 5.5] if side == 'left' else [20., 5.5]
    e = DelayedEstimator(c['speed_time_constant_sec'])
    e.initialize(state[:3], 0, 0, 0, 0)
    # Synthetic plant deliberately differs from the controller's provisional gains.
    true_speed_gain, true_turn_gain = .0033, .0036
    yaw = 0.
    rows, history = [], [state.copy()]
    dt = .05
    outputs = [0., 0.]
    phase = TurnPhase()
    for i in range(1, int(seconds/dt)):
        t = i*dt
        e.advance(t, yaw, sum(outputs)/2*c['command_span_us']/100*c['speed_gain_m_s_per_us'])
        history.append(state.copy())
        # 10Hz delayed vision, plus 4 seconds of complete visual dropout.
        visual = i >= 10 and i % 2 == 0 and not 12 <= t < 16
        if visual:
            measured = history[i-10]
            e.correct(measured[:3], t-.5, .65, .5, measured[3], 4.)
        left, right, detail = route_command(e.state, start, target, yaw, c, phase)
        if detail['distance_m'] <= c['arrival_radius_m']:
            break
        delta = c['output_slew_rate_percent_s']*dt
        outputs = [p+max(-delta, min(delta, q-p)) for p,q in zip(outputs, [left,right])]
        left_us, right_us = [p*c['command_span_us']/100 for p in outputs]
        yaw_target = (right_us-left_us)*true_turn_gain
        yaw += (yaw_target-yaw)*(1-math.exp(-dt/c['yaw_time_constant_sec']))
        state = propagate(state, dt, yaw, (left_us+right_us)/2*true_speed_gain, c['speed_time_constant_sec'])
        rows.append(dict(t=t, true_x=state[0], true_y=state[1], true_heading=state[2],
            estimated_x=e.state[0], estimated_y=e.state[1], estimated_heading=e.state[2],
            visual_received=visual, left_percent=outputs[0], right_percent=outputs[1],
            yaw_rate=yaw, cross_track_m=detail['cross_track_m'], distance_m=detail['distance_m']))
    return rows, dict(simulated=True, side=side, seconds=t, arrival=detail['distance_m'] <= c['arrival_radius_m'],
        final_distance_m=detail['distance_m'], true_speed_gain=true_speed_gain, true_turn_gain=true_turn_gain,
        visual_dropout_sec=[12,16], configured_delay_sec=.5,
        note='Synthetic demonstration only; not a validation of real boat dynamics.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left','right'], default='left')
    parser.add_argument('--output', default='~/Downloads/usv_navigation_logs/simulation')
    args = parser.parse_args()
    rows, result = simulate(args.side)
    directory = Path(args.output).expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    with (directory/'simulation.csv').open('w') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (directory/'simulation_summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(directory=str(directory), **result), indent=2))


if __name__ == '__main__':
    main()
