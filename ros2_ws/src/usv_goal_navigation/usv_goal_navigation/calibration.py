"""Read-only identification from recorded motion; writes a separate report."""
import argparse
import json
from pathlib import Path
import numpy as np


def identify(directory):
    directory = Path(directory)
    selected = {'imu', 'visual', 'pwm', 'tick', 'run_start', 'run_end'}
    rows = []
    with (directory / 'telemetry.jsonl').open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row['event'] in selected:
                rows.append(row)
    config = json.loads((directory/'parameters.json').read_text())
    imu = [r for r in rows if r['event']=='imu']
    visual = [r for r in rows if r['event']=='visual' and r['correction']['accepted']]
    pwm = [r for r in rows if r['event']=='pwm']
    ticks = [r for r in rows if r['event']=='tick' and r['state']=='navigating']
    runs = []
    for run in sorted(set(r['run_id'] for r in ticks)):
        samples = [r for r in ticks if r['run_id']==run]
        runs.append(dict(run_id=run, start=samples[0]['monotonic'], end=samples[-1]['monotonic'],
            seconds=samples[-1]['monotonic']-samples[0]['monotonic'],
            max_cross_track_m=max(abs(r['cross_track_m']) for r in samples),
            differential_saturated_fraction=float(np.mean([abs(r['desired_differential_us']) >= config['maximum_differential_us']-.01 for r in samples]))))
    longest = max(runs, key=lambda r:r['seconds'])
    if longest['seconds'] < 15:
        raise ValueError('Need at least one automatic run longer than 15 seconds')
    lo, hi = longest['start']+2, longest['end']-1
    it = np.array([r['monotonic'] for r in imu])
    raw = np.array([r['message']['angular_velocity']['z'] for r in imu])
    vt = np.array([r['received_monotonic'] for r in visual])
    heading = np.unwrap([r['heading'] for r in visual])
    xy = np.array([r['xy_m'] for r in visual])
    integral = np.r_[0, np.cumsum(np.diff(it)*(raw[1:]+raw[:-1])*.5)]
    alignment = []
    for window in (.3,.6,1.0):
        q = np.arange(lo, hi-window, .05)
        observed = np.interp(q+window, vt, heading)-np.interp(q, vt, heading)
        for sign in (1,-1):
            scores = []
            for delay in np.arange(0, 1.501, .01):
                predicted = sign*(np.interp(q+window-delay,it,integral)-np.interp(q-delay,it,integral))
                error = float(np.sqrt(np.mean((observed-predicted)**2)))
                scores.append((error, float(delay), float(np.corrcoef(observed,predicted)[0,1])))
            rmse, delay, corr = min(scores)
            alignment.append(dict(window_sec=window, imu_sign=sign, delay_sec=delay, rmse_rad=rmse, correlation=corr))
    best = min([r for r in alignment if r['window_sec']==.6], key=lambda r:r['rmse_rad'])
    sign, delay = best['imu_sign'], best['delay_sec']
    pt = np.array([r['monotonic'] for r in pwm])
    increments = np.array([[max(0,r['message']['pulse_width_us'][0]-config['left_start_us']),
                            max(0,r['message']['pulse_width_us'][1]-config['right_start_us'])]
                           if not (r['message']['dry_run'] or r['message']['watchdog_active'] or r['message']['emergency_stop_active'])
                           else [0,0] for r in pwm])
    q = np.arange(lo,hi,.1)
    measured_rate = np.interp(q,it,raw)*sign
    mean = np.interp(q,pt,increments.mean(axis=1))
    differential = np.interp(q,pt,(increments[:,1]-increments[:,0])*config['motor_turn_sign'])
    yaw_fits = []
    for tau in np.arange(.3,3.01,.1):
        a = np.exp(-.1/tau)
        matrix = np.column_stack([(1-a)*differential[:-1], (1-a)*mean[:-1], np.full(len(q)-1,1-a)])
        output = measured_rate[1:]-a*measured_rate[:-1]
        coef = np.linalg.lstsq(matrix,output,rcond=None)[0]
        residual = output-matrix@coef
        yaw_fits.append(dict(tau_sec=float(tau), gain=float(coef[0]),
            common_thrust_bias_gain=float(coef[1]), constant_bias_rad_s=float(coef[2]),
            one_step_rmse_rad_s=float(np.sqrt(np.mean(residual**2)))))
    turn = min(yaw_fits,key=lambda r:r['one_step_rmse_rad_s'])
    # Smooth measured position differences before fitting speed, instead of differentiating single noisy frames.
    width = .8
    dx = np.interp(q+delay+width/2,vt,xy[:,0])-np.interp(q+delay-width/2,vt,xy[:,0])
    dy = np.interp(q+delay+width/2,vt,xy[:,1])-np.interp(q+delay-width/2,vt,xy[:,1])
    h = np.interp(q+delay,vt,heading)
    speed = (dx*np.cos(h)+dy*np.sin(h))/width
    speed_fits = []
    for tau in np.arange(.5,4.01,.1):
        a = np.exp(-.1/tau)
        filtered = np.zeros(len(q));filtered[0]=mean[0]
        for i in range(1,len(q)):
            filtered[i]=a*filtered[i-1]+(1-a)*mean[i-1]
        matrix = np.column_stack([filtered,np.cos(h),np.sin(h)])
        coef = np.linalg.lstsq(matrix,speed,rcond=None)[0]
        residual = speed-matrix@coef
        speed_fits.append(dict(tau_sec=float(tau),gain=float(coef[0]),
            fitted_drift_xy_m_s=[float(v) for v in coef[1:]],rmse_m_s=float(np.sqrt(np.mean(residual**2)))))
    speed_fit = min(speed_fits,key=lambda r:r['rmse_m_s'])
    result = dict(log=str(directory), runs=runs, imu_alignment=alignment,
        best_alignment=best, turn_fit=turn, speed_fit=speed_fit,
        note='Model candidates from longest run; short second run is not independent gain validation. Drift and motor bias are nuisance terms, not automatically calibrated offsets.')
    (directory/'motion_identification.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory')
    args=parser.parse_args()
    print(json.dumps(identify(args.directory),indent=2))


if __name__=='__main__':
    main()
