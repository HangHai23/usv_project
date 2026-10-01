"""Counterfactual closed-loop model comparison, not real trajectory replay."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import yaml
from .control import route_command, TurnPhase
from .estimation import DelayedEstimator, propagate


def trial(config, fitted, factor=1., scenario=None):
    dt=.05
    start=[15.3423593,6.1381820]
    goal=[0.,5.514]
    plant=[*start,-.68,.2]
    if scenario is not None:
        start=list(scenario['start'])
        goal=list(scenario['goal'])
        plant=[*start,scenario['heading'],scenario.get('speed',.2)]
    yaw=0.
    output=[0.,0.]
    history=[plant.copy()]
    est=DelayedEstimator(config['speed_time_constant_sec'])
    est.initialize(plant[:3],0,0,0,0)
    phase=TurnPhase() if 'turn_enter_angle_rad' in config else None
    lateral=[]; rates=[]; saturated=[]; headings=[];trace=[]
    axis=np.array(goal)-np.array(start);axis=axis/np.linalg.norm(axis)
    turn=fitted['turn_fit'];speed=fitted['speed_fit']
    for i in range(1,2401):
        t=i*dt
        measured_rate=yaw*(-1 if config['imu_yaw_rate_reversed'] else 1)
        effective=[p*config['command_span_us']/100 for p in output]
        mean=sum(effective)/2
        est.advance(t,measured_rate,mean*config['speed_gain_m_s_per_us'])
        history.append(plant.copy())
        if i>=10 and i%2==0:
            measured=history[i-4]  # true plant-to-receipt delay 0.20s
            est.correct(measured[:3],t-config['measurement_delay_sec'],config['position_correction_gain'],
                        config['heading_correction_gain'],measured[3],4.)
        left,right,details=route_command(est.state,start,goal,measured_rate,config,phase)
        if details['distance_m']<=config['arrival_radius_m']:
            break
        step=config['output_slew_rate_percent_s']*dt
        output=[p+max(-step,min(step,q-p)) for p,q in zip(output,[left,right])]
        effective=[p*config['command_span_us']/100 for p in output]
        mean=sum(effective)/2
        target_rate=(effective[1]-effective[0])*turn['gain']*factor + mean*turn['common_thrust_bias_gain']+turn['constant_bias_rad_s']
        yaw+=(target_rate-yaw)*(1-math.exp(-dt/(turn['tau_sec']*factor)))
        plant=propagate(plant,dt,yaw,mean*speed['gain']*factor,speed['tau_sec']*factor)
        plant[0]+=speed['fitted_drift_xy_m_s'][0]*dt
        plant[1]+=speed['fitted_drift_xy_m_s'][1]*dt
        cross=-(plant[0]-start[0])*axis[1]+(plant[1]-start[1])*axis[0]
        # Report tracking after initial turn-around separately from launch manoeuvre.
        if t>20:
            lateral.append(cross);rates.append(yaw);headings.append(details['heading_error_rad'])
            saturated.append(abs(details['desired_differential_us'])>=config['maximum_differential_us']-.01)
        trace.append([t,*plant,yaw,cross,*output])
    return dict(arrived=details['distance_m']<=config['arrival_radius_m'],seconds=t,
        cross_track_rms_after_20s_m=float(np.sqrt(np.mean(np.square(lateral)))) if lateral else None,
        peak_heading_error_after_20s_deg=float(np.rad2deg(max(map(abs,headings)))) if headings else None,
        differential_saturated_fraction_after_20s=float(np.mean(saturated)) if saturated else None,
        final_distance_m=details['distance_m']),trace


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory')
    parser.add_argument('--compare-config')
    parser.add_argument('--model-log')
    args=parser.parse_args()
    root=Path(args.directory)
    old=json.loads((root/'parameters.json').read_text())
    fitted=json.loads((Path(args.model_log or root)/'motion_identification.json').read_text())
    if args.compare_config:
        new=yaml.safe_load(Path(args.compare_config).read_text())['goal_navigation']['ros__parameters']
        starts={}
        with (root/'telemetry.jsonl').open() as stream:
            for line in stream:
                try: row=json.loads(line)
                except ValueError: continue
                if row['event']=='tick' and row['state']=='navigating' and row['run_id'] not in starts:
                    starts[row['run_id']]=dict(start=row['start_xy_m'],goal=row['target_xy_m'],
                        heading=row['predicted_pose'][2],speed=row['predicted_pose'][3])
        result={}
        for run,scenario in starts.items():
            result[run]={}
            for name,c in [('before',old),('after',new)]:
                result[run][name]={}
                for factor in (.8,1.,1.2):
                    metrics,trace=trial(c,fitted,factor,scenario)
                    result[run][name][str(factor)]=metrics
        (root/'turn_and_speed_comparison.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        return
    calibrated=dict(old,imu_yaw_rate_reversed=False,measurement_delay_sec=.2,
        speed_gain_m_s_per_us=.00415,turn_gain_rad_s_per_us=.00335,
        speed_time_constant_sec=2.7,yaw_time_constant_sec=1.1)
    candidates={'original':old,'sign_and_model_only':calibrated}
    for kp,look in [(.4,2.5),(.6,2.5),(.8,2.5),(.6,3.0)]:
        candidates[f'kp{kp}_look{look}']=dict(calibrated,heading_kp=kp,lookahead_m=look,
            maximum_yaw_rate_rad_s=.25,maximum_differential_us=110.)
    results={}
    for name,c in candidates.items():
        results[name]={}
        for factor in (.8,1.,1.2):
            result,trace=trial(c,fitted,factor)
            results[name][str(factor)]=result
            if factor==1:
                np.savetxt(root/f'model_{name}.csv',np.array(trace),delimiter=',',comments='',
                    header='time,x,y,heading,speed,yaw_rate,cross_track,left_percent,right_percent')
    (root/'closed_loop_comparison.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    main()
