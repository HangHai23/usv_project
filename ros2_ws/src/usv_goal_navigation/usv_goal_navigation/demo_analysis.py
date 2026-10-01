"""Manual demonstration features, preserving per-motor ESC calibration."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import numpy as np

PWM_PARAMETERS = [
    'forward_only', 'unidirectional_equal_pulse_increment',
    'channel_1_unidirectional_start_us', 'channel_2_unidirectional_start_us',
    'output_limit_percent', 'channel_1_trim_percent', 'channel_2_trim_percent',
    'channel_1', 'channel_2', 'dry_run', 'output_rate',
]


def pwm_features(message, config):
    if not config['forward_only']:
        raise ValueError('This demonstration format currently requires forward-only ESCs')
    starts = [float(config[f'channel_{i}_unidirectional_start_us']) for i in (1,2)]
    limit = float(config['output_limit_percent'])
    maximum = 1000 + 10*limit
    actual_starts = [min(s, maximum) for s in starts]
    spans = ([maximum-min(starts)]*2 if config['unidirectional_equal_pulse_increment']
             else [maximum-s for s in actual_starts])
    pulses = [int(v) for v in message['pulse_width_us']]
    increments = [max(0., p-s) if p>1000 else 0. for p,s in zip(pulses,actual_starts)]
    expected = [1000 if a<=0 or limit<=0 else round(s+w*min(float(a)/limit,1.))
                for a,s,w in zip(message['applied_percent'],actual_starts,spans)]
    return dict(left_pulse_us=pulses[0], right_pulse_us=pulses[1],
        left_start_us=starts[0], right_start_us=starts[1],
        left_increment_us=increments[0], right_increment_us=increments[1],
        mean_increment_us=sum(increments)/2, differential_increment_us=increments[1]-increments[0],
        left_normalized_increment=increments[0]/spans[0] if spans[0]>0 else 0.,
        right_normalized_increment=increments[1]/spans[1] if spans[1]>0 else 0.,
        mapping_consistent=all(abs(p-e)<=1 for p,e in zip(pulses,expected)))


def analyze_demo(directory):
    root=Path(directory)
    meta=json.loads((root/'demonstration.json').read_text())
    config=meta['pwm_parameters']
    rows=[];damaged=0
    with (root/'telemetry.jsonl').open() as stream:
        for line in stream:
            try: rows.append(json.loads(line))
            except ValueError: damaged+=1
    streams={}
    for name in ('rc','imu','pwm','vision','mode'):
        stream=sorted([r for r in rows if r['event']==name],key=lambda r:r['sample_monotonic'])
        streams[name]=([r['sample_monotonic'] for r in stream],stream)
    def latest(name,t,age):
        times,stream=streams[name];i=bisect.bisect_right(times,t)-1
        return stream[i] if i>=0 and t-times[i]<=age else None
    unique={v.get('received_monotonic',v['sample_monotonic']):v for v in streams['vision'][1]}
    vision=[unique[t] for t in sorted(unique)]
    vt=np.array([v.get('received_monotonic',v['sample_monotonic'])-meta['measurement_delay_sec'] for v in vision])
    xy=np.array([v['pose'][:2] for v in vision])
    headings=np.unwrap([v['pose'][2] for v in vision])
    start=meta['start_monotonic'];end=meta.get('end_monotonic',float('inf'))
    goal=meta['target_xy_m'];features=[];last=None;accelerations=[]
    for p in streams['pwm'][1]:
        t=p['sample_monotonic']
        if not start<=t<=end: continue
        m=p['message'];f=pwm_features(m,config)
        f.update(elapsed_sec=t-start,sample_monotonic=t,
            requested_left_percent=m['requested_percent'][0],requested_right_percent=m['requested_percent'][1],
            applied_left_percent=m['applied_percent'][0],applied_right_percent=m['applied_percent'][1],
            left_controller_angle_deg=m['controller_angle_deg'][0],right_controller_angle_deg=m['controller_angle_deg'][1],
            source=m['source'],emergency_stop=m['emergency_stop_active'],watchdog=m['watchdog_active'],dry_run=m['dry_run'])
        rc=latest('rc',t,.25);imu=latest('imu',t,.20);mode=latest('mode',t,.3)
        f.update(ch2_us=rc['message']['microseconds'][1] if rc else None,
                 ch4_us=rc['message']['microseconds'][3] if rc else None,
                 yaw_rate_rad_s=imu['message']['angular_velocity']['z'] if imu else None,
                 angular_velocity_x=imu['message']['angular_velocity']['x'] if imu else None,
                 angular_velocity_y=imu['message']['angular_velocity']['y'] if imu else None,
                 acceleration_x=imu['message']['linear_acceleration']['x'] if imu else None,
                 acceleration_y=imu['message']['linear_acceleration']['y'] if imu else None,
                 acceleration_z=imu['message']['linear_acceleration']['z'] if imu else None)
        # Offline interpolation uses surrounding measured frames, never fills long visual gaps.
        j=int(np.searchsorted(vt,t))
        good=0<j<len(vt) and vt[j]-vt[j-1]<=.5
        f.update(position_available=bool(good),x_m=None,y_m=None,heading_rad=None,
                 goal_error_rad=None,distance_m=None,measured_speed_m_s=None,
                 visual_bracket_sec=float(vt[j]-vt[j-1]) if 0<j<len(vt) else None)
        if good:
            a=(t-vt[j-1])/(vt[j]-vt[j-1]);pos=xy[j-1]*(1-a)+xy[j]*a
            h=headings[j-1]*(1-a)+headings[j]*a
            bearing=math.atan2(goal[1]-pos[1],goal[0]-pos[0]);err=math.atan2(math.sin(bearing-h),math.cos(bearing-h))
            f.update(x_m=float(pos[0]),y_m=float(pos[1]),heading_rad=float(h),goal_error_rad=err,
                distance_m=math.dist(pos,goal))
            ia=int(np.searchsorted(vt,t-.3));ib=int(np.searchsorted(vt,t+.3))
            if ia>0 and ib<len(vt) and np.max(np.diff(vt[ia-1:ib+1]))<=.5:
                vx=(np.interp(t+.3,vt,xy[:,0])-np.interp(t-.3,vt,xy[:,0]))/.6
                vy=(np.interp(t+.3,vt,xy[:,1])-np.interp(t-.3,vt,xy[:,1]))/.6
                f['measured_speed_m_s']=float(vx*math.cos(h)+vy*math.sin(h))
        f['usable_sample']=bool(good and rc and imu and mode and not mode['message']['data']
            and rc['message']['connected'] and not(m['emergency_stop_active'] or m['watchdog_active'] or m['dry_run'])
            and m['source'].startswith('manual:') and f['mapping_consistent'])
        f['mean_increment_rate_us_s']=None
        if last and 0<t-last['sample_monotonic']<=.25:
            rate=(f['mean_increment_us']-last['mean_increment_us'])/(t-last['sample_monotonic'])
            f['mean_increment_rate_us_s']=rate
            if f['usable_sample'] and rate>30:
                accelerations.append(abs(f['goal_error_rad'])*180/math.pi)
        features.append(f);last=f
    if features:
        with (root/'demonstration.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(features[0]));writer.writeheader();writer.writerows(features)
    usable=[f for f in features if f['usable_sample']]
    def quantile(values):
        return [float(v) for v in np.quantile(values,[.1,.5,.9])] if values else None
    result=dict(operator_label=meta.get('label','incomplete'),termination_reason=meta.get('reason'),
        samples=len(features),usable_samples=len(usable),damaged_lines=damaged,
        position_missing_fraction=sum(not f['position_available'] for f in features)/max(1,len(features)),
        mapping_mismatches=sum(not f['mapping_consistent'] for f in features),
        acceleration_heading_error_deg_p10_p50_p90=quantile(accelerations),
        measured_speed_m_s_p10_p50_p90=quantile([f['measured_speed_m_s'] for f in usable if f['measured_speed_m_s'] is not None]),
        final_measured_distance_m=next((f['distance_m'] for f in reversed(features) if f['distance_m'] is not None),None),
        notes=['Operator success is a human label, not an automatic arrival confirmation.',
               'PWM increment is relative to each motor threshold, not measured thrust.',
               'Vision aligned with configured delay; motion can be reviewed before learning.',
               'No learned policy or propulsion parameters are automatically applied.'])
    result['eligible_for_review']=bool(meta.get('label')=='operator_success' and len(usable)>=20
        and result['mapping_mismatches']==0 and result['position_missing_fraction']<.2)
    # Descriptive response bins, not causal gains: inertia/delay/wind remain in these observations.
    result['response_bins']=[]
    for low,high in [(-1000,-200),(-200,-50),(-50,50),(50,200),(200,1001)]:
        group=[f for f in usable if low<=f['differential_increment_us']<high]
        result['response_bins'].append(dict(differential_increment_range_us=[low,high],samples=len(group),
            yaw_rate_rad_s_p10_p50_p90=quantile([f['yaw_rate_rad_s'] for f in group]),
            forward_speed_m_s_p10_p50_p90=quantile([f['measured_speed_m_s'] for f in group if f['measured_speed_m_s'] is not None])))
    result['notes'].append('Response bins are descriptive, not identified causal turning gains; inertia and disturbances remain.')
    (root/'analysis.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    print(json.dumps(analyze_demo(args.directory),indent=2))
