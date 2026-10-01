import json
import time
import numpy as np
import pytest
from usv_goal_navigation.demo_analysis import pwm_features, analyze_demo, PWM_PARAMETERS


def config():
    return dict(forward_only=True, unidirectional_equal_pulse_increment=True,
                channel_1_unidirectional_start_us=1190., channel_2_unidirectional_start_us=1100.,
                output_limit_percent=65., channel_1_trim_percent=0., channel_2_trim_percent=0.,
                channel_1=1, channel_2=2, dry_run=False, output_rate=20.)


def test_independent_thresholds():
    m=dict(pulse_width_us=[1290,1200],applied_percent=[100/550*65]*2)
    f=pwm_features(m,config())
    assert f['differential_increment_us']==0
    assert f['mapping_consistent']
    assert f['left_normalized_increment']==pytest.approx(100/550)
    m['pulse_width_us']=[1290,1290]
    assert not pwm_features(m,config())['mapping_consistent']
    m=dict(pulse_width_us=[1000,1000],applied_percent=[0.,0.])
    assert pwm_features(m,config())['mean_increment_us']==0
    c=config();c['unidirectional_equal_pulse_increment']=False
    assert pwm_features(dict(pulse_width_us=[1420,1375],applied_percent=[32.5,32.5]),c)['mapping_consistent']


def test_analysis_receipt_time_and_labels(tmp_path):
    meta=dict(pwm_parameters=config(),measurement_delay_sec=.2,start_monotonic=10.,
              end_monotonic=12.,target_xy_m=[20.,0.],label='operator_success')
    (tmp_path/'demonstration.json').write_text(json.dumps(meta))
    rows=[]
    for t in np.arange(9.5,12.6,.05):
        for kind,msg in [('rc',dict(microseconds=[1500]*16,connected=True)),
                         ('mode',dict(data=False)),
                         ('imu',dict(angular_velocity=dict(x=0.,y=0.,z=.1),linear_acceleration=dict(x=0.,y=0.,z=9.8)))]:
            rows.append(dict(event=kind,sample_monotonic=float(t)-.001,message=msg))
        rows.append(dict(event='vision',sample_monotonic=float(t)+2,received_monotonic=float(t)+.2,
                         pose=[float(t)-9.5,0.,0.]))
        rows.append(dict(event='pwm',sample_monotonic=float(t),message=dict(
            pulse_width_us=[1290,1200],applied_percent=[100/550*65]*2,requested_percent=[20.,20.],
            controller_angle_deg=[70,65],source='manual:rc_differential',
            emergency_stop_active=False,watchdog_active=False,dry_run=False)))
    (tmp_path/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    r=analyze_demo(tmp_path)
    assert r['eligible_for_review'] and r['mapping_mismatches']==0
    assert r['measured_speed_m_s_p10_p50_p90'][1]==pytest.approx(1.)
    meta['label']='interrupted'
    (tmp_path/'demonstration.json').write_text(json.dumps(meta))
    assert not analyze_demo(tmp_path)['eligible_for_review']
    (tmp_path/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in rows if r['event']!='vision'))
    assert analyze_demo(tmp_path)['usable_samples']==0


def test_live_parameters_and_parameter_change(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rcl_interfaces.msg import ParameterEvent, Parameter
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, String
    from usv_interfaces.msg import CrsfChannels, PwmOutputState
    from usv_goal_navigation.manual_demo import ManualDemo
    # Fake parameter server only: never connects to hardware or publishes commands.
    rclpy.init(args=['--ros-args','-p',f'log_directory:={tmp_path}'])
    pwm=Node('pwm_actuator');live=config()
    live.update(channel_1_unidirectional_start_us=1200.,channel_2_unidirectional_start_us=1070.)
    for k in PWM_PARAMETERS:pwm.declare_parameter(k,live[k])
    demo=ManualDemo();ex=SingleThreadedExecutor();ex.add_node(pwm);ex.add_node(demo)
    def feed():
        demo.receive('rc',CrsfChannels(connected=True))
        demo.receive('imu',Imu())
        demo.receive('mode',Bool(data=False))
        demo.receive('pwm',PwmOutputState(source='manual:rc_differential',pulse_width_us=[1000,1000]))
        demo.observation(String(data=json.dumps(dict(valid=True,session='test',received_monotonic=time.monotonic(),
            observation=dict(coord='test',frame=time.monotonic_ns(),processed_unix_ms=0,field_valid=True,
                size_m=[20,11],goals=[dict(side='left',ends_m=[[0,4],[0,6]])]),
            own_boat=dict(xy_m=[10,5],heading_rad=0)))))
    try:
        deadline=time.monotonic()+5
        while not demo.client.service_is_ready() and time.monotonic()<deadline:ex.spin_once(timeout_sec=.05)
        # Drain initial parameter announcements before the calibration snapshot.
        for _ in range(20):ex.spin_once(timeout_sec=.01)
        feed();demo.request_start()
        while demo.rec is None and time.monotonic()<deadline:
            feed();ex.spin_once(timeout_sec=.01)
        assert demo.rec is not None
        directory=demo.rec.directory
        assert demo.meta['pwm_parameters']==live
        feed()
        demo.parameter_event(ParameterEvent(node='/pwm_actuator',changed_parameters=[Parameter(name='output_limit_percent')]))
        assert demo.rec is None
        assert json.loads((directory/'demonstration.json').read_text())['label']=='invalid'
        assert (directory/'analysis.json').exists()
        assert not any(t.startswith('/propulsion/') for t,_ in demo.get_publisher_names_and_types_by_node('manual_demonstration','/'))
        feed();demo.receive('mode',Bool(data=True))
        demo.request_start();assert demo.pending is None
        feed();demo.receive('rc',CrsfChannels(connected=False))
        demo.request_start();assert demo.pending is None
        feed();demo.begin(live);directory=demo.rec.directory
        demo.finish('operator_success','operator_end')
        assert json.loads((directory/'analysis.json').read_text())['operator_label']=='operator_success'
        feed();demo.begin(live)
        demo.latest['imu']=(time.monotonic()-1,demo.latest['imu'][1]);demo.watchdog()
        assert demo.rec is None and demo.meta['label']=='interrupted'
    finally:
        demo.finish('interrupted','test_end');ex.shutdown();demo.destroy_node();pwm.destroy_node();rclpy.shutdown()
