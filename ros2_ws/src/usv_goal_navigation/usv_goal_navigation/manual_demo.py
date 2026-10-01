"""Keyboard-triggered observation-only manual demonstration recorder."""
import json
import math
import select
import signal
import sys
import termios
import time
import tty
from collections import deque
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterEvent
from usv_interfaces.msg import CrsfChannels, CrsfLinkStatistics, PropulsionCommand, PwmOutputState
from .estimation import MapMemory, point
from .control import goal_center
from .recording import Recorder
from .demo_analysis import PWM_PARAMETERS, analyze_demo


class ManualDemo(Node):
    def __init__(self):
        super().__init__('manual_demonstration')
        defaults=dict(goal_side='left',log_directory='~/Downloads/usv_manual_demonstrations',
            measurement_delay_sec=.2,pre_roll_sec=2.,freshness_sec=.5,
            position_start_freshness_sec=1.,expected_speed_m_s=0.,pwm_node='/pwm_actuator')
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.c={k:self.get_parameter(k).value for k in defaults}
        if self.c['goal_side'] not in ('left','right'):raise ValueError('goal_side must be left/right')
        for k in ('measurement_delay_sec','pre_roll_sec','freshness_sec','position_start_freshness_sec','expected_speed_m_s'):
            if not math.isfinite(self.c[k]) or self.c[k]<0:raise ValueError(f'invalid {k}')
        if self.c['freshness_sec']==0 or self.c['position_start_freshness_sec']==0:raise ValueError('freshness must be positive')
        self.buffer=deque(maxlen=10000);self.latest={};self.rec=None;self.meta=None
        self.map=MapMemory();self.frame=None;self.session=None;self.pose=None;self.pose_time=0.
        self.pending=None;self.pending_since=0.;self.calibration_generation=0
        self.client=self.create_client(GetParameters,self.c['pwm_node'].rstrip('/')+'/get_parameters')
        for topic,kind,cls,qos in [('/rc/channels','rc',CrsfChannels,qos_profile_sensor_data),
            ('/rc/link_statistics','link',CrsfLinkStatistics,qos_profile_sensor_data),
            ('/imu/data_raw','imu',Imu,qos_profile_sensor_data),
            ('/actuators/pwm_state','pwm',PwmOutputState,100),
            ('/control/automatic_mode','mode',Bool,50),
            ('/propulsion/manual_command','manual_command',PropulsionCommand,100),
            ('/propulsion/command','arbitrated_command',PropulsionCommand,100)]:
            self.create_subscription(cls,topic,lambda m,k=kind:self.receive(k,m),qos)
        self.create_subscription(String,'/vision/global_observation',self.observation,100)
        self.create_subscription(String,'/vision/udp_diagnostics',lambda m:self.record('udp',raw=m.data),200)
        self.create_subscription(ParameterEvent,'/parameter_events',self.parameter_event,100)
        self.create_timer(.1,self.watchdog)

    def record(self,kind,**values):
        now=time.monotonic()
        event=dict(kind=kind,sample_monotonic=now,sample_unix_ns=time.time_ns(),**values)
        self.buffer.append(event)
        while self.buffer and now-self.buffer[0]['sample_monotonic']>self.c['pre_roll_sec']:
            self.buffer.popleft()
        if self.rec:self.rec.write(kind,**{k:v for k,v in event.items() if k!='kind'})

    def receive(self,kind,msg):
        value=message_to_ordereddict(msg)
        # Keep malformed/nonfinite sensor data from breaking the entire recorder.
        try: json.dumps(value,allow_nan=False)
        except (ValueError,TypeError):
            self.record('invalid_message',topic_kind=kind);return
        self.latest[kind]=(time.monotonic(),value)
        self.record(kind,message=value)

    def observation(self,msg):
        self.record('observation',raw=msg.data)
        try:
            r=json.loads(msg.data);obs=r.get('observation')
            if r.get('reset_required'):
                if self.rec:self.finish('invalid','coordinate_reset')
                self.map=MapMemory();self.pose=None;self.pose_time=0.;self.frame=None
                return
            if not isinstance(obs,dict):return
            if ((self.map.coord and obs['coord']!=self.map.coord) or
                (self.session is not None and r.get('session')!=self.session)):
                if self.rec:self.finish('invalid','coordinate_or_session_changed')
                self.map=MapMemory();self.frame=None;self.pose=None;self.pose_time=0.
            self.map.ingest(obs);self.session=r.get('session')
            if not r.get('valid') or not obs.get('field_valid'):return
            frame=obs['frame']
            if type(frame) is not int or (self.frame is not None and frame<=self.frame):return
            xy=point(r['own_boat']['xy_m']);h=float(r['own_boat']['heading_rad'])
            received=float(r.get('received_monotonic') or time.monotonic())
            if not math.isfinite(h) or not math.isfinite(received) or not 0<=time.monotonic()-received<2:return
            self.frame=frame;self.pose=[*xy,h];self.pose_time=received
            # Preserve socket receipt separately from this subscriber callback time.
            self.record('vision',pose=self.pose,frame=frame,received_monotonic=received,
                        processed_unix_ms=obs['processed_unix_ms'],coord=self.map.coord)
        except (ValueError,TypeError,KeyError,AttributeError,IndexError):
            self.record('invalid_observation')

    def parameter_event(self,msg):
        if msg.node.rstrip('/')!=self.c['pwm_node'].rstrip('/'):return
        changed={p.name for p in [*msg.new_parameters,*msg.changed_parameters,*msg.deleted_parameters]}
        if changed.intersection(PWM_PARAMETERS):
            self.calibration_generation+=1
            self.record('pwm_configuration_changed',message=message_to_ordereddict(msg))
            if self.rec:self.finish('invalid','pwm_configuration_changed')

    def readiness(self,require_position=True):
        now=time.monotonic()
        for k in ('rc','imu','pwm','mode'):
            if k not in self.latest or now-self.latest[k][0]>self.c['freshness_sec']:
                return f'等待新鲜的 {k} 数据'
        if self.latest['mode'][1]['data']:return '请把CH8切到手动低位'
        if not self.latest['rc'][1]['connected']:return '遥控器未连接'
        p=self.latest['pwm'][1]
        if p['emergency_stop_active'] or p['watchdog_active']:return '急停或动力看门狗处于停机状态'
        if p['dry_run']:return '当前PWM为dry_run，不能作为实船示范'
        if not p['source'].startswith('manual:'):return '最终PWM来源不是手动控制'
        if require_position:
            if self.pose is None or now-self.pose_time>self.c['position_start_freshness_sec']:return '等待本船有效新位置'
            if self.c['goal_side'] not in self.map.goals or self.map.size is None:return '等待目标球门和场地数据'
        return None

    def request_start(self):
        if self.rec or self.pending:
            print('已有采集或启动请求，请先结束。',flush=True);return
        reason=self.readiness()
        if reason:print(f'未开始：{reason}',flush=True);return
        if not self.client.service_is_ready():print('未开始：PWM参数服务不可用',flush=True);return
        self.pending_since=time.monotonic();generation=self.calibration_generation
        self.pending=self.client.call_async(GetParameters.Request(names=PWM_PARAMETERS))
        future=self.pending
        def ready(done):
            if self.pending is not done:return
            self.pending=None
            try:
                config={k:parameter_value_to_python(v) for k,v in zip(PWM_PARAMETERS,done.result().values)}
                if len(config)!=len(PWM_PARAMETERS) or any(v is None for v in config.values()):raise ValueError('PWM参数不完整')
                if not config['forward_only']:raise ValueError('当前采集分析仅支持单向电调')
                if generation!=self.calibration_generation:raise ValueError('参数读取期间发生修改，请再按S')
                reason=self.readiness()
                if reason:raise ValueError(reason)
                self.begin(config)
            except Exception as e:print(f'未开始：{e}',flush=True)
        future.add_done_callback(ready)

    def begin(self,config):
        started=time.monotonic()
        self.rec=Recorder(self.c['log_directory'],self.c)
        self.meta=dict(format_version=1,start_monotonic=started,goal_side=self.c['goal_side'],
            target_xy_m=list(goal_center(self.map.goals[self.c['goal_side']])),
            start_pose=self.pose,start_visual_received_monotonic=self.pose_time,map=self.map.snapshot(),
            measurement_delay_sec=self.c['measurement_delay_sec'],expected_speed_m_s=self.c['expected_speed_m_s'],
            pwm_parameters=config,label='incomplete',reason='recording')
        (self.rec.directory/'demonstration.json').write_text(json.dumps(self.meta,indent=2))
        for event in self.buffer:
            self.rec.write(event['kind'],**{k:v for k,v in event.items() if k!='kind'},pre_roll=True)
        self.rec.write('demo_start',metadata=self.meta)
        print(f"开始采集 → {self.c['goal_side']}球门；左启动{config['channel_1_unidirectional_start_us']}us，右启动{config['channel_2_unidirectional_start_us']}us\n保存位置：{self.rec.directory}\nE成功结束；X非成功；Q退出",flush=True)

    def finish(self,label,reason):
        if self.pending:self.pending=None
        if not self.rec:return
        self.meta.update(end_monotonic=time.monotonic(),label=label,reason=reason,
            last_visual_pose=self.pose,last_visual_age_sec=time.monotonic()-self.pose_time)
        directory=self.rec.directory
        self.rec.write('demo_end',metadata=self.meta)
        (directory/'demonstration.json').write_text(json.dumps(self.meta,indent=2))
        self.rec.close();self.rec=None
        print(f'已保存：{directory}（{label} / {reason}）',flush=True)
        try:
            result=analyze_demo(directory)
            print(f"分析完成：有效样本{result['usable_samples']}/{result['samples']}；映射不一致{result['mapping_mismatches']}。见 analysis.json / demonstration.csv",flush=True)
        except Exception as e:
            self.get_logger().error(f'分析失败，原始记录已保存：{e}')

    def watchdog(self):
        if self.pending and time.monotonic()-self.pending_since>3:
            self.pending=None;print('PWM参数读取超时，请重试S',flush=True)
        if self.rec:
            reason=self.readiness(require_position=False)
            if reason:self.finish('interrupted',reason)


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('请在交互终端运行（SSH需要分配终端），不要将采集程序放到后台。')
    rclpy.init(args=args);node=ManualDemo();saved=termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print('手动示范：CH8保持低位，CH2油门/CH4转向。S开始；E成功结束；X非成功；Q退出。\n只采集，不发布动力指令。',flush=True)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if key=='s':node.request_start()
                elif key=='e':node.finish('operator_success','operator_end')
                elif key=='x':node.finish('not_successful','operator_rejected')
                elif key=='q':break
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved)
        node.finish('interrupted','recorder_exit')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
