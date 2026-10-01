"""Continuous navigation, mode arbitration input, delayed correction, raw logs."""
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import Bool, String
from usv_interfaces.msg import PropulsionCommand, PwmOutputState, CrsfChannels
from .control import goal_center, route_command, TurnPhase
from .estimation import DelayedEstimator, MapMemory, point
from .recording import Recorder

DEFAULTS = dict(
    observation_topic='/vision/global_observation', imu_topic='/imu/data_raw',
    command_topic='/navigation/test_command', status_topic='/navigation/status',
    goal_side='left', armed=False, require_mode=True, measurement_delay_sec=0.2,
    max_prediction_sec=10.0, imu_timeout=0.3, pwm_timeout=0.5, mode_timeout=0.3,
    output_rate=20.0, imu_yaw_rate_reversed=False, imu_bias_rad_s=0.0,
    position_correction_gain=0.65, heading_correction_gain=0.5,
    maximum_position_innovation_m=4.0, maximum_visual_speed_m_s=3.0,
    speed_gain_m_s_per_us=0.00415, turn_gain_rad_s_per_us=0.00335,
    speed_time_constant_sec=2.7, yaw_time_constant_sec=1.1,
    left_start_us=1190.0, right_start_us=1100.0, command_span_us=550.0,
    motor_turn_sign=1.0, cruise_speed_m_s=0.45, maximum_command_percent=35.0,
    maximum_differential_us=150.0, maximum_yaw_rate_rad_s=0.35,
    heading_kp=1.2, yaw_feedback_gain=0.6, speed_feedback_gain=0.25,
    lookahead_m=1.5, slowdown_distance_m=2.0, arrival_radius_m=0.6,
    output_slew_rate_percent_s=20.0,
    yaw_brake_preview_sec=0.5, turn_enter_angle_rad=0.8,
    turn_exit_angle_rad=0.25, turn_exit_yaw_rate_rad_s=0.15, turn_speed_cap_m_s=0.20,
    uncertainty_initial_m=0.2, uncertainty_growth_m_s=0.12,
    map_cache_file='~/usv_project/data/navigation/map_cache.json',
    log_directory='~/Downloads/usv_navigation_logs',
)


class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.c = {k: self.get_parameter(k).value for k in DEFAULTS}
        c = self.c
        for k, default in DEFAULTS.items():
            if type(default) is float and not math.isfinite(c[k]):
                raise ValueError(f'{k} must be finite')
        for k in ('max_prediction_sec', 'imu_timeout', 'pwm_timeout', 'mode_timeout', 'output_rate',
                  'speed_gain_m_s_per_us', 'turn_gain_rad_s_per_us', 'speed_time_constant_sec',
                  'yaw_time_constant_sec', 'command_span_us', 'lookahead_m', 'slowdown_distance_m',
                  'arrival_radius_m', 'maximum_visual_speed_m_s', 'maximum_position_innovation_m',
                  'output_slew_rate_percent_s', 'maximum_yaw_rate_rad_s', 'maximum_differential_us'):
            if c[k] <= 0:
                raise ValueError(f'{k} must be positive')
        if c['goal_side'] not in ('left', 'right') or c['motor_turn_sign'] not in (-1., 1.):
            raise ValueError('invalid goal side or motor turn sign')
        if not 0 <= c['measurement_delay_sec'] <= 2 or not 0 < c['maximum_command_percent'] <= 100:
            raise ValueError('invalid delay or output cap')
        for k in ('position_correction_gain', 'heading_correction_gain'):
            if not 0 < c[k] <= 1:
                raise ValueError(f'{k} must be in (0,1]')
        if not 0 <= c['yaw_brake_preview_sec'] <= 3:
            raise ValueError('yaw_brake_preview_sec must be in 0..3')
        if not 0 < c['turn_exit_angle_rad'] < c['turn_enter_angle_rad'] < math.pi:
            raise ValueError('require 0 < turn_exit_angle_rad < turn_enter_angle_rad < pi')
        if c['turn_exit_yaw_rate_rad_s'] <= 0:
            raise ValueError('turn_exit_yaw_rate_rad_s must be positive')
        if not 0 <= c['turn_speed_cap_m_s'] <= c['cruise_speed_m_s']:
            raise ValueError('turn_speed_cap_m_s must be between zero and cruise speed')
        self.rec = Recorder(c['log_directory'], c)
        self.map = MapMemory()
        self.cache_path = Path(c['map_cache_file']).expanduser()
        try:
            self.cached = json.loads(self.cache_path.read_text())
        except (OSError, ValueError):
            self.cached = None
        self.est = DelayedEstimator(c['speed_time_constant_sec'])
        self.last_imu = self.last_pwm = self.last_mode = self.last_visual = 0.0
        self.rate = self.raw_rate = 0.0
        self.auto = False
        self.pwm = None
        self.frame = -1
        self.previous_visual = None
        self.session = None
        self.fault = None
        self.start = None
        self.turn_phase = TurnPhase()
        self.arrived = False
        self.run_id = 0
        self.run_started = None
        self.running = False
        self.outputs = [0., 0.]
        self.last_tick = time.monotonic()
        self.counts = dict(accepted=0, rejected=0, duplicates=0)
        self.last_correction = {}
        self.command_pub = self.create_publisher(PropulsionCommand, c['command_topic'], 10)
        self.status_pub = self.create_publisher(String, c['status_topic'], 10)
        self.create_subscription(String, c['observation_topic'], self.on_observation, 100)
        self.create_subscription(Imu, c['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PwmOutputState, '/actuators/pwm_state', self.on_pwm, 100)
        self.create_subscription(Bool, '/control/automatic_mode', self.on_mode, 10)
        self.create_subscription(String, '/vision/udp_diagnostics', lambda m: self.rec.write('udp', raw=m.data), 200)
        self.create_subscription(PropulsionCommand, '/propulsion/command', lambda m: self.rec.message('arbitrated_command', m), 100)
        self.create_subscription(CrsfChannels, '/rc/channels', lambda m: self.rec.message('rc', m), qos_profile_sensor_data)
        self.create_subscription(ParameterEvent, '/parameter_events', lambda m: self.rec.message('parameter_event', m), 100)
        self.create_timer(1 / c['output_rate'], self.update)
        self.get_logger().info(f'Navigation v2 logs: {self.rec.directory}')

    def end_run(self, reason):
        if self.running:
            self.rec.write('run_end', run_id=self.run_id, reason=reason)
            self.rec.file.flush()
            summary = dict(run_id=self.run_id, reason=reason, start_xy_m=self.start,
                elapsed_sec=time.monotonic()-self.run_started, final_predicted_state=self.est.state,
                goal_side=self.c['goal_side'], map=self.map.snapshot(), counts=self.counts.copy())
            (self.rec.directory / f'run_{self.run_id:04d}.json').write_text(json.dumps(summary, indent=2))
            self.running = False

    def on_mode(self, msg):
        self.last_mode = time.monotonic()
        if bool(msg.data) != self.auto:
            self.turn_phase.reset()
            self.rec.write('mode', automatic=bool(msg.data))
            self.outputs = [0., 0.]
            self.start = None
            self.arrived = False
            if not msg.data:
                self.end_run('manual_takeover')
            self.auto = bool(msg.data)

    def on_pwm(self, msg):
        self.rec.message('pwm', msg)
        self.advance(time.monotonic())
        self.pwm = msg
        self.last_pwm = time.monotonic()

    def effective_pwm(self, now):
        if self.pwm is None or now - self.last_pwm > self.c['pwm_timeout']:
            return [0., 0.]
        if self.pwm.emergency_stop_active or self.pwm.watchdog_active or self.pwm.dry_run:
            return [0., 0.]
        return [max(0., float(self.pwm.pulse_width_us[0]) - self.c['left_start_us']),
                max(0., float(self.pwm.pulse_width_us[1]) - self.c['right_start_us'])]

    def advance(self, now):
        target = sum(self.effective_pwm(now)) / 2 * self.c['speed_gain_m_s_per_us']
        rate = self.rate if now - self.last_imu <= self.c['imu_timeout'] else 0.0
        self.est.advance(now, rate, target)

    def on_imu(self, msg):
        raw = float(msg.angular_velocity.z)
        values = [raw, msg.angular_velocity.x, msg.angular_velocity.y,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        if not all(math.isfinite(v) for v in values) or msg.angular_velocity_covariance[0] == -1:
            self.rec.write('invalid_imu')
            return
        self.rec.message('imu', msg)
        now = time.monotonic()
        self.advance(now)
        self.raw_rate = raw
        self.rate = (raw - self.c['imu_bias_rad_s']) * (-1 if self.c['imu_yaw_rate_reversed'] else 1)
        self.last_imu = now

    def on_observation(self, msg):
        now = time.monotonic()
        self.rec.write('observation', raw=msg.data)
        try:
            report = json.loads(msg.data)
            if report.get('reset_required'):
                self.fault = 'coordinate_or_session_reset'
                return
            obs = report.get('observation')
            if not isinstance(obs, dict):
                return
            session = report.get('session')
            if self.session is not None and session != self.session:
                self.fault = 'coordinate_or_session_reset'
                return
            if self.map.coord and obs['coord'] != self.map.coord:
                self.fault = 'coordinate_or_session_reset'
                return
            if self.map.coord is None and isinstance(self.cached, dict) and self.cached.get('coord') == obs['coord']:
                self.map.ingest(dict(coord=obs['coord'], field_valid=True,
                    size_m=self.cached['size_m'], goals=[dict(side=s, ends_m=e) for s, e in self.cached['goals'].items()]))
            changed = self.map.ingest(obs)
            if changed:
                self.cache_path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.cache_path.with_suffix('.tmp')
                tmp.write_text(json.dumps(self.map.snapshot(), indent=2))
                tmp.replace(self.cache_path)
                self.rec.write('map', map=self.map.snapshot())
            if not report.get('valid') or obs.get('field_valid') is not True:
                return
            frame = obs['frame']
            if type(frame) is not int or frame <= self.frame:
                self.counts['duplicates'] += 1
                return
            xy = point(report['own_boat']['xy_m'])
            heading = float(report['own_boat']['heading_rad'])
            ms = obs['processed_unix_ms']
            if type(ms) is not int or not math.isfinite(heading):
                raise ValueError('invalid timestamp/heading')
            # Same-machine monotonic reception, not subtraction of unsynchronized wall clocks.
            received = float(report.get('received_monotonic', now))
            if not math.isfinite(received) or received > now or now - received > 2:
                raise ValueError('invalid local receipt timestamp')
            stamp = received - self.c['measurement_delay_sec']
            speed = None
            if self.previous_visual:
                px, py, pm = self.previous_visual
                dt = (ms - pm) / 1000
                if dt <= 0:
                    raise ValueError('nonmonotonic visual timestamp')
                if dt <= 3:
                    vx, vy = (xy[0]-px)/dt, (xy[1]-py)/dt
                    if math.hypot(vx, vy) > self.c['maximum_visual_speed_m_s']:
                        raise ValueError('visual velocity outlier')
                    speed = max(0., vx * math.cos(heading) + vy * math.sin(heading))
            self.advance(now)
            if self.est.state is None:
                self.est.initialize([*xy, heading], stamp, now, self.rate, 0.)
                correction = dict(accepted=True, reason='initialize')
            else:
                correction = self.est.correct([*xy, heading], stamp,
                    self.c['position_correction_gain'], self.c['heading_correction_gain'], speed,
                    self.c['maximum_position_innovation_m'])
            self.last_correction = correction
            self.rec.write('visual', frame=frame, received_monotonic=received,
                measurement_monotonic=stamp, measurement_ms=ms, xy_m=xy, heading=heading,
                speed_m_s=speed, correction=correction)
            if not correction['accepted']:
                self.counts['rejected'] += 1
                return
            self.frame, self.session = frame, session
            self.previous_visual = (*xy, ms)
            self.last_visual = received
            self.counts['accepted'] += 1
        except (ValueError, TypeError, KeyError, AttributeError, IndexError, OverflowError) as e:
            self.counts['rejected'] += 1
            self.rec.write('rejected', reason=str(e))
        except OSError as e:
            self.fault = 'map_io_error'
            self.get_logger().error(str(e))

    def publish(self, left=0., right=0.):
        msg = PropulsionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.channel_1_percent, msg.channel_2_percent = float(left), float(right)
        msg.source = f"goal_navigation:{self.c['goal_side']}"
        self.command_pub.publish(msg)

    def update(self):
        now = time.monotonic()
        dt = min(0.2, now - self.last_tick)
        self.last_tick = now
        self.advance(now)
        c = self.c
        age = now - self.last_visual if self.last_visual else None
        selected = not c['require_mode'] or (self.auto and now - self.last_mode <= c['mode_timeout'])
        goal = self.map.goals.get(c['goal_side'])
        state = 'navigating'
        if self.fault:
            state = self.fault
        elif not selected:
            state = 'manual_or_mode_unavailable'
        elif self.est.state is None or goal is None or self.map.size is None:
            state = 'waiting_for_map_and_position'
        elif now - self.last_imu > c['imu_timeout']:
            state = 'imu_timeout'
        elif age is None or age > c['max_prediction_sec']:
            state = 'visual_timeout'
        elif c['armed'] and (self.pwm is None or now - self.last_pwm > c['pwm_timeout']):
            state = 'pwm_feedback_timeout'
        elif c['armed'] and (self.pwm.emergency_stop_active or self.pwm.watchdog_active):
            state = 'actuator_stopped'
        elif self.arrived:
            state = 'arrived'
        data = {}
        proposed = [0., 0.]
        if state == 'navigating':
            if self.start is None:
                self.turn_phase.reset()
                self.start = self.est.state[:2]
                self.run_id += 1
                self.run_started = now
                self.running = True
                self.rec.write('run_start', run_id=self.run_id, start=self.start, map=self.map.snapshot())
            target = list(goal_center(goal))
            left, right, data = route_command(self.est.state, self.start, target, self.rate, c, self.turn_phase)
            data.update(start_xy_m=self.start, target_xy_m=target)
            if data['distance_m'] <= c['arrival_radius_m']:
                state = 'arrived'
                self.arrived = True
            else:
                delta = c['output_slew_rate_percent_s'] * dt
                proposed = [p + max(-delta, min(delta, q-p)) for p, q in zip(self.outputs, [left, right])]
        self.outputs = proposed
        active = c['armed'] and state == 'navigating'
        self.publish(*(proposed if active else [0., 0.]))
        inc = self.effective_pwm(now)
        predicted = self.est.state
        outside = bool(predicted and self.map.size and not
            (0 <= predicted[0] <= self.map.size[0] and 0 <= predicted[1] <= self.map.size[1]))
        status = dict(state=state, run_id=self.run_id, command_active=active, armed=c['armed'],
            goal_side=c['goal_side'], automatic_selected=selected, frame=self.frame, visual_age_sec=age,
            dead_reckoning=age is not None and age > 0.25, predicted_pose=predicted,
            yaw_rate_rad_s=self.rate, raw_yaw_rate_rad_s=self.raw_rate, effective_pwm_us=inc,
            # ROS fixed arrays contain NumPy scalars; list() does not convert them.
            pulse_width_us=[int(v) for v in self.pwm.pulse_width_us] if self.pwm else None,
            applied_percent=[float(v) for v in self.pwm.applied_percent] if self.pwm else None,
            pwm_feedback_age_sec=now-self.last_pwm if self.last_pwm else None,
            pwm_source=self.pwm.source if self.pwm else None,
            actuator_emergency_stop=self.pwm.emergency_stop_active if self.pwm else None,
            dry_run=self.pwm.dry_run if self.pwm else None,
            arrival_basis='prediction' if age is not None and age > 0.25 else 'recent_visual_fusion',
            uncertainty_estimate_m=c['uncertainty_initial_m'] + (age or 0)*c['uncertainty_growth_m_s'],
            predicted_outside_boundary=outside, map=self.map.snapshot(),
            proposed_left_percent=proposed[0], proposed_right_percent=proposed[1],
            correction=self.last_correction, counts=self.counts.copy(), **data)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.rec.write('tick', **status)
        if state == 'arrived':
            self.end_run('arrived')
        elif not selected:
            self.turn_phase.reset()
            self.end_run('manual_or_mode_unavailable')
            self.start = None
            self.arrived = False


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch may forward SIGINT after the process group already received it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.end_run('shutdown')
        node.rec.close()
        if rclpy.ok():
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
