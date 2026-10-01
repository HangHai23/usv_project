"""CRSF throttle to four servo outputs using the existing PWM board protocol."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray
from usv_interfaces.msg import CrsfChannels

from .gait import leg_angles, throttle_fraction


class Controller(Node):
    def __init__(self):
        super().__init__('quadruped_rc_control')
        defaults = dict(
            dry_run=True, i2c_bus=7, i2c_address=45, throttle_channel=2,
            throttle_reversed=False, deadband_us=20, rc_timeout=0.2,
            emergency_stop_channel=5, emergency_stop_threshold_us=1500,
            output_rate=50.0, maximum_frequency_hz=1.0, support_fraction=0.7,
            servo_channels=[1, 2, 3, 4], centers_deg=[87.0]*4,
            amplitudes_deg=[20.0]*4, directions=[1, 1, -1, -1],
            phase_offsets=[0.0, 0.5, 0.5, 0.0],
        )
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        c = self.cfg
        for key in ('servo_channels', 'centers_deg', 'amplitudes_deg', 'directions', 'phase_offsets'):
            if len(c[key]) != 4:
                raise ValueError(f'{key} must contain four entries: LF, LR, RF, RR')
        if len(set(c['servo_channels'])) != 4 or any(not 1 <= n <= 16 for n in c['servo_channels']):
            raise ValueError('servo channels must be distinct and within 1..16')
        if any(not 1 <= c[k] <= 16 for k in ('throttle_channel', 'emergency_stop_channel')):
            raise ValueError('RC channels must be within 1..16')
        if not 0 <= c['deadband_us'] < 500 or not 0 < c['support_fraction'] < 1:
            raise ValueError('invalid deadband or support fraction')
        if not 0 <= c['i2c_address'] <= 127 or c['i2c_bus'] < 0:
            raise ValueError('invalid I2C configuration')
        for key in ('rc_timeout', 'output_rate', 'maximum_frequency_hz'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        for center, amplitude, direction, offset in zip(c['centers_deg'], c['amplitudes_deg'], c['directions'], c['phase_offsets']):
            if not all(math.isfinite(v) for v in (center, amplitude, offset)) or amplitude < 0 or not 0 <= center-amplitude <= center+amplitude <= 180 or direction not in (-1, 1):
                raise ValueError('invalid servo calibration')
        self.bus = None
        self.phase = 0.0
        self.demand = 0.0
        self.estop = True
        self.last_rc = None
        self.last_tick = time.monotonic()
        self.state = self.create_publisher(Int32MultiArray, '/quadruped/servo_angles', 10)
        self.create_subscription(CrsfChannels, '/rc/channels', self.receive, qos_profile_sensor_data)
        if not c['dry_run']:
            import smbus
            self.bus = smbus.SMBus(c['i2c_bus'])
        try:
            self.write([round(v) for v in c['centers_deg']])
        except Exception:
            if self.bus is not None:
                self.bus.close()
            raise
        self.create_timer(1.0 / c['output_rate'], self.tick)

    def receive(self, msg):
        c = self.cfg
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        if not msg.connected or not -0.05 <= age <= c['rc_timeout']:
            self.last_rc = None
            self.estop = True
            self.demand = 0.0
            self.tick()
            return
        self.last_rc = time.monotonic()
        stop = msg.microseconds[c['emergency_stop_channel']-1]
        if stop > c['emergency_stop_threshold_us']:
            self.estop = True
        elif stop < c['emergency_stop_threshold_us']:
            self.estop = False
        self.demand = throttle_fraction(msg.microseconds[c['throttle_channel']-1], c['deadband_us'])
        if c['throttle_reversed']:
            self.demand *= -1
        if self.estop:
            self.tick()

    def tick(self):
        c = self.cfg
        now = time.monotonic()
        dt = min(now - self.last_tick, 2.0 / c['output_rate'])
        self.last_tick = now
        moving = self.last_rc is not None and now-self.last_rc <= c['rc_timeout'] and not self.estop and self.demand != 0.0
        if moving:
            self.phase = (self.phase + dt * self.demand * c['maximum_frequency_hz']) % 1.0
            angles = leg_angles(self.phase, c['centers_deg'], c['amplitudes_deg'], c['directions'], c['phase_offsets'], c['support_fraction'])
        else:
            self.phase = 0.0
            if self.last_rc is None or now-self.last_rc > c['rc_timeout']:
                self.estop = True
            angles = [round(v) for v in c['centers_deg']]
        self.write(angles)

    def write(self, angles):
        if self.bus is not None:
            for channel, angle in zip(self.cfg['servo_channels'], angles):
                self.bus.write_byte_data(self.cfg['i2c_address'], channel, angle)
        self.state.publish(Int32MultiArray(data=angles))

    def destroy_node(self):
        try:
            self.write([round(v) for v in self.cfg['centers_deg']])
        finally:
            if self.bus is not None:
                self.bus.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
