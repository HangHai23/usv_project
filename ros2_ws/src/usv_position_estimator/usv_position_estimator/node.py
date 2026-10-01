"""ROS wrapper around the planar delayed filter; no motor control or TF output."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from usv_interfaces.msg import GlobalPosition
from .filter import DelayedPositionFilter, quaternion_matrix


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class PositionEstimator(Node):
    def __init__(self):
        super().__init__('usv_position_estimator')
        defaults = dict(position_topic='/usv/global_position', imu_topic='/imu/data_raw',
                        output_topic='/usv/position_prediction', global_frame='map',
                        child_frame='imu_link', imu_world_to_map_yaw_deg=0.0,
                        acceleration_includes_gravity=True, gravity=9.80665,
                        accelerometer_bias_xyz=[0.0,0.0,0.0], history_seconds=3.0,
                        position_std_m=0.10, acceleration_noise_std=0.35,
                        bias_walk_std=0.02, innovation_gate_squared=25.0,
                        position_timeout=1.0, imu_timeout=0.20, maximum_imu_gap=0.20,
                        future_tolerance=0.05, output_rate=100.0, water_z=0.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in ['gravity','history_seconds','position_std_m','acceleration_noise_std',
                    'innovation_gate_squared','position_timeout','imu_timeout',
                    'maximum_imu_gap','output_rate']:
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ['bias_walk_std','future_tolerance']:
            if not math.isfinite(self.cfg[key]) or self.cfg[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        self.bias = np.asarray(self.cfg['accelerometer_bias_xyz'], dtype=float)
        if self.bias.shape != (3,) or not np.all(np.isfinite(self.bias)):
            raise ValueError('accelerometer_bias_xyz must have 3 finite numbers')
        if not all(math.isfinite(self.cfg[k]) for k in ['water_z','imu_world_to_map_yaw_deg']):
            raise ValueError('invalid frame alignment or water_z')
        yaw = math.radians(self.cfg['imu_world_to_map_yaw_deg'])
        self.alignment_q = (0., 0., math.sin(yaw/2), math.cos(yaw/2))
        self.alignment = quaternion_matrix(self.alignment_q)
        self._reset()
        self.last_clock = None
        self.last_warning = -float('inf')
        self.publisher = self.create_publisher(Odometry, self.cfg['output_topic'], 10)
        self.valid_pub = self.create_publisher(Bool, '/usv/position_prediction_valid', 10)
        self.create_subscription(Imu, self.cfg['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(GlobalPosition, self.cfg['position_topic'], self.on_position, 10)
        self.create_timer(1/self.cfg['output_rate'], self.publish)
        self.get_logger().warning('Planar predictor: calibrate IMU world-to-map yaw before navigation; no control output')

    def _reset(self):
        c = self.cfg
        self.filter = DelayedPositionFilter(c['history_seconds'], c['position_std_m'],
                                           c['acceleration_noise_std'], c['bias_walk_std'],
                                           c['innovation_gate_squared'])
        self.last_imu = None
        self.imu_message = None
        self.pending = []
        self.seen = set()
        self.last_published = None

    def clock_now(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.last_clock is not None and now < self.last_clock:
            self._reset()
        self.last_clock = now
        return now

    def warn(self, text):
        now = time.monotonic()
        if now-self.last_warning > 5:
            self.get_logger().warning(text)
            self.last_warning = now

    def on_imu(self, msg):
        now = self.clock_now()
        t = seconds(msg.header.stamp)
        if msg.header.frame_id != self.cfg['child_frame']:
            self.warn('IMU frame differs from child_frame; mounting transform must be configured upstream')
            return
        if t <= 0 or t > now+self.cfg['future_tolerance'] or now-t > self.cfg['imu_timeout']:
            self.warn('Rejected stale/future IMU timestamp')
            return
        if self.last_imu is not None:
            if t <= self.last_imu:
                return
            if t-self.last_imu > self.cfg['maximum_imu_gap']:
                self._reset()
                self.warn('IMU gap: reset filter, awaiting new global position')
        if msg.orientation_covariance[0] == -1 or msg.linear_acceleration_covariance[0] == -1:
            self.warn('IMU orientation/acceleration unavailable')
            return
        q = msg.orientation
        raw = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        if not np.all(np.isfinite(raw)) or not np.all(np.isfinite(gyro)):
            return
        try:
            self.rotation = self.alignment@quaternion_matrix([q.x,q.y,q.z,q.w])
        except ValueError:
            return
        acceleration = self.rotation@(raw-self.bias)
        if self.cfg['acceleration_includes_gravity']:
            acceleration -= [0.,0.,self.cfg['gravity']]
        self.filter.add(t, 'imu', acceleration[:2])
        self.last_imu, self.imu_message = t, msg
        self.drain_positions()

    def on_position(self, msg):
        now = self.clock_now()
        t = seconds(msg.header.stamp)
        if msg.header.frame_id != self.cfg['global_frame']:
            self.warn('Rejected UDP position: frame_id does not match global_frame')
            return
        if t <= 0 or t > now+self.cfg['future_tolerance'] or now-t > self.cfg['history_seconds']:
            self.warn('Rejected UDP position: timestamp outside history/future limits')
            return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key in self.seen:
            return
        if not all(math.isfinite(v) for v in [msg.x,msg.y,msg.z]):
            return
        self.seen.add(key)
        self.seen = {k for k in self.seen if k[0]+k[1]*1e-9 >= now-self.cfg['history_seconds']}
        self.pending.append((t, [msg.x,msg.y]))
        self.pending.sort(key=lambda item: item[0])
        self.pending = self.pending[-100:]
        self.drain_positions()

    def drain_positions(self):
        while self.pending and self.last_imu is not None and self.pending[0][0] <= self.last_imu:
            t, position = self.pending.pop(0)
            if not self.filter.add(t, 'position', position):
                self.warn('Position rejected: older than IMU history or innovation gate exceeded')

    def publish(self):
        now = self.clock_now()
        fresh_imu = self.last_imu is not None and now-self.last_imu <= self.cfg['imu_timeout']
        initialized = self.filter.last_position_time is not None
        valid = fresh_imu and initialized and now-self.filter.last_position_time <= self.cfg['position_timeout']
        self.valid_pub.publish(Bool(data=bool(valid)))
        if not fresh_imu or not initialized or self.last_published == self.last_imu:
            return
        _, x, p, _ = self.filter.current
        msg = Odometry()
        msg.header.stamp = self.imu_message.header.stamp
        msg.header.frame_id = self.cfg['global_frame']
        msg.child_frame_id = self.cfg['child_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x[0]), float(x[1])
        msg.pose.pose.position.z = self.cfg['water_z']
        # Compose map alignment quaternion with measured IMU orientation.
        q = self.imu_message.orientation
        a = self.alignment_q
        msg.pose.pose.orientation.x = a[3]*q.x-a[2]*q.y
        msg.pose.pose.orientation.y = a[3]*q.y+a[2]*q.x
        msg.pose.pose.orientation.z = a[3]*q.z+a[2]*q.w
        msg.pose.pose.orientation.w = a[3]*q.w-a[2]*q.z
        norm = math.sqrt(sum(getattr(msg.pose.pose.orientation,k)**2 for k in ['x','y','z','w']))
        for k in ['x','y','z','w']:
            setattr(msg.pose.pose.orientation,k,getattr(msg.pose.pose.orientation,k)/norm)
        velocity = self.rotation.T@np.array([x[2],x[3],0.])
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float,velocity)
        msg.twist.twist.angular = self.imu_message.angular_velocity
        pose_cov = np.eye(6)*1e6
        pose_cov[:2,:2] = p[:2,:2]
        attitude_cov = np.array(self.imu_message.orientation_covariance).reshape(3,3)
        if np.all(np.isfinite(attitude_cov)) and np.all(np.diag(attitude_cov)>0):
            pose_cov[3:,3:] = attitude_cov
        twist_cov = np.eye(6)*1e6
        v_cov = np.eye(3)*1e6
        v_cov[:2,:2] = p[2:4,2:4]
        twist_cov[:3,:3] = self.rotation.T@v_cov@self.rotation
        msg.pose.covariance = pose_cov.ravel().tolist()
        msg.twist.covariance = twist_cov.ravel().tolist()
        self.publisher.publish(msg)
        self.last_published = self.last_imu


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PositionEstimator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
