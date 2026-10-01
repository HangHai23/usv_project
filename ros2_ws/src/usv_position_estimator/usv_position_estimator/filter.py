"""Planar linear Kalman filter with bounded, timestamp-ordered event replay."""
from bisect import bisect_right
import numpy as np


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-6:
        raise ValueError('invalid IMU quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class DelayedPositionFilter:
    def __init__(self, history=3.0, position_std=0.10, accel_std=0.35,
                 bias_walk_std=0.02, gate=25.0):
        self.history, self.position_std = history, position_std
        self.accel_std, self.bias_walk_std, self.gate = accel_std, bias_walk_std, gate
        self.events = []
        self.base = None
        self.current = None
        self.last_position_time = None
        self.first_position_time = None
        self.sequence = 0

    def _propagate(self, snapshot, t):
        old_t, x, p, acceleration = snapshot
        dt = t-old_t
        f = np.eye(6)
        f[:2, 2:4] = np.eye(2)*dt
        f[:2, 4:6] = -np.eye(2)*dt*dt/2
        f[2:4, 4:6] = -np.eye(2)*dt
        g = np.zeros((6, 2))
        g[:2] = np.eye(2)*dt*dt/2
        g[2:4] = np.eye(2)*dt
        # Continuous acceleration noise spectral density and bias random walk.
        q = np.zeros((6, 6))
        q[:2, :2] = np.eye(2)*self.accel_std**2*dt**3/3
        q[:2, 2:4] = q[2:4, :2] = np.eye(2)*self.accel_std**2*dt**2/2
        q[2:4, 2:4] = np.eye(2)*self.accel_std**2*dt
        q[4:6, 4:6] = np.eye(2)*self.bias_walk_std**2*dt
        return (t, f@x+g@acceleration, f@p@f.T+q, acceleration.copy())

    def add(self, t, kind, values):
        values = np.asarray(values, dtype=float)
        if not np.isfinite(t) or values.shape != (2,) or not np.all(np.isfinite(values)):
            return False
        if self.base is None:
            self.base = (t, np.zeros(6), np.diag([1e6,1e6,25.,25.,0.25,0.25]), np.zeros(2))
            self.current = self.base
        if t < self.base[0]:
            return False
        if kind == 'position' and self.first_position_time is not None and t < self.first_position_time:
            return False
        self.sequence += 1
        key = (t, 0 if kind == 'imu' else 1, self.sequence)
        index = bisect_right([e['key'] for e in self.events], key)
        initial = kind == 'position' and self.first_position_time is None
        event = dict(key=key, kind=kind, value=values, accepted=None, initial=initial)
        if initial:
            self.first_position_time = t
        self.events.insert(index, event)
        snapshot = self.base if index == 0 else self.events[index-1]['snapshot']
        for e in self.events[index:]:
            snapshot = self._propagate(snapshot, e['key'][0])
            stamp, x, p, acceleration = snapshot
            if e['kind'] == 'imu':
                acceleration = e['value'].copy()
            elif e['initial']:
                # Absolute coordinates can be far from zero (e.g. projected UTM).
                # Initialize the origin without interpreting it as a velocity jump.
                x = x.copy()
                x[:2] = e['value']
                x[2:4] = 0.0
                p = p.copy()
                p[:2, :] = 0.0
                p[:, :2] = 0.0
                p[:2, :2] = np.eye(2)*self.position_std**2
                e['accepted'] = True
                self.last_position_time = max(stamp, self.last_position_time or stamp)
            else:
                innovation = e['value']-x[:2]
                s = p[:2, :2]+np.eye(2)*self.position_std**2
                if e['accepted'] is None:
                    e['accepted'] = float(innovation@np.linalg.solve(s, innovation)) <= self.gate
                if e['accepted']:
                    k = np.linalg.solve(s, p[:2, :]).T
                    x = x+k@innovation
                    a = np.eye(6)
                    a[:, :2] -= k
                    p = a@p@a.T+k@(np.eye(2)*self.position_std**2)@k.T
                    self.last_position_time = max(stamp, self.last_position_time or stamp)
            snapshot = (stamp, x, (p+p.T)/2, acceleration)
            e['snapshot'] = snapshot
        self.current = snapshot
        cutoff = self.current[0]-self.history
        count = 0
        for e in self.events:
            if e['key'][0] >= cutoff:
                break
            self.base = e['snapshot']
            count += 1
        if count:
            del self.events[:count]
        return kind == 'imu' or bool(event['accepted'])
