"""Capture a stationary-water IMU reference without modifying hardware calibration."""
import argparse
import json
import math
import statistics
import time
from pathlib import Path
from datetime import datetime
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', default='~/Downloads/usv_navigation_logs/imu_reference')
    args = parser.parse_args()
    directory = Path(args.output).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    rows = []
    rclpy.init()
    node = rclpy.create_node('navigation_imu_reference')
    stream = (directory / 'imu.jsonl').open('w')
    def receive(msg):
        t = time.monotonic()
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        if not all(math.isfinite(v) for v in values):
            return
        rows.append([t, *values])
        stream.write(json.dumps(dict(monotonic=t, unix_ns=time.time_ns(), message=message_to_ordereddict(msg))) + '\n')
    node.create_subscription(Imu, '/imu/data_raw', receive, qos_profile_sensor_data)
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    stream.close()
    result = dict(samples=len(rows), note='Water motion is included; do not automatically subtract mean as gyro bias.')
    if len(rows) > 1:
        result['frequency_hz'] = (len(rows)-1)/(rows[-1][0]-rows[0][0])
        for i, name in enumerate(['gx', 'gy', 'gz', 'ax', 'ay', 'az'], 1):
            values = [r[i] for r in rows]
            result[name] = dict(mean=statistics.mean(values), std=statistics.pstdev(values),
                                min=min(values), max=max(values))
        result['yaw_drift_integral_rad'] = sum((b[0]-a[0])*a[3] for a,b in zip(rows, rows[1:]))
    (directory / 'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(directory=str(directory), **result), indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
