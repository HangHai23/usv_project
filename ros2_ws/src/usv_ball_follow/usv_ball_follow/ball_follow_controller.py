#!/usr/bin/env python3
"""Cascaded bearing/yaw-rate controller publishing automatic propulsion."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from usv_interfaces.msg import BallTargetState, PropulsionCommand

from .control_math import (
    clamp,
    desired_yaw_rate,
    mix_same_direction_or_pivot,
    scheduled_throttle,
    slew,
)


class BallFollowController(Node):
    def __init__(self) -> None:
        super().__init__("ball_follow_controller")
        self.declare_parameter("target_topic", "/ball/target_state")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/propulsion/automatic_command")
        self.declare_parameter("bearing_deadband_rad", 0.035)
        self.declare_parameter("bearing_kp", 2.0)
        self.declare_parameter("maximum_yaw_rate_rad_s", 0.70)
        self.declare_parameter("yaw_rate_kp", 28.0)
        self.declare_parameter("maximum_steering_percent", 25.0)
        self.declare_parameter("imu_yaw_rate_reversed", False)
        self.declare_parameter("steering_reversed", False)
        self.declare_parameter("cruise_throttle_percent", 15.0)
        self.declare_parameter("steering_slowdown_angle_rad", 0.45)
        self.declare_parameter("stop_area_ratio", 0.20)
        self.declare_parameter("predicting_throttle_scale", 0.30)
        self.declare_parameter("output_slew_rate_percent_s", 60.0)
        self.declare_parameter("target_timeout", 0.20)
        self.declare_parameter("imu_timeout", 0.20)
        self.declare_parameter("output_rate", 20.0)

        parameter_names = [
            "bearing_deadband_rad", "bearing_kp", "maximum_yaw_rate_rad_s",
            "yaw_rate_kp", "maximum_steering_percent", "cruise_throttle_percent",
            "steering_slowdown_angle_rad", "stop_area_ratio",
            "predicting_throttle_scale", "output_slew_rate_percent_s",
            "target_timeout", "imu_timeout", "output_rate",
        ]
        values = {name: float(self.get_parameter(name).value) for name in parameter_names}
        self._deadband = values["bearing_deadband_rad"]
        self._bearing_kp = values["bearing_kp"]
        self._max_yaw_rate = values["maximum_yaw_rate_rad_s"]
        self._yaw_rate_kp = values["yaw_rate_kp"]
        self._max_steering = values["maximum_steering_percent"]
        self._cruise = values["cruise_throttle_percent"]
        self._slowdown_angle = values["steering_slowdown_angle_rad"]
        self._stop_area = values["stop_area_ratio"]
        self._predicting_scale = values["predicting_throttle_scale"]
        self._slew_rate = values["output_slew_rate_percent_s"]
        self._target_timeout = values["target_timeout"]
        self._imu_timeout = values["imu_timeout"]
        self._output_rate = values["output_rate"]
        self._imu_reversed = bool(self.get_parameter("imu_yaw_rate_reversed").value)
        self._steering_reversed = bool(self.get_parameter("steering_reversed").value)
        if min(self._max_yaw_rate, self._max_steering, self._slowdown_angle,
               self._stop_area, self._slew_rate, self._target_timeout,
               self._imu_timeout, self._output_rate) <= 0.0:
            raise ValueError("controller limits, timeouts, rates and stop_area must be positive")
        if not 0.0 <= self._predicting_scale <= 1.0:
            raise ValueError("predicting_throttle_scale must be in 0..1")
        if not 0.0 <= self._cruise <= 100.0:
            raise ValueError("cruise_throttle_percent must be in 0..100")
        if not 0.0 < self._max_steering <= 100.0:
            raise ValueError("maximum_steering_percent must be in 0..100")

        self._lock = threading.Lock()
        self._target = None
        self._last_target = 0.0
        self._yaw_rate = 0.0
        self._last_imu = 0.0
        self._outputs = [0.0, 0.0]
        self._publisher = self.create_publisher(
            PropulsionCommand, str(self.get_parameter("output_topic").value), 10
        )
        self._desired_rate_publisher = self.create_publisher(
            Float32, "/ball/desired_yaw_rate", 10
        )
        self._active_publisher = self.create_publisher(Bool, "/ball/follow_active", 10)
        self.create_subscription(
            BallTargetState,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self._output_rate, self._update)
        self.get_logger().info(
            f"Ball-follow cascaded controller ready at {self._output_rate:.1f} Hz; "
            f"cruise={self._cruise:.1f}%, steering limit={self._max_steering:.1f}%"
        )

    def _on_target(self, message: BallTargetState) -> None:
        with self._lock:
            self._target = message
            self._last_target = time.monotonic()

    def _on_imu(self, message: Imu) -> None:
        yaw_rate = float(message.angular_velocity.z)
        if not math.isfinite(yaw_rate):
            return
        if self._imu_reversed:
            yaw_rate = -yaw_rate
        with self._lock:
            self._yaw_rate = yaw_rate
            self._last_imu = time.monotonic()

    def _publish_zero(self) -> None:
        self._outputs = [0.0, 0.0]
        self._publish_command(0.0, 0.0)
        self._desired_rate_publisher.publish(Float32(data=0.0))
        self._active_publisher.publish(Bool(data=False))

    def _publish_command(self, left: float, right: float) -> None:
        message = PropulsionCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.channel_1_percent = float(clamp(left, -100.0, 100.0))
        message.channel_2_percent = float(clamp(right, -100.0, 100.0))
        message.source = "ball_follow"
        self._publisher.publish(message)

    def _update(self) -> None:
        now = time.monotonic()
        with self._lock:
            target = self._target
            target_fresh = target is not None and now - self._last_target <= self._target_timeout
            imu_fresh = self._last_imu > 0.0 and now - self._last_imu <= self._imu_timeout
            yaw_rate = self._yaw_rate
        if (
            not target_fresh
            or not target.target_valid
            or not imu_fresh
            or not math.isfinite(float(target.bearing_rad))
            or not math.isfinite(float(target.area_ratio))
        ):
            self._publish_zero()
            return

        desired_rate = desired_yaw_rate(
            float(target.bearing_rad), self._deadband, self._bearing_kp, self._max_yaw_rate
        )
        steering = self._yaw_rate_kp * (desired_rate - yaw_rate)
        steering = clamp(steering, -self._max_steering, self._max_steering)
        if self._steering_reversed:
            steering = -steering
        throttle = scheduled_throttle(
            self._cruise,
            abs(float(target.bearing_rad)),
            self._slowdown_angle,
            float(target.area_ratio),
            self._stop_area,
        )
        if target.tracking_state == BallTargetState.PREDICTING:
            throttle *= self._predicting_scale
        left_target, right_target = mix_same_direction_or_pivot(throttle, steering)
        maximum_delta = self._slew_rate / self._output_rate
        left = slew(self._outputs[0], left_target, maximum_delta)
        right = slew(self._outputs[1], right_target, maximum_delta)
        self._outputs = [left, right]
        self._publish_command(left, right)
        self._desired_rate_publisher.publish(Float32(data=float(desired_rate)))
        self._active_publisher.publish(Bool(data=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
