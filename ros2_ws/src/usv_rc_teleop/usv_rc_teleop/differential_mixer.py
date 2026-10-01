#!/usr/bin/env python3
"""Mix CRSF throttle/steering channels into left/right propulsion demands."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from usv_interfaces.msg import CrsfChannels, PropulsionCommand

from .channel3_esc1_mapper import centered_us_to_percent


def mix_differential(throttle: float, steering: float) -> tuple[float, float]:
    """Constrained differential mix.

    While translating, both motors retain the throttle sign and the inside motor
    is reduced as far as zero. At zero throttle, steering commands an in-place
    turn with equal and opposite motor demands.
    """
    throttle = max(-100.0, min(100.0, float(throttle)))
    steering = max(-100.0, min(100.0, float(steering)))
    if throttle == 0.0:
        return steering, -steering

    turn = steering / 100.0
    # Reverse the differential relationship while backing so that a positive
    # steering command still requests the same yaw direction.
    effective_turn = turn if throttle > 0.0 else -turn
    left = throttle
    right = throttle
    if effective_turn > 0.0:
        right *= 1.0 - effective_turn
    elif effective_turn < 0.0:
        left *= 1.0 + effective_turn
    return left, right


class DifferentialMixer(Node):
    def __init__(self) -> None:
        super().__init__("rc_differential_mixer")
        self.declare_parameter("throttle_channel", 2)
        self.declare_parameter("steering_channel", 4)
        self.declare_parameter("minimum_us", 1000)
        self.declare_parameter("center_us", 1500)
        self.declare_parameter("maximum_us", 2000)
        self.declare_parameter("throttle_deadband_us", 20)
        self.declare_parameter("steering_deadband_us", 20)
        self.declare_parameter("throttle_reversed", False)
        self.declare_parameter("steering_reversed", False)
        self.declare_parameter("left_motor_reversed", False)
        self.declare_parameter("right_motor_reversed", False)
        self.declare_parameter("maximum_message_age", 0.2)

        self._throttle_index = int(self.get_parameter("throttle_channel").value) - 1
        self._steering_index = int(self.get_parameter("steering_channel").value) - 1
        self._minimum_us = int(self.get_parameter("minimum_us").value)
        self._center_us = int(self.get_parameter("center_us").value)
        self._maximum_us = int(self.get_parameter("maximum_us").value)
        self._throttle_deadband_us = int(
            self.get_parameter("throttle_deadband_us").value
        )
        self._steering_deadband_us = int(
            self.get_parameter("steering_deadband_us").value
        )
        self._throttle_sign = (
            -1.0 if bool(self.get_parameter("throttle_reversed").value) else 1.0
        )
        self._steering_sign = (
            -1.0 if bool(self.get_parameter("steering_reversed").value) else 1.0
        )
        self._left_sign = (
            -1.0 if bool(self.get_parameter("left_motor_reversed").value) else 1.0
        )
        self._right_sign = (
            -1.0 if bool(self.get_parameter("right_motor_reversed").value) else 1.0
        )
        self._maximum_message_age = float(
            self.get_parameter("maximum_message_age").value
        )

        if not 0 <= self._throttle_index < 16 or not 0 <= self._steering_index < 16:
            raise ValueError("throttle_channel and steering_channel must be in 1..16")
        if self._throttle_index == self._steering_index:
            raise ValueError("throttle_channel and steering_channel must differ")
        if not math.isfinite(self._maximum_message_age) or self._maximum_message_age <= 0:
            raise ValueError("maximum_message_age must be positive")
        for deadband in (self._throttle_deadband_us, self._steering_deadband_us):
            centered_us_to_percent(
                self._center_us,
                self._minimum_us,
                self._center_us,
                self._maximum_us,
                deadband,
            )

        self._publisher = self.create_publisher(
            PropulsionCommand, "/propulsion/manual_command", 1
        )
        self.create_subscription(
            CrsfChannels, "/rc/channels", self._channels_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Differential control: CH{self._throttle_index + 1}=throttle, "
            f"CH{self._steering_index + 1}=steering; outputs 1=left, 2=right"
        )

    def _decode_channel(self, pulse_us: int, deadband_us: int) -> float:
        return centered_us_to_percent(
            pulse_us, self._minimum_us, self._center_us,
            self._maximum_us, deadband_us,
        )

    def _channels_callback(self, message: CrsfChannels) -> None:
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (now - stamp).nanoseconds / 1.0e9
        valid = message.connected and -0.05 <= age <= self._maximum_message_age
        left = 0.0
        right = 0.0
        if valid:
            throttle = self._throttle_sign * self._decode_channel(
                int(message.microseconds[self._throttle_index]),
                self._throttle_deadband_us,
            )
            steering = self._steering_sign * self._decode_channel(
                int(message.microseconds[self._steering_index]),
                self._steering_deadband_us,
            )
            left, right = mix_differential(throttle, steering)
            left *= self._left_sign
            right *= self._right_sign

        command = PropulsionCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "rc_receiver"
        command.channel_1_percent = left
        command.channel_2_percent = right
        command.source = "rc_differential" if valid else "rc_failsafe"
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DifferentialMixer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
