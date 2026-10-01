#!/usr/bin/env python3
"""Map centered CRSF channel 3 input to signed ESC channel 1 demand."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from usv_interfaces.msg import CrsfChannels, PropulsionCommand


def centered_us_to_percent(
    pulse_us: int,
    minimum_us: int,
    center_us: int,
    maximum_us: int,
    center_deadband_us: int,
) -> float:
    if not minimum_us < center_us < maximum_us:
        raise ValueError("center_us must be between minimum_us and maximum_us")
    if center_deadband_us < 0 or center_deadband_us >= min(
        center_us - minimum_us, maximum_us - center_us
    ):
        raise ValueError("center_deadband_us is outside the usable input range")
    lower_center = center_us - center_deadband_us
    upper_center = center_us + center_deadband_us
    if lower_center <= pulse_us <= upper_center:
        return 0.0
    if pulse_us <= minimum_us:
        return -100.0
    if pulse_us >= maximum_us:
        return 100.0
    if pulse_us < lower_center:
        return -100.0 * (lower_center - pulse_us) / (lower_center - minimum_us)
    return 100.0 * (pulse_us - upper_center) / (maximum_us - upper_center)


class Channel3Esc1Mapper(Node):
    def __init__(self) -> None:
        super().__init__("channel3_esc1_mapper")
        self.declare_parameter("input_channel", 3)
        self.declare_parameter("minimum_us", 1000)
        self.declare_parameter("center_us", 1500)
        self.declare_parameter("maximum_us", 2000)
        self.declare_parameter("center_deadband_us", 0)
        self.declare_parameter("maximum_message_age", 0.2)

        self._channel_index = int(self.get_parameter("input_channel").value) - 1
        self._minimum_us = int(self.get_parameter("minimum_us").value)
        self._center_us = int(self.get_parameter("center_us").value)
        self._maximum_us = int(self.get_parameter("maximum_us").value)
        self._center_deadband_us = int(
            self.get_parameter("center_deadband_us").value
        )
        self._maximum_message_age = float(
            self.get_parameter("maximum_message_age").value
        )
        if not 0 <= self._channel_index < 16:
            raise ValueError("input_channel must be in the range 1..16")
        if not math.isfinite(self._maximum_message_age) or self._maximum_message_age <= 0:
            raise ValueError("maximum_message_age must be positive")
        centered_us_to_percent(
            self._center_us,
            self._minimum_us,
            self._center_us,
            self._maximum_us,
            self._center_deadband_us,
        )

        self._publisher = self.create_publisher(
            PropulsionCommand, "/propulsion/command", 1
        )
        self.create_subscription(
            CrsfChannels,
            "/rc/channels",
            self._channels_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Mapping RC channel {self._channel_index + 1} "
            f"({self._minimum_us}..{self._center_us}..{self._maximum_us} us) "
            "to ESC1 signed demand"
        )

    def _channels_callback(self, message: CrsfChannels) -> None:
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        age = (now - stamp).nanoseconds / 1.0e9
        valid = message.connected and -0.05 <= age <= self._maximum_message_age
        percent = 0.0
        if valid:
            percent = centered_us_to_percent(
                int(message.microseconds[self._channel_index]),
                self._minimum_us,
                self._center_us,
                self._maximum_us,
                self._center_deadband_us,
            )

        command = PropulsionCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "rc_receiver"
        command.channel_1_percent = percent
        command.channel_2_percent = 0.0
        command.source = "rc_channel3_test" if valid else "rc_failsafe"
        self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Channel3Esc1Mapper()
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
