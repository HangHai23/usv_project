#!/usr/bin/env python3
"""Run one motor at one pulse width, stop it, then save the observed result."""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from usv_interfaces.msg import CrsfChannels, PropulsionCommand, PwmOutputState
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", type=int, choices=(1, 2), required=True)
    parser.add_argument("--pulse-us", type=int, required=True)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "Downloads" / "usv_start_threshold",
    )
    args = parser.parse_args()
    if not 1000 <= args.pulse_us <= 2000:
        parser.error("--pulse-us must be in 1000..2000")
    if not math.isfinite(args.duration) or not 0.5 <= args.duration <= 10.0:
        parser.error("--duration must be in 0.5..10 seconds")
    return args


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output = args.output_dir / f"channel{args.channel}_{args.pulse_us}us_{stamp}.yaml"

    rclpy.init()
    node = Node("esc_start_threshold_test")
    publisher = node.create_publisher(
        PropulsionCommand, "/esc_start_test/command", 10
    )
    state = {
        "imu": None,
        "imu_time": 0.0,
        "pwm": None,
        "pwm_time": 0.0,
        "rc": None,
        "rc_time": 0.0,
    }

    def update(name, message):
        state[name] = message
        state[name + "_time"] = time.monotonic()

    node.create_subscription(
        Imu,
        "/imu/data_raw",
        lambda message: update("imu", message),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        PwmOutputState,
        "/actuators/pwm_state",
        lambda message: update("pwm", message),
        10,
    )
    node.create_subscription(
        CrsfChannels,
        "/rc/channels",
        lambda message: update("rc", message),
        qos_profile_sensor_data,
    )

    def publish(left_percent=0.0, right_percent=0.0):
        message = PropulsionCommand()
        message.header.stamp = node.get_clock().now().to_msg()
        message.channel_1_percent = left_percent
        message.channel_2_percent = right_percent
        message.source = "esc_start_threshold"
        publisher.publish(message)

    record = {
        "schema_version": 1,
        "test": "single_motor_start_threshold",
        "started_at": datetime.now().astimezone().isoformat(),
        "channel": args.channel,
        "requested_pulse_us": args.pulse_us,
        "duration_s": args.duration,
        "status": "aborted",
        "reason": "unexpected_exit",
        "observed_result": "not_recorded",
        "actual_pulse_us": None,
        "dry_run": None,
        "imu_yaw_rate": {"mean_rad_s": None, "minimum_rad_s": None, "maximum_rad_s": None},
    }
    yaw_samples = []
    started = None
    wait_started = time.monotonic()
    failure = None
    try:
        print("等待IMU、PWM执行器和遥控器；CH5必须处于低位。")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            imu, pwm, rc = state["imu"], state["pwm"], state["rc"]
            fresh = (
                imu is not None
                and now - state["imu_time"] < 0.2
                and pwm is not None
                and now - state["pwm_time"] < 0.3
            )
            rc_ready = (
                pwm is not None
                and (
                    pwm.dry_run
                    or (
                        rc is not None
                        and rc.connected
                        and now - state["rc_time"] < 0.3
                        and rc.microseconds[4] < 1500
                    )
                )
            )
            ready = fresh and rc_ready and not (
                pwm is not None and pwm.emergency_stop_active
            )
            if not ready:
                publish()
                if started is not None:
                    failure = "IMU、执行器、遥控器或CH5状态在试验中失效"
                    break
                if now - wait_started > 10.0:
                    failure = "等待设备就绪超时"
                    break
                continue

            if started is None:
                started = now
                record["dry_run"] = bool(pwm.dry_run)
                print(
                    f"开始：通道{args.channel}={args.pulse_us}us，"
                    f"另一通道=1000us，持续{args.duration:.1f}秒。"
                )
            if now - started >= args.duration:
                record["status"] = "completed"
                record["reason"] = "duration_reached"
                break

            percent = (args.pulse_us - 1000) / 10.0
            publish(percent if args.channel == 1 else 0.0,
                    percent if args.channel == 2 else 0.0)
            yaw_samples.append(float(imu.angular_velocity.z))
            record["actual_pulse_us"] = int(pwm.pulse_width_us[args.channel - 1])
    except KeyboardInterrupt:
        failure = "keyboard_interrupt"
    finally:
        for _ in range(10):
            if not rclpy.ok():
                break
            publish()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failure:
        record["reason"] = failure
    if yaw_samples:
        record["imu_yaw_rate"] = {
            "mean_rad_s": sum(yaw_samples) / len(yaw_samples),
            "minimum_rad_s": min(yaw_samples),
            "maximum_rad_s": max(yaw_samples),
        }

    if record["status"] == "completed" and not record["dry_run"]:
        print("动力已恢复1000us。请输入实际观察结果：")
        print("  1 = 稳定启动；2 = 仅抖动/偶尔启动；3 = 完全未启动")
        try:
            answer = input("结果 [1/2/3]：").strip()
        except EOFError:
            answer = ""
        record["observed_result"] = {
            "1": "stable_start",
            "2": "unstable_or_twitch",
            "3": "no_start",
        }.get(answer, "invalid_or_skipped")
    elif record["dry_run"]:
        record["observed_result"] = "dry_run_only"

    record["finished_at"] = datetime.now().astimezone().isoformat()
    with output.open("x", encoding="utf-8") as file:
        yaml.safe_dump(record, file, allow_unicode=True, sort_keys=False)
    print(f"结果文件：{output}")


if __name__ == "__main__":
    main()
