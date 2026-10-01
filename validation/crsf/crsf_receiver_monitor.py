#!/usr/bin/env python3
"""实时接收并解析 ExpressLRS 接收机输出的 CRSF 数据。"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import Optional

import serial


CRSF_BAUDRATE = 420000
CRSF_MAX_FRAME_SIZE = 64
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MID = 992
CRSF_CHANNEL_MAX = 1811
CRSF_US_MIN = 988
CRSF_US_MAX = 2012


def crc8_dvb_s2(data: bytes) -> int:
    """Calculate the CRC-8/DVB-S2 checksum used by CRSF."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class CrsfStreamParser:
    """Extract checksum-valid CRSF frames from an arbitrary byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buffer.extend(data)
        frames: list[bytes] = []

        while len(self._buffer) >= 4:
            payload_length = self._buffer[1]
            if payload_length < 2 or payload_length > CRSF_MAX_FRAME_SIZE - 2:
                del self._buffer[0]
                self.discarded_bytes += 1
                continue

            frame_length = payload_length + 2
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                del self._buffer[0]
                self.crc_errors += 1
                self.discarded_bytes += 1
                continue

            frames.append(frame)
            del self._buffer[:frame_length]

        return frames


def decode_channels(payload: bytes) -> list[int]:
    if len(payload) != 22:
        raise ValueError(f"RC channel payload must contain 22 bytes, got {len(payload)}")
    packed = int.from_bytes(payload, byteorder="little")
    return [(packed >> (channel * 11)) & 0x07FF for channel in range(CRSF_CHANNEL_COUNT)]


def channel_to_microseconds(raw_value: int) -> int:
    span_in = CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN
    span_out = CRSF_US_MAX - CRSF_US_MIN
    return round(CRSF_US_MIN + (raw_value - CRSF_CHANNEL_MIN) * span_out / span_in)


def signed_byte(value: int) -> int:
    return value - 256 if value >= 128 else value


@dataclass
class LinkStatistics:
    uplink_rssi_1: int
    uplink_rssi_2: int
    uplink_quality: int
    uplink_snr: int
    active_antenna: int
    rf_mode: int
    tx_power_code: int
    downlink_rssi: int
    downlink_quality: int
    downlink_snr: int

    @classmethod
    def from_payload(cls, payload: bytes) -> "LinkStatistics":
        if len(payload) < 10:
            raise ValueError("Link statistics payload is shorter than 10 bytes")
        return cls(
            uplink_rssi_1=-payload[0],
            uplink_rssi_2=-payload[1],
            uplink_quality=payload[2],
            uplink_snr=signed_byte(payload[3]),
            active_antenna=payload[4],
            rf_mode=payload[5],
            tx_power_code=payload[6],
            downlink_rssi=-payload[7],
            downlink_quality=payload[8],
            downlink_snr=signed_byte(payload[9]),
        )


def channel_bar(value_us: int, width: int = 17) -> str:
    ratio = (value_us - CRSF_US_MIN) / (CRSF_US_MAX - CRSF_US_MIN)
    position = max(0, min(width - 1, round(ratio * (width - 1))))
    cells = ["-"] * width
    cells[width // 2] = "|"
    cells[position] = "●"
    return "".join(cells)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="实时解析 ExpressLRS/CRSF 接收机数据")
    parser.add_argument("--port", default="/dev/ttyTHS1", help="UART device (default: /dev/ttyTHS1)")
    parser.add_argument("--baudrate", type=int, default=CRSF_BAUDRATE, help="UART baud rate")
    parser.add_argument("--refresh", type=float, default=5.0, help="screen refresh rate in Hz")
    parser.add_argument("--timeout", type=float, default=0.5, help="RC frame loss warning time in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="exit after N seconds; 0 runs forever")
    parser.add_argument("--no-clear", action="store_true", help="print snapshots without clearing terminal")
    return parser


def render(
    port: str,
    baudrate: int,
    channels_raw: Optional[list[int]],
    link: Optional[LinkStatistics],
    frame_counts: Counter,
    rc_rate: float,
    age: float,
    loss_timeout: float,
    parser: CrsfStreamParser,
    clear_screen: bool,
) -> None:
    if clear_screen:
        print("\033[2J\033[H", end="")
    status = "CONNECTED" if age <= loss_timeout else "NO RC FRAMES"
    print("CRSF receiver monitor  (Ctrl+C to quit)")
    print(f"Port: {port} @ {baudrate} 8N1    Status: {status}    RC rate: {rc_rate:6.1f} Hz")
    print(
        f"Valid frames: {sum(frame_counts.values())}    "
        f"RC: {frame_counts[CRSF_FRAMETYPE_RC_CHANNELS_PACKED]}    "
        f"Link: {frame_counts[CRSF_FRAMETYPE_LINK_STATISTICS]}    "
        f"CRC/resync errors: {parser.crc_errors}"
    )
    print()

    if channels_raw is None:
        print("Waiting for CRSF RC_CHANNELS_PACKED (0x16) frames...")
    else:
        channels_us = [channel_to_microseconds(value) for value in channels_raw]
        for row in range(0, CRSF_CHANNEL_COUNT, 4):
            fields = []
            for index in range(row, row + 4):
                fields.append(
                    f"CH{index + 1:02d} {channels_us[index]:4d}us "
                    f"[{channel_bar(channels_us[index])}]"
                )
            print("  ".join(fields))
        print("Raw:", " ".join(f"{value:4d}" for value in channels_raw))

    print()
    if link is None:
        print("Link statistics: waiting for frame type 0x14...")
    else:
        print(
            "Uplink: "
            f"RSSI1={link.uplink_rssi_1:4d} dBm  "
            f"RSSI2={link.uplink_rssi_2:4d} dBm  "
            f"LQ={link.uplink_quality:3d}%  SNR={link.uplink_snr:4d} dB  "
            f"antenna={link.active_antenna}  rf_mode={link.rf_mode}  "
            f"power_code={link.tx_power_code}"
        )
        print(
            "Downlink: "
            f"RSSI={link.downlink_rssi:4d} dBm  "
            f"LQ={link.downlink_quality:3d}%  SNR={link.downlink_snr:4d} dB"
        )
    sys.stdout.flush()


def main() -> int:
    args = build_argument_parser().parse_args()
    if args.refresh <= 0 or args.timeout <= 0 or args.duration < 0:
        print("refresh/timeout must be positive and duration cannot be negative", file=sys.stderr)
        return 2

    try:
        serial_port = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
    except serial.SerialException as exc:
        print(f"Cannot open {args.port}: {exc}", file=sys.stderr)
        print("Check wiring, device path, and membership of the dialout group.", file=sys.stderr)
        return 1

    parser = CrsfStreamParser()
    counts: Counter = Counter()
    latest_channels: Optional[list[int]] = None
    latest_link: Optional[LinkStatistics] = None
    start = time.monotonic()
    last_render = start
    last_rate_time = start
    last_rate_count = 0
    rc_rate = 0.0
    last_rc_time = 0.0
    running = True

    def stop(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        serial_port.reset_input_buffer()
        while running:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break

            for frame in parser.feed(serial_port.read(1024)):
                frame_type = frame[2]
                payload = frame[3:-1]
                counts[frame_type] += 1
                if frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                    try:
                        latest_channels = decode_channels(payload)
                        last_rc_time = now
                    except ValueError:
                        pass
                elif frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
                    try:
                        latest_link = LinkStatistics.from_payload(payload)
                    except ValueError:
                        pass

            if now - last_rate_time >= 1.0:
                rc_count = counts[CRSF_FRAMETYPE_RC_CHANNELS_PACKED]
                rc_rate = (rc_count - last_rate_count) / (now - last_rate_time)
                last_rate_count = rc_count
                last_rate_time = now

            if now - last_render >= 1.0 / args.refresh:
                age = now - last_rc_time if last_rc_time else float("inf")
                render(
                    args.port,
                    args.baudrate,
                    latest_channels,
                    latest_link,
                    counts,
                    rc_rate,
                    age,
                    args.timeout,
                    parser,
                    not args.no_clear,
                )
                last_render = now
    finally:
        serial_port.close()

    print("\nCRSF monitor stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
