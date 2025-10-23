#!/usr/bin/env python3
"""
Real-time Bluetooth Low Energy capture and plotting for the ADXL366 streamer.

This script connects to the Zephyr firmware using the custom acceleration
service, subscribes to the notification characteristic, and renders a live
plot of the X/Y/Z acceleration components.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import logging
import signal
import struct
import sys
import time

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - dependency hint
    print(
        "matplotlib is required to run this script. "
        "Install it with `pip install matplotlib`.",
        file=sys.stderr,
    )
    sys.exit(1)

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:  # pragma: no cover - dependency hint
    print(
        "bleak is required to run this script. "
        "Install it with `pip install bleak`.",
        file=sys.stderr,
    )
    sys.exit(1)

ACCEL_SERVICE_UUID = "045b2c8b-45f0-4c69-9d23-5a4cedf0d079"
ACCEL_MEASUREMENT_UUID = "045b2c8c-45f0-4c69-9d23-5a4cedf0d079"
BYTES_PER_SAMPLE = 12


class AccelerometerPlotter:
    """Maintains a scrolling window of acceleration data and renders the plots."""

    def __init__(self, window_seconds: float, max_rate: float) -> None:
        self.window = max(window_seconds, 1.0)
        history_length = max(1, int(self.window * max_rate))
        self._history: deque[tuple[float, float, float, float]] = deque(
            maxlen=history_length
        )

        plt.ion()
        self.fig, self.axes = plt.subplots(3, 1, sharex=True)
        self.lines = []

        labels = ("X", "Y", "Z")
        for axis, label in zip(self.axes, labels):
            line, = axis.plot([], [], label=f"{label} (m/s²)", linewidth=1.5)
            axis.set_ylabel(f"{label} (m/s²)")
            axis.grid(True, which="both", linestyle="--", linewidth=0.5)
            axis.legend(loc="upper left")
            self.lines.append(line)
            axis.axhline(0.0, color="0.4", linewidth=0.6, linestyle=":")

        self.axes[-1].set_xlabel("Time (s)")
        self.fig.tight_layout()
        plt.show()

    def add_sample(self, timestamp: float, x: float, y: float, z: float) -> None:
        self._history.append((timestamp, x, y, z))

    def is_open(self) -> bool:
        return plt.fignum_exists(self.fig.number)

    def render(self) -> None:
        if not self._history:
            return

        latest_time = self._history[-1][0]
        cutoff = latest_time - self.window
        while len(self._history) > 1 and self._history[0][0] < cutoff:
            self._history.popleft()

        times = [sample[0] - self._history[0][0] for sample in self._history]
        axis_samples = list(zip(*(sample[1:] for sample in self._history)))

        for axis, line, values in zip(self.axes, self.lines, axis_samples):
            line.set_data(times, values)

            v_min = min(values)
            v_max = max(values)
            span = max(1e-3, v_max - v_min)
            padding = max(0.25, span * 0.15)
            axis.set_ylim(v_min - padding, v_max + padding)

        x_max = max(self.window, times[-1] if times else self.window)
        for axis in self.axes:
            axis.set_xlim(max(0.0, x_max - self.window), x_max)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)


class AccelerometerStream:
    """Handles BLE notifications and forwards samples to the plotter."""

    def __init__(self, plotter: AccelerometerPlotter) -> None:
        self._plotter = plotter
        self._start = time.monotonic()
        self.sample_count = 0

    def handle_notification(self, _handle: int, data: bytearray) -> None:
        if len(data) < BYTES_PER_SAMPLE:
            logging.debug("Ignored short notification (%d bytes)", len(data))
            return

        x, y, z = struct.unpack("<fff", data[:BYTES_PER_SAMPLE])
        timestamp = time.monotonic() - self._start
        self.sample_count += 1
        self._plotter.add_sample(timestamp, x, y, z)


async def find_device_by_name(name: str, timeout: float):
    """Scan for a BLE device whose name contains the given substring."""
    target = name.lower()

    if hasattr(BleakScanner, "find_device_by_filter"):
        def _match(device, advertisement):
            local_name = ""
            if advertisement and advertisement.local_name:
                local_name = advertisement.local_name
            elif device and device.name:
                local_name = device.name
            return local_name and target in local_name.lower()

        return await BleakScanner.find_device_by_filter(_match, timeout=timeout)

    end_time = time.monotonic() + timeout
    while time.monotonic() < end_time:
        remaining = end_time - time.monotonic()
        scan_timeout = max(0.5, min(3.0, remaining))
        devices = await BleakScanner.discover(timeout=scan_timeout)
        for device in devices:
            if device.name and target in device.name.lower():
                return device
    return None


async def resolve_target(address: str | None, name: str, timeout: float):
    """Determine the BLE address to connect to, scanning if necessary."""
    if address:
        logging.info("Using provided address %s", address)
        return address

    logging.info("Scanning for device name containing '%s'...", name)
    device = await find_device_by_name(name, timeout)
    if not device:
        raise RuntimeError(f"Unable to find a device named like '{name}'.")

    logging.info(
        "Found device %s (%s)", device.name or "Unnamed", device.address
    )
    return device.address


async def plot_loop(plotter: AccelerometerPlotter, stop_event: asyncio.Event, interval: float) -> None:
    """Refresh the matplotlib window until the stop event is set."""
    try:
        while not stop_event.is_set():
            plotter.render()
            if not plotter.is_open():
                stop_event.set()
                break
            await asyncio.sleep(interval)
    finally:
        plt.ioff()


async def stream_and_plot(args) -> None:
    address = await resolve_target(args.address, args.name, args.scan_timeout)
    plotter = AccelerometerPlotter(args.window, args.max_rate)
    stream = AccelerometerStream(plotter)
    stop_event = asyncio.Event()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, AttributeError):
            signal.signal(sig, lambda *_: stop_event.set())

    try:
        async with BleakClient(address) as client:
            logging.info("Connected to %s", address)
            await client.start_notify(ACCEL_MEASUREMENT_UUID, stream.handle_notification)
            logging.info("Subscribed to acceleration notifications.")

            plot_task = asyncio.create_task(
                plot_loop(plotter, stop_event, args.update_interval)
            )

            try:
                if args.duration:
                    try:
                        await asyncio.wait_for(stop_event.wait(), timeout=args.duration)
                        if not stop_event.is_set():
                            logging.info("Requested duration elapsed; stopping capture.")
                            stop_event.set()
                    except asyncio.TimeoutError:
                        logging.info("Requested duration elapsed; stopping capture.")
                        stop_event.set()
                else:
                    await stop_event.wait()
            finally:
                await client.stop_notify(ACCEL_MEASUREMENT_UUID)
                stop_event.set()
                await plot_task

    except BleakError as exc:
        raise RuntimeError(f"BLE failure: {exc}") from exc


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture and plot ADXL366 acceleration data over Bluetooth LE."
    )
    parser.add_argument(
        "--address",
        help="BLE address of the device. If omitted, the script scans by name.",
    )
    parser.add_argument(
        "--name",
        default="ADXL366 Streamer",
        help="Substring of the advertised device name to match during scanning.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        help="Length of the rolling plot window in seconds (default: 10).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Stop automatically after the given number of seconds.",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=15.0,
        help="How long to scan for the device before giving up (seconds).",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=120.0,
        help="Expected maximum sample rate in samples/second (default: 120).",
    )
    parser.add_argument(
        "--update-interval",
        type=float,
        default=0.05,
        help="Plot refresh interval in seconds (default: 0.05).",
    )
    parser.add_argument(
        "--log",
        default="info",
        choices=("debug", "info", "warning", "error"),
        help="Log level for console output (default: info).",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log.upper()),
        format="[%(levelname)s] %(message)s",
    )

    try:
        asyncio.run(stream_and_plot(args))
    except KeyboardInterrupt:
        logging.info("Interrupted, shutting down.")
    except Exception as exc:  # pragma: no cover - surface errors to user
        logging.error("%s", exc)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
