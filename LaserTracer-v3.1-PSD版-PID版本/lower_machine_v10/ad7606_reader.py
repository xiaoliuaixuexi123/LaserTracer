"""LubanCat4 lower-machine AD7606/PDP90A acquisition utility.

Hardware mapping (40-pin header):
    AD7606 DOUTA  -> SPI0 MISO, physical pin 21
    AD7606 RD/SCLK -> SPI0 CLK, physical pin 23
    AD7606 CS      -> SPI0 CS0, physical pin 24
    AD7606 BUSY    -> GPIO3_D4, global GPIO 124, gpiochip3 line 28
    AD7606 CA+CB   -> GPIO3_A6, global GPIO 102, gpiochip3 line 6
    AD7606 RESET   -> GPIO3_B7, global GPIO 111, gpiochip3 line 15

The module deliberately imports spidev and gpiod only when hardware is opened,
so decoding and PSD normalization can be unit-tested on a development PC.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import time
from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path
from typing import Optional, Sequence, Tuple


CHANNEL_COUNT = 8
FRAME_SIZE_BYTES = CHANNEL_COUNT * 2
ADC_COUNTS_PER_FULL_SCALE = 32768.0


class Ad7606Error(RuntimeError):
    """Base exception for AD7606 acquisition failures."""


class BusyTimeoutError(Ad7606Error):
    """Raised when BUSY does not return low within the configured timeout."""


@dataclass(frozen=True)
class Ad7606Sample:
    sequence: int
    timestamp_ns: int
    raw: Tuple[int, ...]
    voltages: Tuple[float, ...]
    busy_completion_seen: bool


@dataclass(frozen=True)
class PsdReading:
    x_voltage: float
    y_voltage: float
    sum_voltage: float
    x_normalized: Optional[float]
    y_normalized: Optional[float]
    valid: bool
    reason: str


def decode_ad7606_frame(frame: bytes) -> Tuple[int, ...]:
    """Decode one DOUTA frame containing V1..V8 as big-endian int16."""
    if len(frame) != FRAME_SIZE_BYTES:
        raise ValueError(
            f"AD7606 frame must be {FRAME_SIZE_BYTES} bytes, got {len(frame)}"
        )
    return tuple(struct.unpack(">8h", frame))


def counts_to_voltages(
    raw: Sequence[int], full_scale_voltage: float = 5.0
) -> Tuple[float, ...]:
    """Convert signed ADC codes to volts for a bipolar input range."""
    if len(raw) != CHANNEL_COUNT:
        raise ValueError(f"expected {CHANNEL_COUNT} channels, got {len(raw)}")
    if not math.isfinite(full_scale_voltage) or full_scale_voltage <= 0:
        raise ValueError("full_scale_voltage must be finite and positive")
    return tuple(float(code) * full_scale_voltage / ADC_COUNTS_PER_FULL_SCALE for code in raw)


def calculate_psd_reading(
    voltages: Sequence[float],
    offsets: Sequence[float] = (0.0, 0.0, 0.0),
    sum_min_voltage: float = 0.2,
    sum_max_voltage: float = 3.8,
) -> PsdReading:
    """Normalize PDP90A X/Y by SUM and reject missing/saturated light.

    ``offsets`` contains the dark/zero offsets for X, Y and SUM. The maximum
    threshold is applied to the measured SUM voltage, while the minimum is
    applied after subtracting the dark SUM offset.
    """
    if len(voltages) < 3:
        raise ValueError("PSD calculation requires at least channels 1, 2 and 3")
    if len(offsets) != 3:
        raise ValueError("offsets must contain X, Y and SUM values")
    if sum_min_voltage < 0 or sum_max_voltage <= sum_min_voltage:
        raise ValueError("SUM thresholds must satisfy 0 <= min < max")

    x_voltage = float(voltages[0])
    y_voltage = float(voltages[1])
    sum_voltage = float(voltages[2])
    x_offset, y_offset, sum_offset = (float(value) for value in offsets)

    values = (x_voltage, y_voltage, sum_voltage, x_offset, y_offset, sum_offset)
    if not all(math.isfinite(value) for value in values):
        return PsdReading(
            x_voltage, y_voltage, sum_voltage, None, None, False, "non_finite"
        )

    corrected_sum = sum_voltage - sum_offset
    if corrected_sum < sum_min_voltage:
        return PsdReading(
            x_voltage, y_voltage, sum_voltage, None, None, False, "low_sum"
        )
    if sum_voltage > sum_max_voltage:
        return PsdReading(
            x_voltage, y_voltage, sum_voltage, None, None, False, "sum_saturated"
        )

    return PsdReading(
        x_voltage=x_voltage,
        y_voltage=y_voltage,
        sum_voltage=sum_voltage,
        x_normalized=(x_voltage - x_offset) / corrected_sum,
        y_normalized=(y_voltage - y_offset) / corrected_sum,
        valid=True,
        reason="ok",
    )


class GpiodLines:
    """Small compatibility wrapper for python-gpiod 1.x and 2.x."""

    def __init__(
        self,
        chip_path: str,
        busy_line: int,
        convst_line: int,
        reset_line: int,
    ) -> None:
        self.chip_path = chip_path
        self.busy_line = int(busy_line)
        self.convst_line = int(convst_line)
        self.reset_line = int(reset_line)
        self._version = 0
        self._gpiod = None
        self._request = None
        self._chip = None
        self._busy = None
        self._convst = None
        self._reset = None

    def open(self) -> None:
        try:
            import gpiod  # type: ignore
        except ImportError as exc:
            raise Ad7606Error(
                "python gpiod is not installed; install the distro python3-gpiod package"
            ) from exc

        self._gpiod = gpiod
        if hasattr(gpiod, "request_lines"):
            self._open_v2(gpiod)
        else:
            self._open_v1(gpiod)

    def _open_v2(self, gpiod) -> None:
        direction = gpiod.line.Direction
        value = gpiod.line.Value
        config = {
            self.busy_line: gpiod.LineSettings(
                direction=direction.INPUT,
                edge_detection=gpiod.line.Edge.BOTH,
            ),
            self.convst_line: gpiod.LineSettings(
                direction=direction.OUTPUT,
                output_value=value.ACTIVE,
            ),
            self.reset_line: gpiod.LineSettings(
                direction=direction.OUTPUT,
                output_value=value.INACTIVE,
            ),
        }
        self._request = gpiod.request_lines(
            self.chip_path,
            consumer="ad7606-reader",
            config=config,
        )
        self._version = 2

    def _open_v1(self, gpiod) -> None:
        self._chip = gpiod.Chip(self.chip_path)
        self._busy = self._chip.get_line(self.busy_line)
        self._convst = self._chip.get_line(self.convst_line)
        self._reset = self._chip.get_line(self.reset_line)
        self._busy.request(
            consumer="ad7606-reader",
            type=gpiod.LINE_REQ_EV_BOTH_EDGES,
        )
        self._convst.request(
            consumer="ad7606-reader",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[1],
        )
        self._reset.request(
            consumer="ad7606-reader",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0],
        )
        self._version = 1

    def _v2_value(self, state: bool):
        return self._gpiod.line.Value.ACTIVE if state else self._gpiod.line.Value.INACTIVE

    def set_convst(self, state: bool) -> None:
        if self._version == 2:
            self._request.set_value(self.convst_line, self._v2_value(state))
        elif self._version == 1:
            self._convst.set_value(1 if state else 0)
        else:
            raise Ad7606Error("GPIO lines are not open")

    def set_reset(self, state: bool) -> None:
        if self._version == 2:
            self._request.set_value(self.reset_line, self._v2_value(state))
        elif self._version == 1:
            self._reset.set_value(1 if state else 0)
        else:
            raise Ad7606Error("GPIO lines are not open")

    def get_busy(self) -> bool:
        if self._version == 2:
            return self._request.get_value(self.busy_line) == self._gpiod.line.Value.ACTIVE
        if self._version == 1:
            return bool(self._busy.get_value())
        raise Ad7606Error("GPIO lines are not open")

    def discard_busy_events(self) -> None:
        """Remove stale BUSY events queued before a new conversion."""
        if self._version == 2:
            while self._request.wait_edge_events(timeout=timedelta(0)):
                self._request.read_edge_events()
            return
        if self._version == 1:
            while self._busy.event_wait(sec=0, nsec=0):
                self._busy.event_read()
            return
        raise Ad7606Error("GPIO lines are not open")

    def wait_busy_falling(self, timeout_s: float) -> bool:
        """Wait for the kernel-captured BUSY falling edge.

        The falling edge is sufficient because it indicates that the new
        conversion data is ready. Kernel edge queues avoid losing the roughly
        4 us BUSY pulse while the Python thread is not scheduled.
        """
        deadline = time.monotonic() + timeout_s

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BusyTimeoutError(
                    f"AD7606 BUSY falling edge not received within {timeout_s * 1000:.3f} ms"
                )

            if self._version == 2:
                if not self._request.wait_edge_events(timeout=timedelta(seconds=remaining)):
                    continue
                for event in self._request.read_edge_events():
                    if event.event_type == event.Type.FALLING_EDGE:
                        return True
                continue

            if self._version == 1:
                seconds = int(remaining)
                nanoseconds = int((remaining - seconds) * 1_000_000_000)
                if not self._busy.event_wait(sec=seconds, nsec=nanoseconds):
                    continue
                event = self._busy.event_read()
                if event.type == self._gpiod.LineEvent.FALLING_EDGE:
                    return True
                continue

            raise Ad7606Error("GPIO lines are not open")

    def close(self) -> None:
        if self._version == 2 and self._request is not None:
            self._request.release()
        elif self._version == 1:
            for line in (self._busy, self._convst, self._reset):
                if line is not None:
                    line.release()
            if self._chip is not None:
                self._chip.close()
        self._version = 0


class SpidevPort:
    def __init__(self, bus: int, device: int, speed_hz: int, mode: int = 2) -> None:
        self.bus = int(bus)
        self.device = int(device)
        self.speed_hz = int(speed_hz)
        self.mode = int(mode)
        self._spi = None

    def open(self) -> None:
        try:
            import spidev  # type: ignore
        except ImportError as exc:
            raise Ad7606Error(
                "python spidev is not installed; install the distro python3-spidev package"
            ) from exc
        self._spi = spidev.SpiDev()
        self._spi.open(self.bus, self.device)
        self._spi.mode = self.mode
        self._spi.max_speed_hz = self.speed_hz
        self._spi.bits_per_word = 8

    def read_frame(self) -> bytes:
        if self._spi is None:
            raise Ad7606Error("SPI port is not open")
        return bytes(self._spi.xfer2([0] * FRAME_SIZE_BYTES, self.speed_hz))

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None


def _spin_delay_us(delay_us: float) -> None:
    deadline = time.monotonic_ns() + max(1, int(delay_us * 1000.0))
    while time.monotonic_ns() < deadline:
        pass


class Ad7606Reader:
    """Trigger conversions and synchronously read all eight AD7606 channels."""

    def __init__(
        self,
        spi: SpidevPort,
        gpio: GpiodLines,
        full_scale_voltage: float = 5.0,
        busy_timeout_s: float = 0.002,
        conversion_pulse_us: float = 2.0,
        accept_unobserved_busy_pulse: bool = True,
    ) -> None:
        self.spi = spi
        self.gpio = gpio
        self.full_scale_voltage = float(full_scale_voltage)
        self.busy_timeout_s = float(busy_timeout_s)
        self.conversion_pulse_us = float(conversion_pulse_us)
        self.accept_unobserved_busy_pulse = bool(accept_unobserved_busy_pulse)
        self._sequence = 0
        self._open = False

    def open(self) -> None:
        self.gpio.open()
        try:
            self.spi.open()
            self.reset()
        except Exception:
            self.spi.close()
            self.gpio.close()
            raise
        self._open = True

    def reset(self) -> None:
        self.gpio.set_convst(True)
        self.gpio.set_reset(True)
        _spin_delay_us(2.0)
        self.gpio.set_reset(False)
        time.sleep(0.001)

    def _trigger_and_wait(self) -> bool:
        event_wait_supported = all(
            hasattr(self.gpio, name)
            for name in ("discard_busy_events", "wait_busy_falling")
        )
        if event_wait_supported:
            self.gpio.discard_busy_events()

        self.gpio.set_convst(False)
        _spin_delay_us(self.conversion_pulse_us)
        self.gpio.set_convst(True)

        if event_wait_supported:
            try:
                busy_completion_seen = self.gpio.wait_busy_falling(self.busy_timeout_s)
            except BusyTimeoutError:
                if self.accept_unobserved_busy_pulse and not self.gpio.get_busy():
                    return False
                raise
            return busy_completion_seen

        started_ns = time.monotonic_ns()
        deadline_ns = started_ns + int(self.busy_timeout_s * 1_000_000_000)
        busy_completion_seen = False
        # AD7606 conversion is typically about 4 us with oversampling disabled.
        # A userspace GPIO read may miss the complete high pulse, so non-strict
        # mode accepts BUSY low after a conservative 10 us settling interval.
        accept_low_after_ns = started_ns + 10_000

        while time.monotonic_ns() < deadline_ns:
            busy = self.gpio.get_busy()
            if busy:
                busy_completion_seen = True
            elif busy_completion_seen:
                return True
            elif self.accept_unobserved_busy_pulse and time.monotonic_ns() >= accept_low_after_ns:
                return False

        raise BusyTimeoutError(
            f"AD7606 BUSY did not complete within {self.busy_timeout_s * 1000:.3f} ms"
        )

    def read_sample(self) -> Ad7606Sample:
        if not self._open:
            raise Ad7606Error("AD7606 reader is not open")
        busy_completion_seen = self._trigger_and_wait()
        frame = self.spi.read_frame()
        raw = decode_ad7606_frame(frame)
        voltages = counts_to_voltages(raw, self.full_scale_voltage)
        self._sequence += 1
        return Ad7606Sample(
            sequence=self._sequence,
            timestamp_ns=time.monotonic_ns(),
            raw=raw,
            voltages=voltages,
            busy_completion_seen=busy_completion_seen,
        )

    def close(self) -> None:
        self._open = False
        self.spi.close()
        self.gpio.close()

    def __enter__(self) -> "Ad7606Reader":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read AD7606 and report PDP90A X/Y/SUM")
    parser.add_argument("--spi-bus", type=int, default=0)
    parser.add_argument("--spi-device", type=int, default=0)
    parser.add_argument("--spi-speed", type=int, default=2_000_000)
    parser.add_argument("--gpiochip", default="/dev/gpiochip3")
    parser.add_argument("--busy-line", type=int, default=28)
    parser.add_argument("--convst-line", type=int, default=6)
    parser.add_argument("--reset-line", type=int, default=15)
    parser.add_argument("--rate-hz", type=float, default=400.0)
    parser.add_argument("--print-hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl+C")
    parser.add_argument("--sum-min", type=float, default=0.2)
    parser.add_argument("--sum-max", type=float, default=4.5)
    parser.add_argument("--x-offset", type=float, default=0.0)
    parser.add_argument("--y-offset", type=float, default=0.0)
    parser.add_argument("--sum-offset", type=float, default=0.0)
    parser.add_argument("--strict-busy", action="store_true")
    parser.add_argument("--csv", type=Path, help="optional CSV output path")
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.rate_hz <= 0 or args.print_hz <= 0:
        raise ValueError("rate-hz and print-hz must be positive")
    if args.duration < 0:
        raise ValueError("duration must be non-negative")


def run_capture(args: argparse.Namespace) -> int:
    _validate_args(args)
    spi = SpidevPort(args.spi_bus, args.spi_device, args.spi_speed, mode=2)
    gpio = GpiodLines(args.gpiochip, args.busy_line, args.convst_line, args.reset_line)
    reader = Ad7606Reader(
        spi,
        gpio,
        full_scale_voltage=5.0,
        accept_unobserved_busy_pulse=not args.strict_busy,
    )

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = args.csv.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            ["sequence", "timestamp_ns"]
            + [f"ch{i}_v" for i in range(1, 9)]
            + ["x_norm", "y_norm", "valid", "reason", "busy_completion_seen"]
        )

    offsets = (args.x_offset, args.y_offset, args.sum_offset)
    period_ns = max(1, int(1_000_000_000 / args.rate_hz))
    print_period_ns = max(1, int(1_000_000_000 / args.print_hz))
    start_ns = time.monotonic_ns()
    next_sample_ns = start_ns
    next_print_ns = start_ns
    deadline_misses = 0
    busy_seen_count = 0
    sample_count = 0

    print("seq       X/V       Y/V     SUM/V      X/SUM      Y/SUM  valid  BUSY")
    try:
        with reader:
            while args.duration == 0 or (time.monotonic_ns() - start_ns) < args.duration * 1e9:
                sample = reader.read_sample()
                psd = calculate_psd_reading(
                    sample.voltages,
                    offsets=offsets,
                    sum_min_voltage=args.sum_min,
                    sum_max_voltage=args.sum_max,
                )
                sample_count += 1
                busy_seen_count += int(sample.busy_completion_seen)

                if csv_writer is not None:
                    csv_writer.writerow(
                        [sample.sequence, sample.timestamp_ns]
                        + [f"{value:.9f}" for value in sample.voltages]
                        + [
                            "" if psd.x_normalized is None else f"{psd.x_normalized:.9f}",
                            "" if psd.y_normalized is None else f"{psd.y_normalized:.9f}",
                            int(psd.valid),
                            psd.reason,
                            int(sample.busy_completion_seen),
                        ]
                    )

                now_ns = time.monotonic_ns()
                if now_ns >= next_print_ns:
                    x_norm = "--" if psd.x_normalized is None else f"{psd.x_normalized:+.6f}"
                    y_norm = "--" if psd.y_normalized is None else f"{psd.y_normalized:+.6f}"
                    print(
                        f"{sample.sequence:6d}  {psd.x_voltage:+8.5f}  {psd.y_voltage:+8.5f}  "
                        f"{psd.sum_voltage:+8.5f}  {x_norm:>10}  {y_norm:>10}  "
                        f"{str(psd.valid):>5}  {int(sample.busy_completion_seen)}"
                    )
                    next_print_ns = now_ns + print_period_ns

                next_sample_ns += period_ns
                remaining_ns = next_sample_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1_000_000_000)
                else:
                    deadline_misses += 1
                    next_sample_ns = time.monotonic_ns()
    except KeyboardInterrupt:
        print("\nCapture stopped by user")
    finally:
        if csv_file is not None:
            csv_file.close()

    elapsed_s = max((time.monotonic_ns() - start_ns) / 1e9, 1e-9)
    print(
        f"samples={sample_count}, average_rate={sample_count / elapsed_s:.2f} Hz, "
        f"deadline_misses={deadline_misses}, BUSY_seen={busy_seen_count}/{sample_count}"
    )
    if sample_count and busy_seen_count == 0:
        print("WARNING: BUSY completion was not observed; verify GPIO mapping or retry with --strict-busy")
    return 0


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    try:
        return run_capture(args)
    except (Ad7606Error, OSError, ValueError) as exc:
        parser.error(str(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
