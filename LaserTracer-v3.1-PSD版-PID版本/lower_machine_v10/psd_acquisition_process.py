"""Run AD7606 acquisition outside the motor-control process's Python GIL."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from typing import Any, Dict, Optional

try:
    from .ad7606_reader import (
        Ad7606Reader, Ad7606Sample, BusyTimeoutError, GpiodLines, SpidevPort,
    )
except ImportError:
    from ad7606_reader import (  # type: ignore
        Ad7606Reader, Ad7606Sample, BusyTimeoutError, GpiodLines, SpidevPort,
    )


GEN = 0
HAS_SAMPLE = 1
SEQUENCE = 2
TIMESTAMP_NS = 3
VOLTAGES = 4
RAW = 12
BUSY_SEEN = 20
TIMEOUTS = 21
CONSECUTIVE_TIMEOUTS = 22
READ_MEAN_MS = 23
READ_MAX_MS = 24
OVERRUNS = 25
LAG_MAX_MS = 26
FATAL = 27
STATE_SIZE = 28


def _build_reader(config: Dict[str, Any]) -> Ad7606Reader:
    hardware = config["hardware"]
    return Ad7606Reader(
        SpidevPort(
            hardware["spi_bus"], hardware["spi_device"],
            hardware["spi_speed_hz"], mode=2,
        ),
        GpiodLines(
            hardware["gpiochip"], hardware["busy_line"],
            hardware["convst_line"], hardware["reset_line"],
        ),
        busy_timeout_s=float(hardware.get("busy_timeout_s", 0.002)),
        accept_unobserved_busy_pulse=not bool(hardware.get("strict_busy", True)),
    )


def _run_acquisition(config, rate_hz, max_timeouts, state, stop_event, ready_event, errors):
    reader = None
    generation = 0
    total_timeouts = 0
    consecutive_timeouts = 0
    read_count = 0
    read_total_ms = 0.0
    read_max_ms = 0.0
    overruns = 0
    lag_max_ms = 0.0
    period_s = 0.0 if rate_hz is None else 1.0 / rate_hz
    next_sample = time.monotonic()
    try:
        reader = _build_reader(config)
        reader.open()
        while not stop_event.is_set():
            generation += 1
            read_started = time.monotonic()
            if period_s > 0.0:
                lag_max_ms = max(
                    lag_max_ms, max(0.0, read_started - next_sample) * 1000.0,
                )
            sample = None
            busy_error = None
            try:
                sample = reader.read_sample()
            except BusyTimeoutError as exc:
                busy_error = exc
                total_timeouts += 1
                consecutive_timeouts += 1
            read_ms = (time.monotonic() - read_started) * 1000.0
            read_count += 1
            read_total_ms += read_ms
            read_max_ms = max(read_max_ms, read_ms)
            if busy_error is None:
                consecutive_timeouts = 0

            with state.get_lock():
                shared = state.get_obj()
                shared[GEN] = generation
                shared[HAS_SAMPLE] = int(sample is not None)
                if sample is not None:
                    shared[SEQUENCE] = sample.sequence
                    shared[TIMESTAMP_NS] = sample.timestamp_ns
                    shared[VOLTAGES:VOLTAGES + 8] = sample.voltages
                    shared[RAW:RAW + 8] = sample.raw
                    shared[BUSY_SEEN] = int(sample.busy_completion_seen)
                shared[TIMEOUTS] = total_timeouts
                shared[CONSECUTIVE_TIMEOUTS] = consecutive_timeouts
                shared[READ_MEAN_MS] = read_total_ms / read_count
                shared[READ_MAX_MS] = read_max_ms
                shared[OVERRUNS] = overruns
                shared[LAG_MAX_MS] = lag_max_ms
            ready_event.set()
            if consecutive_timeouts >= max_timeouts:
                raise busy_error

            if period_s > 0.0:
                next_sample += period_s
                delay = next_sample - time.monotonic()
                if delay > 0.0:
                    # A one-millisecond slot is short enough to remain responsive
                    # to stop without a separate wakeup pipe.
                    time.sleep(delay)
                else:
                    overruns += 1
                    next_sample = time.monotonic()
    except Exception as exc:
        with state.get_lock():
            state.get_obj()[FATAL] = 1
        try:
            errors.put_nowait(f"{type(exc).__name__}: {exc}")
        except queue.Full:
            pass
        ready_event.set()
    finally:
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass
        ready_event.set()


class PsdAcquisitionProcess:
    """Latest-sample shared memory; old samples never queue behind motor feedback."""

    def __init__(self, config: Dict[str, Any], rate_hz: Optional[float], worker=None):
        if rate_hz is not None and rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self._context = mp.get_context("spawn")
        self._state = self._context.Array("d", STATE_SIZE)
        self._stop_event = self._context.Event()
        self._ready_event = self._context.Event()
        self._errors = self._context.Queue(maxsize=1)
        self._process = self._context.Process(
            target=_run_acquisition if worker is None else worker,
            args=(
                config, rate_hz,
                int(config["controller"].get("max_consecutive_adc_timeouts", 5)),
                self._state, self._stop_event, self._ready_event, self._errors,
            ),
            name="psd-ad7606-acquisition",
            daemon=True,
        )

    def start(self, timeout_s: float = 3.0) -> None:
        self._process.start()
        if not self._ready_event.wait(timeout_s):
            self.stop()
            raise RuntimeError("Timed out while opening AD7606 acquisition process")
        error = self.error()
        if error:
            self.stop()
            raise RuntimeError(error)

    def error(self) -> Optional[str]:
        with self._state.get_lock():
            fatal = bool(self._state.get_obj()[FATAL])
        if fatal:
            try:
                return self._errors.get_nowait()
            except queue.Empty:
                return "PSD acquisition process failed"
        if self._ready_event.is_set() and not self._process.is_alive():
            return f"PSD acquisition process exited ({self._process.exitcode})"
        return None

    def snapshot(self) -> Dict[str, Any]:
        with self._state.get_lock():
            values = self._state.get_obj()[:]
        sample = None
        if values[HAS_SAMPLE]:
            sample = Ad7606Sample(
                sequence=int(values[SEQUENCE]),
                timestamp_ns=int(values[TIMESTAMP_NS]),
                raw=tuple(int(v) for v in values[RAW:RAW + 8]),
                voltages=tuple(values[VOLTAGES:VOLTAGES + 8]),
                busy_completion_seen=bool(values[BUSY_SEEN]),
            )
        return {
            "generation": int(values[GEN]),
            "sample": sample,
            "samples": int(values[SEQUENCE]),
            "adc_timeouts": int(values[TIMEOUTS]),
            "consecutive_adc_timeouts": int(values[CONSECUTIVE_TIMEOUTS]),
            "last_adc_error": (
                "AD7606 BUSY timeout" if values[CONSECUTIVE_TIMEOUTS] else None
            ),
            "adc_read_mean_ms": values[READ_MEAN_MS],
            "adc_read_max_ms": values[READ_MAX_MS],
            "adc_schedule_overruns": int(values[OVERRUNS]),
            "adc_schedule_lag_max_ms": values[LAG_MAX_MS],
        }

    def stop(self) -> None:
        self._stop_event.set()
        if self._process.pid is not None:
            self._process.join(1.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(1.0)
