"""V10 multirate PSD acquisition with fast far-zone position control."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple

try:
    from .ad7606_reader import (
        Ad7606Reader,
        BusyTimeoutError,
        GpiodLines,
        SpidevPort,
        calculate_psd_reading,
    )
    from .psd_tracker import PsdTrackingController
except ImportError:
    from ad7606_reader import (  # type: ignore
        Ad7606Reader,
        BusyTimeoutError,
        GpiodLines,
        SpidevPort,
        calculate_psd_reading,
    )
    from psd_tracker import PsdTrackingController  # type: ignore


ApplyStep = Callable[[float, float, bool, float], Tuple[float, float]]
HoldPosition = Callable[[], Tuple[float, float]]


def _wait_until(deadline: float, stop_event: threading.Event) -> bool:
    """Wait with sub-10 ms precision while remaining promptly stoppable."""
    while not stop_event.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return True
        if remaining > 0.001:
            time.sleep(max(remaining - 0.0005, 0.0))
        else:
            time.sleep(0)
    return False


def _position_step_time_scale(dt_s: float, reference_rate_hz: float) -> float:
    """Keep incremental-position gain per second bounded by its reference rate."""
    if not math.isfinite(dt_s) or dt_s < 0.0:
        raise ValueError("position command interval must be finite and non-negative")
    if not math.isfinite(reference_rate_hz) or reference_rate_hz <= 0.0:
        raise ValueError("position step reference rate must be finite and positive")
    return min(1.0, dt_s * reference_rate_hz)


class PsdTrackingService:
    """Sample the PSD independently from the slower motor command loop."""

    def __init__(
        self,
        config_path: str | Path,
        apply_step: ApplyStep,
        hold_position: Optional[HoldPosition] = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.apply_step = apply_step
        self.hold_position = hold_position
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._startup_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._startup_error: Optional[str] = None
        self._status: Dict[str, Any] = {
            "running": False,
            "state": "stopped",
            "samples": 0,
            "control_cycles": 0,
            "control_deadline_misses": 0,
            "commands": 0,
            "holds": 0,
            "direction_reversals": 0,
            "adc_timeouts": 0,
            "last_error": None,
        }

    @property
    def is_running(self) -> bool:
        with self._lock:
            return bool(self._status["running"])

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def _set_status(self, **values: Any) -> None:
        with self._lock:
            self._status.update(values)

    def start(self, timeout_s: float = 3.0) -> Tuple[bool, str]:
        with self._lock:
            if self._status["running"]:
                return True, "Predictive PSD position tracking is already running"
            self._status = {
                "running": False,
                "state": "starting",
                "samples": 0,
                "control_cycles": 0,
                "control_deadline_misses": 0,
                "commands": 0,
                "holds": 0,
                "direction_reversals": 0,
                "adc_timeouts": 0,
                "last_error": None,
            }
            self._startup_error = None
            self._stop_event.clear()
            self._startup_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="psd-position-multirate-v10",
                daemon=True,
            )
            self._thread.start()

        if not self._startup_event.wait(timeout_s):
            self._stop_event.set()
            return False, "Timed out while opening AD7606/GPIO"
        if self._startup_error:
            return False, self._startup_error
        return True, "V10 multirate PSD position tracking started"

    def stop(self, timeout_s: float = 2.0) -> Tuple[bool, str]:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout_s)
        if thread is not None and thread.is_alive():
            return False, "Tracking threads did not stop within the timeout"
        with self._lock:
            self._status["running"] = False
            self._status["state"] = "stopped"
        return True, "V10 multirate PSD position tracking stopped"

    def _load_config(self) -> Dict[str, Any]:
        return json.loads(self.config_path.read_text(encoding="utf-8"))

    @staticmethod
    def _build_reader(config: Dict[str, Any]) -> Ad7606Reader:
        hardware = config["hardware"]
        return Ad7606Reader(
            SpidevPort(
                hardware["spi_bus"],
                hardware["spi_device"],
                hardware["spi_speed_hz"],
                mode=2,
            ),
            GpiodLines(
                hardware["gpiochip"],
                hardware["busy_line"],
                hardware["convst_line"],
                hardware["reset_line"],
            ),
            busy_timeout_s=float(hardware.get("busy_timeout_s", 0.002)),
            accept_unobserved_busy_pulse=not bool(hardware.get("strict_busy", True)),
        )

    def _run(self) -> None:
        reader: Optional[Ad7606Reader] = None
        controller: Optional[PsdTrackingController] = None
        acquisition_thread: Optional[threading.Thread] = None
        acquisition_error = []
        snapshot_lock = threading.Lock()
        snapshot: Dict[str, Any] = {
            "generation": 0,
            "sample": None,
            "samples": 0,
            "adc_timeouts": 0,
            "consecutive_adc_timeouts": 0,
            "last_adc_error": None,
        }

        try:
            config = self._load_config()
            control = config["controller"]
            sample_rate_hz = float(control.get("sample_rate_hz", 500.0))
            command_rate_hz = float(control.get("command_rate_hz", 250.0))
            position_step_reference_rate_hz = float(
                control.get("position_step_reference_rate_hz", 200.0)
            )
            status_update_rate_hz = float(
                control.get("status_update_rate_hz", 10.0)
            )
            status_log_rate_hz = float(control.get("status_log_rate_hz", 2.0))
            motion_log_rate_hz = float(control.get("motion_log_rate_hz", 10.0))
            reversal_brake_s = float(control.get("reversal_brake_s", 0.030))
            reversal_min_step_deg = float(
                control.get("reversal_min_step_deg", 0.0)
            )
            reversal_confirm_commands = int(
                control.get("reversal_confirm_commands", 1)
            )
            reversal_cosine_threshold = float(
                control.get("reversal_cosine_threshold", -1.0)
            )
            hold_min_interval_s = float(control.get("hold_min_interval_s", 0.200))
            center_hold_min_interval_s = float(
                control.get("center_hold_min_interval_s", 0.050)
            )
            max_adc_timeouts = int(control.get("max_consecutive_adc_timeouts", 5))
            if min(
                sample_rate_hz,
                command_rate_hz,
                position_step_reference_rate_hz,
                status_update_rate_hz,
                status_log_rate_hz,
                motion_log_rate_hz,
            ) <= 0.0:
                raise ValueError("sample, command and log rates must be positive")
            if command_rate_hz > sample_rate_hz:
                raise ValueError("command rate cannot exceed sample rate")
            if max_adc_timeouts < 1:
                raise ValueError("max_consecutive_adc_timeouts must be positive")
            if min(
                reversal_brake_s,
                reversal_min_step_deg,
                hold_min_interval_s,
                center_hold_min_interval_s,
            ) < 0.0:
                raise ValueError("brake and hold intervals cannot be negative")
            if reversal_confirm_commands < 1:
                raise ValueError("reversal_confirm_commands must be positive")
            if not -1.0 <= reversal_cosine_threshold <= 1.0:
                raise ValueError("reversal_cosine_threshold must be between -1 and 1")

            controller = PsdTrackingController(config)
            reader = self._build_reader(config)
            reader.open()
            controller.start()

            offsets = config["dark_offsets_v"]
            limits = config["commissioning_limits"]
            offset_values = (offsets["x"], offsets["y"], offsets["sum"])
            sample_period = 1.0 / sample_rate_hz
            command_period = 1.0 / command_rate_hz
            status_update_period = 1.0 / status_update_rate_hz
            log_period = 1.0 / status_log_rate_hz
            motion_log_period = 1.0 / motion_log_rate_hz
            started_at = time.monotonic()

            def acquisition_loop() -> None:
                next_sample = time.monotonic()
                consecutive_timeouts = 0
                total_timeouts = 0
                generation = 0
                while not self._stop_event.is_set():
                    generation += 1
                    try:
                        sample = reader.read_sample()
                        consecutive_timeouts = 0
                        with snapshot_lock:
                            snapshot.update(
                                generation=generation,
                                sample=sample,
                                samples=sample.sequence,
                                adc_timeouts=total_timeouts,
                                consecutive_adc_timeouts=0,
                                last_adc_error=None,
                            )
                    except BusyTimeoutError as exc:
                        consecutive_timeouts += 1
                        total_timeouts += 1
                        with snapshot_lock:
                            snapshot.update(
                                generation=generation,
                                sample=None,
                                adc_timeouts=total_timeouts,
                                consecutive_adc_timeouts=consecutive_timeouts,
                                last_adc_error=str(exc),
                            )
                        print(
                            "[PSD分区高速][ADC重试] "
                            f"BUSY超时 {consecutive_timeouts}/{max_adc_timeouts}，"
                            "当前控制周期不发送电机命令"
                        )
                        if consecutive_timeouts >= max_adc_timeouts:
                            acquisition_error.append(exc)
                            self._stop_event.set()
                            break
                    except Exception as exc:
                        acquisition_error.append(exc)
                        self._stop_event.set()
                        break

                    next_sample += sample_period
                    delay = next_sample - time.monotonic()
                    if delay > 0.0:
                        if not _wait_until(next_sample, self._stop_event):
                            break
                    else:
                        next_sample = time.monotonic()

            acquisition_thread = threading.Thread(
                target=acquisition_loop,
                name="psd-acquisition-v10",
                daemon=True,
            )
            acquisition_thread.start()
            self._set_status(running=True, state="acquiring")
            self._startup_event.set()
            print(
                "[PSD分区高速] 已启动 | "
                f"采样目标={sample_rate_hz:.1f}Hz 控制目标={command_rate_hz:.1f}Hz | "
                f"步长基准={position_step_reference_rate_hz:.1f}Hz "
                f"步长={controller.motor_step_deg:.3f}° "
                f"远区最大修正={controller.max_single_correction_deg:.3f}° | "
                "分区频率="
                f"{controller.position_zones['fine']['command_rate_hz']:.0f}/"
                f"{controller.position_zones['medium']['command_rate_hz']:.0f}/"
                f"{controller.position_zones['far']['command_rate_hz']:.0f}Hz "
                f"反向制动={reversal_brake_s * 1000.0:.0f}ms×"
                f"{reversal_confirm_commands}次确认"
            )

            next_command = time.monotonic()
            next_status_update = next_command
            next_log = next_command
            next_motion_log = next_command
            last_generation = -1
            latest_reading = None
            latest_update = None
            previous_controller_state = None
            hold_request_id = 0
            last_logged_state = None
            last_hold_request_id = 0
            last_nonzero_step = [0.0, 0.0]
            pending_reversal_count = 0
            last_zone_command_at = {
                "fine": float("-inf"),
                "medium": float("-inf"),
                "far": float("-inf"),
            }
            motion_command_times = deque()
            last_hold_at = float("-inf")
            reversal_brake_until = 0.0
            resume_after_brake = False
            last_motion_command_at: Optional[float] = None
            control_cycles = 0
            control_deadline_misses = 0
            command_count = 0
            hold_count = 0
            reversal_count = 0
            status_details: Dict[str, Any] = {}

            def skip_overrun_slots() -> None:
                nonlocal next_command, control_deadline_misses
                finished_at = time.monotonic()
                if next_command >= finished_at:
                    return
                missed = int((finished_at - next_command) / command_period) + 1
                next_command += missed * command_period
                control_deadline_misses += missed

            while not self._stop_event.is_set():
                delay = next_command - time.monotonic()
                if delay > 0.0:
                    if not _wait_until(next_command, self._stop_event):
                        break
                next_command += command_period
                control_cycles += 1
                cycle_now = time.monotonic()
                cycle_elapsed = max(cycle_now - started_at, 1e-9)

                with snapshot_lock:
                    local = dict(snapshot)
                generation = local["generation"]
                now = time.monotonic()
                new_generation = generation > 0 and generation != last_generation
                if new_generation:
                    last_generation = generation
                    sample = local["sample"]
                    if sample is not None:
                        latest_reading = calculate_psd_reading(
                            sample.voltages,
                            offsets=offset_values,
                            sum_min_voltage=float(limits["sum_min_v"]),
                            sum_max_voltage=float(limits["sum_max_v"]),
                        )
                        latest_update = controller.update(
                            latest_reading.x_normalized,
                            latest_reading.y_normalized,
                            valid=latest_reading.valid,
                            sample_time_s=(
                                sample.timestamp_ns / 1_000_000_000.0
                            ),
                        )
                    else:
                        latest_reading = None
                        latest_update = controller.update(
                            None,
                            None,
                            valid=False,
                        )

                    state_name = latest_update.state.value
                    entered_center = (
                        state_name == "locked"
                        and previous_controller_state != "locked"
                    )
                    entered_signal_lost = (
                        state_name == "signal_lost"
                        and previous_controller_state != "signal_lost"
                    )
                    if entered_center or entered_signal_lost:
                        hold_request_id += 1
                    if entered_signal_lost:
                        reason_text = (
                            latest_reading.reason
                            if latest_reading is not None
                            else local.get("last_adc_error") or "adc_busy_timeout"
                        )
                        sum_text = (
                            f"{latest_reading.sum_voltage:.4f}V"
                            if latest_reading is not None
                            else "不可用"
                        )
                        print(
                            "[PSD分区高速][信号丢失] "
                            f"原因={reason_text} SUM={sum_text} "
                            f"连续无效阈值={controller.lost_after_invalid_samples}点"
                        )
                    previous_controller_state = state_name

                reading = latest_reading
                update = latest_update
                while (
                    motion_command_times
                    and motion_command_times[0] < now - 1.0
                ):
                    motion_command_times.popleft()

                if now >= next_status_update:
                    next_status_update = now + status_update_period
                    samples = int(local.get("samples", 0))
                    measured_sample_rate = (
                        (samples - 1) / cycle_elapsed if samples > 1 else 0.0
                    )
                    measured_control_rate = (
                        (control_cycles - 1) / cycle_elapsed
                        if control_cycles > 1
                        else 0.0
                    )
                    controller_state = (
                        update.state.value if update is not None else "acquiring"
                    )
                    status_state = (
                        "adc_retry"
                        if int(local.get("consecutive_adc_timeouts", 0)) > 0
                        else controller_state
                    )
                    status_values: Dict[str, Any] = {
                        "state": status_state,
                        "samples": samples,
                        "control_cycles": control_cycles,
                        "control_deadline_misses": control_deadline_misses,
                        "commands": command_count,
                        "holds": hold_count,
                        "direction_reversals": reversal_count,
                        "measured_sample_rate_hz": measured_sample_rate,
                        "measured_control_rate_hz": measured_control_rate,
                        "measured_command_rate_hz": float(
                            len(motion_command_times)
                        ),
                        "adc_timeouts": int(local.get("adc_timeouts", 0)),
                        "consecutive_adc_timeouts": int(
                            local.get("consecutive_adc_timeouts", 0)
                        ),
                        "last_error": local.get("last_adc_error"),
                    }
                    if reading is not None:
                        status_values.update(
                            x_norm=reading.x_normalized,
                            y_norm=reading.y_normalized,
                            sum_v=reading.sum_voltage,
                            psd_valid=reading.valid,
                            psd_reason=reading.reason,
                        )
                    status_values.update(status_details)
                    self._set_status(**status_values)

                if (
                    hold_request_id != last_hold_request_id
                    and self.hold_position is not None
                ):
                    state_name = update.state.value if update is not None else "acquiring"
                    if state_name == "signal_lost":
                        allow_hold = True
                    elif state_name == "locked":
                        allow_hold = now - last_hold_at >= center_hold_min_interval_s
                    else:
                        allow_hold = now - last_hold_at >= hold_min_interval_s
                    last_hold_request_id = hold_request_id
                    if allow_hold:
                        hold_started = time.monotonic()
                        hold_pitch, hold_yaw = self.hold_position()
                        hold_ms = (time.monotonic() - hold_started) * 1000.0
                        hold_count += 1
                        status_details.update(
                            last_hold_ms=hold_ms,
                            pitch_target_deg=hold_pitch,
                            yaw_target_deg=hold_yaw,
                        )
                        last_hold_at = now
                        last_nonzero_step = [0.0, 0.0]
                        pending_reversal_count = 0
                        last_motion_command_at = None
                        print(
                            "[PSD分区高速][主动保持] "
                            f"取消旧目标，保持 pitch={hold_pitch:+.6f}° "
                            f"yaw={hold_yaw:+.6f}° | 往返={hold_ms:.2f}ms"
                        )

                if new_generation and update is not None:
                    if update.should_move and reading is not None:
                        pitch_delta, yaw_delta = update.step_correction_deg
                        zone_name = update.control_zone
                        zone_rate_hz = max(float(update.command_rate_hz), 1e-9)
                        zone_period_s = 1.0 / zone_rate_hz
                        zone_due = (
                            zone_rate_hz >= command_rate_hz - 1e-9
                            or now - last_zone_command_at[zone_name] >= zone_period_s
                        )
                        if not zone_due or now < reversal_brake_until:
                            skip_overrun_slots()
                            continue
                        command_dt_s = (
                            command_period
                            if last_motion_command_at is None
                            else max(now - last_motion_command_at, 0.0)
                        )
                        step_time_scale = _position_step_time_scale(
                            command_dt_s,
                            position_step_reference_rate_hz,
                        )
                        pitch_delta = round(pitch_delta * step_time_scale, 6)
                        yaw_delta = round(yaw_delta * step_time_scale, 6)
                        current_step = (pitch_delta, yaw_delta)
                        current_magnitude = math.hypot(*current_step)
                        previous_magnitude = math.hypot(*last_nonzero_step)
                        reversal_cosine = 1.0
                        if current_magnitude > 0.0 and previous_magnitude > 0.0:
                            reversal_cosine = (
                                current_step[0] * last_nonzero_step[0]
                                + current_step[1] * last_nonzero_step[1]
                            ) / (current_magnitude * previous_magnitude)
                        reversal_pending = (
                            current_magnitude >= reversal_min_step_deg
                            and previous_magnitude >= reversal_min_step_deg
                            and reversal_cosine <= reversal_cosine_threshold
                        )
                        if reversal_pending:
                            pending_reversal_count += 1
                        else:
                            pending_reversal_count = 0
                        reversal_confirmed = (
                            pending_reversal_count >= reversal_confirm_commands
                        )

                        # A single opposite command is usually PSD noise or
                        # cross-axis coupling. Wait for confirmation without
                        # changing the remembered direction or motor target.
                        if reversal_pending and not reversal_confirmed:
                            last_zone_command_at[zone_name] = now
                            skip_overrun_slots()
                            continue

                        if reversal_confirmed and self.hold_position is not None:
                            brake_started = time.monotonic()
                            hold_pitch, hold_yaw = self.hold_position()
                            brake_ms = (time.monotonic() - brake_started) * 1000.0
                            reversal_brake_until = now + reversal_brake_s
                            resume_after_brake = False
                            last_nonzero_step = [0.0, 0.0]
                            pending_reversal_count = 0
                            last_motion_command_at = None
                            last_zone_command_at[zone_name] = now
                            reversal_count += 1
                            status_details.update(
                                pitch_target_deg=hold_pitch,
                                yaw_target_deg=hold_yaw,
                            )
                            print(
                                "[PSD分区高速][反向制动] "
                                f"连续{reversal_confirm_commands}次确认方向翻转，"
                                f"保持反馈位置{reversal_brake_s * 1000.0:.0f}ms | "
                                f"pitch={hold_pitch:+.6f}° yaw={hold_yaw:+.6f}° | "
                                f"下发={brake_ms:.2f}ms"
                            )
                            skip_overrun_slots()
                            continue

                        # hold_position() already re-anchors the server targets
                        # to feedback. Do not reset again on the next far-zone
                        # command, otherwise accumulated lead is discarded.
                        reset_target = update.reset_target
                        motor_started = time.monotonic()
                        pitch_target, yaw_target = self.apply_step(
                            pitch_delta,
                            yaw_delta,
                            reset_target,
                            float(update.max_target_lead_deg),
                        )
                        motor_dispatch_ms = (
                            time.monotonic() - motor_started
                        ) * 1000.0
                        resume_after_brake = False
                        last_zone_command_at[zone_name] = now
                        last_motion_command_at = now
                        command_count += 1
                        motion_command_times.append(now)
                        status_details.update(
                            control_zone=update.control_zone,
                            target_reset=reset_target,
                            last_motor_dispatch_ms=motor_dispatch_ms,
                            pitch_target_deg=pitch_target,
                            yaw_target_deg=yaw_target,
                            pitch_step_deg=pitch_delta,
                            yaw_step_deg=yaw_delta,
                            position_step_time_scale=step_time_scale,
                            motion_command_interval_ms=command_dt_s * 1000.0,
                        )
                        error_x, error_y = update.error_norm or (0.0, 0.0)
                        continuous_pitch, continuous_yaw = (
                            update.continuous_correction_deg
                        )
                        predicted_x, predicted_y = (
                            update.predicted_error_norm or (error_x, error_y)
                        )
                        rate_x, rate_y = update.error_rate_norm_s
                        for axis in range(2):
                            if current_step[axis] != 0.0:
                                last_nonzero_step[axis] = current_step[axis]
                        if now >= next_motion_log:
                            next_motion_log = now + motion_log_period
                            live_status = self.status()
                            print(
                                "[PSD分区高速][运动] "
                                f"区域={update.control_zone} "
                                f"分区上限={zone_rate_hz:.0f}Hz "
                                f"动态领先={update.max_target_lead_deg:.3f}° "
                                f"预测={'开' if update.prediction_active else '关'} "
                                f"阻尼={'开' if update.damping_active else '关'} "
                                f"重置目标={reset_target} | "
                                f"误差 X={error_x:+.6f} Y={error_y:+.6f} | "
                                f"预测 X={predicted_x:+.6f} Y={predicted_y:+.6f} | "
                                f"趋势 X={rate_x:+.3f}/s Y={rate_y:+.3f}/s | "
                                f"比例 pitch={continuous_pitch:+.4f}° "
                                f"yaw={continuous_yaw:+.4f}° | "
                                f"修正 pitch={pitch_delta:+.3f}° "
                                f"yaw={yaw_delta:+.3f}° | "
                                f"步长时间缩放={step_time_scale:.3f} "
                                f"间隔={command_dt_s * 1000.0:.2f}ms | "
                                f"SUM={reading.sum_voltage:.4f}V | "
                                f"目标 pitch={pitch_target:+.6f}° "
                                f"yaw={yaw_target:+.6f}° | "
                                f"目标入队={motor_dispatch_ms:.3f}ms | "
                                f"实际采样={live_status.get('measured_sample_rate_hz', 0.0):.1f}Hz "
                                f"实际控制={live_status.get('measured_control_rate_hz', 0.0):.1f}Hz "
                                f"近1秒运动命令={live_status.get('measured_command_rate_hz', 0.0):.1f}Hz"
                            )

                state = update.state.value if update is not None else "acquiring"
                if state != last_logged_state or now >= next_log:
                    next_log = now + log_period
                    last_logged_state = state
                    status = self.status()
                    if state == "locked" and reading is not None:
                        error_x, error_y = update.error_norm or (0.0, 0.0)
                        print(
                            "[PSD分区高速][不动] PSD在死区内 | "
                            f"误差 X={error_x:+.6f} Y={error_y:+.6f} | "
                            f"SUM={reading.sum_voltage:.4f}V | "
                            f"实际采样={status.get('measured_sample_rate_hz', 0.0):.1f}Hz "
                            f"实际控制={status.get('measured_control_rate_hz', 0.0):.1f}Hz"
                        )
                    elif state in ("acquiring", "signal_lost"):
                        print(
                            "[PSD分区高速][不动] "
                            f"state={state}，等待有效PSD | "
                            f"实际采样={status.get('measured_sample_rate_hz', 0.0):.1f}Hz"
                        )

                # Align to the next future slot after an overrun. This skips
                # missed slots without a catch-up burst or an extra full-period
                # delay, allowing the loop to approach its configured rate because motor RX is
                # handled by a separate thread and TX does not wait for replies.
                skip_overrun_slots()

            if acquisition_error:
                raise acquisition_error[0]

        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}"
            print(f"[PSD分区高速][故障] {message}")
            self._startup_error = message
            self._set_status(running=False, state="error", last_error=message)
            self._startup_event.set()
        finally:
            self._stop_event.set()
            if acquisition_thread is not None and acquisition_thread.is_alive():
                acquisition_thread.join(1.0)
            if controller is not None and self.hold_position is not None:
                try:
                    hold_pitch, hold_yaw = self.hold_position()
                    print(
                        "[PSD分区高速][停止保持] "
                        f"pitch={hold_pitch:+.6f}° yaw={hold_yaw:+.6f}°"
                    )
                except Exception as hold_exc:
                    print(
                        "[PSD分区高速][停止保持警告] "
                        f"{type(hold_exc).__name__}: {hold_exc}"
                    )
            if controller is not None:
                controller.stop()
            if reader is not None:
                try:
                    reader.close()
                except Exception:
                    pass
            with self._lock:
                self._status["running"] = False
                if self._status["state"] != "error":
                    self._status["state"] = "stopped"
                samples = self._status.get("samples", 0)
                commands = self._status.get("commands", 0)
                control_rate = self._status.get("measured_control_rate_hz", 0.0)
                command_rate = self._status.get("measured_command_rate_hz", 0.0)
                deadline_misses = self._status.get("control_deadline_misses", 0)
                sample_rate = self._status.get("measured_sample_rate_hz", 0.0)
            print(
                "[PSD分区高速] 已停止 | "
                f"samples={samples} commands={commands} "
                f"measured_sample_rate={sample_rate:.1f}Hz "
                f"measured_control_rate={control_rate:.1f}Hz "
                f"measured_motion_command_rate={command_rate:.1f}Hz "
                f"control_deadline_misses={deadline_misses}"
            )
            self._startup_event.set()
