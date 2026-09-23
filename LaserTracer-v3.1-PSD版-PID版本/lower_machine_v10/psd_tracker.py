"""Pure PSD tracking-control core for the RK3588 lower machine.

This module deliberately contains no GPIO, SPI, serial or socket operations.
It converts one calibrated PSD reading into a bounded incremental pitch/yaw
position command. Hardware acquisition and motor transport are connected by
the lower-machine device service in a later integration layer.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple


class TrackerState(str, Enum):
    STOPPED = "stopped"
    ACQUIRING = "acquiring"
    TRACKING = "tracking"
    LOCKED = "locked"
    SIGNAL_LOST = "signal_lost"


@dataclass(frozen=True)
class TrackingUpdate:
    state: TrackerState
    filtered_norm: Optional[Tuple[float, float]]
    error_norm: Optional[Tuple[float, float]]
    continuous_correction_deg: Tuple[float, float]
    step_correction_deg: Tuple[float, float]
    should_move: bool
    reason: str
    control_zone: str = "none"
    reset_target: bool = False
    predicted_error_norm: Optional[Tuple[float, float]] = None
    error_rate_norm_s: Tuple[float, float] = (0.0, 0.0)
    max_target_lead_deg: float = 0.0
    command_rate_hz: float = 0.0
    prediction_active: bool = False
    damping_active: bool = False
    dynamic_lead_scale: float = 0.0


def _require_pair(value: Sequence[float], name: str) -> Tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name} must contain two values")
    pair = (float(value[0]), float(value[1]))
    if not all(math.isfinite(item) for item in pair):
        raise ValueError(f"{name} values must be finite")
    return pair


def feedback_relative_target(
    feedback_position: float,
    delta: float,
    max_lead_deg: float,
) -> float:
    """Apply only this PSD correction to the newest measured motor position."""
    values = (feedback_position, delta, max_lead_deg)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("tracking target values must be finite")
    if max_lead_deg <= 0.0:
        raise ValueError("max_target_lead_deg must be positive")
    candidate = float(feedback_position) + float(delta)
    lower = float(feedback_position) - float(max_lead_deg)
    upper = float(feedback_position) + float(max_lead_deg)
    return round(min(max(candidate, lower), upper), 6)


class PsdTrackingController:
    """Stateful PSD-to-motor incremental position controller.

    The calibrated correction matrix already includes the negative feedback
    sign. Therefore the controller evaluates::

        motor_delta = correction_matrix * (measured_norm - target_norm)

    A configurable proportional gain is applied to the calibrated correction.
    The result is quantized to the motor's minimum position increment and
    bounded per update.
    """

    def __init__(self, config: Mapping[str, Any]) -> None:
        target = config["tracking_setpoint_norm"]
        self.target_norm = (float(target["x"]), float(target["y"]))

        matrix = config["inverse_model"]["correction_deg_per_norm"]
        if len(matrix) != 2:
            raise ValueError("correction matrix must have two rows")
        self.correction_matrix = (
            _require_pair(matrix[0], "correction matrix row 0"),
            _require_pair(matrix[1], "correction matrix row 1"),
        )

        deadband = config["commissioning_limits"]["deadband_norm"]
        self.deadband_norm = (float(deadband["x"]), float(deadband["y"]))
        limits = config["commissioning_limits"]
        self.max_single_correction_deg = float(limits["max_single_correction_deg"])

        controller = config.get("controller", {})
        self.filter_alpha = float(controller.get("filter_alpha", 0.22))
        self.release_deadband_multiplier = float(
            controller.get("release_deadband_multiplier", 1.5)
        )
        self.release_confirm_samples = int(
            controller.get("release_confirm_samples", 1)
        )
        self.lock_confirm_samples = int(
            controller.get("lock_confirm_samples", 1)
        )
        self.acquire_valid_samples = int(controller.get("acquire_valid_samples", 5))
        self.lost_after_invalid_samples = int(
            controller.get("lost_after_invalid_samples", 3)
        )
        self.motor_step_deg = float(controller.get("motor_step_deg", 0.003))
        self.axis_command_threshold_deg = float(
            controller.get("axis_command_threshold_deg", self.motor_step_deg / 4.0)
        )
        self.proportional_gain = float(controller.get("proportional_gain", 0.85))
        self.sample_rate_hz = float(controller.get("sample_rate_hz", 500.0))
        self.command_rate_hz = float(controller.get("command_rate_hz", 250.0))
        self.default_max_target_lead_deg = float(
            controller.get("max_target_lead_deg", self.max_single_correction_deg)
        )
        self.prediction_horizon_s = float(
            controller.get("prediction_horizon_s", 0.045)
        )
        self.derivative_alpha = float(controller.get("derivative_alpha", 0.18))
        self.max_prediction_deadband_multiple = float(
            controller.get("max_prediction_deadband_multiple", 2.0)
        )
        self.prediction_min_error_ratio = float(
            controller.get("prediction_min_error_ratio", 2.0)
        )
        self.prediction_full_error_ratio = float(
            controller.get(
                "prediction_full_error_ratio",
                self.prediction_min_error_ratio + 4.0,
            )
        )
        self.prediction_confirm_samples = int(
            controller.get("prediction_confirm_samples", 1)
        )
        self.prediction_min_rate_norm_s = float(
            controller.get("prediction_min_rate_norm_s", 0.0)
        )
        self.damping_horizon_s = float(
            controller.get("damping_horizon_s", 0.0)
        )
        self.max_damping_deadband_multiple = float(
            controller.get("max_damping_deadband_multiple", 1.0)
        )
        zones = controller.get("position_zones", {})
        self.position_zones = {
            "fine": {
                "max_error_ratio": float(
                    zones.get("fine", {}).get("max_error_ratio", 3.0)
                ),
                "proportional_gain": float(
                    zones.get("fine", {}).get("proportional_gain", 0.35)
                ),
                "max_correction_deg": float(
                    zones.get("fine", {}).get("max_correction_deg", 0.003)
                ),
                "max_target_lead_deg": float(
                    zones.get("fine", {}).get(
                        "max_target_lead_deg", self.default_max_target_lead_deg
                    )
                ),
                "min_target_lead_deg": float(
                    zones.get("fine", {}).get(
                        "min_target_lead_deg",
                        zones.get("fine", {}).get(
                            "max_target_lead_deg", self.default_max_target_lead_deg
                        ),
                    )
                ),
                "full_lead_error_ratio": float(
                    zones.get("fine", {}).get("full_lead_error_ratio", 3.0)
                ),
                "command_rate_hz": float(
                    zones.get("fine", {}).get("command_rate_hz", 25.0)
                ),
                "reset_target_each_command": bool(
                    zones.get("fine", {}).get("reset_target_each_command", True)
                ),
            },
            "medium": {
                "max_error_ratio": float(
                    zones.get("medium", {}).get("max_error_ratio", 10.0)
                ),
                "proportional_gain": float(
                    zones.get("medium", {}).get("proportional_gain", 0.65)
                ),
                "max_correction_deg": float(
                    zones.get("medium", {}).get("max_correction_deg", 0.009)
                ),
                "max_target_lead_deg": float(
                    zones.get("medium", {}).get(
                        "max_target_lead_deg", self.default_max_target_lead_deg
                    )
                ),
                "min_target_lead_deg": float(
                    zones.get("medium", {}).get(
                        "min_target_lead_deg",
                        zones.get("medium", {}).get(
                            "max_target_lead_deg", self.default_max_target_lead_deg
                        ),
                    )
                ),
                "full_lead_error_ratio": float(
                    zones.get("medium", {}).get("full_lead_error_ratio", 10.0)
                ),
                "command_rate_hz": float(
                    zones.get("medium", {}).get("command_rate_hz", 80.0)
                ),
                "reset_target_each_command": bool(
                    zones.get("medium", {}).get("reset_target_each_command", False)
                ),
            },
            "far": {
                "proportional_gain": float(
                    zones.get("far", {}).get(
                        "proportional_gain", self.proportional_gain
                    )
                ),
                "max_correction_deg": float(
                    zones.get("far", {}).get(
                        "max_correction_deg", self.max_single_correction_deg
                    )
                ),
                "max_target_lead_deg": float(
                    zones.get("far", {}).get(
                        "max_target_lead_deg", self.default_max_target_lead_deg
                    )
                ),
                "min_target_lead_deg": float(
                    zones.get("far", {}).get(
                        "min_target_lead_deg",
                        zones.get("far", {}).get(
                            "max_target_lead_deg", self.default_max_target_lead_deg
                        ),
                    )
                ),
                "full_lead_error_ratio": float(
                    zones.get("far", {}).get("full_lead_error_ratio", 20.0)
                ),
                "command_rate_hz": float(
                    zones.get("far", {}).get(
                        "command_rate_hz", self.command_rate_hz
                    )
                ),
                "reset_target_each_command": bool(
                    zones.get("far", {}).get("reset_target_each_command", False)
                ),
            },
        }
        for zone_name, zone in self.position_zones.items():
            requested_limit = float(zone["max_correction_deg"])
            if requested_limit > self.max_single_correction_deg:
                zone["max_correction_deg"] = self.max_single_correction_deg
                print(
                    "[PSD分区高速][参数修正] "
                    f"{zone_name}区单次修正上限 {requested_limit:.6f}° "
                    f"超过全局上限 {self.max_single_correction_deg:.6f}°，"
                    "已按全局上限运行"
                )
        self._validate_config()

        self.enabled = False
        self.state = TrackerState.STOPPED
        self._filtered_norm: Optional[Tuple[float, float]] = None
        self._consecutive_valid = 0
        self._consecutive_invalid = 0
        self._release_violation_count = 0
        self._lock_candidate_count = 0
        self._previous_error: Optional[Tuple[float, float]] = None
        self._previous_sample_time_s: Optional[float] = None
        self._error_rate_norm_s = (0.0, 0.0)
        self._prediction_direction = [0, 0]
        self._prediction_streak = [0, 0]

    @classmethod
    def from_json(cls, path: str | Path) -> "PsdTrackingController":
        config = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(config)

    def _validate_config(self) -> None:
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must satisfy 0 < alpha <= 1")
        if self.release_deadband_multiplier < 1.0:
            raise ValueError("release_deadband_multiplier must be at least 1")
        if self.release_confirm_samples < 1:
            raise ValueError("release_confirm_samples must be positive")
        if self.lock_confirm_samples < 1:
            raise ValueError("lock_confirm_samples must be positive")
        if self.acquire_valid_samples < 1:
            raise ValueError("acquire_valid_samples must be positive")
        if self.lost_after_invalid_samples < 1:
            raise ValueError("lost_after_invalid_samples must be positive")
        if self.motor_step_deg <= 0.0:
            raise ValueError("motor_step_deg must be positive")
        if self.max_single_correction_deg < self.motor_step_deg:
            raise ValueError("maximum correction cannot be smaller than one motor step")
        if not 0.0 <= self.axis_command_threshold_deg <= self.max_single_correction_deg:
            raise ValueError("axis command threshold is outside the correction range")
        if not 0.0 < self.proportional_gain <= 1.0:
            raise ValueError("proportional_gain must satisfy 0 < Kp <= 1")
        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        if not 0.0 <= self.prediction_horizon_s <= 0.1:
            raise ValueError("prediction_horizon_s must be between 0 and 0.1")
        if not 0.0 < self.derivative_alpha <= 1.0:
            raise ValueError("derivative_alpha must satisfy 0 < alpha <= 1")
        if self.max_prediction_deadband_multiple < 0.0:
            raise ValueError("max_prediction_deadband_multiple cannot be negative")
        if self.prediction_min_error_ratio < 1.0:
            raise ValueError("prediction_min_error_ratio must be at least 1")
        if self.prediction_full_error_ratio < self.prediction_min_error_ratio:
            raise ValueError(
                "prediction_full_error_ratio cannot be below the prediction threshold"
            )
        if self.prediction_confirm_samples < 1:
            raise ValueError("prediction_confirm_samples must be positive")
        if self.prediction_min_rate_norm_s < 0.0:
            raise ValueError("prediction_min_rate_norm_s cannot be negative")
        if not 0.0 <= self.damping_horizon_s <= 0.1:
            raise ValueError("damping_horizon_s must be between 0 and 0.1")
        if self.max_damping_deadband_multiple < 0.0:
            raise ValueError("max_damping_deadband_multiple cannot be negative")
        fine = self.position_zones["fine"]
        medium = self.position_zones["medium"]
        if not 1.0 <= fine["max_error_ratio"] < medium["max_error_ratio"]:
            raise ValueError("position zone error ratios must be increasing")
        for name, zone in self.position_zones.items():
            if not 0.0 < zone["proportional_gain"] <= 1.0:
                raise ValueError(f"{name} zone proportional gain is invalid")
            if not self.motor_step_deg <= zone["max_correction_deg"] <= self.max_single_correction_deg:
                raise ValueError(
                    f"{name} zone correction limit is invalid: "
                    f"motor_step={self.motor_step_deg}, "
                    f"zone_limit={zone['max_correction_deg']}, "
                    f"global_limit={self.max_single_correction_deg}"
                )
            if zone["max_target_lead_deg"] <= 0.0:
                raise ValueError(f"{name} zone target lead is invalid")
            if not 0.0 < zone["min_target_lead_deg"] <= zone["max_target_lead_deg"]:
                raise ValueError(f"{name} zone dynamic target lead range is invalid")
            if zone["full_lead_error_ratio"] <= 0.0:
                raise ValueError(f"{name} zone full lead error ratio is invalid")
            if not 0.0 < zone["command_rate_hz"] <= self.command_rate_hz:
                raise ValueError(f"{name} zone command rate is invalid")
        if any(value <= 0.0 for value in self.deadband_norm):
            raise ValueError("deadband values must be positive")

    def start(self) -> None:
        self.enabled = True
        self.state = TrackerState.ACQUIRING
        self._filtered_norm = None
        self._consecutive_valid = 0
        self._consecutive_invalid = 0
        self._release_violation_count = 0
        self._lock_candidate_count = 0
        self._previous_error = None
        self._previous_sample_time_s = None
        self._error_rate_norm_s = (0.0, 0.0)
        self._prediction_direction = [0, 0]
        self._prediction_streak = [0, 0]

    def stop(self) -> None:
        self.enabled = False
        self.state = TrackerState.STOPPED
        self._consecutive_valid = 0
        self._consecutive_invalid = 0
        self._release_violation_count = 0
        self._lock_candidate_count = 0
        self._previous_error = None
        self._previous_sample_time_s = None
        self._error_rate_norm_s = (0.0, 0.0)
        self._prediction_direction = [0, 0]
        self._prediction_streak = [0, 0]

    def _reset_prediction_gate(self) -> None:
        self._prediction_direction = [0, 0]
        self._prediction_streak = [0, 0]

    def _no_move(self, reason: str) -> TrackingUpdate:
        error = None
        if self._filtered_norm is not None:
            error = (
                self._filtered_norm[0] - self.target_norm[0],
                self._filtered_norm[1] - self.target_norm[1],
            )
        return TrackingUpdate(
            state=self.state,
            filtered_norm=self._filtered_norm,
            error_norm=error,
            continuous_correction_deg=(0.0, 0.0),
            step_correction_deg=(0.0, 0.0),
            should_move=False,
            reason=reason,
        )

    def _filter(self, measured: Tuple[float, float]) -> Tuple[float, float]:
        if self._filtered_norm is None:
            self._filtered_norm = measured
        else:
            alpha = self.filter_alpha
            self._filtered_norm = (
                alpha * measured[0] + (1.0 - alpha) * self._filtered_norm[0],
                alpha * measured[1] + (1.0 - alpha) * self._filtered_norm[1],
            )
        return self._filtered_norm

    def _select_position_zone(self, error: Tuple[float, float]):
        error_ratio = max(
            abs(error[0]) / self.deadband_norm[0],
            abs(error[1]) / self.deadband_norm[1],
        )
        if error_ratio <= self.position_zones["fine"]["max_error_ratio"]:
            return "fine", self.position_zones["fine"]
        if error_ratio <= self.position_zones["medium"]["max_error_ratio"]:
            return "medium", self.position_zones["medium"]
        return "far", self.position_zones["far"]

    def _dynamic_target_lead(
        self,
        zone_name: str,
        zone: Mapping[str, Any],
        error_ratio: float,
    ) -> Tuple[float, float]:
        if zone_name == "fine":
            zone_start_ratio = 1.0
        elif zone_name == "medium":
            zone_start_ratio = self.position_zones["fine"]["max_error_ratio"]
        else:
            zone_start_ratio = self.position_zones["medium"]["max_error_ratio"]
        full_ratio = max(
            float(zone["full_lead_error_ratio"]),
            zone_start_ratio + 1e-9,
        )
        scale = min(
            1.0,
            max(0.0, (error_ratio - zone_start_ratio) / (full_ratio - zone_start_ratio)),
        )
        minimum = float(zone["min_target_lead_deg"])
        maximum = float(zone["max_target_lead_deg"])
        return minimum + (maximum - minimum) * scale, scale

    def _quantize_axis(self, correction: float, max_correction_deg: float) -> float:
        if abs(correction) < self.axis_command_threshold_deg:
            return 0.0
        magnitude = min(abs(correction), max_correction_deg)
        steps = max(1, int(round(magnitude / self.motor_step_deg)))
        quantized = min(steps * self.motor_step_deg, max_correction_deg)
        return math.copysign(quantized, correction)

    def update(
        self,
        x_normalized: Optional[float],
        y_normalized: Optional[float],
        *,
        valid: bool,
        sample_time_s: Optional[float] = None,
    ) -> TrackingUpdate:
        if not self.enabled:
            return self._no_move("disabled")

        finite = (
            x_normalized is not None
            and y_normalized is not None
            and math.isfinite(float(x_normalized))
            and math.isfinite(float(y_normalized))
        )
        if not valid or not finite:
            self._reset_prediction_gate()
            self._consecutive_valid = 0
            self._consecutive_invalid += 1
            self._release_violation_count = 0
            self._lock_candidate_count = 0
            self._previous_error = None
            self._previous_sample_time_s = None
            self._error_rate_norm_s = (0.0, 0.0)
            if self._consecutive_invalid >= self.lost_after_invalid_samples:
                self.state = TrackerState.SIGNAL_LOST
                # The spot may reappear at a different location after a long
                # optical dropout. Never blend that fresh reading with the
                # last position observed before losing the beam.
                self._filtered_norm = None
            else:
                self.state = TrackerState.ACQUIRING
            return self._no_move("invalid_psd")

        self._consecutive_invalid = 0
        self._consecutive_valid += 1
        filtered = self._filter((float(x_normalized), float(y_normalized)))

        # update() runs on the motor-feedback control clock, not on every PSD
        # acquisition. Real sample timestamps take precedence below.
        nominal_dt = 1.0 / self.command_rate_hz
        derivative_dt = nominal_dt
        current_sample_time_s: Optional[float] = None
        if sample_time_s is not None and math.isfinite(float(sample_time_s)):
            current_sample_time_s = float(sample_time_s)
            if self._previous_sample_time_s is not None:
                measured_dt = current_sample_time_s - self._previous_sample_time_s
                if math.isfinite(measured_dt) and measured_dt > 0.0:
                    derivative_dt = measured_dt
            self._previous_sample_time_s = current_sample_time_s
        else:
            self._previous_sample_time_s = None

        if self._consecutive_valid < self.acquire_valid_samples:
            self.state = TrackerState.ACQUIRING
            return self._no_move("waiting_for_valid_samples")

        error = (
            filtered[0] - self.target_norm[0],
            filtered[1] - self.target_norm[1],
        )
        if self._previous_error is None:
            self._error_rate_norm_s = (0.0, 0.0)
        else:
            raw_rate = (
                (error[0] - self._previous_error[0]) / derivative_dt,
                (error[1] - self._previous_error[1]) / derivative_dt,
            )
            alpha_d = self.derivative_alpha
            self._error_rate_norm_s = (
                alpha_d * raw_rate[0]
                + (1.0 - alpha_d) * self._error_rate_norm_s[0],
                alpha_d * raw_rate[1]
                + (1.0 - alpha_d) * self._error_rate_norm_s[1],
            )
        self._previous_error = error
        inside_lock_deadband = (
            abs(error[0]) <= self.deadband_norm[0]
            and abs(error[1]) <= self.deadband_norm[1]
        )
        inside_release_deadband = (
            abs(error[0]) <= self.deadband_norm[0] * self.release_deadband_multiplier
            and abs(error[1]) <= self.deadband_norm[1] * self.release_deadband_multiplier
        )
        if inside_lock_deadband:
            self._reset_prediction_gate()
            self._release_violation_count = 0
            if self.state != TrackerState.LOCKED:
                self._lock_candidate_count += 1
                if self._lock_candidate_count < self.lock_confirm_samples:
                    if self.state in (TrackerState.ACQUIRING, TrackerState.SIGNAL_LOST):
                        self.state = TrackerState.ACQUIRING
                    else:
                        self.state = TrackerState.TRACKING
                    return TrackingUpdate(
                        state=self.state,
                        filtered_norm=filtered,
                        error_norm=error,
                        continuous_correction_deg=(0.0, 0.0),
                        step_correction_deg=(0.0, 0.0),
                        should_move=False,
                        reason="confirming_lock",
                    )
            self.state = TrackerState.LOCKED
            return TrackingUpdate(
                state=self.state,
                filtered_norm=filtered,
                error_norm=error,
                continuous_correction_deg=(0.0, 0.0),
                step_correction_deg=(0.0, 0.0),
                should_move=False,
                reason="inside_deadband",
            )

        self._lock_candidate_count = 0

        if self.state == TrackerState.LOCKED:
            if inside_release_deadband:
                self._reset_prediction_gate()
                self._release_violation_count = 0
                return TrackingUpdate(
                    state=self.state,
                    filtered_norm=filtered,
                    error_norm=error,
                    continuous_correction_deg=(0.0, 0.0),
                    step_correction_deg=(0.0, 0.0),
                    should_move=False,
                    reason="inside_release_hysteresis",
                )
            self._release_violation_count += 1
            if self._release_violation_count < self.release_confirm_samples:
                return TrackingUpdate(
                    state=self.state,
                    filtered_norm=filtered,
                    error_norm=error,
                    continuous_correction_deg=(0.0, 0.0),
                    step_correction_deg=(0.0, 0.0),
                    should_move=False,
                    reason="confirming_release",
                )

        self._release_violation_count = 0

        zone_name, zone = self._select_position_zone(error)
        zone_gain = zone["proportional_gain"]
        zone_limit = zone["max_correction_deg"]
        error_ratio = max(
            abs(error[0]) / self.deadband_norm[0],
            abs(error[1]) / self.deadband_norm[1],
        )
        control_error_values = list(error)
        damping_active = False
        if self.damping_horizon_s > 0.0:
            for axis in range(2):
                rate = self._error_rate_norm_s[axis]
                # When the spot is already returning to center, shorten the
                # apparent error according to closing speed. Clamp the damping
                # contribution and never allow it to reverse the error sign.
                if error[axis] * rate < 0.0:
                    maximum_damping = (
                        self.deadband_norm[axis]
                        * self.max_damping_deadband_multiple
                    )
                    damping_delta = max(
                        -maximum_damping,
                        min(maximum_damping, rate * self.damping_horizon_s),
                    )
                    candidate = error[axis] + damping_delta
                    if error[axis] * candidate < 0.0:
                        candidate = 0.0
                    if candidate != error[axis]:
                        damping_active = True
                        control_error_values[axis] = candidate

        prediction_active = False
        if (
            zone_name == "far"
            and
            self.prediction_horizon_s > 0.0
            and error_ratio >= self.prediction_min_error_ratio
        ):
            ratio_span = max(
                self.prediction_full_error_ratio
                - self.prediction_min_error_ratio,
                1e-9,
            )
            prediction_scale = min(
                1.0,
                max(
                    0.0,
                    (error_ratio - self.prediction_min_error_ratio) / ratio_span,
                ),
            )
            for axis in range(2):
                rate = self._error_rate_norm_s[axis]
                moving_away = (
                    error[axis] * rate > 0.0
                    and abs(rate) >= self.prediction_min_rate_norm_s
                )
                if moving_away:
                    direction = 1 if rate > 0.0 else -1
                    if self._prediction_direction[axis] == direction:
                        self._prediction_streak[axis] += 1
                    else:
                        self._prediction_direction[axis] = direction
                        self._prediction_streak[axis] = 1
                else:
                    self._prediction_direction[axis] = 0
                    self._prediction_streak[axis] = 0

                # 只预测持续远离中心的分量；回中心或瞬时翻转立即取消预测。
                if (
                    moving_away
                    and self._prediction_streak[axis]
                    >= self.prediction_confirm_samples
                ):
                    maximum_lead = (
                        self.deadband_norm[axis]
                        * self.max_prediction_deadband_multiple
                    )
                    lead = max(
                        -maximum_lead,
                        min(
                            maximum_lead,
                            rate * self.prediction_horizon_s * prediction_scale,
                        ),
                    )
                    control_error_values[axis] += lead
                    if lead != 0.0:
                        prediction_active = True
        else:
            self._reset_prediction_gate()
        control_error = (control_error_values[0], control_error_values[1])
        dynamic_lead_deg, dynamic_lead_scale = self._dynamic_target_lead(
            zone_name,
            zone,
            error_ratio,
        )
        # The calibrated inverse matrix converts PSD error to a full motor
        # correction. Each zone applies its own gain and movement limit.
        continuous = (
            zone_gain
            * (
                self.correction_matrix[0][0] * control_error[0]
                + self.correction_matrix[0][1] * control_error[1]
            ),
            zone_gain
            * (
                self.correction_matrix[1][0] * control_error[0]
                + self.correction_matrix[1][1] * control_error[1]
            ),
        )
        step = (
            self._quantize_axis(continuous[0], zone_limit),
            self._quantize_axis(continuous[1], zone_limit),
        )

        # If the PSD is outside the deadband but both coupled motor components
        # fall below the per-axis threshold, move the stronger component by one
        # minimum step so the loop cannot stall forever.
        if step == (0.0, 0.0):
            axis = 0 if abs(continuous[0]) >= abs(continuous[1]) else 1
            forced = math.copysign(self.motor_step_deg, continuous[axis])
            step = (forced, 0.0) if axis == 0 else (0.0, forced)

        self.state = TrackerState.TRACKING
        return TrackingUpdate(
            state=self.state,
            filtered_norm=filtered,
            error_norm=error,
            continuous_correction_deg=continuous,
            step_correction_deg=step,
            should_move=True,
            reason="correction_ready",
            control_zone=zone_name,
            reset_target=bool(zone["reset_target_each_command"]),
            predicted_error_norm=control_error,
            error_rate_norm_s=self._error_rate_norm_s,
            max_target_lead_deg=dynamic_lead_deg,
            command_rate_hz=float(zone["command_rate_hz"]),
            prediction_active=prediction_active,
            damping_active=damping_active,
            dynamic_lead_scale=dynamic_lead_scale,
        )
