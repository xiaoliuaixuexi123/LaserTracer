"""Offline smoke test for version 10; no hardware is opened."""

import json
import serial
import threading
import time
from copy import deepcopy
from pathlib import Path

from ad7606_reader import Ad7606Sample
from psd_tracker import (
    PsdTrackingController,
    TrackerState,
    feedback_relative_target,
)
from tracking_service import PsdTrackingService, _position_step_time_scale
from raspberrypi_to_pyside import RaspberryPiDeviceServer


config_path = Path(__file__).with_name("psd_calibration.json")
config = json.loads(config_path.read_text(encoding="utf-8"))
assert config["controller"]["sample_rate_hz"] == 1000.0
assert config["controller"]["acquisition_backend"] == "thread"
assert config["controller"]["command_rate_hz"] == 290.0
assert config["controller"]["control_clock"] == "motor_feedback"
assert config["controller"]["motor_link_rate_hz"] == 290.0
assert config["controller"]["position_step_reference_rate_hz"] == 200.0
assert config["controller"]["status_update_rate_hz"] == 5.0
assert config["controller"]["status_log_rate_hz"] == 1.0
assert config["controller"]["motion_log_rate_hz"] == 1.0
assert config["controller"]["motor_feedback_log_rate_hz"] == 1.0
assert {
    zone["command_rate_hz"]
    for zone in config["controller"]["position_zones"].values()
} == {290.0}
assert RaspberryPiDeviceServer.DEFAULT_POLE_PAIRS == 21
parameter_server = object.__new__(RaspberryPiDeviceServer)
parameter_server.motors = {
    "motor1": {
        "model": "HO7213", "canid": "01", "pole_pairs": 21,
        "Kp": 6, "Kd": 240, "velocity": 1.0, "currency": 3.0,
    },
    "motor2": {
        "model": "HO7213", "canid": "02", "pole_pairs": 21,
        "Kp": 16, "Kd": 160, "velocity": 1.0, "currency": 3.0,
    },
}
assert RaspberryPiDeviceServer._validate_tracking_motor_parameters(
    parameter_server
) == []
parameter_payload = RaspberryPiDeviceServer._build_canfd_payload(
    parameter_server,
    parameter_server.motors["motor1"],
    position=0.0,
    velocity=1.0,
    current=3.0,
    mode=RaspberryPiDeviceServer.MOTOR_MODE_POSITION,
    run=True,
)
assert parameter_payload[11] == RaspberryPiDeviceServer.MOTOR_MODE_POSITION
assert parameter_payload[12] == 1
assert parameter_payload[13] == 6
assert parameter_payload[14] == 240
assert int.from_bytes(parameter_payload[9:11], "big", signed=True) == 983
auto_upload_payload = bytearray(8)
auto_upload_payload[0:3] = bytes((0x00, 0x06, 0x01))
auto_upload_command = RaspberryPiDeviceServer._format_canfd_extended_command(
    parameter_server,
    "01",
    auto_upload_payload,
)
assert auto_upload_command == "D000000018000601" + "00" * 5 + "\r"
parameter_server._motor_auto_upload_variants = {}
assert RaspberryPiDeviceServer._format_auto_upload_command(
    parameter_server, "motor1", True
) == "D000000018000601" + "00" * 5 + "\r"
try:
    RaspberryPiDeviceServer._format_auto_upload_command(
        parameter_server, "motor1", True, "canfd_extended_brs_dlc8"
    )
    raise AssertionError("Undocumented auto-upload variants must be rejected")
except ValueError:
    pass
assert RaspberryPiDeviceServer._format_auto_upload_command(
    parameter_server, "motor1", True, "canfd_extended_dlc16_padded"
) == "D00000001A000601" + "00" * 13 + "\r"
try:
    parameter_server._format_auto_upload_command(
        "motor1", True, "canfd_extended_dlc16_legacy_index05"
    )
    raise AssertionError('CAN2.0 index05 must not be guessed in CANFD mode')
except ValueError:
    pass

invalid_parameter_server = object.__new__(RaspberryPiDeviceServer)
invalid_parameter_server.motors = deepcopy(parameter_server.motors)
invalid_parameter_server.motors["motor2"] = dict(
    invalid_parameter_server.motors["motor2"], pole_pairs=14, currency=4.5
)
parameter_errors = RaspberryPiDeviceServer._validate_tracking_motor_parameters(
    invalid_parameter_server
)
assert any("pole_pairs must be 21" in error for error in parameter_errors)
assert any("current limit" in error for error in parameter_errors)
assert hasattr(RaspberryPiDeviceServer, "_run_safe_rate_benchmark")
assert hasattr(RaspberryPiDeviceServer, "_benchmark_psd_until")
assert hasattr(RaspberryPiDeviceServer, "_benchmark_motor_until")
assert hasattr(RaspberryPiDeviceServer, "_run_motor_position_step_test")
assert hasattr(RaspberryPiDeviceServer, "_run_motor_position_step_test_once")
assert hasattr(RaspberryPiDeviceServer, "_analyze_motor_step_samples")
assert hasattr(RaspberryPiDeviceServer, "_summarize_motor_step_trials")
assert hasattr(RaspberryPiDeviceServer, "_start_motor_position_step_test_async")
assert hasattr(RaspberryPiDeviceServer, "_start_motor_position_step_test_once_async")
assert hasattr(RaspberryPiDeviceServer, "_set_canfd_auto_upload")
assert hasattr(RaspberryPiDeviceServer, "_test_canfd_auto_upload")
assert hasattr(RaspberryPiDeviceServer, "_observe_upload_phase")
slcan_buffer = bytearray(b"garbage")
slcan_frames, slcan_nacks = RaspberryPiDeviceServer._extract_slcan_frames(
    slcan_buffer,
    b"\x07d0018AABBCCDD\r",
)
assert slcan_nacks == 1
assert slcan_frames == ["d0018AABBCCDD"]
assert slcan_buffer == bytearray()
assert config["commissioning_limits"]["max_single_correction_deg"] == 0.030
assert config["controller"]["max_target_lead_deg"] == 0.040
assert config["controller"]["prediction_horizon_s"] == 0.000
assert config["controller"]["damping_horizon_s"] == 0.000
assert config["controller"]["position_zones"]["fine"]["proportional_gain"] == 0.08
assert config["controller"]["position_zones"]["medium"]["proportional_gain"] == 0.35
assert config["controller"]["position_zones"]["far"]["proportional_gain"] == 0.55
assert config["controller"]["sample_rate_hz"] == 1000.0
assert config["controller"]["command_rate_hz"] == 290.0
assert config["controller"]["motor_link_rate_hz"] == 290.0
assert config["controller"]["minimum_feedback_rate_hz"] == 250.0
assert config["controller"]["prediction_confirm_samples"] == 4
assert config["controller"]["reversal_confirm_commands"] == 4
assert config["controller"]["release_confirm_samples"] == 19
assert config["controller"]["lock_confirm_samples"] == 9
assert config["controller"]["lost_after_invalid_samples"] == 23
assert config["controller"]["center_hold_min_interval_s"] == 0.250
clamped_config = deepcopy(config)
clamped_config["controller"]["position_zones"]["far"]["max_correction_deg"] = 0.060
clamped_controller = PsdTrackingController(clamped_config)
assert clamped_controller.position_zones["far"]["max_correction_deg"] == 0.030
target = config["tracking_setpoint_norm"]
forward = config["forward_model"]["jacobian_norm_per_deg"]
offsets = config["dark_offsets_v"]

lock_controller = PsdTrackingController(config)
lock_controller.start()
lock_update = None
for _ in range(
    config["controller"]["acquire_valid_samples"]
    + config["controller"]["lock_confirm_samples"]
    - 2
):
    lock_update = lock_controller.update(target["x"], target["y"], valid=True)
assert lock_update is not None
assert lock_update.reason == "confirming_lock"
assert lock_update.state != TrackerState.LOCKED
lock_update = lock_controller.update(target["x"], target["y"], valid=True)
assert lock_update.state == TrackerState.LOCKED
assert lock_update.reason == "inside_deadband"

dropout_controller = PsdTrackingController(config)
dropout_controller.start()
for _ in range(config["controller"]["acquire_valid_samples"]):
    dropout_controller.update(target["x"], target["y"], valid=True)
for _ in range(config["controller"]["lost_after_invalid_samples"] - 1):
    dropout_update = dropout_controller.update(None, None, valid=False)
    assert dropout_update.state != TrackerState.SIGNAL_LOST
dropout_update = dropout_controller.update(None, None, valid=False)
assert dropout_update.state == TrackerState.SIGNAL_LOST

logic_config = deepcopy(config)
logic_config["controller"]["filter_alpha"] = 1.0
logic_config["controller"]["lock_confirm_samples"] = 1
logic_controller = PsdTrackingController(logic_config)
logic_controller.start()
for _ in range(logic_config["controller"]["acquire_valid_samples"]):
    locked = logic_controller.update(target["x"], target["y"], valid=True)
assert locked.state == TrackerState.LOCKED
release_x = (
    target["x"]
    + logic_controller.deadband_norm[0]
    * logic_controller.release_deadband_multiplier
    * 1.1
)
for _ in range(logic_controller.release_confirm_samples - 1):
    confirming = logic_controller.update(release_x, target["y"], valid=True)
    assert confirming.state == TrackerState.LOCKED
    assert not confirming.should_move
released = logic_controller.update(release_x, target["y"], valid=True)
assert released.state == TrackerState.TRACKING
assert released.should_move

fine_controller = PsdTrackingController(logic_config)
fine_controller.start()
fine_x = target["x"] + forward[0][1] * 0.006
fine_y = target["y"] + forward[1][1] * 0.006
for _ in range(logic_config["controller"]["acquire_valid_samples"]):
    fine_update = fine_controller.update(fine_x, fine_y, valid=True)
assert fine_update.should_move
assert fine_update.step_correction_deg == (0.0, -0.001)
assert fine_update.control_zone == "fine"
assert fine_update.reset_target

proportional_controller = PsdTrackingController(logic_config)
proportional_controller.start()
proportional_x = target["x"] + forward[0][1] * 0.018
proportional_y = target["y"] + forward[1][1] * 0.018
for _ in range(logic_config["controller"]["acquire_valid_samples"]):
    proportional_update = proportional_controller.update(
        proportional_x, proportional_y, valid=True
    )
assert proportional_update.should_move
assert abs(proportional_update.continuous_correction_deg[0]) < 1e-9
assert abs(proportional_update.continuous_correction_deg[1] + 0.0063) < 1e-9
assert proportional_update.step_correction_deg == (0.0, -0.006)
assert proportional_update.control_zone == "medium"
assert proportional_update.reset_target
assert 0.003 < proportional_update.max_target_lead_deg < 0.012
assert proportional_update.command_rate_hz == 290.0
assert not proportional_update.prediction_active

fast_controller = PsdTrackingController(logic_config)
fast_controller.start()
fast_x = target["x"] + forward[0][1] * 0.100
fast_y = target["y"] + forward[1][1] * 0.100
for _ in range(logic_config["controller"]["acquire_valid_samples"]):
    fast_update = fast_controller.update(fast_x, fast_y, valid=True)
assert fast_update.should_move
assert fast_update.step_correction_deg == (0.0, -0.030)
assert fast_update.control_zone == "far"
assert fast_update.max_target_lead_deg == 0.040
assert fast_update.command_rate_hz == 290.0

prediction_config = deepcopy(logic_config)
prediction_config["controller"]["acquire_valid_samples"] = 1
prediction_config["controller"]["release_confirm_samples"] = 1
prediction_config["controller"]["derivative_alpha"] = 1.0
prediction_config["controller"]["prediction_confirm_samples"] = 3
prediction_config["controller"]["prediction_horizon_s"] = 0.030
prediction_controller = PsdTrackingController(prediction_config)
prediction_controller.start()
prediction_controller.update(target["x"], target["y"], valid=True)
prediction_update = None
for multiplier in (7.0, 8.0, 9.0, 10.0):
    moving_x = target["x"] + prediction_controller.deadband_norm[0] * multiplier
    prediction_update = prediction_controller.update(moving_x, target["y"], valid=True)
assert prediction_update is not None
assert prediction_update.should_move
assert prediction_update.predicted_error_norm is not None
assert abs(prediction_update.predicted_error_norm[0]) > abs(prediction_update.error_norm[0])
assert prediction_update.control_zone == "far"
assert prediction_update.prediction_active

damping_config = deepcopy(logic_config)
damping_config["controller"]["acquire_valid_samples"] = 1
damping_config["controller"]["release_confirm_samples"] = 1
damping_config["controller"]["derivative_alpha"] = 1.0
damping_config["controller"]["damping_horizon_s"] = 0.015
damping_controller = PsdTrackingController(damping_config)
damping_controller.start()
damping_controller.update(target["x"], target["y"], valid=True)
damping_controller.update(
    target["x"] + damping_controller.deadband_norm[0] * 10.0,
    target["y"],
    valid=True,
)
damping_update = damping_controller.update(
    target["x"] + damping_controller.deadband_norm[0] * 8.0,
    target["y"],
    valid=True,
)
assert damping_update.damping_active
assert not damping_update.prediction_active
assert abs(damping_update.predicted_error_norm[0]) < abs(damping_update.error_norm[0])

timed_config = deepcopy(logic_config)
timed_config["controller"]["acquire_valid_samples"] = 1
timed_config["controller"]["release_confirm_samples"] = 1
timed_config["controller"]["derivative_alpha"] = 1.0
timed_controller = PsdTrackingController(timed_config)
timed_controller.start()
timed_controller.update(
    target["x"] + 0.100,
    target["y"],
    valid=True,
    sample_time_s=10.000,
)
timed_update = timed_controller.update(
    target["x"] + 0.110,
    target["y"],
    valid=True,
    sample_time_s=10.010,
)
assert abs(timed_update.error_rate_norm_s[0] - 1.0) < 1e-9
timed_controller.update(None, None, valid=False, sample_time_s=10.020)
timed_update = timed_controller.update(
    target["x"] + 0.120,
    target["y"],
    valid=True,
    sample_time_s=20.000,
)
assert timed_update.error_rate_norm_s == (0.0, 0.0)

adequate_step = RaspberryPiDeviceServer._analyze_motor_step_samples(
    [(0.005, 0.000), (0.020, 0.020), (0.040, 0.040),
     (0.060, 0.048), (0.080, 0.050), (0.120, 0.050)],
    0.000,
    0.050,
)
assert adequate_step["assessment"] == "adequate"
assert adequate_step["command_to_first_feedback_ms"] == 5.0
assert adequate_step["first_motion_latency_ms"] == 20.0
assert adequate_step["rise_time_10_ms"] == 20.0
assert adequate_step["rise_time_90_ms"] == 60.0
assert adequate_step["overshoot_percent"] == 0.0

slow_step = RaspberryPiDeviceServer._analyze_motor_step_samples(
    [(0.010, 0.000), (0.100, 0.010), (0.200, 0.020),
     (0.300, 0.030), (0.400, 0.035)],
    0.000,
    0.050,
)
assert slow_step["assessment"] == "slow"
assert slow_step["rise_time_90_ms"] is None

underdamped_step = RaspberryPiDeviceServer._analyze_motor_step_samples(
    [(0.005, 0.000), (0.020, 0.040), (0.040, 0.060),
     (0.060, 0.040), (0.080, 0.058), (0.120, 0.050)],
    0.000,
    0.050,
)
assert underdamped_step["assessment"] == "underdamped"
assert underdamped_step["overshoot_percent"] >= 20.0

identification_trials = []
for step_deg, rise_ms in ((0.05, 250.0), (0.10, 260.0), (0.20, 270.0)):
    phase = {
        "rise_time_90_ms": rise_ms,
        "settling_time_ms": rise_ms + 100.0,
        "overshoot_percent": 0.0,
        "target_crossings": 0,
        "feedback_hz": 100.0,
    }
    identification_trials.append({
        "step_deg": step_deg,
        "repetition": 1,
        "outward": dict(phase),
        "return": dict(phase),
    })
identification_summary = RaspberryPiDeviceServer._summarize_motor_step_trials(
    identification_trials,
    (0.05, 0.10, 0.20),
)
assert identification_summary["limitation"] == "position_loop_or_filter_limited"
assert identification_summary["assessment"] == "slow"

assert abs(_position_step_time_scale(1.0 / 250.0, 200.0) - 0.8) < 1e-12
assert abs(_position_step_time_scale(1.0 / 1000.0, 200.0) - 0.2) < 1e-12
assert _position_step_time_scale(1.0 / 200.0, 200.0) == 1.0
assert _position_step_time_scale(1.0 / 100.0, 200.0) == 1.0
assert abs(_position_step_time_scale(0.001, 200.0) - 0.2) < 1e-12

assert feedback_relative_target(0.000, 0.010, 0.010) == 0.010
assert feedback_relative_target(0.005, 0.010, 0.010) == 0.015
assert feedback_relative_target(0.000, -0.006, 0.010) == -0.006

fake_server = object.__new__(RaspberryPiDeviceServer)
fake_server.motor_transaction_lock = threading.RLock()
fake_server._tracking_target_lock = threading.RLock()
fake_server.motor_feedback_condition = threading.Condition()
fake_server._motor_tx_condition = threading.Condition()
fake_server.motor_serial = None
fake_server.current_positions = {"motor1": 0.0, "motor2": 0.0}
fake_server.current_targets = {"motor1": 0.0, "motor2": 0.0}
fake_server._tracking_requested_targets = dict(fake_server.current_targets)
fake_server._tracking_tx_pending = None
fake_server._tracking_tx_superseded = 0
fake_server._tracking_max_target_lead_deg = 0.018
fake_server._tracking_feedback_log_period_s = 0.1
fake_server._next_tracking_feedback_log = float("inf")
fake_server._tracking_feedback_stale_warn_s = 0.050
fake_server._tracking_feedback_stale_stop_s = 0.500
fake_server._tracking_stats_started_at = time.monotonic()
fake_server._tracking_tx_batches = 0
fake_server._tracking_tx_frames = 0
fake_server._tracking_batches_since_idle = 0
fake_server._tracking_last_batch_at = None
fake_server._tracking_active_burst_started_at = time.monotonic()
fake_server._tracking_rx_baseline = {"motor1": 0, "motor2": 0}
fake_server._tracking_tx_baseline = 0
fake_server._motor_rx_counts = {"motor1": 0, "motor2": 0}
fake_server._motor_feedback_modes = {
    "motor1": "periodic_auto_upload",
    "motor2": "periodic_auto_upload",
}
fake_server._motor_feedback_timestamps = {
    "motor1": time.monotonic(),
    "motor2": time.monotonic(),
}
fake_server._motor_rx_parse_errors = 0
fake_server._tracking_parse_error_baseline = 0
fake_server._last_tracking_write_ms = 0.0
fake_server._motor_tx_timeouts = 0
fake_server._motor_tx_failures = 0
fake_server._motor_tx_last_error = None
fake_server._motor_tx_dropped_batches = 0
fake_server._motor_tx_rate_hz = 1000.0
fake_server._motor_link_recoveries = 0
sent_batches = []


def fake_send_batch(targets, count_tracking=True):
    sent_batches.append(dict(targets))
    fake_server.current_targets.update(targets)
    if count_tracking:
        fake_server._tracking_tx_batches += 1
        fake_server._tracking_tx_frames += 2
        fake_server._tracking_batches_since_idle += 1
    return 0.1


def fake_queue_batch(targets):
    sent_batches.append(dict(targets))
    fake_server._tracking_requested_targets.update(targets)
    return 0.01


fake_server._send_tracking_target_batch = fake_send_batch
fake_server._queue_tracking_target_batch = fake_queue_batch
pitch_target, yaw_target = RaspberryPiDeviceServer._apply_tracking_step(
    fake_server, 0.0, -0.010, False
)
assert sent_batches == [{"motor1": 0.0, "motor2": -0.010}]
assert pitch_target == 0.0 and yaw_target == -0.010
RaspberryPiDeviceServer._apply_tracking_step(fake_server, 0.0, -0.010, False)
assert sent_batches[-1] == {"motor1": 0.0, "motor2": -0.010}
fake_server.current_positions["motor2"] = 0.0
fake_server.current_targets["motor2"] = 0.0
RaspberryPiDeviceServer._apply_tracking_step(
    fake_server, 0.0, -0.060, False, 0.020
)
assert sent_batches[-1] == {"motor1": 0.0, "motor2": -0.020}
fake_server.current_positions["motor2"] = -0.004
fake_server.current_targets["motor1"] = 0.008
RaspberryPiDeviceServer._apply_tracking_step(fake_server, 0.0, 0.003, True)
assert sent_batches[-1] == {"motor1": 0.0, "motor2": -0.001}

# Automatic upload is independent of position TX, so stale feedback is a real
# fault even after an idle interval.
stale_time = time.monotonic() - 1.0
fake_server._motor_feedback_timestamps = {
    "motor1": stale_time,
    "motor2": stale_time,
}
fake_server._tracking_active_burst_started_at = stale_time
fake_server._tracking_last_batch_at = time.monotonic()
try:
    RaspberryPiDeviceServer._apply_tracking_step(
        fake_server, 0.001, 0.001, False
    )
except RuntimeError as exc:
    assert "Motor feedback stale" in str(exc)
else:
    raise AssertionError("stale automatic feedback must stop tracking")

fake_server.current_positions = {"motor1": 0.002, "motor2": -0.004}
fake_server.current_targets = {"motor1": 0.010, "motor2": -0.010}
hold_pitch, hold_yaw = RaspberryPiDeviceServer._hold_tracking_position(fake_server)
assert (hold_pitch, hold_yaw) == (0.002, -0.004)
assert sent_batches[-1] == {"motor1": 0.002, "motor2": -0.004}


queue_server = object.__new__(RaspberryPiDeviceServer)
queue_server._motor_tx_condition = threading.Condition()
queue_server._tracking_tx_pending = None
queue_server._tracking_tx_generation = 0
queue_server._tracking_tx_superseded = 0
queue_server._tracking_requested_targets = {}
RaspberryPiDeviceServer._queue_tracking_target_batch(
    queue_server, {"motor1": 0.1, "motor2": 0.2}
)
RaspberryPiDeviceServer._queue_tracking_target_batch(
    queue_server, {"motor1": 0.3, "motor2": 0.4}
)
assert queue_server._tracking_tx_pending == (
    2, {"motor1": 0.3, "motor2": 0.4}
)
assert queue_server._tracking_tx_superseded == 1
assert queue_server._tracking_requested_targets == {
    "motor1": 0.3, "motor2": 0.4,
}

worker_server = object.__new__(RaspberryPiDeviceServer)
worker_server._motor_tx_condition = threading.Condition()
worker_server._motor_tx_stop_event = threading.Event()
worker_server._motor_tx_thread = None
worker_server._motor_tx_rate_hz = 250.0
worker_server._motor_link_rate_hz = 250.0
worker_server._motor_link_keepalive = False
worker_server._motor_link_watchdog = False
worker_server._motor_link_last_targets = None
worker_server._service_link_watchdog = lambda: None
worker_server._tracking_tx_pending = None
worker_server._tracking_tx_generation = 0
worker_server._tracking_tx_sent_generation = 0
worker_server._tracking_tx_superseded = 0
worker_server._tracking_requested_targets = {}
worker_server.motor_transaction_lock = threading.RLock()
worker_writes = []


def fake_worker_send(targets):
    worker_writes.append(dict(targets))
    return 0.1


worker_server._send_tracking_target_batch = fake_worker_send
RaspberryPiDeviceServer._queue_tracking_target_batch(
    worker_server, {"motor1": 1.0, "motor2": 2.0}
)
RaspberryPiDeviceServer._queue_tracking_target_batch(
    worker_server, {"motor1": 3.0, "motor2": 4.0}
)
RaspberryPiDeviceServer._start_motor_tx_thread(worker_server)
worker_deadline = time.monotonic() + 0.2
while not worker_writes and time.monotonic() < worker_deadline:
    time.sleep(0.001)
worker_server._motor_tx_stop_event.set()
with worker_server._motor_tx_condition:
    worker_server._motor_tx_condition.notify_all()
worker_server._motor_tx_thread.join(0.2)
assert worker_writes == [{"motor1": 3.0, "motor2": 4.0}]
assert worker_server._tracking_tx_sent_generation == 2


class FakeMotorSerial:
    def __init__(self):
        self.is_open = True
        self.out_waiting = 0
        self.writes = []
        self.flush_count = 0
        self.reset_output_count = 0
        self.fail_next_write = False

    def write(self, payload):
        if self.fail_next_write:
            self.fail_next_write = False
            raise serial.SerialTimeoutException("simulated write timeout")
        self.writes.append(payload)
        return len(payload)

    def flush(self):
        self.flush_count += 1

    def reset_output_buffer(self):
        self.reset_output_count += 1


one_shot_server = object.__new__(RaspberryPiDeviceServer)
one_shot_server.motor_transaction_lock = threading.RLock()
one_shot_server.motor_feedback_condition = threading.Condition()
one_shot_server._motor_step_test_stop_event = threading.Event()
one_shot_server._motor_feedback_capture = None
one_shot_server.motors = {
    "motor1": {
        "canid": "01", "Kp": 20, "Kd": 208, "velocity": 2.0,
        "currency": 3.0, "mode": 2, "run": True,
    },
}
one_shot_server.current_positions = {"motor1": 0.0}
one_shot_server.current_targets = {"motor1": 0.0}
one_shot_server._latest_motor_feedback = {}
one_shot_server._motor_feedback_timestamps = {}
one_shot_server._motor_feedback_generation = {}
one_shot_server._motor_rx_counts = {}
one_shot_server._motor_rx_frames = 0
one_shot_calls = []


def fake_one_shot_send(motor, **kwargs):
    one_shot_calls.append(dict(kwargs))

    def publish_feedback():
        for position in (0.000, 0.002, 0.010, 0.030, 0.046, 0.050):
            time.sleep(0.003)
            RaspberryPiDeviceServer._record_motor_feedback(
                one_shot_server,
                "motor1",
                {
                    "position": position,
                    "velocity": 0.5,
                    "current": 0.2,
                    "mode": 2,
                    "run": True,
                },
            )

    threading.Thread(target=publish_feedback, daemon=True).start()
    return "fake"


one_shot_server._send_canfd_motor_frame = fake_one_shot_send
one_shot_result = RaspberryPiDeviceServer._capture_motor_step_phase_auto_upload(
    one_shot_server,
    "motor1",
    0.050,
    0.000,
    0.030,
)
assert len(one_shot_calls) == 1
assert one_shot_result["command_frames_sent"] == 1
assert one_shot_result["auto_upload_feedback"] is True
assert one_shot_result["samples"] == 6
assert one_shot_result["rise_time_90_ms"] is not None


batch_server = object.__new__(RaspberryPiDeviceServer)
batch_server.motor_serial = FakeMotorSerial()
batch_server.serial_lock = threading.Lock()
batch_server.motor_feedback_condition = threading.Condition()
batch_server._motor_rx_counts = {}
batch_server._tracking_rx_baseline = {}
batch_server.motors = {
    "motor1": {
        "canid": "01", "Kp": 18, "Kd": 208, "velocity": 1,
        "currency": 3, "mode": 2, "run": True,
    },
    "motor2": {
        "canid": "02", "Kp": 10, "Kd": 112, "velocity": 1,
        "currency": 3, "mode": 2, "run": True,
    },
}
batch_server.current_targets = {}
batch_server._tracking_serial_backlog_drop_bytes = 512
batch_server._tracking_tx_backoff_s = 0.020
batch_server._tracking_tx_batches = 0
batch_server._tracking_tx_frames = 0
batch_server._tracking_tx_baseline = 0
batch_server._tracking_batches_since_idle = 0
batch_server._tracking_last_batch_at = None
batch_server._tracking_active_burst_started_at = None
batch_server._tracking_feedback_stale_stop_s = 0.500
batch_server._tracking_feedback_log_period_s = 0.2
batch_server._last_tracking_write_ms = 0.0
batch_server._motor_tx_timeouts = 0
batch_server._motor_tx_dropped_batches = 0
batch_server._motor_tx_backoff_until = 0.0
batch_server._motor_link_tx_batches = 0
batch_server._next_motor_tx_warning = float("inf")
RaspberryPiDeviceServer._send_tracking_target_batch(
    batch_server,
    {"motor1": 0.100, "motor2": -0.200},
)
assert len(batch_server.motor_serial.writes) == 1
assert batch_server.motor_serial.writes[0].count(b"\r") == 2
assert len(batch_server.motor_serial.writes[0]) == 76
assert batch_server.motor_serial.flush_count == 0
assert batch_server._tracking_tx_batches == 1
assert batch_server._tracking_tx_frames == 2

last_good_targets = dict(batch_server.current_targets)
batch_server.motor_serial.fail_next_write = True
dropped_write = RaspberryPiDeviceServer._send_tracking_target_batch(
    batch_server,
    {"motor1": 0.200, "motor2": -0.300},
)
assert dropped_write is None
assert batch_server.current_targets == last_good_targets
assert batch_server._motor_tx_timeouts == 1
assert batch_server._motor_tx_dropped_batches == 1
assert batch_server.motor_serial.reset_output_count == 1
assert batch_server.motor_serial.writes[-1] == b"\r"
assert batch_server._tracking_tx_batches == 1

dispatch_server = object.__new__(RaspberryPiDeviceServer)
dispatch_server.motors = {
    "motor1": {"canid": "01", "pole_pairs": 14},
    "motor2": {"canid": "02", "pole_pairs": 14},
}
dispatch_server.motor_feedback_condition = threading.Condition()
dispatch_server.current_positions = {}
dispatch_server._latest_motor_feedback = {}
dispatch_server._motor_feedback_generation = {}
dispatch_server._motor_feedback_timestamps = {}
dispatch_server._motor_rx_counts = {}
dispatch_server._motor_rx_frames = 0
dispatch_server._motor_rx_parse_errors = 0
feedback_payload = bytearray(16)
feedback_payload[11] = 2
feedback_payload[12] = 1
feedback_payload[14] = 25
feedback_payload[15] = 1
feedback_frame = "d064A" + feedback_payload.hex()
RaspberryPiDeviceServer._dispatch_motor_feedback_frame(
    dispatch_server, feedback_frame
)
assert dispatch_server.current_positions["motor1"] == 0.0
assert dispatch_server._motor_feedback_generation["motor1"] == 1
assert dispatch_server._motor_rx_counts["motor1"] == 1
assert dispatch_server._motor_rx_counts.get("motor2", 0) == 0
waited_feedback = RaspberryPiDeviceServer._wait_for_motor_feedback(
    dispatch_server, "motor1", 0, 0.01
)
assert waited_feedback["slave_id"] == 1
assert waited_feedback["mode"] == 2
extended_feedback_frame = "D00000064A" + feedback_payload.hex()
RaspberryPiDeviceServer._dispatch_motor_feedback_frame(
    dispatch_server, extended_feedback_frame
)
brs_feedback_frame = "b064A" + feedback_payload.hex()
RaspberryPiDeviceServer._dispatch_motor_feedback_frame(
    dispatch_server, brs_feedback_frame
)
assert dispatch_server._motor_feedback_generation["motor1"] == 3
assert dispatch_server._motor_rx_raw_types == {"d": 1, "D": 1, "b": 1}
assert dispatch_server._motor_rx_parse_errors == 0

x = target["x"] + forward[0][0] * 0.012
y = target["y"] + forward[1][0] * 0.012
corrected_sum = 2.0
voltages = (
    offsets["x"] + x * corrected_sum,
    offsets["y"] + y * corrected_sum,
    corrected_sum + offsets["sum"],
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
center_voltages = (
    offsets["x"] + target["x"] * corrected_sum,
    offsets["y"] + target["y"] * corrected_sum,
    corrected_sum + offsets["sum"],
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)


class FakeReader:
    def __init__(self):
        self.sequence = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_sample(self):
        self.sequence += 1
        active_voltages = voltages if self.sequence <= 30 else center_voltages
        return Ad7606Sample(
            sequence=self.sequence,
            timestamp_ns=time.monotonic_ns(),
            raw=(0,) * 8,
            voltages=active_voltages,
            busy_completion_seen=True,
        )


commands = []
holds = []
position = [0.0, 0.0]


def apply_step(pitch_delta, yaw_delta, reset_target, max_target_lead_deg):
    commands.append(
        (pitch_delta, yaw_delta, reset_target, max_target_lead_deg)
    )
    position[0] += pitch_delta
    position[1] += yaw_delta
    return position[0], position[1]


def hold_position():
    holds.append(tuple(position))
    return position[0], position[1]


timer_config = deepcopy(config)
timer_config["controller"].update(
    control_clock="timer",
    acquisition_backend="thread",
    sample_rate_hz=500.0,
    command_rate_hz=250.0,
    filter_alpha=0.18,
    release_confirm_samples=16,
    lock_confirm_samples=8,
    acquire_valid_samples=5,
    lost_after_invalid_samples=20,
)
for zone in timer_config["controller"]["position_zones"].values():
    zone["command_rate_hz"] = 250.0
service = PsdTrackingService(config_path, apply_step, hold_position)
service._load_config = lambda: timer_config
service._build_reader = lambda unused_config: FakeReader()
started, message = service.start()
assert started, message

deadline = time.monotonic() + 1.0
while time.monotonic() < deadline:
    status = service.status()
    if status["samples"] >= 80 and len(commands) >= 3 and len(holds) >= 1:
        break
    time.sleep(0.01)

status = service.status()
assert status["running"]
assert status["samples"] >= 80
assert len(commands) >= 3
assert len(holds) >= 1
assert status["measured_sample_rate_hz"] > 100.0
assert status["samples"] > status["control_cycles"]
assert (
    status["measured_sample_rate_hz"]
    > status["measured_control_rate_hz"]
)
assert abs(commands[0][0] + 0.0008) < 1e-12
assert commands[0][2]
# Windows' wait granularity is much coarser than RK3588/Linux. The exact
# 250 Hz deadline is verified from configuration and must be measured on-board;
# this offline test only guards against a blocked control loop.
assert status["measured_control_rate_hz"] > 20.0

stopped, message = service.stop()
assert stopped, message
print("10_position_multirate_fast self-test: PASS")
