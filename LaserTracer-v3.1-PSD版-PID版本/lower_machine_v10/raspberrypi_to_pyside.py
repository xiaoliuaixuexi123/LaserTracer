import socket
import json
import threading
import serial
import time
import math
import statistics
from pathlib import Path

try:
    from .tracking_service import PsdTrackingService
    from .psd_tracker import bounded_accumulated_target
except ImportError:
    from tracking_service import PsdTrackingService
    from psd_tracker import bounded_accumulated_target


class RaspberryPiDeviceServer:
    """RK3588下位机：接收JSON命令并控制电机和激光测距仪。"""

    SOFTWARE_BUILD = '2026-09-22-command-reply-v14'
    CANFD_DLC_16 = 'A'
    CANFD_FRAME_SIZE = 64
    CANFD_POSITION_COUNTS_PER_REV = 1048576
    CANFD_VELOCITY_SCALE = 8388608
    CANFD_VELOCITY_FULL_SCALE_HZ = 1000.0
    CANFD_CURRENT_SCALE = 32768
    CANFD_CURRENT_FULL_SCALE_A = 100.0
    # This installation uses HO7213 motors (21 pole pairs).  Keep the value in
    # each motor configuration as well so velocity command/feedback scaling is
    # explicit at both ends of the TCP link.
    DEFAULT_POLE_PAIRS = 21
    MOTOR_MODE_CURRENT = 0
    MOTOR_MODE_VELOCITY = 1
    MOTOR_MODE_POSITION = 2
    MOTOR_FEEDBACK_POLL_S = 0.0005
    MOTOR_SERIAL_READ_TIMEOUT_S = 0.001
    # The configured command rate is a scheduling target, not a reason to kill
    # tracking. The USB adapter can occasionally need more than one 4 ms slot to
    # accept a 76-byte dual-axis batch, so allow a bounded longer write.
    MOTOR_SERIAL_WRITE_TIMEOUT_S = 0.020

    def __init__(self, host='0.0.0.0', port=8888,
                 motor_port='/dev/ttyACM0', motor_baud=1000000,
                 laser_port='/dev/ttyUSB0', laser_baud=921600):
        self.host = host
        self.port = port
        self.running = False
        self.xy_data = None 
        self.serial_lock = threading.Lock()
        self._client_send_lock = threading.Lock()
        self.motor_transaction_lock = threading.RLock()
        self._tracking_target_lock = threading.RLock()
        self.motor_feedback_condition = threading.Condition()

        # 电机字典必须在反馈线程启动前存在。
        self.motors = {}

        # 记录当前目标位置，仅用于上位机位置命令缓存。
        self.current_targets = {}
        # 记录电机返回的实际位置；跟踪增量必须以反馈位置为基准，避免目标累加超前。
        self.current_positions = {}
        self._latest_motor_feedback = {}
        self._motor_feedback_generation = {}
        self._motor_feedback_timestamps = {}
        self._motor_rx_counts = {}
        self._motor_rx_frames = 0
        self._motor_rx_raw_types = {}
        self._motor_rx_parse_errors = 0
        self._motor_adapter_nacks = 0
        self._motor_rx_stop_event = threading.Event()
        self._motor_rx_thread = None
        self._motor_feedback_modes = {}
        # Serial RX must never perform TCP writes.  A second worker publishes
        # only the newest cached feedback at a bounded rate, so a slow upper
        # machine cannot stall CANFD reception.
        self._motor_feedback_publish_stop_event = threading.Event()
        self._motor_feedback_publish_thread = None
        self._motor_feedback_publish_rate_hz = 5.0
        self._motor_feedback_published_generation = {}
        print(f'[下位机版本] {self.SOFTWARE_BUILD}')
        # The tracking controller only publishes the newest dual-axis target.
        # A dedicated TX worker performs the potentially blocking USB-serial
        # write so it cannot stretch the 4 ms motor-command period.
        self._motor_tx_condition = threading.Condition()
        self._motor_tx_stop_event = threading.Event()
        self._motor_tx_thread = None
        self._motor_tx_rate_hz = 250.0
        self._tracking_tx_pending = None
        self._tracking_tx_generation = 0
        self._tracking_tx_sent_generation = 0
        self._tracking_tx_superseded = 0
        self._tracking_requested_targets = {}
        self._rate_benchmark_lock = threading.Lock()
        self._motor_step_test_stop_event = threading.Event()
        self._motor_step_test_thread = None
        # A passive diagnostic capture is armed only around a one-shot target.
        # The RX thread appends timestamped feedback here; normal tracking keeps
        # using the latest-value fields above and pays only one None check.
        self._motor_feedback_capture = None
        self._tracking_tx_batches = 0
        self._tracking_tx_frames = 0
        self._tracking_batches_since_idle = 0
        self._tracking_last_batch_at = None
        self._tracking_active_burst_started_at = None
        self._tracking_stats_started_at = time.monotonic()
        self._tracking_rx_baseline = {}
        self._tracking_tx_baseline = 0
        self._tracking_parse_error_baseline = 0
        self._last_tracking_write_ms = 0.0
        self._motor_tx_timeouts = 0
        self._motor_tx_failures = 0
        self._motor_tx_last_error = None
        self._motor_tx_dropped_batches = 0
        self._motor_tx_backoff_until = 0.0
        self._next_motor_tx_warning = 0.0

        # 串口配置 - 电机
        try:
            self.motor_serial = serial.Serial(
                port=motor_port,
                baudrate=motor_baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.MOTOR_SERIAL_READ_TIMEOUT_S,
                write_timeout=self.MOTOR_SERIAL_WRITE_TIMEOUT_S,
            )
            print(f"✓ 电机串口 {motor_port} 打开成功")
            self.init_motor_serial()
        except Exception as e:
            print(f"✗ 打开电机串口失败: {e}")
            self.motor_serial = None

        # 串口配置 - 激光测距仪
        try:
            self.laser_serial = serial.Serial(
                port=laser_port,
                baudrate=laser_baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.laser_serial.reset_input_buffer()
            self.laser_serial.reset_output_buffer()
            print(f"✓ 激光测距仪串口 {laser_port} 打开成功")

            # 启动测距仪数据读取线程
            self.laser_data_thread = threading.Thread(target=self._laser_data_reader, daemon=True)
            self.laser_data_thread.start()
        except Exception as e:
            print(f"✗ 打开激光测距仪串口失败: {e}")
            self.laser_serial = None

        # 激光测距仪最新数据
        self.latest_laser_data = None
        self.laser_data_lock = threading.Lock()

        # TCP服务器
        self.server_socket = None
        self.client_socket = None

        # 测距仪数据推送控制
        self.laser_streaming = True  # 是否自动推送测距仪数据

        tracking_config = Path(__file__).with_name('psd_calibration.json')
        self._tracking_feedback_log_period_s = 0.1
        self._next_tracking_feedback_log = 0.0
        self._tracking_max_target_lead_deg = 0.010
        self._tracking_feedback_stale_warn_s = 0.050
        self._tracking_feedback_stale_stop_s = 0.500
        self._tracking_serial_backlog_drop_bytes = 512
        self._tracking_tx_backoff_s = 0.020
        try:
            tracking_settings = json.loads(tracking_config.read_text(encoding='utf-8'))
            tracking_controller = tracking_settings.get('controller', {})
            self._motor_tx_rate_hz = float(
                tracking_controller.get('command_rate_hz', self._motor_tx_rate_hz)
            )
            self.MOTOR_FEEDBACK_POLL_S = float(
                tracking_controller.get(
                    'motor_feedback_poll_s', self.MOTOR_FEEDBACK_POLL_S
                )
            )
            feedback_log_rate_hz = float(
                tracking_controller.get('motor_feedback_log_rate_hz', 10.0)
            )
            self._motor_feedback_publish_rate_hz = float(
                tracking_controller.get(
                    'status_update_rate_hz',
                    self._motor_feedback_publish_rate_hz,
                )
            )
            self._tracking_max_target_lead_deg = float(
                tracking_controller.get(
                    'max_target_lead_deg', self._tracking_max_target_lead_deg
                )
            )
            self._tracking_feedback_stale_warn_s = float(
                tracking_controller.get(
                    'motor_feedback_stale_warn_s',
                    self._tracking_feedback_stale_warn_s,
                )
            )
            self._tracking_feedback_stale_stop_s = float(
                tracking_controller.get(
                    'motor_feedback_stale_stop_s',
                    self._tracking_feedback_stale_stop_s,
                )
            )
            self._tracking_serial_backlog_drop_bytes = int(
                tracking_controller.get(
                    'motor_serial_backlog_drop_bytes',
                    tracking_controller.get(
                        'motor_serial_backlog_stop_bytes',
                        self._tracking_serial_backlog_drop_bytes,
                    ),
                )
            )
            self._tracking_tx_backoff_s = float(
                tracking_controller.get(
                    'motor_serial_timeout_backoff_s',
                    self._tracking_tx_backoff_s,
                )
            )
            if min(
                self.MOTOR_FEEDBACK_POLL_S,
                feedback_log_rate_hz,
                self._motor_tx_rate_hz,
                self._motor_feedback_publish_rate_hz,
            ) <= 0.0:
                raise ValueError('motor feedback timing values must be positive')
            if self._tracking_max_target_lead_deg <= 0.0:
                raise ValueError('max_target_lead_deg must be positive')
            if not (
                0.0 < self._tracking_feedback_stale_warn_s
                < self._tracking_feedback_stale_stop_s
            ):
                raise ValueError(
                    'motor feedback stale thresholds must satisfy 0 < warn < stop'
                )
            if self._tracking_serial_backlog_drop_bytes < 76:
                raise ValueError('motor serial backlog limit must be at least 76 bytes')
            if self._tracking_tx_backoff_s <= 0.0:
                raise ValueError('motor serial timeout backoff must be positive')
            self._tracking_feedback_log_period_s = 1.0 / feedback_log_rate_hz
        except Exception as exc:
            print(f"[PSD分区高速][警告] 读取电机反馈配置失败，使用默认值: {exc}")
        if self.motor_serial is not None:
            self._start_motor_rx_thread()
            self._start_motor_tx_thread()
            self._start_motor_feedback_publish_thread()
        self.tracking_service = PsdTrackingService(
            tracking_config,
            self._apply_tracking_step,
            self._hold_tracking_position,
        )

    def _ensure_motor_defaults(self, motor):
        """补齐 CANFD 控制所需的本地状态，不改变上位机传入的接口。"""
        motor.setdefault('model', 'HO7213')
        motor.setdefault('mode', self.MOTOR_MODE_POSITION)
        motor.setdefault('run', False)
        motor.setdefault('target_position', 0.0)
        motor.setdefault('target_velocity', 0.0)
        motor.setdefault('target_current', motor.get('currency', 0.0))
        motor.setdefault('pole_pairs', self.DEFAULT_POLE_PAIRS)

    def _log_motor_parameters(self, motor_id, context):
        """Print the CANFD parameters that subsequent motor frames will carry."""
        motor = self.motors[motor_id]
        self._ensure_motor_defaults(motor)
        model = str(motor.get('model', 'unknown'))
        pole_pairs = int(motor.get('pole_pairs', self.DEFAULT_POLE_PAIRS))
        kp = int(motor.get('Kp', 0))
        kd = int(motor.get('Kd', 0))
        velocity = float(motor.get('velocity', 0.0))
        current = float(motor.get('currency', 0.0))
        mode = self._mode_value(motor.get('mode', self.MOTOR_MODE_POSITION))
        can_id = self._normalize_standard_can_id(motor.get('canid', 0))
        velocity_counts = self._velocity_to_canfd_counts(velocity, pole_pairs)
        current_counts = self._current_to_canfd_counts(current)
        print(
            f'[HO7213参数][{context}] {motor_id} 型号={model} CAN=0x{can_id} '
            f'极对数={pole_pairs} mode={mode} Kp={kp} Kd={kd} '
            f'速度上限={velocity:.3f}rad/s({velocity_counts}count) '
            f'电流上限={current:.3f}A({current_counts}count)'
        )
        return {
            'model': model,
            'pole_pairs': pole_pairs,
            'Kp': kp,
            'Kd': kd,
            'velocity': velocity,
            'current': current,
            'mode': mode,
        }

    def _validate_tracking_motor_parameters(self):
        errors = []
        for motor_id in ('motor1', 'motor2'):
            values = self._log_motor_parameters(motor_id, '跟踪启动检查')
            if values['model'].upper() == 'HO7213':
                if values['pole_pairs'] != 21:
                    errors.append(
                        f'{motor_id} HO7213 pole_pairs must be 21, got '
                        f"{values['pole_pairs']}"
                    )
                if not 0.0 < values['current'] <= 4.0:
                    errors.append(
                        f'{motor_id} HO7213 current limit must satisfy 0 < A <= 4, '
                        f"got {values['current']}"
                    )
            if not 0 <= values['Kp'] <= 0xFF:
                errors.append(f"{motor_id} Kp is outside 0..255")
            if not 0 <= values['Kd'] <= 0xFF:
                errors.append(f"{motor_id} Kd is outside 0..255")
            if values['velocity'] <= 0.0:
                errors.append(f"{motor_id} position velocity must be positive")
        return errors

    def _normalize_standard_can_id(self, canid):
        if isinstance(canid, str):
            text = canid.strip().lower()
            if text.startswith('0x'):
                value = int(text, 16)
            else:
                value = int(text, 16)
        else:
            value = int(canid)

        if value < 0 or value > 0x7FF:
            raise ValueError(f"CAN ID out of standard-frame range: {canid}")
        return f"{value:03X}"

    def _mode_value(self, mode):
        if isinstance(mode, str):
            return {
                'current': self.MOTOR_MODE_CURRENT,
                'torque': self.MOTOR_MODE_CURRENT,
                'velocity': self.MOTOR_MODE_VELOCITY,
                'speed': self.MOTOR_MODE_VELOCITY,
                'position': self.MOTOR_MODE_POSITION,
            }.get(mode.lower(), self.MOTOR_MODE_POSITION)
        return int(mode)

    def _clamp_int(self, value, min_value, max_value):
        return max(min_value, min(max_value, int(round(value))))

    def _signed_to_bytes(self, value, byte_count):
        bits = byte_count * 8
        min_value = -(1 << (bits - 1))
        max_value = (1 << (bits - 1)) - 1
        value = self._clamp_int(value, min_value, max_value)
        return value.to_bytes(byte_count, byteorder='big', signed=True)

    def _bytes_to_signed(self, data):
        return int.from_bytes(bytes(data), byteorder='big', signed=True)

    def _position_to_canfd_counts(self, position_deg):
        return int(round(float(position_deg) / 360.0 * self.CANFD_POSITION_COUNTS_PER_REV))

    def _velocity_to_canfd_counts(self, velocity_rad_s, pole_pairs):
        electrical_hz = float(velocity_rad_s) * float(pole_pairs) / (2.0 * math.pi)
        return int(round(electrical_hz / self.CANFD_VELOCITY_FULL_SCALE_HZ * self.CANFD_VELOCITY_SCALE))

    def _current_to_canfd_counts(self, current_a):
        return int(round(float(current_a) / self.CANFD_CURRENT_FULL_SCALE_A * self.CANFD_CURRENT_SCALE))

    def _build_canfd_payload(self, motor, position=None, velocity=None, current=None,
                             mode=None, run=None, zero=0):
        self._ensure_motor_defaults(motor)

        if position is None:
            position = motor.get('target_position', 0.0)
        else:
            motor['target_position'] = float(position)

        if velocity is None:
            velocity = motor.get('target_velocity', 0.0)
        else:
            motor['target_velocity'] = float(velocity)

        if current is None:
            current = motor.get('target_current', motor.get('currency', 0.0))
        else:
            motor['target_current'] = float(current)

        if mode is None:
            mode = self._mode_value(motor.get('mode', self.MOTOR_MODE_POSITION))
        else:
            mode = self._mode_value(mode)
            motor['mode'] = mode

        if run is None:
            run = bool(motor.get('run', False))
        else:
            run = bool(run)
            motor['run'] = run

        kp = self._clamp_int(motor.get('Kp', 0), 0, 0xFF)
        kd = self._clamp_int(motor.get('Kd', 0), 0, 0xFF)
        pole_pairs = motor.get('pole_pairs', self.DEFAULT_POLE_PAIRS)

        data = bytearray(16)
        data[0:5] = self._signed_to_bytes(self._position_to_canfd_counts(position), 5)
        data[5:9] = self._signed_to_bytes(self._velocity_to_canfd_counts(velocity, pole_pairs), 4)
        data[9:11] = self._signed_to_bytes(self._current_to_canfd_counts(current), 2)
        # CANFD send bytes 11-15: ModeSel, RunCmd, Kp, Kd, Zero.
        data[11] = self._clamp_int(mode, self.MOTOR_MODE_CURRENT, self.MOTOR_MODE_POSITION)
        data[12] = 1 if run else 0
        data[13] = kp
        data[14] = kd
        data[15] = self._clamp_int(zero, 0, 0xFF)
        return data

    def _format_canfd_command(self, canid, payload):
        if len(payload) != 16:
            raise ValueError(f"CANFD payload must be 16 bytes, got {len(payload)}")
        payload_hex = ''.join(format(byte, '02x') for byte in payload)
        return f"d{self._normalize_standard_can_id(canid)}{self.CANFD_DLC_16}{payload_hex}\r"

    def _send_canfd_motor_frame(self, motor, **kwargs):
        payload = self._build_canfd_payload(motor, **kwargs)
        command = self._format_canfd_command(motor['canid'], payload)
        self.write_motor_data(command)
        return command

    def _expected_slave_id(self, motor_id):
        motor = self.motors.get(motor_id)
        if not motor:
            return None
        try:
            return int(str(motor['canid']).strip(), 16)
        except Exception:
            return None

    def _start_motor_rx_thread(self):
        if self._motor_rx_thread is not None and self._motor_rx_thread.is_alive():
            return
        self._motor_rx_stop_event.clear()
        self._motor_rx_thread = threading.Thread(
            target=self._motor_rx_loop,
            name='motor-canfd-rx-v08',
            daemon=True,
        )
        self._motor_rx_thread.start()
        print(
            '[PSD分区高速] 电机反馈接收线程已启动 | '
            f'串口读超时={self.MOTOR_SERIAL_READ_TIMEOUT_S * 1000.0:.1f}ms '
            f'写超时={self.MOTOR_SERIAL_WRITE_TIMEOUT_S * 1000.0:.1f}ms'
        )

    def _start_motor_feedback_publish_thread(self):
        if (
            self._motor_feedback_publish_thread is not None
            and self._motor_feedback_publish_thread.is_alive()
        ):
            return
        self._motor_feedback_publish_stop_event.clear()
        self._motor_feedback_publish_thread = threading.Thread(
            target=self._motor_feedback_publish_loop,
            name='motor-feedback-to-upper-machine',
            daemon=True,
        )
        self._motor_feedback_publish_thread.start()
        print(
            '[电机反馈] 上位机推送线程已启动 | '
            f'最高推送速率={self._motor_feedback_publish_rate_hz:.1f}Hz '
            '策略=每轴仅保留最新反馈'
        )

    def _motor_feedback_publish_loop(self):
        """Publish coalesced feedback without ever blocking the serial RX worker."""
        period_s = 1.0 / self._motor_feedback_publish_rate_hz
        next_publish_at = time.monotonic()
        while not self._motor_feedback_publish_stop_event.is_set():
            with self.motor_feedback_condition:
                changed = any(
                    generation
                    > self._motor_feedback_published_generation.get(motor_id, 0)
                    for motor_id, generation in self._motor_feedback_generation.items()
                )
                if not changed:
                    self.motor_feedback_condition.wait(min(period_s, 0.1))
                    continue

            now = time.monotonic()
            delay = next_publish_at - now
            if delay > 0.0 and self._motor_feedback_publish_stop_event.wait(delay):
                break

            # Keep the newest sample pending while no upper machine is
            # connected, so it is delivered immediately after reconnection.
            if self.client_socket is None:
                next_publish_at = time.monotonic() + period_s
                continue

            with self.motor_feedback_condition:
                generations = dict(self._motor_feedback_generation)
                payload = {
                    motor_id: dict(feedback)
                    for motor_id, feedback in self._latest_motor_feedback.items()
                    if generations.get(motor_id, 0)
                    > self._motor_feedback_published_generation.get(motor_id, 0)
                }
            if payload:
                self.send_response({
                    'device': 'motor',
                    'action': 'feedback_stream',
                    'type': 'data_stream',
                    'status': 'success',
                    'data': payload,
                })
                with self.motor_feedback_condition:
                    for motor_id in payload:
                        self._motor_feedback_published_generation[motor_id] = (
                            generations[motor_id]
                        )
            next_publish_at = time.monotonic() + period_s

    @staticmethod
    def _extract_slcan_frames(buffer, chunk):
        """Split CR frames and consume standalone SLCAN BEL/NAK bytes."""
        frames = []
        nack_count = 0
        for value in chunk:
            if value == 0x07:
                # Lawicel-compatible adapters return BEL without a trailing CR
                # for an unsupported or malformed command.  It terminates the
                # current response; retaining it would corrupt the next frame.
                buffer.clear()
                nack_count += 1
                continue
            if value == 0x0D:
                frame = bytes(buffer).decode('ascii', errors='ignore').strip()
                buffer.clear()
                if frame:
                    frames.append(frame)
                continue
            if value == 0x0A:
                continue
            buffer.append(value)
        return frames, nack_count

    def _motor_rx_loop(self):
        """Continuously drain SLCAN feedback without blocking the TX loop."""
        buffer = bytearray()
        while not self._motor_rx_stop_event.is_set():
            port = self.motor_serial
            if port is None or not port.is_open:
                self._motor_rx_stop_event.wait(0.001)
                continue
            try:
                waiting = int(port.in_waiting)
                chunk = port.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue
                frames, nack_count = self._extract_slcan_frames(buffer, chunk)
                self._motor_adapter_nacks += nack_count
                for frame in frames:
                    self._dispatch_motor_feedback_frame(frame)
                if len(buffer) > 4096:
                    buffer.clear()
                    self._motor_rx_parse_errors += 1
            except Exception as exc:
                if not self._motor_rx_stop_event.is_set():
                    self._motor_rx_parse_errors += 1
                    print(
                        '[PSD分区高速][反馈线程警告] '
                        f'{type(exc).__name__}: {exc}'
                    )
                    self._motor_rx_stop_event.wait(0.002)

    def _start_motor_tx_thread(self):
        if self._motor_tx_thread is not None and self._motor_tx_thread.is_alive():
            return
        self._motor_tx_stop_event.clear()
        self._motor_tx_thread = threading.Thread(
            target=self._motor_tx_loop,
            name='motor-canfd-tracking-tx',
            daemon=True,
        )
        self._motor_tx_thread.start()
        print(
            '[PSD分区高速] 电机目标发送线程已启动 | '
            f'目标周期={1000.0 / self._motor_tx_rate_hz:.2f}ms '
            '队列策略=仅保留最新双轴目标'
        )

    def _motor_tx_loop(self):
        """Write at most one newest dual-axis target per configured TX slot."""
        period_s = 1.0 / self._motor_tx_rate_hz
        next_slot = time.monotonic()
        while not self._motor_tx_stop_event.is_set():
            with self._motor_tx_condition:
                while (
                    self._tracking_tx_pending is None
                    and not self._motor_tx_stop_event.is_set()
                ):
                    self._motor_tx_condition.wait(0.1)
                if self._motor_tx_stop_event.is_set():
                    break

            now = time.monotonic()
            if next_slot < now:
                next_slot = now
            delay = next_slot - now
            if delay > 0.0 and self._motor_tx_stop_event.wait(delay):
                break

            # Serialise against manual transactions and stop/hold.  Taking the
            # pending item under the same transaction lock guarantees that a
            # final hold command cannot be followed by an older queued target.
            with self.motor_transaction_lock:
                with self._motor_tx_condition:
                    pending = self._tracking_tx_pending
                    self._tracking_tx_pending = None
                if pending is not None:
                    generation, targets = pending
                    try:
                        write_ms = self._send_tracking_target_batch(targets)
                    except Exception as exc:
                        self._motor_tx_failures += 1
                        self._motor_tx_dropped_batches += 1
                        self._motor_tx_last_error = (
                            f'{type(exc).__name__}: {exc}'
                        )
                        self._motor_tx_backoff_until = (
                            time.monotonic() + self._tracking_tx_backoff_s
                        )
                        self._recover_motor_serial_output()
                        self._warn_motor_tx_drop(
                            f'发送线程异常={self._motor_tx_last_error}'
                        )
                        write_ms = None
                    if write_ms is not None:
                        with self._motor_tx_condition:
                            self._tracking_tx_sent_generation = generation

            next_slot += period_s
            finished_at = time.monotonic()
            if next_slot < finished_at:
                # A slow write already rate-limits the link.  Resume from its
                # completion instead of adding another whole command-period delay.
                next_slot = finished_at

    def _queue_tracking_target_batch(self, targets):
        """Publish a target without waiting for the USB-serial write."""
        started = time.monotonic()
        queued_targets = {
            motor_id: float(targets[motor_id])
            for motor_id in ('motor1', 'motor2')
        }
        with self._motor_tx_condition:
            if self._tracking_tx_pending is not None:
                self._tracking_tx_superseded += 1
            self._tracking_tx_generation += 1
            self._tracking_tx_pending = (
                self._tracking_tx_generation,
                queued_targets,
            )
            self._tracking_requested_targets.update(queued_targets)
            self._motor_tx_condition.notify()
        return (time.monotonic() - started) * 1000.0

    def _dispatch_motor_feedback_frame(self, frame):
        frame_prefix = frame[:1]
        raw_types = getattr(self, '_motor_rx_raw_types', None)
        if raw_types is None:
            raw_types = {}
            self._motor_rx_raw_types = raw_types
        raw_types[frame_prefix] = raw_types.get(frame_prefix, 0) + 1
        matched = False
        for motor_id in tuple(self.motors):
            feedback = self.parse_motor_feedback(frame, motor_id)
            if feedback is None:
                continue
            self._record_motor_feedback(motor_id, feedback)
            matched = True
            break
        if not matched and frame_prefix.lower() in ('d', 'b', 't'):
            self._motor_rx_parse_errors += 1

    def _wait_for_motor_feedback(self, motor_id, generation, timeout):
        deadline = time.monotonic() + float(timeout)
        with self.motor_feedback_condition:
            while self._motor_feedback_generation.get(motor_id, 0) <= generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.motor_feedback_condition.wait(remaining)
            feedback = self._latest_motor_feedback.get(motor_id)
            return dict(feedback) if isinstance(feedback, dict) else feedback

    def _send_canfd_motor_frame_and_read(self, motor_id, motor, timeout=0.3, **kwargs):
        """Send a command and wait for the next background-received sample.

        This is retained for operations that need sequencing (mode changes and
        zeroing). The motor returns one status frame for the command and the
        independent RX thread publishes it. High-rate position commands use the
        send-only path so the control loop never waits for that reply.
        """
        with self.motor_transaction_lock:
            with self.motor_feedback_condition:
                generation = self._motor_feedback_generation.get(motor_id, 0)
            self._send_canfd_motor_frame(motor, **kwargs)
            return self._wait_for_motor_feedback(motor_id, generation, timeout)

    def _latest_motor_feedback_snapshot(self, motor_id):
        """Return the newest asynchronously received feedback and its age."""
        now = time.monotonic()
        with self.motor_feedback_condition:
            feedback = self._latest_motor_feedback.get(motor_id)
            timestamp = self._motor_feedback_timestamps.get(motor_id)
            data = dict(feedback) if isinstance(feedback, dict) else None
        if data is not None and timestamp is not None:
            data['feedback_age_ms'] = (now - timestamp) * 1000.0
        return data

    def _record_motor_feedback(self, motor_id, feedback):
        if not isinstance(feedback, dict):
            return
        position = feedback.get('position')
        try:
            position = float(position)
        except (TypeError, ValueError):
            return
        if not math.isfinite(position):
            return
        now = time.monotonic()
        with self.motor_feedback_condition:
            self.current_positions[motor_id] = position
            self._latest_motor_feedback[motor_id] = dict(feedback)
            self._motor_feedback_timestamps[motor_id] = now
            self._motor_feedback_generation[motor_id] = (
                self._motor_feedback_generation.get(motor_id, 0) + 1
            )
            self._motor_rx_counts[motor_id] = self._motor_rx_counts.get(motor_id, 0) + 1
            self._motor_rx_frames += 1
            motor = self.motors.get(motor_id)
            if motor is not None:
                self._update_motor_state_from_feedback(motor, feedback)
            capture = getattr(self, '_motor_feedback_capture', None)
            if (
                isinstance(capture, dict)
                and capture.get('armed')
                and capture.get('motor_id') == motor_id
            ):
                started_at = float(capture['started_at'])
                capture['samples'].append((now - started_at, position))
                capture['details'].append({
                    'elapsed_s': now - started_at,
                    'position': position,
                    'velocity': feedback.get('velocity'),
                    'current': feedback.get('current'),
                })
            self.motor_feedback_condition.notify_all()

    def _update_motor_state_from_feedback(self, motor, feedback):
        if not isinstance(feedback, dict):
            return
        if 'run' in feedback:
            motor['run'] = bool(feedback['run'])
        if 'mode' in feedback:
            motor['mode'] = feedback['mode']

    def init_motor_serial(self):
        """初始化电机串口"""
        if not self.motor_serial:
            return
        self.write_motor_data('C\r')
        self.write_motor_data('S8\r')
        self.write_motor_data('Y5\r')
        self.write_motor_data('O\r')
        time.sleep(0.1)

    def write_motor_data(self, data, flush=True):
        """写入数据到电机串口"""
        if self.motor_serial:
            with self.serial_lock:
                encoded = data.encode('ascii')
                written = self.motor_serial.write(encoded)
                if written != len(encoded):
                    raise RuntimeError(
                        f'Incomplete motor serial write: {written}/{len(encoded)} bytes'
                    )
                if flush:
                    self.motor_serial.flush()

    def _send_tracking_target_batch(self, targets, count_tracking=True):
        """Send both position targets without waiting for motor feedback.

        Each HO7213 position command produces one status reply. The dedicated
        RX thread drains those replies; this TX path only writes the newest
        dual-axis target and never sends a separate position query or waits.
        """
        if self.motor_serial is None or not self.motor_serial.is_open:
            raise RuntimeError('Motor serial port is not available')

        commands = []
        for motor_id in ('motor1', 'motor2'):
            if motor_id not in self.motors:
                raise RuntimeError(f'Motor is not initialized: {motor_id}')
            target = float(targets[motor_id])
            motor = self.motors[motor_id]
            payload = self._build_canfd_payload(
                motor,
                position=target,
                velocity=motor.get('velocity', 0),
                current=motor.get('currency', 0),
                mode=self.MOTOR_MODE_POSITION,
            )
            commands.append(self._format_canfd_command(motor['canid'], payload))

        started = time.monotonic()
        if started < self._motor_tx_backoff_until:
            self._motor_tx_dropped_batches += 1
            return None

        backlog = int(getattr(self.motor_serial, 'out_waiting', 0))
        if backlog > self._tracking_serial_backlog_drop_bytes:
            self._motor_tx_dropped_batches += 1
            self._motor_tx_backoff_until = started + self._tracking_tx_backoff_s
            self._recover_motor_serial_output()
            self._warn_motor_tx_drop(
                f'串口积压={backlog}B，丢弃本周期旧目标并继续'
            )
            return None

        if count_tracking and (
            self._tracking_last_batch_at is None
            or started - self._tracking_last_batch_at
            > self._tracking_feedback_stale_stop_s
        ):
            self._tracking_batches_since_idle = 0
            self._tracking_active_burst_started_at = started
            self._tracking_tx_baseline = self._tracking_tx_batches
            self._next_tracking_feedback_log = (
                started + self._tracking_feedback_log_period_s
            )
            with self.motor_feedback_condition:
                self._tracking_rx_baseline = dict(self._motor_rx_counts)
        try:
            self.write_motor_data(''.join(commands), flush=False)
        except serial.SerialTimeoutException:
            self._motor_tx_timeouts += 1
            self._motor_tx_dropped_batches += 1
            self._motor_tx_backoff_until = (
                time.monotonic() + self._tracking_tx_backoff_s
            )
            self._recover_motor_serial_output()
            self._warn_motor_tx_drop(
                '串口写超时，丢弃本周期目标并继续'
            )
            return None
        self._last_tracking_write_ms = (time.monotonic() - started) * 1000.0
        for motor_id, target in targets.items():
            self.current_targets[motor_id] = float(target)
        if count_tracking:
            self._tracking_tx_batches += 1
            self._tracking_tx_frames += 2
            self._tracking_batches_since_idle += 1
            self._tracking_last_batch_at = started
        return self._last_tracking_write_ms

    def _recover_motor_serial_output(self):
        """Discard a partial queued frame and terminate any fragment at adapter."""
        try:
            with self.serial_lock:
                self.motor_serial.reset_output_buffer()
                self.motor_serial.write(b'\r')
        except (serial.SerialException, OSError):
            # A subsequent normal write will distinguish congestion from a
            # real disconnect; timeout recovery itself must remain non-fatal.
            pass

    def _warn_motor_tx_drop(self, reason):
        now = time.monotonic()
        if now < self._next_motor_tx_warning:
            return
        self._next_motor_tx_warning = now + 1.0
        print(
            '[PSD分区高速][发送降级] '
            f'{reason} | 写超时累计={self._motor_tx_timeouts} '
            f'丢弃批次累计={self._motor_tx_dropped_batches}'
        )

    def _reset_tracking_link_stats(self):
        now = time.monotonic()
        with self._motor_tx_condition:
            self._tracking_tx_pending = None
            self._tracking_tx_superseded = 0
            self._tracking_tx_sent_generation = self._tracking_tx_generation
        with self.motor_feedback_condition:
            self._tracking_rx_baseline = dict(self._motor_rx_counts)
        self._tracking_stats_started_at = now
        self._tracking_parse_error_baseline = self._motor_rx_parse_errors
        self._tracking_tx_batches = 0
        self._tracking_tx_frames = 0
        self._tracking_batches_since_idle = 0
        self._tracking_last_batch_at = None
        self._tracking_active_burst_started_at = None
        self._tracking_tx_baseline = 0
        self._last_tracking_write_ms = 0.0
        self._motor_tx_timeouts = 0
        self._motor_tx_failures = 0
        self._motor_tx_last_error = None
        self._motor_tx_dropped_batches = 0
        self._motor_tx_backoff_until = 0.0
        self._next_motor_tx_warning = 0.0
        self._next_tracking_feedback_log = (
            now + self._tracking_feedback_log_period_s
        )

    def _tracking_link_metrics(self, now=None):
        if now is None:
            now = time.monotonic()
        rate_started_at = (
            self._tracking_active_burst_started_at
            if self._tracking_active_burst_started_at is not None
            else self._tracking_stats_started_at
        )
        elapsed = max(now - rate_started_at, 1e-9)
        with self.motor_feedback_condition:
            ages_ms = {
                motor_id: (
                    (now - self._motor_feedback_timestamps[motor_id]) * 1000.0
                    if motor_id in self._motor_feedback_timestamps
                    else float('inf')
                )
                for motor_id in ('motor1', 'motor2')
            }
            rx_hz = {
                motor_id: (
                    self._motor_rx_counts.get(motor_id, 0)
                    - self._tracking_rx_baseline.get(motor_id, 0)
                ) / elapsed
                for motor_id in ('motor1', 'motor2')
            }
        with self._motor_tx_condition:
            superseded_targets = self._tracking_tx_superseded
            target_pending = self._tracking_tx_pending is not None
        return {
            'tx_batch_hz': (
                self._tracking_tx_batches - self._tracking_tx_baseline
            ) / elapsed,
            'tx_frame_hz': (
                2 * (self._tracking_tx_batches - self._tracking_tx_baseline)
            ) / elapsed,
            'rx_hz': rx_hz,
            'feedback_age_ms': ages_ms,
            'write_ms': self._last_tracking_write_ms,
            'tx_timeouts': self._motor_tx_timeouts,
            'tx_failures': self._motor_tx_failures,
            'tx_last_error': self._motor_tx_last_error,
            'dropped_batches': self._motor_tx_dropped_batches,
            'superseded_targets': superseded_targets,
            'target_pending': target_pending,
            'parse_errors': (
                self._motor_rx_parse_errors - self._tracking_parse_error_baseline
            ),
            'serial_backlog_bytes': int(
                getattr(self.motor_serial, 'out_waiting', 0)
            ) if self.motor_serial is not None else 0,
        }

    def _check_tracking_feedback_freshness(self, metrics):
        stop_ms = self._tracking_feedback_stale_stop_s * 1000.0
        stale = [
            motor_id
            for motor_id, age_ms in metrics['feedback_age_ms'].items()
            if age_ms > stop_ms
        ]
        if stale:
            ages = ', '.join(
                f"{motor_id}={metrics['feedback_age_ms'][motor_id]:.1f}ms"
                for motor_id in stale
            )
            raise RuntimeError(f'Motor feedback stale: {ages}')

    def read_motor_data(self, size=CANFD_FRAME_SIZE):
        """从电机串口读取数据"""
        if not self.motor_serial:
            return ""
        try:
            if self.motor_serial.in_waiting > 0:
                return self.motor_serial.read_until(b'\r', size=size).decode(errors='ignore')
        except Exception as e:
            print(f"读取电机数据失败: {e}")
        return ""

    def write_laser_data(self, command):
        """写入命令到激光测距仪串口"""
        if not self.laser_serial:
            return False
        try:
            if not command.endswith('\r\n') and not command.endswith('\n'):
                command = f"{command}\r\n"
            self.laser_serial.write(command.encode('ascii'))
            self.laser_serial.flush()
            return True
        except Exception as e:
            print(f"发送激光命令失败: {e}")
            return False

    def _laser_data_reader(self):
        """激光测距仪数据读取线程"""
        buffer = ""
        while self.running or True:  # 持续运行
            if self.laser_serial and self.laser_serial.is_open:
                try:
                    if self.laser_serial.in_waiting > 0:
                        data = self.laser_serial.read(self.laser_serial.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data

                        # 处理完整行
                        while '\n' in buffer or '\r' in buffer:
                            if '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                            else:
                                line, buffer = buffer.split('\r', 1)

                            line = line.strip()
                            if line:
                                with self.laser_data_lock:
                                    self.latest_laser_data = line
                                tracking_service = getattr(
                                    self, 'tracking_service', None
                                )
                                if not (
                                    tracking_service is not None
                                    and tracking_service.is_running
                                ):
                                    print(f"激光数据: {line}")

                                # 如果启用了数据流推送,实时发送给上位机
                                if self.laser_streaming and self.client_socket:
                                    self.push_laser_data(line)

                except Exception as e:
                    print(f"读取激光数据错误: {e}")

            time.sleep(0.01)  # 10ms延迟

    def push_laser_data(self, data):
        """推送激光数据给上位机"""
        try:
            message = {
                'device': 'laser',
                'type': 'data_stream',
                'data': data,
                'timestamp': time.time()
            }
            json_data = json.dumps(message, ensure_ascii=False) + '\n'
            with self._client_send_lock:
                if self.client_socket:
                    self.client_socket.sendall(json_data.encode('utf-8'))
        except Exception as e:
            print(f"推送激光数据失败: {e}")

    def start_server(self):
        """启动TCP服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True

        print(f"✓ 服务器启动,监听 {self.host}:{self.port}")

        while self.running:
            try:
                print("等待上位机连接...")
                self.client_socket, addr = self.server_socket.accept()
                print(f"✓ 上位机已连接: {addr}")
                # 处理客户端请求
                self.handle_client()

            except Exception as e:
                if self.running:
                    print(f"服务器错误: {e}")

    def handle_client(self):
        """处理客户端请求"""
        buffer = ""

        while self.running:
            try:
                data = self.client_socket.recv(4096).decode('utf-8')
                if not data:
                    print("客户端断开连接")
                    break

                buffer += data

                # 处理完整的JSON命令(以换行符分隔)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self.process_command(line.strip())

            except Exception as e:
                print(f"处理客户端数据错误: {e}")
                break
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
            self.laser_streaming = False  # 断开连接时停止数据流
        self._motor_step_test_stop_event.set()
        self.tracking_service.stop()

    def process_command(self, json_str):
        """处理JSON命令"""
        try:
            command = json.loads(json_str)
            device = command.get('device')
            action = command.get('action')

            print(f"收到命令 - 设备: {device}, 动作: {action}")

            response = {'device': device, 'action': action, 'status': 'error', 'message': 'Unknown action'}

            # 根据设备类型分发命令
            if device == 'motor':
                response = self.handle_motor_command(command)
            elif device == 'laser':
                response = self.handle_laser_command(command)
            elif device == 'tracking':
                response = self.handle_tracking_command(command)
            else:
                response = {'device': device, 'action': action, 'status': 'error', 'message': 'Unknown device'}

            # 发送响应
            self.send_response(response)

        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
            self.send_response({'status': 'error', 'message': 'Invalid JSON'})
        except Exception as e:
            print(f"命令处理错误: {e}")
            self.send_response({'status': 'error', 'message': str(e)})

    def send_response(self, response):
        """发送响应给上位机"""
        if self.client_socket:
            try:
                json_data = json.dumps(response, ensure_ascii=False) + '\n'
                with self._client_send_lock:
                    if self.client_socket:
                        self.client_socket.sendall(json_data.encode('utf-8'))
            except Exception as e:
                print(f"发送响应失败: {e}")

    # ==================== 电机命令处理 ====================
    def handle_motor_command(self, command):
        """处理电机命令"""
        action = command.get('action')
        motor_id = command.get('motor_id')

        diagnostic_running = (
            self._motor_step_test_thread is not None
            and self._motor_step_test_thread.is_alive()
        )
        if diagnostic_running and action not in (
            'get_status', 'get_all_status', 'disable', 'disable_all'
        ):
            return {
                'device': 'motor',
                'action': action,
                'status': 'error',
                'message': 'Motor position-loop identification is running',
            }

        if self.tracking_service.is_running:
            if action in ('disable', 'disable_all'):
                self.tracking_service.stop()
            elif action == 'get_status':
                return self._cached_motor_status_response(motor_id)
            elif action == 'get_all_status':
                return {
                    'device': 'motor',
                    'action': action,
                    'status': 'success',
                    'data': {
                        mid: self._cached_motor_status_response(mid).get('data')
                        for mid in self.motors
                    },
                }
            else:
                return {
                    'device': 'motor',
                    'action': action,
                    'status': 'error',
                    'message': 'PSD tracking is running; stop tracking before manual motor control',
                }

        if action in ('disable', 'disable_all'):
            self._motor_step_test_stop_event.set()

        if action == 'add_motor':
            config = command.get('config', {})
            previous = self.motors.get(motor_id, {})
            for key in (
                'model', 'mode', 'run', 'target_position',
                'target_velocity', 'pole_pairs',
            ):
                if key in previous and key not in config:
                    config[key] = previous[key]
            self._ensure_motor_defaults(config)
            self.motors[motor_id] = config
            self._log_motor_parameters(motor_id, '添加电机')
            return {'device': 'motor', 'action': action, 'status': 'success', 'message': f'Motor {motor_id} added'}

        elif action == 'init':
            return self.motor_init(motor_id)

        elif action == 'init_all':
            results = {
                mid: self.motor_init(mid)
                for mid in self.motors
            }
            failed = [
                mid for mid, result in results.items()
                if result.get('status') != 'success'
            ]
            return {
                'device': 'motor',
                'action': action,
                'status': 'error' if failed else 'success',
                'message': (
                    'CANFD command-reply initialization failed: ' + ', '.join(failed)
                    if failed else
                    'All motors initialized with asynchronous command replies'
                ),
                'data': results,
            }

        elif action == 'enable':
            return self.motor_enable(motor_id)

        elif action == 'disable':
            return self.motor_disable(motor_id)
        
        elif action == 'open_xy_control':
            return {'device': 'motor', 'action': action, 'status': 'success',
                    'message': 'XY control disabled on Raspberry Pi'}
        
        elif action == 'close_xy_control':
            return {'device': 'motor', 'action': action, 'status': 'success',
                    'message': 'XY control disabled on Raspberry Pi'}

        elif action == 'enable_all':
            data = {}
            for mid in self.motors:
                result = self.motor_enable(mid)
                data[mid] = result.get('data')
            return {'device': 'motor', 'action': action, 'status': 'success', 'message': 'All motors enabled',
                    'data': data}

        elif action == 'disable_all':
            data = {}
            for mid in self.motors:
                result = self.motor_disable(mid)
                data[mid] = result.get('data')
            return {'device': 'motor', 'action': action, 'status': 'success', 'message': 'All motors disabled',
                    'data': data}

        elif action == 'position_control':
            return self.motor_position_control(motor_id)
        
        elif action == 'velocity_control':
            return self.motor_velocity_control(motor_id)

        elif action == 'set_zero':
            return self.motor_set_zero(motor_id)

        elif action == 'set_all_zero':
            for mid in self.motors:
                self.motor_set_zero(mid)
            return {'device': 'motor', 'action': action, 'status': 'success', 'message': 'All motors zero set'}

        elif action == 'move_to_position':
            position = command.get('position', 0)
            return self.motor_move_to_position(motor_id, position)

        elif action == 'move_with_velocity':
            velocity = command.get('velocity', 0)
            current = command.get('current', command.get('currency', None))
            return self.motor_move_with_velocity(motor_id, velocity, current)

        elif action == 'set_params':
            params = command.get('params', {})
            if motor_id in self.motors:
                self.motors[motor_id].update(params)
                self._log_motor_parameters(motor_id, '更新参数')
                return {'device': 'motor', 'action': action, 'status': 'success',
                        'message': f'Motor {motor_id} params updated'}
            return {'device': 'motor', 'action': action, 'status': 'error', 'message': f'Motor {motor_id} not found'}

        elif action == 'get_status':
            return self.motor_get_status(motor_id)

        elif action == 'get_all_status':
            status = {}
            for mid in self.motors:
                result = self.motor_get_status(mid)
                status[mid] = result.get('data')
            return {'device': 'motor', 'action': action, 'status': 'success', 'data': status}

        return {'device': 'motor', 'action': action, 'status': 'error', 'message': 'Unknown motor action'}

    def _cached_motor_status_response(self, motor_id):
        if motor_id not in self.motors:
            return {
                'device': 'motor',
                'action': 'get_status',
                'status': 'error',
                'message': f'Motor {motor_id} not found',
            }
        data = self._latest_motor_feedback_snapshot(motor_id)
        if data is None:
            return {
                'device': 'motor',
                'action': 'get_status',
                'status': 'error',
                'message': f'No cached feedback for {motor_id}',
            }
        return {
            'device': 'motor',
            'action': 'get_status',
            'status': 'success',
            'message': f'Cached motor status for {motor_id}',
            'data': data,
        }

    def _prepare_tracking_command_replies(self):
        """Select asynchronous command replies without writing Index 06."""
        missing = [
            motor_id for motor_id in ('motor1', 'motor2')
            if motor_id not in self.motors
        ]
        if missing:
            raise RuntimeError('Motors not initialized: ' + ', '.join(missing))
        for motor_id in ('motor1', 'motor2'):
            with self.motor_feedback_condition:
                measured_position = self.current_positions.get(motor_id)
            if measured_position is None:
                raise RuntimeError(
                    f'{motor_id} has no command-reply position feedback; '
                    'initialize the motor first'
                )
            self._motor_feedback_modes[motor_id] = 'command_reply'
        print('[PSD分区高速][反馈模式] 双轴位置命令异步应答；不启用Index 06')

    # ==================== PSD跟踪命令处理 ====================
    def _tracking_preflight(self):
        if self.motor_serial is None:
            return False, 'Motor serial port is not available'
        if self._motor_rx_thread is None or not self._motor_rx_thread.is_alive():
            return False, 'Motor feedback receiver thread is not running'
        if self._motor_tx_thread is None or not self._motor_tx_thread.is_alive():
            return False, 'Motor target sender thread is not running'
        missing = [motor_id for motor_id in ('motor1', 'motor2') if motor_id not in self.motors]
        if missing:
            return False, f"Motors not initialized: {', '.join(missing)}"
        parameter_errors = self._validate_tracking_motor_parameters()
        if parameter_errors:
            return False, 'Motor parameter check failed: ' + '; '.join(parameter_errors)
        disabled = [
            motor_id for motor_id in ('motor1', 'motor2')
            if not bool(self.motors[motor_id].get('run', False))
        ]
        if disabled:
            return False, f"Motors not enabled: {', '.join(disabled)}"
        missing_targets = [
            motor_id for motor_id in ('motor1', 'motor2')
            if motor_id not in self.current_targets
        ]
        if missing_targets:
            return False, 'Read or command the current motor positions before starting tracking'
        now = time.monotonic()
        with self.motor_feedback_condition:
            missing_feedback = [
                motor_id for motor_id in ('motor1', 'motor2')
                if motor_id not in self.current_positions
                or motor_id not in self._motor_feedback_timestamps
            ]
        if missing_feedback:
            return False, (
                'A motor command reply is required before fast tracking: '
                + ', '.join(missing_feedback)
            )
        return True, 'ready'

    def _apply_tracking_step(
        self,
        pitch_delta,
        yaw_delta,
        reset_accumulator=False,
        max_target_lead_deg=None,
    ):
        with self._tracking_target_lock:
            target_lead = (
                self._tracking_max_target_lead_deg
                if max_target_lead_deg is None
                else float(max_target_lead_deg)
            )
            if target_lead <= 0.0:
                raise ValueError('max_target_lead_deg must be positive')
            with self.motor_feedback_condition:
                feedback_positions = {
                    motor_id: float(self.current_positions[motor_id])
                    for motor_id in ('motor1', 'motor2')
                }
            if reset_accumulator:
                for motor_id in ('motor1', 'motor2'):
                    self._tracking_requested_targets[motor_id] = feedback_positions[motor_id]
            axis_plan = (
                ('motor1', 'pitch', float(pitch_delta)),
                ('motor2', 'yaw', float(yaw_delta)),
            )
            targets = {}
            sent_axes = []
            for motor_id, axis_name, delta in axis_plan:
                feedback_position = feedback_positions[motor_id]
                previous_target = float(
                    self._tracking_requested_targets.get(
                        motor_id,
                        self.current_targets.get(motor_id, feedback_position),
                    )
                )
                if abs(delta) < 1e-12:
                    targets[motor_id] = previous_target
                    continue

                targets[motor_id] = bounded_accumulated_target(
                    previous_target,
                    feedback_position,
                    delta,
                    target_lead,
                )
                sent_axes.append(axis_name)

            queue_ms = self._queue_tracking_target_batch(targets)
            pitch_target = targets['motor1']
            yaw_target = targets['motor2']
            queue_text = f'{queue_ms:.3f}ms'
            now = time.monotonic()
            metrics = self._tracking_link_metrics(now)
            burst_elapsed = (
                now - self._tracking_active_burst_started_at
                if self._tracking_active_burst_started_at is not None
                else 0.0
            )
            last_batch_at = self._tracking_last_batch_at
            tx_recent = (
                last_batch_at is not None
                and now - last_batch_at <= self._tracking_feedback_stale_stop_s
            )
            if (
                tx_recent
                and burst_elapsed >= self._tracking_feedback_stale_stop_s
            ):
                # Only check freshness after commands have actually been sent
                # continuously. A new burst gets a full grace period, so an
                # idle-period timestamp cannot stop recovery immediately.
                self._check_tracking_feedback_freshness(metrics)
            if now >= self._next_tracking_feedback_log:
                self._next_tracking_feedback_log = (
                    now + self._tracking_feedback_log_period_s
                )
                with self.motor_feedback_condition:
                    pitch_feedback = float(self.current_positions['motor1'])
                    yaw_feedback = float(self.current_positions['motor2'])
                warn_ms = self._tracking_feedback_stale_warn_s * 1000.0
                if (
                    not tx_recent
                    or burst_elapsed < self._tracking_feedback_stale_warn_s
                ):
                    feedback_state = '等待命令异步反馈'
                elif max(metrics['feedback_age_ms'].values()) > warn_ms:
                    feedback_state = '偏慢'
                else:
                    feedback_state = '正常'
                print(
                    '[PSD分区高速][电机链路] '
                    f"修正轴={','.join(sent_axes)} | "
                    f"目标 pitch={pitch_target:+.6f} yaw={yaw_target:+.6f} | "
                    f"反馈 pitch={pitch_feedback:+.6f} yaw={yaw_feedback:+.6f} | "
                    f"领先 pitch={pitch_target - pitch_feedback:+.6f} "
                    f"yaw={yaw_target - yaw_feedback:+.6f} | "
                    f"目标入队={queue_text} 实际写={metrics['write_ms']:.2f}ms "
                    f"TX={metrics['tx_batch_hz']:.1f}Hz | "
                    f"RX pitch={metrics['rx_hz']['motor1']:.1f}Hz "
                    f"yaw={metrics['rx_hz']['motor2']:.1f}Hz | "
                    f"反馈龄期 pitch={metrics['feedback_age_ms']['motor1']:.1f}ms "
                    f"yaw={metrics['feedback_age_ms']['motor2']:.1f}ms "
                    f"状态={feedback_state} | "
                    f"积压={metrics['serial_backlog_bytes']}B "
                    f"覆盖旧目标累计={metrics['superseded_targets']} "
                    f"写超时={metrics['tx_timeouts']} "
                    f"写异常={metrics['tx_failures']} "
                    f"丢弃={metrics['dropped_batches']} "
                    f"解析错误={metrics['parse_errors']}"
                )
            return pitch_target, yaw_target

    def _hold_tracking_position(self):
        """Cancel accumulated targets and actively hold measured positions."""
        with self._tracking_target_lock:
            with self.motor_transaction_lock:
                with self.motor_feedback_condition:
                    hold_targets = {
                        motor_id: round(float(self.current_positions[motor_id]), 6)
                        for motor_id in ('motor1', 'motor2')
                    }
                with self._motor_tx_condition:
                    self._tracking_tx_pending = None
                    self._tracking_requested_targets.update(hold_targets)
                self._send_tracking_target_batch(
                    hold_targets,
                    count_tracking=False,
                )
                return hold_targets['motor1'], hold_targets['motor2']

    def _benchmark_psd_until(self, reader, started_at, deadline):
        samples = 0
        while time.monotonic() < deadline:
            reader.read_sample()
            samples += 1
        elapsed = max(time.monotonic() - started_at, 1e-9)
        return {
            'samples': samples,
            'elapsed_s': round(elapsed, 6),
            'sample_hz': round(samples / elapsed, 1),
        }

    def _wait_for_feedback_pair(self, generations, timeout_s):
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        with self.motor_feedback_condition:
            while any(
                self._motor_feedback_generation.get(motor_id, 0)
                <= generations[motor_id]
                for motor_id in ('motor1', 'motor2')
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.motor_feedback_condition.wait(remaining)
        return True

    def _benchmark_motor_until(self, targets, started_at, deadline):
        sent_pairs = 0
        completed_pairs = 0
        feedback_timeouts = 0
        consecutive_timeouts = 0
        with self.motor_feedback_condition:
            rx_baseline = {
                motor_id: self._motor_rx_counts.get(motor_id, 0)
                for motor_id in ('motor1', 'motor2')
            }

        while time.monotonic() < deadline:
            with self.motor_transaction_lock:
                with self.motor_feedback_condition:
                    generations = {
                        motor_id: self._motor_feedback_generation.get(motor_id, 0)
                        for motor_id in ('motor1', 'motor2')
                    }
                write_ms = self._send_tracking_target_batch(
                    targets,
                    count_tracking=False,
                )
                if write_ms is None:
                    next_command_at += command_period_s
                    time.sleep(min(0.001, command_period_s))
                    continue
                sent_pairs += 1
                remaining = deadline - time.monotonic()
                received = self._wait_for_feedback_pair(
                    generations,
                    min(max(remaining, 0.0), 0.050),
                )
            if received:
                completed_pairs += 1
                consecutive_timeouts = 0
            else:
                feedback_timeouts += 1
                consecutive_timeouts += 1
                if consecutive_timeouts >= 3:
                    break

        elapsed = max(time.monotonic() - started_at, 1e-9)
        with self.motor_feedback_condition:
            rx_frames = {
                motor_id: (
                    self._motor_rx_counts.get(motor_id, 0)
                    - rx_baseline[motor_id]
                )
                for motor_id in ('motor1', 'motor2')
            }
        return {
            'sent_pairs': sent_pairs,
            'completed_pairs': completed_pairs,
            'feedback_timeouts': feedback_timeouts,
            'elapsed_s': round(elapsed, 6),
            'tx_pair_hz': round(sent_pairs / elapsed, 1),
            'completed_pair_hz': round(completed_pairs / elapsed, 1),
            'rx_motor1_hz': round(rx_frames['motor1'] / elapsed, 1),
            'rx_motor2_hz': round(rx_frames['motor2'] / elapsed, 1),
        }

    @staticmethod
    def _analyze_motor_step_samples(samples, initial_position, target_position):
        """Summarize one fixed-target motor step without assuming a plant model."""
        if len(samples) < 2:
            raise RuntimeError('Not enough motor feedback samples for step analysis')

        initial_position = float(initial_position)
        target_position = float(target_position)
        step_deg = target_position - initial_position
        step_magnitude = abs(step_deg)
        if step_magnitude < 0.005:
            final_error_deg = float(samples[-1][1]) - target_position
            return {
                'initial_position_deg': round(initial_position, 6),
                'target_position_deg': round(target_position, 6),
                'measured_step_deg': round(step_deg, 6),
                'samples': len(samples),
                'command_to_first_feedback_ms': round(float(samples[0][0]) * 1000.0, 3),
                'first_motion_latency_ms': None,
                'rise_time_10_ms': None,
                'rise_time_90_ms': None,
                'settling_time_ms': 0.0,
                'tolerance_deg': 0.002,
                'overshoot_percent': 0.0,
                'target_crossings': 0,
                'final_error_deg': round(final_error_deg, 6),
                'tail_peak_to_peak_deg': 0.0,
                'assessment': 'no_step_needed',
                'diagnosis': '起始位置已经接近返回目标，无需分析该方向阶跃',
            }
        direction = 1.0 if step_deg > 0.0 else -1.0
        tolerance_deg = max(0.002, step_magnitude * 0.05)

        directed_progress = [
            direction * (float(position) - initial_position)
            for _, position in samples
        ]
        command_to_first_feedback_s = float(samples[0][0])
        motion_threshold_deg = max(0.001, step_magnitude * 0.02)
        first_motion_latency_s = next(
            (
                float(elapsed_s)
                for (elapsed_s, _), progress in zip(samples, directed_progress)
                if progress >= motion_threshold_deg
            ),
            None,
        )
        rise_time_10_s = next(
            (
                float(elapsed_s)
                for (elapsed_s, _), progress in zip(samples, directed_progress)
                if progress >= step_magnitude * 0.10
            ),
            None,
        )
        rise_time_90_s = next(
            (
                float(elapsed_s)
                for (elapsed_s, _), progress in zip(samples, directed_progress)
                if progress >= step_magnitude * 0.90
            ),
            None,
        )
        peak_progress = max(directed_progress)
        overshoot_percent = max(
            0.0,
            (peak_progress - step_magnitude) / step_magnitude * 100.0,
        )

        settling_time_s = None
        for index, (elapsed_s, _) in enumerate(samples):
            if all(
                abs(float(position) - target_position) <= tolerance_deg
                for _, position in samples[index:]
            ):
                settling_time_s = float(elapsed_s)
                break

        crossings = 0
        previous_sign = 0
        for _, position in samples:
            error = float(position) - target_position
            if abs(error) <= tolerance_deg:
                continue
            sign = 1 if error > 0.0 else -1
            if previous_sign and sign != previous_sign:
                crossings += 1
            previous_sign = sign

        tail_count = max(2, len(samples) // 5)
        tail_positions = [float(position) for _, position in samples[-tail_count:]]
        tail_peak_to_peak_deg = max(tail_positions) - min(tail_positions)
        final_error_deg = float(samples[-1][1]) - target_position

        if overshoot_percent > 10.0 or crossings >= 2:
            assessment = 'underdamped'
            diagnosis = '固定目标下存在明显超调或反复越过，优先增加D或降低P'
        elif rise_time_90_s is None:
            assessment = 'slow'
            diagnosis = '测试时段内未达到90%，P可能偏低，也需检查速度/电流限制'
        elif rise_time_90_s > 0.100:
            assessment = 'slow'
            diagnosis = '90%上升时间超过100ms，P可能偏低或执行器受到限速'
        elif settling_time_s is None:
            assessment = 'not_settled'
            diagnosis = '已接近目标但测试时段内未稳定，需检查阻尼和机械扰动'
        else:
            assessment = 'adequate'
            diagnosis = '固定目标响应速度和阻尼均正常，电机位置环P基本够用'

        return {
            'initial_position_deg': round(initial_position, 6),
            'target_position_deg': round(target_position, 6),
            'measured_step_deg': round(step_deg, 6),
            'samples': len(samples),
            'command_to_first_feedback_ms': round(
                command_to_first_feedback_s * 1000.0,
                3,
            ),
            'first_motion_latency_ms': (
                None
                if first_motion_latency_s is None
                else round(first_motion_latency_s * 1000.0, 3)
            ),
            'rise_time_10_ms': (
                None if rise_time_10_s is None else round(rise_time_10_s * 1000.0, 3)
            ),
            'rise_time_90_ms': (
                None if rise_time_90_s is None else round(rise_time_90_s * 1000.0, 3)
            ),
            'settling_time_ms': (
                None if settling_time_s is None else round(settling_time_s * 1000.0, 3)
            ),
            'tolerance_deg': round(tolerance_deg, 6),
            'overshoot_percent': round(overshoot_percent, 2),
            'target_crossings': crossings,
            'final_error_deg': round(final_error_deg, 6),
            'tail_peak_to_peak_deg': round(tail_peak_to_peak_deg, 6),
            'assessment': assessment,
            'diagnosis': diagnosis,
        }

    def _capture_motor_step_phase(
        self,
        motor_id,
        targets,
        initial_position,
        duration_s,
        command_rate_hz=100.0,
    ):
        command_rate_hz = float(command_rate_hz)
        if not 10.0 <= command_rate_hz <= 200.0:
            raise ValueError('motor step command_rate_hz must be between 10 and 200')
        started_at = time.monotonic()
        deadline = started_at + float(duration_s)
        command_period_s = 1.0 / command_rate_hz
        next_command_at = started_at
        samples = []
        sent_pairs = 0
        feedback_timeouts = 0
        consecutive_timeouts = 0

        while time.monotonic() < deadline:
            if self._motor_step_test_stop_event.is_set():
                raise RuntimeError('Motor position-loop test cancelled')
            now = time.monotonic()
            if now < next_command_at:
                time.sleep(next_command_at - now)
            if time.monotonic() >= deadline:
                break
            with self.motor_transaction_lock:
                with self.motor_feedback_condition:
                    generations = {
                        mid: self._motor_feedback_generation.get(mid, 0)
                        for mid in ('motor1', 'motor2')
                    }
                write_ms = self._send_tracking_target_batch(
                    targets,
                    count_tracking=False,
                )
                if write_ms is None:
                    time.sleep(0.001)
                    continue
                sent_pairs += 1
                remaining = deadline - time.monotonic()
                received = self._wait_for_feedback_pair(
                    generations,
                    min(max(remaining, 0.0), 0.050),
                )
            if not received:
                feedback_timeouts += 1
                consecutive_timeouts += 1
                if consecutive_timeouts >= 3:
                    break
                continue

            consecutive_timeouts = 0
            captured_at = time.monotonic()
            with self.motor_feedback_condition:
                position = float(self.current_positions[motor_id])
            samples.append((captured_at - started_at, position))
            next_command_at += command_period_s
            if next_command_at < captured_at:
                next_command_at = captured_at

        analysis = self._analyze_motor_step_samples(
            samples,
            initial_position,
            targets[motor_id],
        )
        elapsed_s = max(time.monotonic() - started_at, 1e-9)
        analysis.update({
            'elapsed_s': round(elapsed_s, 6),
            'feedback_hz': round(len(samples) / elapsed_s, 1),
            'requested_command_rate_hz': command_rate_hz,
            'sent_pairs': sent_pairs,
            'feedback_timeouts': feedback_timeouts,
        })
        return analysis

    def _enable_and_verify_canfd_auto_upload(
        self,
        motor_id,
        hold_position,
        timeout_s=0.150,
        run=True,
        strict=True,
    ):
        """Enable passive feedback, arm it with one hold frame, and verify it."""
        timeout_s = float(timeout_s)
        with self.motor_feedback_condition:
            enable_generation = self._motor_feedback_generation.get(motor_id, 0)
        with self.motor_transaction_lock:
            self._set_canfd_auto_upload(motor_id, True)

        # Observe feedback after requesting upload, then send one stationary
        # hold frame. A status frame here is not a parameter-value readback.
        # This arming sequence is retained for compatibility; the CAN2.0
        # multi-turn sequencing note is not a CANFD protocol guarantee.
        enable_deadline = time.monotonic() + min(timeout_s, 0.050)
        with self.motor_feedback_condition:
            while self._motor_feedback_generation.get(motor_id, 0) <= enable_generation:
                remaining = enable_deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.motor_feedback_condition.wait(remaining)
            enable_ack_received = (
                self._motor_feedback_generation.get(motor_id, 0) > enable_generation
            )
            hold_generation = self._motor_feedback_generation.get(motor_id, 0)
        raw_type_baseline = dict(getattr(self, '_motor_rx_raw_types', {}))
        parse_error_baseline = self._motor_rx_parse_errors

        motor = self.motors[motor_id]
        with self.motor_transaction_lock:
            self._send_canfd_motor_frame(
                motor,
                position=float(hold_position),
                velocity=motor.get('velocity', 0.0),
                current=motor.get('currency', 0.0),
                mode=self.MOTOR_MODE_POSITION,
                run=bool(run),
            )
            self.current_targets[motor_id] = float(hold_position)

        # One new frame can merely be the hold-command acknowledgement.  Four
        # new frames after it prove that periodic automatic feedback is active.
        started_at = time.monotonic()
        deadline = started_at + timeout_s
        with self.motor_feedback_condition:
            while self._motor_feedback_generation.get(motor_id, 0) < hold_generation + 4:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.motor_feedback_condition.wait(remaining)
            received = (
                self._motor_feedback_generation.get(motor_id, 0) - hold_generation
            )
        elapsed_s = max(time.monotonic() - started_at, 1e-9)
        if received < 4:
            raw_type_delta = {
                prefix: count - raw_type_baseline.get(prefix, 0)
                for prefix, count in getattr(
                    self,
                    '_motor_rx_raw_types',
                    {},
                ).items()
                if count - raw_type_baseline.get(prefix, 0) > 0
            }
            parse_error_delta = self._motor_rx_parse_errors - parse_error_baseline
            details = (
                f'index=0x06, enable_ack={enable_ack_received}, only {received} frame(s) '
                f'after hold command in {elapsed_s * 1000.0:.1f}ms, '
                f'raw_types={raw_type_delta}, parse_errors={parse_error_delta}'
            )
            if strict or received < 1:
                try:
                    with self.motor_transaction_lock:
                        self._set_canfd_auto_upload(motor_id, False)
                except Exception:
                    pass
                raise RuntimeError(
                    f'{motor_id} CANFD periodic upload was not verified; '
                    + details
                )
            # Some firmware builds acknowledge 0x000601 but still emit only
            # one status frame per control command.  Keep TX non-blocking and
            # let the independent RX thread consume those replies, while
            # clearly marking that the expected 1 ms periodic stream is absent.
            self._motor_feedback_modes[motor_id] = 'command_reply_fallback'
            print(
                '[电机自动反馈][降级] '
                f'{motor_id} 未检测到连续主动上传，改用位置命令异步应答 | {details}'
            )
            return 0.0
        periodic_frames = max(received - 1, 0)
        rate_hz = round(periodic_frames / elapsed_s, 1)
        self._motor_feedback_modes[motor_id] = 'periodic_auto_upload'
        return rate_hz

    def _capture_motor_step_phase_auto_upload(
        self,
        motor_id,
        target_position,
        initial_position,
        duration_s,
    ):
        """Send one position target and passively capture auto-upload feedback."""
        motor = self.motors[motor_id]
        capture = {
            'motor_id': motor_id,
            'armed': False,
            'started_at': 0.0,
            'samples': [],
            'details': [],
        }
        with self.motor_feedback_condition:
            if self._motor_feedback_capture is not None:
                raise RuntimeError('Another passive motor feedback capture is active')
            self._motor_feedback_capture = capture

        write_ms = None
        try:
            with self.motor_transaction_lock:
                started_at = time.monotonic()
                with self.motor_feedback_condition:
                    capture['started_at'] = started_at
                    capture['armed'] = True
                write_started_at = time.monotonic()
                self._send_canfd_motor_frame(
                    motor,
                    position=float(target_position),
                    velocity=motor.get('velocity', 0.0),
                    current=motor.get('currency', 0.0),
                    mode=self.MOTOR_MODE_POSITION,
                    run=True,
                )
                write_ms = (time.monotonic() - write_started_at) * 1000.0
                self.current_targets[motor_id] = float(target_position)

            deadline = started_at + float(duration_s)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                if self._motor_step_test_stop_event.wait(min(remaining, 0.020)):
                    raise RuntimeError('Motor position-loop test cancelled')
        finally:
            with self.motor_feedback_condition:
                capture['armed'] = False
                if self._motor_feedback_capture is capture:
                    self._motor_feedback_capture = None
                samples = list(capture['samples'])
                details = list(capture['details'])

        if len(samples) < 2:
            raise RuntimeError(
                f'{motor_id} automatic feedback stopped during one-shot step '
                f'({len(samples)} sample(s))'
            )
        analysis = self._analyze_motor_step_samples(
            samples,
            initial_position,
            target_position,
        )
        elapsed_s = max(float(duration_s), samples[-1][0], 1e-9)

        def peak_absolute(name):
            values = []
            for item in details:
                try:
                    value = float(item.get(name))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(abs(value))
            return None if not values else round(max(values), 6)

        analysis.update({
            'elapsed_s': round(elapsed_s, 6),
            'feedback_hz': round(len(samples) / elapsed_s, 1),
            'command_frames_sent': 1,
            'command_write_ms': round(float(write_ms), 3),
            'auto_upload_feedback': True,
            'peak_abs_velocity_rad_s': peak_absolute('velocity'),
            'peak_abs_current_a': peak_absolute('current'),
        })
        return analysis

    @staticmethod
    def _summarize_motor_step_trials(trials, step_degrees):
        amplitude_summaries = {}
        for step_deg in step_degrees:
            matching = [
                trial for trial in trials
                if abs(float(trial['step_deg']) - float(step_deg)) < 1e-9
            ]
            phases = [
                trial[phase_name]
                for trial in matching
                for phase_name in ('outward', 'return')
            ]

            def median_metric(key):
                values = [
                    float(phase[key])
                    for phase in phases
                    if phase.get(key) is not None
                ]
                return None if not values else round(statistics.median(values), 3)

            amplitude_summaries[f'{float(step_deg):.3f}'] = {
                'trials': len(matching),
                'median_command_to_first_feedback_ms': median_metric(
                    'command_to_first_feedback_ms'
                ),
                'median_first_motion_latency_ms': median_metric(
                    'first_motion_latency_ms'
                ),
                'median_rise_time_10_ms': median_metric('rise_time_10_ms'),
                'median_rise_time_90_ms': median_metric('rise_time_90_ms'),
                'median_settling_time_ms': median_metric('settling_time_ms'),
                'median_overshoot_percent': median_metric('overshoot_percent'),
                'max_overshoot_percent': round(
                    max(float(phase['overshoot_percent']) for phase in phases),
                    2,
                ),
                'max_target_crossings': max(
                    int(phase['target_crossings']) for phase in phases
                ),
                'median_feedback_hz': median_metric('feedback_hz'),
            }

        smallest = amplitude_summaries[f'{float(step_degrees[0]):.3f}'][
            'median_rise_time_90_ms'
        ]
        largest = amplitude_summaries[f'{float(step_degrees[-1]):.3f}'][
            'median_rise_time_90_ms'
        ]
        scaling_ratio = None
        if smallest is not None and largest is not None and smallest > 0.0:
            scaling_ratio = largest / smallest

        maximum_overshoot = max(
            summary['max_overshoot_percent']
            for summary in amplitude_summaries.values()
        )
        maximum_crossings = max(
            summary['max_target_crossings']
            for summary in amplitude_summaries.values()
        )
        if scaling_ratio is None:
            limitation = 'insufficient_data'
            diagnosis = '有效t90数据不足，无法判断主要限制'
        elif scaling_ratio <= 1.35:
            limitation = 'position_loop_or_filter_limited'
            diagnosis = '不同幅值t90接近，主要受位置环增益、阻尼或内部滤波限制'
        elif scaling_ratio >= 2.50:
            limitation = 'velocity_or_acceleration_limited'
            diagnosis = 't90随幅值显著增长，主要受速度或加速度限制'
        else:
            limitation = 'mixed_limited'
            diagnosis = '位置环动态与速度/加速度限制共同影响响应'

        if maximum_overshoot > 10.0 or maximum_crossings >= 2:
            assessment = 'underdamped'
        elif smallest is None or smallest > 100.0:
            assessment = 'slow'
        else:
            assessment = 'adequate'
        return {
            'by_amplitude': amplitude_summaries,
            'large_to_small_t90_ratio': (
                None if scaling_ratio is None else round(scaling_ratio, 3)
            ),
            'limitation': limitation,
            'assessment': assessment,
            'diagnosis': diagnosis,
        }

    def _run_motor_position_step_test(
        self,
        step_degrees=(0.05, 0.10, 0.20),
        repeats=3,
        duration_s=1.2,
        command_rate_hz=100.0,
    ):
        """Identify each position loop at fixed rate and multiple amplitudes."""
        step_degrees = tuple(float(value) for value in step_degrees)
        repeats = int(repeats)
        duration_s = float(duration_s)
        command_rate_hz = float(command_rate_hz)
        if not step_degrees or any(
            not 0.01 <= abs(step_deg) <= 0.20
            for step_deg in step_degrees
        ):
            raise ValueError('all step amplitudes must be between 0.01 and 0.20 degrees')
        if any(step_deg <= 0.0 for step_deg in step_degrees):
            raise ValueError('step amplitudes must be positive')
        if tuple(sorted(step_degrees)) != step_degrees:
            raise ValueError('step amplitudes must be increasing')
        if not 1 <= repeats <= 5:
            raise ValueError('repeats must be between 1 and 5')
        if not 0.5 <= duration_s <= 2.0:
            raise ValueError('duration_s must be between 0.5 and 2.0 seconds')
        if not 10.0 <= command_rate_hz <= 200.0:
            raise ValueError('command_rate_hz must be between 10 and 200')
        if self.tracking_service.is_running:
            raise RuntimeError('Stop PSD tracking before running the motor step test')
        ready, message = self._tracking_preflight()
        if not ready:
            raise RuntimeError(message)

        now = time.monotonic()
        with self.motor_feedback_condition:
            stale_feedback = [
                motor_id
                for motor_id in ('motor1', 'motor2')
                if now - self._motor_feedback_timestamps.get(motor_id, 0.0) > 2.0
            ]
        if stale_feedback:
            raise RuntimeError(
                'Query both motor positions immediately before step test: '
                + ', '.join(stale_feedback)
            )
        wrong_mode = [
            motor_id
            for motor_id in ('motor1', 'motor2')
            if self._mode_value(self.motors[motor_id].get('mode'))
            != self.MOTOR_MODE_POSITION
        ]
        if wrong_mode:
            raise RuntimeError(
                'Motor step test requires position mode: ' + ', '.join(wrong_mode)
            )
        if not self._rate_benchmark_lock.acquire(blocking=False):
            raise RuntimeError('Another motor diagnostic is already running')

        original_targets = {}
        results = {}
        try:
            with self.motor_feedback_condition:
                original_targets = {
                    motor_id: round(float(self.current_positions[motor_id]), 6)
                    for motor_id in ('motor1', 'motor2')
                }
            with self._motor_tx_condition:
                self._tracking_tx_pending = None
                self._tracking_requested_targets.update(original_targets)

            self._send_tracking_target_batch(
                original_targets,
                count_tracking=False,
            )
            for motor_id in ('motor1', 'motor2'):
                trials = []
                for step_deg in step_degrees:
                    for repetition in range(1, repeats + 1):
                        if self._motor_step_test_stop_event.is_set():
                            raise RuntimeError('Motor position-loop test cancelled')
                        with self.motor_feedback_condition:
                            outward_initial = float(self.current_positions[motor_id])
                        outward_targets = dict(original_targets)
                        outward_targets[motor_id] = round(
                            original_targets[motor_id] + step_deg,
                            6,
                        )
                        print(
                            '[电机位置环辨识][阶跃] '
                            f'{motor_id} 幅值={step_deg:.3f}° '
                            f'第{repetition}/{repeats}次 | '
                            f'{outward_initial:+.6f}° -> '
                            f"{outward_targets[motor_id]:+.6f}°"
                        )
                        outward = self._capture_motor_step_phase(
                            motor_id,
                            outward_targets,
                            outward_initial,
                            duration_s,
                            command_rate_hz,
                        )

                        with self.motor_feedback_condition:
                            return_initial = float(self.current_positions[motor_id])
                        print(
                            '[电机位置环辨识][返回] '
                            f'{motor_id} 幅值={step_deg:.3f}° '
                            f'第{repetition}/{repeats}次 | '
                            f'{return_initial:+.6f}° -> '
                            f"{original_targets[motor_id]:+.6f}°"
                        )
                        returned = self._capture_motor_step_phase(
                            motor_id,
                            original_targets,
                            return_initial,
                            duration_s,
                            command_rate_hz,
                        )
                        trials.append({
                            'step_deg': step_deg,
                            'repetition': repetition,
                            'outward': outward,
                            'return': returned,
                        })

                summary = self._summarize_motor_step_trials(
                    trials,
                    step_degrees,
                )
                motor = self.motors[motor_id]
                results[motor_id] = {
                    'configured_Kp': int(motor.get('Kp', 0)),
                    'configured_Kd': int(motor.get('Kd', 0)),
                    'configured_velocity_rad_s': float(motor.get('velocity', 0.0)),
                    'configured_current_a': float(motor.get('currency', 0.0)),
                    'trials': trials,
                    'summary': summary,
                    'assessment': summary['assessment'],
                }
                print(
                    '[电机位置环辨识][汇总] '
                    f"{motor_id} Kp={results[motor_id]['configured_Kp']} "
                    f"Kd={results[motor_id]['configured_Kd']} "
                    f"速度={results[motor_id]['configured_velocity_rad_s']:.3f}rad/s "
                    f"电流={results[motor_id]['configured_current_a']:.3f}A "
                    f"小/大幅t90比={summary['large_to_small_t90_ratio']} "
                    f"限制={summary['limitation']} "
                    f"判断={summary['assessment']}"
                )

            return {
                'step_degrees': list(step_degrees),
                'repeats': repeats,
                'phase_duration_s': duration_s,
                'command_rate_hz': command_rate_hz,
                'original_positions_deg': original_targets,
                'motors': results,
            }
        finally:
            try:
                if original_targets:
                    with self._tracking_target_lock:
                        with self._motor_tx_condition:
                            self._tracking_tx_pending = None
                            self._tracking_requested_targets.update(original_targets)
                        self._send_tracking_target_batch(
                            original_targets,
                            count_tracking=False,
                        )
            finally:
                self._rate_benchmark_lock.release()

    def _run_motor_position_step_test_once(
        self,
        step_degrees=(0.05, 0.10, 0.20),
        repeats=3,
        duration_s=1.2,
    ):
        """Measure true one-command steps using HO7213 automatic feedback."""
        step_degrees = tuple(float(value) for value in step_degrees)
        repeats = int(repeats)
        duration_s = float(duration_s)
        if not step_degrees or any(
            not 0.01 <= abs(step_deg) <= 0.20
            for step_deg in step_degrees
        ):
            raise ValueError('all step amplitudes must be between 0.01 and 0.20 degrees')
        if any(step_deg <= 0.0 for step_deg in step_degrees):
            raise ValueError('step amplitudes must be positive')
        if tuple(sorted(step_degrees)) != step_degrees:
            raise ValueError('step amplitudes must be increasing')
        if not 1 <= repeats <= 5:
            raise ValueError('repeats must be between 1 and 5')
        if not 0.5 <= duration_s <= 2.0:
            raise ValueError('duration_s must be between 0.5 and 2.0 seconds')
        if self.tracking_service.is_running:
            raise RuntimeError('Stop PSD tracking before running the one-shot step test')
        ready, message = self._tracking_preflight()
        if not ready:
            raise RuntimeError(message)

        now = time.monotonic()
        with self.motor_feedback_condition:
            stale_feedback = [
                motor_id
                for motor_id in ('motor1', 'motor2')
                if now - self._motor_feedback_timestamps.get(motor_id, 0.0) > 2.0
            ]
        if stale_feedback:
            raise RuntimeError(
                'Query both motor positions immediately before one-shot step test: '
                + ', '.join(stale_feedback)
            )
        wrong_mode = [
            motor_id
            for motor_id in ('motor1', 'motor2')
            if self._mode_value(self.motors[motor_id].get('mode'))
            != self.MOTOR_MODE_POSITION
        ]
        if wrong_mode:
            raise RuntimeError(
                'One-shot step test requires position mode: ' + ', '.join(wrong_mode)
            )
        if not self._rate_benchmark_lock.acquire(blocking=False):
            raise RuntimeError('Another motor diagnostic is already running')

        original_targets = {}
        results = {}
        try:
            with self.motor_feedback_condition:
                original_targets = {
                    motor_id: round(float(self.current_positions[motor_id]), 6)
                    for motor_id in ('motor1', 'motor2')
                }
            with self._motor_tx_condition:
                self._tracking_tx_pending = None
                self._tracking_requested_targets.update(original_targets)

            # Clear an upload mode possibly left by an interrupted earlier run,
            # then establish a known stationary starting target.
            with self.motor_transaction_lock:
                for motor_id in ('motor1', 'motor2'):
                    self._set_canfd_auto_upload(motor_id, False)
                self._send_tracking_target_batch(
                    original_targets,
                    count_tracking=False,
                )
            time.sleep(0.020)

            for motor_id in ('motor1', 'motor2'):
                verified_upload_hz = self._enable_and_verify_canfd_auto_upload(
                    motor_id,
                    original_targets[motor_id],
                )
                print(
                    '[电机单次阶跃][自动反馈] '
                    f'{motor_id} 已验证，启动阶段速率≈{verified_upload_hz:.1f}Hz'
                )
                trials = []
                try:
                    for step_deg in step_degrees:
                        for repetition in range(1, repeats + 1):
                            if self._motor_step_test_stop_event.is_set():
                                raise RuntimeError('Motor position-loop test cancelled')
                            with self.motor_feedback_condition:
                                outward_initial = float(self.current_positions[motor_id])
                            outward_target = round(
                                original_targets[motor_id] + step_deg,
                                6,
                            )
                            print(
                                '[电机单次阶跃][阶跃] '
                                f'{motor_id} 幅值={step_deg:.3f}° '
                                f'第{repetition}/{repeats}次 | '
                                f'{outward_initial:+.6f}° -> {outward_target:+.6f}° | '
                                '位置命令=仅1帧'
                            )
                            outward = self._capture_motor_step_phase_auto_upload(
                                motor_id,
                                outward_target,
                                outward_initial,
                                duration_s,
                            )

                            with self.motor_feedback_condition:
                                return_initial = float(self.current_positions[motor_id])
                            print(
                                '[电机单次阶跃][返回] '
                                f'{motor_id} 幅值={step_deg:.3f}° '
                                f'第{repetition}/{repeats}次 | '
                                f'{return_initial:+.6f}° -> '
                                f"{original_targets[motor_id]:+.6f}° | 位置命令=仅1帧"
                            )
                            returned = self._capture_motor_step_phase_auto_upload(
                                motor_id,
                                original_targets[motor_id],
                                return_initial,
                                duration_s,
                            )
                            trials.append({
                                'step_deg': step_deg,
                                'repetition': repetition,
                                'outward': outward,
                                'return': returned,
                            })
                finally:
                    try:
                        with self.motor_transaction_lock:
                            self._set_canfd_auto_upload(motor_id, False)
                    except Exception as exc:
                        print(
                            '[电机单次阶跃][警告] '
                            f'关闭{motor_id}自动反馈失败: {type(exc).__name__}: {exc}'
                        )
                    time.sleep(0.010)

                summary = self._summarize_motor_step_trials(
                    trials,
                    step_degrees,
                )
                motor = self.motors[motor_id]
                results[motor_id] = {
                    'configured_Kp': int(motor.get('Kp', 0)),
                    'configured_Kd': int(motor.get('Kd', 0)),
                    'configured_velocity_rad_s': float(motor.get('velocity', 0.0)),
                    'configured_current_a': float(motor.get('currency', 0.0)),
                    'verified_auto_upload_start_hz': verified_upload_hz,
                    'trials': trials,
                    'summary': summary,
                    'assessment': summary['assessment'],
                }
                print(
                    '[电机单次阶跃][汇总] '
                    f"{motor_id} Kp={results[motor_id]['configured_Kp']} "
                    f"Kd={results[motor_id]['configured_Kd']} "
                    f"限制={summary['limitation']} "
                    f"判断={summary['assessment']}"
                )

            return {
                'measurement_mode': 'single_command_canfd_auto_upload',
                'step_degrees': list(step_degrees),
                'repeats': repeats,
                'phase_duration_s': duration_s,
                'position_commands_per_phase': 1,
                'original_positions_deg': original_targets,
                'motors': results,
            }
        finally:
            with self.motor_feedback_condition:
                self._motor_feedback_capture = None
            try:
                if original_targets:
                    with self._tracking_target_lock:
                        with self._motor_tx_condition:
                            self._tracking_tx_pending = None
                            self._tracking_requested_targets.update(original_targets)
                with self.motor_transaction_lock:
                    if original_targets:
                        # Runtime feedback is permanently asynchronous.  The
                        # diagnostic temporarily isolates one motor at a time;
                        # restore both upload streams before returning.
                        for motor_id in ('motor1', 'motor2'):
                            self._set_canfd_auto_upload(motor_id, True)
                        self._send_tracking_target_batch(
                            original_targets,
                            count_tracking=False,
                        )
            finally:
                self._rate_benchmark_lock.release()

    def _observe_upload_phase(self, motor_id, command, duration_s, parameter_index=None):
        """Capture raw RX and all TX without resetting the adapter or sending polls."""
        observation = UploadObservation(
            self._expected_slave_id(motor_id), duration_s, parameter_index,
        )
        with self.motor_feedback_condition:
            if getattr(self, '_motor_diagnostic_capture', None) is not None:
                raise RuntimeError('Another motor diagnostic capture is active')
            self._motor_diagnostic_capture = observation
        nacks_before = self._motor_adapter_nacks
        errors_before = self._motor_rx_parse_errors
        try:
            self.write_motor_data(command, flush=False)
            deadline = observation.started_at + duration_s
            while time.monotonic() < deadline:
                time.sleep(min(0.010, max(deadline - time.monotonic(), 0.0)))
        finally:
            with self.motor_feedback_condition:
                self._motor_diagnostic_capture = None
        result = observation.summary()
        result['adapter_nacks'] = self._motor_adapter_nacks - nacks_before
        result['parse_errors'] = self._motor_rx_parse_errors - errors_before
        print(
            f"[CANFD上传核查] {motor_id} TX={result['tx_commands']} "
            f"原始帧={result['raw_frames']} 状态帧={result['status_frames']} "
            f"参数应答={result['parameter_replies']} "
            f"50ms后四段={result['late_status_buckets']} "
            f"NACK={result['adapter_nacks']} 原始样本={result['first_frames']}"
        )
        return result

    def _test_canfd_auto_upload(self, duration_s=1.0):
        """Documented index06 only; require a quiet/on/quiet causal test.

        This diagnostic sends NO position, velocity, enable/disable-servo or PID
        commands. It changes only the upload request and reads two parameters.
        Transport ACKs and generic status replies do not verify parameter reads.
        """
        duration_s = float(duration_s)
        if not math.isfinite(duration_s) or not 0.5 <= duration_s <= 5.0:
            raise ValueError('auto-upload duration_s must be between 0.5 and 5.0')
        if self.tracking_service.is_running:
            raise RuntimeError('Stop PSD tracking before testing CANFD auto-upload')
        if self._motor_rx_thread is None or not self._motor_rx_thread.is_alive():
            raise RuntimeError('Motor feedback receiver thread is not running')
        motor_ids = ('motor1', 'motor2')
        if any(mid not in self.motors for mid in motor_ids):
            raise RuntimeError('Initialize both motors before testing')
        if not self._rate_benchmark_lock.acquire(blocking=False):
            raise RuntimeError('Another motor diagnostic is already running')
        results = {}
        selected = {}
        variants = ('canfd_extended_dlc16_padded', 'canfd_extended_dlc8')
        original_variants = dict(self._motor_auto_upload_variants)
        cleanup_errors = []
        result = {
            'software_build': self.SOFTWARE_BUILD,
            'measurement_mode': 'raw_off_on_off_no_motion_no_channel_reset',
            'position_commands_sent': 0,
            'duration_s': duration_s,
            'candidate_variants': list(variants),
            'motors': results,
            'cleanup_errors': cleanup_errors,
        }
        try:
            # The RX thread uses a separate lock and keeps draining throughout.
            # Holding the transaction lock prevents manual/TX-worker traffic.
            with self.motor_transaction_lock:
                with self._motor_tx_condition:
                    self._tracking_tx_pending = None
                for mid in motor_ids:
                    self.write_motor_data(
                        self._format_auto_upload_command(mid, False, variants[0]),
                        flush=False,
                    )
                time.sleep(0.100)
                for mid in motor_ids:
                    reads = {}
                    for index in (0x00, 0x01):
                        payload = bytes((0x01, index)) + bytes(14)
                        command = self._format_canfd_extended_command(
                            self.motors[mid]['canid'], payload,
                        )
                        probe = self._observe_upload_phase(mid, command, 0.25, index)
                        probe['assessment'] = (
                            'documented_parameter_reply'
                            if probe['parameter_replies']
                            else 'generic_status_only' if probe['status_frames']
                            else 'no_documented_parameter_reply'
                        )
                        reads[f'0x{index:02X}'] = probe
                    trials = {}
                    results[mid] = {
                        'parameter_reads': reads,
                        'variants': trials,
                        'selected_variant': None,
                        'assessment': 'periodic_upload_not_verified',
                        'estimated_auto_upload_hz': 0.0,
                    }
                    for variant in variants:
                        off = self._format_auto_upload_command(mid, False, variant)
                        on = self._format_auto_upload_command(mid, True, variant)
                        print(f'[CANFD上传核查阶段] {mid} {variant} 关闭基线')
                        before = self._observe_upload_phase(mid, off, duration_s)
                        print(f'[CANFD上传核查阶段] {mid} {variant} 开启后纯接收')
                        enabled = self._observe_upload_phase(mid, on, duration_s)
                        print(f'[CANFD上传核查阶段] {mid} {variant} 再关闭验证')
                        disabled = self._observe_upload_phase(mid, off, duration_s)
                        assessment = verify_upload_cycle(before, enabled, disabled)
                        if any(p['adapter_nacks'] for p in (before, enabled, disabled)):
                            assessment = 'adapter_rejected_command'
                        trials[variant] = {
                            'before': before, 'enabled': enabled, 'disabled': disabled,
                            'assessment': assessment,
                        }
                        print(f'[CANFD上传核查结论] {mid} {variant}: {assessment}')
                        if assessment == 'periodic_auto_upload':
                            selected[mid] = variant
                            results[mid].update({
                                'selected_variant': variant,
                                'assessment': assessment,
                                'estimated_auto_upload_hz': enabled['late_status_hz'],
                            })
                            break
        finally:
            # Never reopen the CAN channel while a motor may still be streaming,
            # and never automatically re-enable a format whose test failed.
            try:
                with self.motor_transaction_lock:
                    for mid in motor_ids:
                        for variant in variants:
                            try:
                                self.write_motor_data(
                                    self._format_auto_upload_command(mid, False, variant),
                                    flush=False,
                                )
                            except Exception as exc:
                                cleanup_errors.append(f'{mid}/{variant}: {exc}')
                        self._motor_auto_upload_enabled[mid] = False
                        self._motor_feedback_modes[mid] = 'disabled'
                        if mid in selected:
                            self._motor_auto_upload_variants[mid] = selected[mid]
                        elif mid in original_variants:
                            self._motor_auto_upload_variants[mid] = original_variants[mid]
                        else:
                            self._motor_auto_upload_variants.pop(mid, None)
            finally:
                self._rate_benchmark_lock.release()
        result['final_upload_state'] = (
            'disable_requested' if not cleanup_errors else 'cleanup_failed'
        )
        result['all_verified'] = len(selected) == len(motor_ids)
        return result

    def _run_safe_rate_benchmark(self, duration_s):
        """Measure uncapped throughput while both motors hold position."""
        duration_s = float(duration_s)
        if not 0.5 <= duration_s <= 10.0:
            raise ValueError('benchmark duration_s must be between 0.5 and 10.0')
        if self.tracking_service.is_running:
            raise RuntimeError('Stop PSD tracking before running the benchmark')
        ready, message = self._tracking_preflight()
        if not ready:
            raise RuntimeError(message)
        now = time.monotonic()
        with self.motor_feedback_condition:
            stale_feedback = [
                motor_id
                for motor_id in ('motor1', 'motor2')
                if now - self._motor_feedback_timestamps.get(motor_id, 0.0) > 2.0
            ]
        if stale_feedback:
            raise RuntimeError(
                'Query both motor positions immediately before benchmark: '
                + ', '.join(stale_feedback)
            )
        wrong_mode = [
            motor_id
            for motor_id in ('motor1', 'motor2')
            if self._mode_value(self.motors[motor_id].get('mode'))
            != self.MOTOR_MODE_POSITION
        ]
        if wrong_mode:
            raise RuntimeError(
                'Benchmark requires position mode: ' + ', '.join(wrong_mode)
            )
        if not self._rate_benchmark_lock.acquire(blocking=False):
            raise RuntimeError('A rate benchmark is already running')

        reader = None
        try:
            with self._tracking_target_lock:
                with self.motor_feedback_condition:
                    hold_targets = {
                        motor_id: round(float(self.current_positions[motor_id]), 6)
                        for motor_id in ('motor1', 'motor2')
                    }
                with self._motor_tx_condition:
                    self._tracking_tx_pending = None
                    self._tracking_requested_targets.update(hold_targets)
                self._send_tracking_target_batch(
                    hold_targets,
                    count_tracking=False,
                )

            config = json.loads(
                Path(__file__).with_name('psd_calibration.json').read_text(
                    encoding='utf-8'
                )
            )

            reader = PsdTrackingService._build_reader(config)
            reader.open()
            psd_started = time.monotonic()
            psd_only = self._benchmark_psd_until(
                reader,
                psd_started,
                psd_started + duration_s,
            )
            reader.close()
            reader = None

            motor_started = time.monotonic()
            motor_only = self._benchmark_motor_until(
                hold_targets,
                motor_started,
                motor_started + duration_s,
            )

            reader = PsdTrackingService._build_reader(config)
            reader.open()
            combined_started = time.monotonic()
            combined_deadline = combined_started + duration_s
            psd_result = {}
            psd_error = []

            def combined_psd_loop():
                try:
                    psd_result.update(
                        self._benchmark_psd_until(
                            reader,
                            combined_started,
                            combined_deadline,
                        )
                    )
                except Exception as exc:
                    psd_error.append(exc)

            psd_thread = threading.Thread(
                target=combined_psd_loop,
                name='safe-rate-benchmark-psd',
                daemon=True,
            )
            psd_thread.start()
            combined_motor = self._benchmark_motor_until(
                hold_targets,
                combined_started,
                combined_deadline,
            )
            psd_thread.join(duration_s + 1.0)
            if psd_thread.is_alive():
                raise RuntimeError('Combined PSD benchmark thread did not stop')
            if psd_error:
                raise psd_error[0]
            reader.close()
            reader = None

            # Reassert the same hold target after the stress test.
            with self.motor_transaction_lock:
                self._send_tracking_target_batch(
                    hold_targets,
                    count_tracking=False,
                )

            result = {
                'safe_hold_targets_deg': hold_targets,
                'phase_duration_s': duration_s,
                'psd_only': psd_only,
                'motor_only': motor_only,
                'combined': {
                    'psd': psd_result,
                    'motor': combined_motor,
                },
            }
            print(
                '[安全测速][结果] '
                f"PSD单独={psd_only['sample_hz']:.1f}Hz | "
                f"电机单独TX/RX={motor_only['tx_pair_hz']:.1f}/"
                f"{motor_only['completed_pair_hz']:.1f}Hz | "
                f"同时PSD={psd_result['sample_hz']:.1f}Hz "
                f"电机TX/RX={combined_motor['tx_pair_hz']:.1f}/"
                f"{combined_motor['completed_pair_hz']:.1f}Hz"
            )
            return result
        finally:
            if reader is not None:
                try:
                    reader.close()
                except Exception:
                    pass
            self._rate_benchmark_lock.release()

    def _start_motor_position_step_test_async(self, command):
        if (
            self._motor_step_test_thread is not None
            and self._motor_step_test_thread.is_alive()
        ):
            raise RuntimeError('Motor position-loop test is already running')
        self._motor_step_test_stop_event.clear()

        test_kwargs = {
            'step_degrees': command.get(
                'step_degrees',
                (0.05, 0.10, 0.20),
            ),
            'repeats': command.get('repeats', 3),
            'duration_s': command.get('duration_s', 1.2),
            'command_rate_hz': command.get('command_rate_hz', 100.0),
        }

        def worker():
            try:
                result = self._run_motor_position_step_test(**test_kwargs)
                response = {
                    'device': 'tracking',
                    'action': 'test_motor_position_loop',
                    'status': 'success',
                    'message': 'Fixed-rate multi-amplitude motor identification completed',
                    'data': result,
                }
            except Exception as exc:
                cancelled = self._motor_step_test_stop_event.is_set()
                response = {
                    'device': 'tracking',
                    'action': 'test_motor_position_loop',
                    'status': 'cancelled' if cancelled else 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
            self.send_response(response)

        self._motor_step_test_thread = threading.Thread(
            target=worker,
            name='motor-position-loop-identification',
            daemon=True,
        )
        self._motor_step_test_thread.start()
        estimated_s = (
            2.0
            * 2.0
            * len(tuple(test_kwargs['step_degrees']))
            * int(test_kwargs['repeats'])
            * float(test_kwargs['duration_s'])
        )
        return {
            'device': 'tracking',
            'action': 'test_motor_position_loop',
            'status': 'started',
            'message': 'Fixed-rate multi-amplitude motor identification started',
            'data': {'estimated_duration_s': round(estimated_s, 1)},
        }

    def _start_motor_position_step_test_once_async(self, command):
        if (
            self._motor_step_test_thread is not None
            and self._motor_step_test_thread.is_alive()
        ):
            raise RuntimeError('Motor position-loop test is already running')
        self._motor_step_test_stop_event.clear()

        test_kwargs = {
            'step_degrees': command.get(
                'step_degrees',
                (0.05, 0.10, 0.20),
            ),
            'repeats': command.get('repeats', 3),
            'duration_s': command.get('duration_s', 1.2),
        }

        def worker():
            try:
                result = self._run_motor_position_step_test_once(**test_kwargs)
                response = {
                    'device': 'tracking',
                    'action': 'test_motor_position_loop_once',
                    'status': 'success',
                    'message': 'One-shot CANFD auto-upload motor identification completed',
                    'data': result,
                }
            except Exception as exc:
                cancelled = self._motor_step_test_stop_event.is_set()
                response = {
                    'device': 'tracking',
                    'action': 'test_motor_position_loop_once',
                    'status': 'cancelled' if cancelled else 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
            self.send_response(response)

        self._motor_step_test_thread = threading.Thread(
            target=worker,
            name='motor-position-loop-one-shot-identification',
            daemon=True,
        )
        self._motor_step_test_thread.start()
        estimated_s = (
            2.0
            * 2.0
            * len(tuple(test_kwargs['step_degrees']))
            * int(test_kwargs['repeats'])
            * float(test_kwargs['duration_s'])
        )
        return {
            'device': 'tracking',
            'action': 'test_motor_position_loop_once',
            'status': 'started',
            'message': 'One-shot CANFD auto-upload motor identification started',
            'data': {'estimated_duration_s': round(estimated_s, 1)},
        }

    def handle_tracking_command(self, command):
        action = command.get('action')
        if action == 'test_canfd_auto_upload':
            try:
                duration_s = float(command.get('duration_s', 1.0))
                result = self._test_canfd_auto_upload(duration_s)
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'success',
                    'message': (
                        'Periodic upload verified on both motors; disabled after test'
                        if result['all_verified'] and not result['cleanup_errors']
                        else 'Diagnostic completed; inspect all_verified and cleanup_errors'
                    ),
                    'data': result,
                }
            except Exception as exc:
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
        if action == 'test_motor_position_loop_once':
            try:
                return self._start_motor_position_step_test_once_async(command)
            except Exception as exc:
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
        if action == 'test_motor_position_loop':
            try:
                return self._start_motor_position_step_test_async(command)
            except Exception as exc:
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
        if action == 'stop_motor_position_loop_test':
            running = (
                self._motor_step_test_thread is not None
                and self._motor_step_test_thread.is_alive()
            )
            self._motor_step_test_stop_event.set()
            return {
                'device': 'tracking',
                'action': action,
                'status': 'success',
                'message': (
                    'Motor position-loop test stop requested'
                    if running else
                    'Motor position-loop test is not running'
                ),
            }
        if action == 'benchmark_rates':
            try:
                duration_s = float(command.get('duration_s', 2.0))
                result = self._run_safe_rate_benchmark(duration_s)
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'success',
                    'message': 'Safe uncapped rate benchmark completed',
                    'data': result,
                }
            except Exception as exc:
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'error',
                    'message': f'{type(exc).__name__}: {exc}',
                }
        if action == 'start':
            if (
                self._motor_step_test_thread is not None
                and self._motor_step_test_thread.is_alive()
            ):
                return {
                    'device': 'tracking', 'action': action, 'status': 'error',
                    'message': 'Stop motor position-loop identification before PSD tracking',
                }
            try:
                self._prepare_tracking_command_replies()
            except Exception as exc:
                return {
                    'device': 'tracking',
                    'action': action,
                    'status': 'error',
                    'message': (
                        'Unable to prepare CANFD command-reply feedback: '
                        f'{type(exc).__name__}: {exc}'
                    ),
                    'data': self.tracking_service.status(),
                }
            ready, message = self._tracking_preflight()
            if not ready:
                return {
                    'device': 'tracking', 'action': action, 'status': 'error',
                    'message': message, 'data': self.tracking_service.status(),
                }
            for motor_id in ('motor1', 'motor2'):
                motor = self.motors[motor_id]
                if self._mode_value(motor.get('mode')) != self.MOTOR_MODE_POSITION:
                    mode_result = self.motor_position_control(motor_id)
                    if mode_result.get('status') != 'success':
                        return {
                            'device': 'tracking',
                            'action': action,
                            'status': 'error',
                            'message': f'Unable to switch {motor_id} to position mode',
                        }
            # Preflight has confirmed at least one earlier command reply per
            # motor. Do not issue a separate position/status query here.
            # Start every tracking session from the latest measured position;
            # subsequent commands may accumulate only within the configured
            # target-lead bound.
            for motor_id in ('motor1', 'motor2'):
                self.current_targets[motor_id] = float(
                    self.current_positions[motor_id]
                )
            with self._motor_tx_condition:
                self._tracking_requested_targets = dict(self.current_targets)
                self._tracking_tx_pending = None
            self._reset_tracking_link_stats()
            ok, message = self.tracking_service.start()
            return {
                'device': 'tracking', 'action': action,
                'status': 'success' if ok else 'error',
                'message': message, 'data': self.tracking_service.status(),
            }

        if action == 'stop':
            ok, message = self.tracking_service.stop()
            return {
                'device': 'tracking', 'action': action,
                'status': 'success' if ok else 'error',
                'message': message, 'data': self.tracking_service.status(),
            }

        if action == 'status':
            return {
                'device': 'tracking', 'action': action, 'status': 'success',
                'message': 'PSD tracking status', 'data': self.tracking_service.status(),
            }

        return {
            'device': 'tracking', 'action': action, 'status': 'error',
            'message': 'Unknown tracking action',
        }

    def motor_init(self, motor_id):
        """初始化电机，并用一帧停止命令取得当前位置应答。"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'init', 'status': 'error', 'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        self._ensure_motor_defaults(motor)
        motor['run'] = False
        motor['target_position'] = 0.0
        motor['target_velocity'] = 0.0
        motor['target_current'] = 0.0
        feedback = self._send_canfd_motor_frame_and_read(
            motor_id,
            motor,
            timeout=0.5,
            position=0.0,
            velocity=0.0,
            current=0.0,
            mode=self.MOTOR_MODE_POSITION,
            run=False,
        )
        if not isinstance(feedback, dict):
            return {
                'device': 'motor',
                'action': 'init',
                'status': 'error',
                'message': (
                    f'Motor {motor_id} did not return a CANFD command reply'
                ),
            }
        measured_position = float(feedback['position'])
        motor['target_position'] = measured_position
        self.current_targets[motor_id] = measured_position
        self._motor_feedback_modes[motor_id] = 'command_reply'
        feedback['feedback_mode'] = 'command_reply'
        print(
            '[电机命令应答] '
            f'{motor_id} 初始化完成，当前位置={measured_position:+.6f}°'
        )
        return {
            'device': 'motor',
            'action': 'init',
            'status': 'success',
            'message': (
                f'Motor {motor_id} initialized; feedback mode=command_reply'
            ),
            'data': feedback,
        }

    def motor_enable(self, motor_id):
        """使能电机"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'enable', 'status': 'error', 'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        feedback = self._send_canfd_motor_frame_and_read(motor_id, motor, timeout=0.3, run=True)
        self._update_motor_state_from_feedback(motor, feedback)
        motor['run'] = True

        return {'device': 'motor', 'action': 'enable', 'status': 'success', 'message': f'Motor {motor_id} enabled',
                'data': feedback}

    def motor_disable(self, motor_id):
        """失能电机"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'disable', 'status': 'error', 'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        feedback = self._send_canfd_motor_frame_and_read(motor_id, motor, timeout=0.3, run=False)
        self._update_motor_state_from_feedback(motor, feedback)
        motor['run'] = False

        return {'device': 'motor', 'action': 'disable', 'status': 'success', 'message': f'Motor {motor_id} disabled',
                'data': feedback}

    def motor_position_control(self, motor_id):
        """无扰切到位置模式，并把实时位置作为新的位置目标。"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'position_control', 'status': 'error',
                    'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        self._ensure_motor_defaults(motor)
        if motor_id not in self.current_positions:
            return {
                'device': 'motor', 'action': 'position_control', 'status': 'error',
                'message': f'Motor {motor_id} has no position feedback; read status before switching',
            }
        was_running = bool(motor.get('run', False))
        current = motor.get('currency', motor.get('target_current', 0.0))
        current_position = float(self.current_positions[motor_id])
        position_before = current_position
        stop_feedback = None
        if self._mode_value(motor.get('mode')) == self.MOTOR_MODE_VELOCITY:
            stop_feedback = self._send_canfd_motor_frame_and_read(
                motor_id, motor, timeout=0.15,
                position=current_position, velocity=0.0, current=current,
                mode=self.MOTOR_MODE_VELOCITY, run=was_running,
            )
            if isinstance(stop_feedback, dict):
                try:
                    current_position = float(stop_feedback['position'])
                except (KeyError, TypeError, ValueError):
                    pass
        current_position = float(self.current_positions.get(motor_id, current_position))
        position_velocity = float(motor.get('velocity', 1.0))
        if position_velocity <= 0.0:
            position_velocity = 1.0
            motor['velocity'] = position_velocity
            print(f'[电机模式][参数修复] {motor_id} position velocity restored to 1.0rad/s')
        feedback = self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.3,
            position=current_position, velocity=position_velocity, current=current,
            mode=self.MOTOR_MODE_POSITION, run=was_running,
        )
        if not isinstance(feedback, dict):
            return {
                'device': 'motor', 'action': 'position_control', 'status': 'error',
                'message': f'Motor {motor_id} did not confirm position mode',
            }
        self._update_motor_state_from_feedback(motor, feedback)
        if self._mode_value(feedback.get('mode', self.MOTOR_MODE_POSITION)) != self.MOTOR_MODE_POSITION:
            return {
                'device': 'motor', 'action': 'position_control', 'status': 'error',
                'message': f'Motor {motor_id} feedback still reports mode={feedback.get("mode")}',
                'data': feedback,
            }
        motor['mode'] = self.MOTOR_MODE_POSITION
        motor['target_position'] = current_position
        self.current_targets[motor_id] = current_position
        print(
            '[电机模式][切换诊断] '
            f'{motor_id} velocity -> position | before={position_before:+.6f}° '
            f'zero_velocity_feedback={None if not isinstance(stop_feedback, dict) else stop_feedback.get("position")}° '
            f'hold={current_position:+.6f}° after={feedback.get("position")}° '
            f'velocity={feedback.get("velocity")}rad/s run={int(was_running)}'
        )

        return {'device': 'motor', 'action': 'position_control', 'status': 'success',
                'message': f'Motor {motor_id} switched to position mode at current position',
                'data': feedback}
    
    def motor_velocity_control(self, motor_id):
        """在当前位置无扰切到零速度模式。"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'velocity_control', 'status': 'error',
                    'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        self._ensure_motor_defaults(motor)
        if motor_id not in self.current_positions:
            return {
                'device': 'motor', 'action': 'velocity_control', 'status': 'error',
                'message': f'Motor {motor_id} has no position feedback; read status before switching',
            }
        was_running = bool(motor.get('run', False))
        current_position = float(self.current_positions[motor_id])
        current = motor.get('currency', motor.get('target_current', 0.0))
        feedback = self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.3,
            position=current_position, velocity=0.0, current=current,
            mode=self.MOTOR_MODE_VELOCITY, run=was_running,
        )
        if not isinstance(feedback, dict):
            return {
                'device': 'motor', 'action': 'velocity_control', 'status': 'error',
                'message': f'Motor {motor_id} did not confirm velocity mode',
            }
        self._update_motor_state_from_feedback(motor, feedback)
        if self._mode_value(feedback.get('mode', self.MOTOR_MODE_VELOCITY)) != self.MOTOR_MODE_VELOCITY:
            return {
                'device': 'motor', 'action': 'velocity_control', 'status': 'error',
                'message': f'Motor {motor_id} feedback still reports mode={feedback.get("mode")}',
                'data': feedback,
            }
        motor['mode'] = self.MOTOR_MODE_VELOCITY
        motor['target_position'] = current_position
        motor['target_velocity'] = 0.0
        self.current_targets[motor_id] = current_position
        print(
            '[电机模式][切换诊断] '
            f'{motor_id} position -> velocity | before={current_position:+.6f}° '
            f'after={feedback.get("position")}° velocity={feedback.get("velocity")}rad/s '
            f'mode={feedback.get("mode")} run={feedback.get("run")}'
        )

        return {'device': 'motor', 'action': 'velocity_control', 'status': 'success',
                'message': f'Motor {motor_id} switched to zero-velocity mode at current position',
                'data': feedback}

    def motor_set_zero(self, motor_id):
        """设置零点"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'set_zero', 'status': 'error',
                    'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        was_running = bool(motor.get('run', False))

        # CANFD zero command must be sent while disabled, and the zero byte
        # must be cleared after a single command frame. If the motor was
        # enabled before zeroing, restore RunCmd after clearing Zero.
        self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.3, position=0, velocity=0, current=0, run=False, zero=0
        )

        feedback = self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.5, position=0, velocity=0, current=0, run=False, zero=1
        )

        clear_feedback = self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.3, position=0, velocity=0, current=0, run=False, zero=0
        )

        verify_feedback = self._send_canfd_motor_frame_and_read(
            motor_id, motor, timeout=0.3, position=0, velocity=0, current=0, run=False, zero=0
        )

        restore_feedback = None
        if was_running:
            restore_feedback = self._send_canfd_motor_frame_and_read(
                motor_id, motor, timeout=0.3, position=0, velocity=0, current=0, run=True, zero=0
            )

        final_feedback = restore_feedback or verify_feedback or clear_feedback or feedback
        self._update_motor_state_from_feedback(motor, final_feedback)
        motor['run'] = was_running

        motor['target_position'] = 0.0
        self.current_targets[motor_id] = 0.0
        self.current_positions[motor_id] = 0.0

        return {'device': 'motor', 'action': 'set_zero', 'status': 'success', 'message': f'Motor {motor_id} zero set',
                'data': final_feedback}

    def motor_move_to_position(self, motor_id, position):
        """只下发位置；实际位置由独立接收线程异步更新。"""
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'move_to_position', 'status': 'error',
                    'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        with self.motor_transaction_lock:
            self._send_canfd_motor_frame(
                motor,
                position=position,
                velocity=motor.get('velocity', 0),
                current=motor.get('currency', 0),
                mode=self.MOTOR_MODE_POSITION,
            )
        self.current_targets[motor_id] = float(position)
        return {
            'device': 'motor',
            'action': 'move_to_position',
            'status': 'success',
            'message': (
                f'Motor {motor_id} position command sent: {position}°; '
                'feedback is received asynchronously'
            ),
            'data': {
                'target_position': float(position),
                'feedback_source': 'position_command_reply',
            },
        }

    def motor_move_with_velocity(self, motor_id, velocity, current=None):
        """速度模式运动。
        velocity: 目标速度，单位 rad/s。
        current: 电流限制，单位 A；不传则使用电机配置里的 currency。
        """
        if motor_id not in self.motors:
            return {'device': 'motor', 'action': 'move_with_velocity', 'status': 'error',
                    'message': f'Motor {motor_id} not found'}

        motor = self.motors[motor_id]
        if current is None:
            current = motor.get('currency', motor.get('target_current', 0))
        else:
            motor['currency'] = current

        motor_data = self._send_canfd_motor_frame_and_read(
            motor_id,
            motor,
            timeout=0.3,
            velocity=velocity,
            current=current,
            mode=self.MOTOR_MODE_VELOCITY
        )

        return {
            'device': 'motor',
            'action': 'move_with_velocity',
            'status': 'success',
            'message': f'Motor {motor_id} velocity set to {velocity} rad/s',
            'data': motor_data
        }

    def motor_get_status(self, motor_id):
        """返回独立接收线程缓存的最新 CANFD 自动反馈。"""
        return self._cached_motor_status_response(motor_id)

    def parse_motor_feedback(self, rx_data_ascii, motor_id):
        """解析电机反馈数据"""
        if not rx_data_ascii:
            return None

        try:
            frames = [
                frame.strip()
                for frame in rx_data_ascii.replace('\n', '\r').split('\r')
                if frame.strip()
            ]

            for frame in reversed(frames):
                frame_prefix = frame[0]
                frame_type = frame_prefix.lower()

                if frame_type in ('d', 'b'):
                    # CANFD-SLCAN uses d/b + 3-digit ID + DLC for standard
                    # frames, and D/B + 8-digit ID + DLC for extended frames.
                    # Automatic feedback firmware variants may choose either.
                    header_length = 5 if frame_prefix.islower() else 10
                    if len(frame) < header_length + 32:
                        continue
                    rx_data_hex = frame[
                        header_length:header_length + 32
                    ]
                    rx_data = bytearray(
                        int(rx_data_hex[i:i + 2], 16)
                        for i in range(0, len(rx_data_hex), 2)
                    )
                    pole_pairs = self.motors.get(motor_id, {}).get('pole_pairs', self.DEFAULT_POLE_PAIRS)

                    position_counts = self._bytes_to_signed(rx_data[0:5])
                    velocity_counts = self._bytes_to_signed(rx_data[5:9])
                    current_counts = self._bytes_to_signed(rx_data[9:11])

                    position = position_counts / self.CANFD_POSITION_COUNTS_PER_REV * 360.0
                    electrical_hz = (
                        velocity_counts / self.CANFD_VELOCITY_SCALE * self.CANFD_VELOCITY_FULL_SCALE_HZ
                    )
                    velocity = electrical_hz / float(pole_pairs) * 2.0 * math.pi
                    current = current_counts / self.CANFD_CURRENT_SCALE * self.CANFD_CURRENT_FULL_SCALE_A
                    temperature = int.from_bytes(bytes([rx_data[14]]), byteorder='big', signed=True)
                    slave_id = rx_data[15]
                    expected_slave_id = self._expected_slave_id(motor_id)
                    if expected_slave_id is not None and slave_id != expected_slave_id:
                        continue

                    return {
                        'id': motor_id,
                        'position': round(position, 3),
                        'velocity': round(velocity, 3),
                        'current': round(current, 3),
                        'mode': rx_data[11],
                        'run': bool(rx_data[12] & 0x0F),
                        'error_code': (rx_data[12] >> 4) & 0x0F,
                        'temperature': temperature,
                        'temperature_c': temperature,
                        'slave_id': slave_id
                    }

                if frame_type == 't' and len(frame) >= 5:
                    dlc = int(frame[4], 16)
                    rx_data_hex = frame[5:5 + dlc * 2]
                    if len(rx_data_hex) < 12:
                        continue
                    rx_data = bytearray(
                        int(rx_data_hex[i:i + 2], 16)
                        for i in range(0, len(rx_data_hex), 2)
                    )

                    motor_position = (rx_data[1] << 8) + rx_data[2]
                    motor_velocity = ((rx_data[3] & 0xFF) << 4) + ((rx_data[4] & 0xF0) >> 4) - 2048
                    motor_current = ((rx_data[4] & 0x0F) << 8) + rx_data[5] - 2048

                    position = ((motor_position - 0x8000) / 0x8000) * 360
                    velocity = motor_velocity / 0x800 * 58.639
                    current = motor_current / 0x800 * 4

                    return {
                        'id': motor_id,
                        'position': round(position, 3),
                        'velocity': round(velocity, 3),
                        'current': round(current, 3)
                    }
        except Exception as e:
            print(f"解析反馈数据错误: {e}")
        return None

    # ==================== 激光测距仪命令处理 ====================
    def handle_laser_command(self, command):
        """处理激光测距仪命令"""
        action = command.get('action')

        if action == 'enter_control_mode':
            success = self.write_laser_data('CPU1')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': 'Enter control mode'}

        elif action == 'laser_on':
            delay = command.get('delay', 10)
            success = self.write_laser_data(f'LON,{delay}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Laser on with {delay}s delay'}

        elif action == 'laser_off':
            delay = command.get('delay', 5)
            success = self.write_laser_data(f'LFF,{delay}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Laser off with {delay}s delay'}

        elif action == 'laser_sfc':
            sfc = command.get('sfc')
            success = self.write_laser_data(f'SFC,{sfc}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Laser SFC with {sfc}'}

        elif action == 'laser_lfc':
            lfc = command.get('lfc')
            success = self.write_laser_data(f'LFC,{lfc}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Laser LFC with {lfc}'}

        elif action == 'laser_target_num':
            tar = command.get('tar', 2)
            success = self.write_laser_data(f'TAR,{tar}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Laser Tar with {tar}'}

        elif action == 'query_temperature':
            success = self.write_laser_data('TEM')
            print("查询温度命令已发送")
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': 'Query temperature'}

        elif action == 'set_com1_temp':
            value = command.get('value', 0)
            success = self.write_laser_data(f'TE1,{value}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Set COM1 temp to {value}'}

        elif action == 'set_com2_temp':
            value = command.get('value', 0)
            success = self.write_laser_data(f'TE2,{value}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Set COM2 temp to {value}'}

        elif action == 'increase_current':
            level = command.get('level', 1)
            success = self.write_laser_data(f'INC,{level}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Increase current level {level}'}

        elif action == 'decrease_current':
            level = command.get('level', 1)
            success = self.write_laser_data(f'DEC,{level}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Decrease current level {level}'}

        elif action == 'set_threshold_ref':
            value = command.get('value', 0)
            success = self.write_laser_data(f'TH1,{value}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Set threshold ref to {value}'}

        elif action == 'set_threshold_sig':
            value = command.get('value', 0)
            success = self.write_laser_data(f'TH2,{value}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Set threshold sig to {value}'}
        
        elif action == 'set_sig':
            value = command.get('value', 0)
            success = self.write_laser_data(f'SIG,{value}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Set SIG to {value}'}

        elif action == 'enter_adjust_mode':
            success = self.write_laser_data('AJT')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': 'Enter adjust mode'}

        elif action == 'measure_distance_mode':
            success = self.write_laser_data('MEA,1')
            # 开启测距模式时,自动启用数据流推送
            if success:
                self.laser_streaming = True
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': 'Enter measure distance mode'}

        elif action == 'stop_measurement':
            success = self.write_laser_data('1')
            # 停止测距时,关闭数据流推送
            # if success:
            #     self.laser_streaming = False
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': 'Stop measurement'}

        elif action == 'debug_mode':
            enable = command.get('enable', True)
            success = self.write_laser_data(f'DBG,{1 if enable else 0}')
            return {'device': 'laser', 'action': action, 'status': 'success' if success else 'error',
                    'message': f'Debug mode {"enabled" if enable else "disabled"}'}

        elif action == 'start_data_stream':
            # 手动开启数据流推送
            self.laser_streaming = True
            return {'device': 'laser', 'action': action, 'status': 'success',
                    'message': 'Data streaming started'}

        elif action == 'stop_data_stream':
            # 手动关闭数据流推送
            self.laser_streaming = False
            return {'device': 'laser', 'action': action, 'status': 'success',
                    'message': 'Data streaming stopped'}

        elif action == 'get_status':
            with self.laser_data_lock:
                data = self.latest_laser_data
            return {'device': 'laser', 'action': action, 'status': 'success',
                    'data': {'latest_reading': data, 'streaming': self.laser_streaming}}

        return {'device': 'laser', 'action': action, 'status': 'error', 'message': 'Unknown laser action'}

    def stop_server(self):
        """停止服务器"""
        self.running = False
        self._motor_step_test_stop_event.set()
        self.tracking_service.stop()
        self._motor_tx_stop_event.set()
        with self._motor_tx_condition:
            self._tracking_tx_pending = None
            self._motor_tx_condition.notify_all()
        if self._motor_tx_thread is not None and self._motor_tx_thread.is_alive():
            self._motor_tx_thread.join(1.0)
        # Stop the firmware stream while the serial RX thread is still alive,
        # otherwise the controller keeps filling the USB adapter after exit.
        if self.motor_serial is not None and self.motor_serial.is_open:
            with self.motor_transaction_lock:
                for motor_id in tuple(self.motors):
                    if not self._motor_auto_upload_enabled.get(motor_id, False):
                        continue
                    try:
                        self._set_canfd_auto_upload(motor_id, False)
                    except Exception as exc:
                        print(
                            '[电机自动反馈][关闭警告] '
                            f'{motor_id}: {type(exc).__name__}: {exc}'
                        )
        self._motor_feedback_publish_stop_event.set()
        with self.motor_feedback_condition:
            self.motor_feedback_condition.notify_all()
        if (
            self._motor_feedback_publish_thread is not None
            and self._motor_feedback_publish_thread.is_alive()
        ):
            self._motor_feedback_publish_thread.join(1.0)
        self._motor_rx_stop_event.set()
        if self._motor_rx_thread is not None and self._motor_rx_thread.is_alive():
            self._motor_rx_thread.join(1.0)
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        if self.motor_serial:
            self.motor_serial.close()
        if self.laser_serial:
            self.laser_serial.close()


# 使用示例
if __name__ == '__main__':
    # 创建服务器
    server = RaspberryPiDeviceServer(
        host='0.0.0.0',
        port=8888,
        motor_port='/dev/ttyACM0',
        motor_baud=1000000,
        laser_port='/dev/ttyUSB0',
        laser_baud=921600
    )

    try:
        # 启动服务器
        server.start_server()
    except KeyboardInterrupt:
        print("\n停止服务器...")
        server.stop_server()
