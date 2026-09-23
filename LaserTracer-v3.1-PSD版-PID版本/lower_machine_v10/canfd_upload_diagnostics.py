"""Evidence for CANFD upload tests; no motor I/O in the decoder/observer."""

import threading
import time


DLC_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def decode_frame(frame):
    """Decode strict SLCAN data frames (optional four-digit RX timestamp)."""
    if not frame or frame[0] not in 'dDbBtT':
        return None
    header = 5 if frame[0].islower() else 10
    try:
        can_id = int(frame[1:header - 1], 16)
        dlc = int(frame[header - 1], 16)
        if frame[0] in 'tT' and dlc > 8:
            return None
        length = DLC_LENGTHS[dlc]
        end = header + length * 2
        if len(frame) not in (end, end + 4):
            return None
        payload = bytes.fromhex(frame[header:end])
        if len(payload) != length:
            return None
        return can_id, payload
    except (ValueError, IndexError):
        return None


def is_parameter_reply(frame, motor_can_id, index):
    """Manual p50: standard FD, ID=motor ID, payload=01,index,value,...,0."""
    decoded = decode_frame(frame)
    if decoded is None or frame[0] not in 'db':
        return False
    can_id, data = decoded
    value_end = 4 if index == 0 else 6
    return (can_id == motor_can_id and len(data) == 16
            and data[:2] == bytes((1, index)) and not any(data[value_end:]))


class UploadObservation:
    """Collect bounded samples and counts before normal feedback parsing."""

    def __init__(self, motor_can_id, duration_s, parameter_index=None, clock=time.monotonic):
        self.motor_can_id = motor_can_id
        self.duration_s = duration_s
        self.parameter_index = parameter_index
        self.clock = clock
        self.started_at = clock()
        self.lock = threading.Lock()
        self.frames = 0
        self.status_frames = 0
        self.parameter_replies = 0
        self.buckets = [0] * 4
        self.types = {}
        self.first_frames = []
        self.last_frame = None
        self.tx_count = 0
        self.tx = []
        self.guard_s = min(0.050, duration_s / 4)

    def record_tx(self, text):
        with self.lock:
            for command in text.split('\r'):
                if command:
                    self.tx_count += 1
                    if len(self.tx) < 8:
                        self.tx.append(command)

    def record_rx(self, frame):
        elapsed = self.clock() - self.started_at
        decoded = decode_frame(frame)
        matched_parameter = (
            self.parameter_index is not None
            and is_parameter_reply(frame, self.motor_can_id, self.parameter_index)
        )
        with self.lock:
            self.frames += 1
            self.types[frame[:1]] = self.types.get(frame[:1], 0) + 1
            sample = {'ms': round(elapsed * 1000, 3), 'frame': frame}
            if len(self.first_frames) < 5:
                self.first_frames.append(sample)
            self.last_frame = sample
            if matched_parameter:
                self.parameter_replies += 1
            elif decoded and frame[0] in 'dDbB':
                _, data = decoded
                if len(data) == 16 and data[-1] == self.motor_can_id and data[11] <= 2:
                    self.status_frames += 1
                    if self.guard_s <= elapsed < self.duration_s:
                        bucket = min(3, int((elapsed - self.guard_s)
                                            / (self.duration_s - self.guard_s) * 4))
                        self.buckets[bucket] += 1
        # A parameter reply is not a position/status sample.
        return matched_parameter

    def summary(self):
        with self.lock:
            late_frames = sum(self.buckets)
            continuous = sum(n > 0 for n in self.buckets) >= 3 and self.buckets[-1] > 0
            return {
                'raw_frames': self.frames,
                'status_frames': self.status_frames,
                'parameter_replies': self.parameter_replies,
                'raw_types': dict(self.types),
                'first_frames': list(self.first_frames),
                'last_frame': self.last_frame,
                'tx_count': self.tx_count,
                'tx_commands': list(self.tx),
                'duration_s': self.duration_s,
                'ack_guard_ms': self.guard_s * 1000,
                'late_status_buckets': list(self.buckets),
                'late_status_frames': late_frames,
                'continuous_status': bool(continuous),
                'late_status_hz': round(late_frames / (self.duration_s - self.guard_s), 1),
            }


def verify_upload_cycle(before, enabled, disabled):
    """Only a quiet -> sustained -> quiet transition proves the switch works."""
    if any(phase['tx_count'] != 1 for phase in (before, enabled, disabled)):
        return 'interfering_commands'
    if before['late_status_frames'] or disabled['late_status_frames']:
        return 'off_state_not_quiet'
    if enabled['continuous_status']:
        return 'periodic_auto_upload'
    if enabled['late_status_frames']:
        return 'sporadic_feedback_unconfirmed'
    if enabled['status_frames']:
        return 'status_reply_without_periodic_upload'
    return 'no_matching_feedback'
