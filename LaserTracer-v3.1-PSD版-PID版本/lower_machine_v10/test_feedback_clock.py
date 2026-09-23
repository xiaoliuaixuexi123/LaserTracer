"""Offline checks for the paired-motor-feedback control clock."""

import json
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from ad7606_reader import Ad7606Sample
from raspberrypi_to_pyside import RaspberryPiDeviceServer
from tracking_service import PsdTrackingService


class FakeReader:
    def __init__(self):
        self.sequence = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_sample(self):
        self.sequence += 1
        return Ad7606Sample(
            self.sequence,
            time.monotonic_ns(),
            (0,) * 8,
            (0.05558, 0.04165, 1.8608) + (0.0,) * 5,
            True,
        )


class FeedbackClockTests(unittest.TestCase):
    def setUp(self):
        self.server = object.__new__(RaspberryPiDeviceServer)
        self.server.motor_feedback_condition = threading.Condition()
        self.server._motor_feedback_generation = {'motor1': 1, 'motor2': 1}

    def advance(self, motor_id):
        with self.server.motor_feedback_condition:
            self.server._motor_feedback_generation[motor_id] += 1
            self.server.motor_feedback_condition.notify_all()

    def test_waits_for_both_new_motor_samples(self):
        baseline = self.server._wait_for_tracking_feedback_pair(None, 0.0)
        received = []
        worker = threading.Thread(
            target=lambda: received.append(
                self.server._wait_for_tracking_feedback_pair(baseline, 0.3)
            )
        )
        worker.start()
        self.advance('motor1')
        time.sleep(0.02)
        self.assertEqual(received, [])
        self.advance('motor2')
        worker.join(0.2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(received, [{'motor1': 2, 'motor2': 2}])

    def test_tracking_updates_only_after_paired_feedback(self):
        config_path = Path(__file__).with_name('psd_calibration.json')
        config = json.loads(config_path.read_text(encoding='utf-8'))
        config['controller'].update(
            acquisition_backend='thread',
            sample_rate_hz=200.0,
            command_rate_hz=100.0,
            minimum_feedback_rate_hz=80.0,
            status_update_rate_hz=100.0,
            motor_feedback_stale_stop_s=0.5,
        )
        for zone in config['controller']['position_zones'].values():
            zone['command_rate_hz'] = 100.0

        service = PsdTrackingService(
            config_path,
            lambda *args: self.fail('Centered PSD must not request movement'),
            wait_motor_feedback_pair=self.server._wait_for_tracking_feedback_pair,
        )
        service._load_config = lambda: config
        with patch.object(PsdTrackingService, '_build_reader', return_value=FakeReader()):
            started, message = service.start()
            self.assertTrue(started, message)
            try:
                time.sleep(0.04)
                self.assertEqual(service.status()['control_cycles'], 0)
                self.advance('motor1')
                time.sleep(0.03)
                self.assertEqual(service.status()['control_cycles'], 0)
                self.advance('motor2')
                deadline = time.monotonic() + 0.2
                while service.status()['control_cycles'] < 1 and time.monotonic() < deadline:
                    time.sleep(0.005)
                status = service.status()
                self.assertEqual(status['control_cycles'], 1)
                self.assertEqual(status['controller_updates'], 1)
                self.assertGreater(status['recent_feedback_pair_rate_hz'], 0.0)
                self.assertGreater(status['recent_control_rate_hz'], 0.0)
            finally:
                service.stop()

    def test_tracking_faults_when_paired_feedback_stops(self):
        config_path = Path(__file__).with_name('psd_calibration.json')
        config = json.loads(config_path.read_text(encoding='utf-8'))
        config['controller']['acquisition_backend'] = 'thread'
        config['controller']['motor_feedback_stale_stop_s'] = 0.08
        service = PsdTrackingService(
            config_path,
            lambda *args: self.fail('No movement is expected'),
            wait_motor_feedback_pair=self.server._wait_for_tracking_feedback_pair,
        )
        service._load_config = lambda: config
        with patch.object(PsdTrackingService, '_build_reader', return_value=FakeReader()):
            started, message = service.start()
            self.assertTrue(started, message)
            deadline = time.monotonic() + 0.3
            while service.status()['state'] != 'error' and time.monotonic() < deadline:
                time.sleep(0.005)
            status = service.status()
            self.assertEqual(status['state'], 'error')
            self.assertIn('Paired motor feedback stopped', status['last_error'])
            service.stop()

    def test_tracking_process_backend_reads_shared_latest_sample(self):
        config_path = Path(__file__).with_name('psd_calibration.json')
        config = json.loads(config_path.read_text(encoding='utf-8'))
        config['controller'].update(
            acquisition_backend='process',
            sample_rate_hz=200.0,
            command_rate_hz=100.0,
            minimum_feedback_rate_hz=80.0,
            status_update_rate_hz=100.0,
        )
        for zone in config['controller']['position_zones'].values():
            zone['command_rate_hz'] = 100.0
        sample = FakeReader().read_sample()

        class FakeProcess:
            def __init__(self, config, rate_hz):
                pass

            def start(self):
                pass

            def snapshot(self):
                return {
                    'generation': 1, 'sample': sample, 'samples': 1,
                    'adc_timeouts': 0, 'consecutive_adc_timeouts': 0,
                    'last_adc_error': None, 'adc_read_mean_ms': 0.2,
                    'adc_read_max_ms': 0.2, 'adc_schedule_overruns': 0,
                    'adc_schedule_lag_max_ms': 0.0,
                }

            def error(self):
                return None

            def stop(self):
                pass

        service = PsdTrackingService(
            config_path,
            lambda *args: self.fail('Centered PSD must not request movement'),
            wait_motor_feedback_pair=self.server._wait_for_tracking_feedback_pair,
        )
        service._load_config = lambda: config
        with patch('tracking_service.PsdAcquisitionProcess', FakeProcess):
            started, message = service.start()
            self.assertTrue(started, message)
            try:
                self.advance('motor1')
                self.advance('motor2')
                deadline = time.monotonic() + 0.2
                while service.status()['control_cycles'] < 1 and time.monotonic() < deadline:
                    time.sleep(0.005)
                status = service.status()
                self.assertEqual(status['acquisition_backend'], 'process')
                self.assertEqual(status['controller_updates'], 1)
            finally:
                service.stop()

    def test_tracking_preflight_requires_verified_motor_rate(self):
        server = self.server
        now = time.monotonic()
        server.motor_serial = object()
        server._motor_rx_thread = SimpleNamespace(is_alive=lambda: True)
        server._motor_tx_thread = SimpleNamespace(is_alive=lambda: True)
        server.motors = {
            motor_id: {'run': True, 'mode': 2}
            for motor_id in ('motor1', 'motor2')
        }
        server.current_targets = {'motor1': 0.0, 'motor2': 0.0}
        server.current_positions = dict(server.current_targets)
        server._motor_feedback_timestamps = {
            motor_id: now for motor_id in ('motor1', 'motor2')
        }
        server._validate_tracking_motor_parameters = lambda: []
        server._tracking_control_clock = 'motor_feedback'
        server._tracking_command_rate_hz = 290.0
        server._motor_tx_rate_hz = 290.0
        server._tracking_min_feedback_rate_hz = 250.0
        server._motor_link_last_targets = dict(server.current_targets)
        server._motor_link_last_window_s = 1.0
        server._motor_link_last_rates_at = now
        server._motor_link_watchdog_min_ratio = 0.9
        server._motor_link_last_rates = {'motor1': 271.0, 'motor2': 265.0}
        self.assertEqual(server._tracking_preflight(), (True, 'ready'))
        server._motor_link_last_rates['motor2'] = 249.0
        ready, message = server._tracking_preflight()
        self.assertFalse(ready)
        self.assertIn('below the required 250', message)

    def test_link_sampling_reuses_commanded_target_when_feedback_stale(self):
        server = self.server
        server._tracking_target_lock = threading.RLock()
        server.motor_transaction_lock = threading.RLock()
        server.motors = {
            motor_id: {'run': True, 'mode': 2}
            for motor_id in ('motor1', 'motor2')
        }
        server.current_positions = {'motor1': 0.0, 'motor2': 0.0}
        server.current_targets = {'motor1': 0.25, 'motor2': -0.125}
        server._motor_feedback_timestamps = {
            motor_id: time.monotonic() - 10.0
            for motor_id in ('motor1', 'motor2')
        }
        sent = []
        server._send_tracking_target_batch = lambda targets, **kwargs: (
            sent.append(targets) or 0.1
        )
        server._wait_for_feedback_pair = lambda generations, timeout: True
        self.assertEqual(
            server.start_motor_link_sampling(),
            {'motor1': 0.25, 'motor2': -0.125},
        )
        self.assertEqual(sent, [{'motor1': 0.25, 'motor2': -0.125}])

    def test_link_sampling_requires_known_commanded_target(self):
        server = self.server
        server._tracking_target_lock = threading.RLock()
        server.motor_transaction_lock = threading.RLock()
        server.motors = {
            motor_id: {'run': True, 'mode': 2}
            for motor_id in ('motor1', 'motor2')
        }
        server.current_positions = {'motor1': 0.0, 'motor2': 0.0}
        server.current_targets = {}
        server._motor_feedback_timestamps = {
            motor_id: time.monotonic() - 10.0
            for motor_id in ('motor1', 'motor2')
        }
        with self.assertRaisesRegex(RuntimeError, 'No known commanded target'):
            server.start_motor_link_sampling()

    def test_watchdog_does_not_call_sender_slowdown_a_bus_limit(self):
        server = self.server
        now = time.monotonic()
        server._update_link_rate_window = lambda: True
        server._motor_link_watchdog = True
        server._motor_link_probe_reset_pending = False
        server._motor_link_keepalive_pause_depth = 0
        server._motor_link_last_rates_at = now
        server._motor_link_last_send_at = now
        server._motor_link_probe_counts = {'motor1': 100, 'motor2': 100}
        server._motor_link_last_rates = {'motor1': 92.0, 'motor2': 92.0}
        server._motor_link_last_tx_hz = 94.0
        server._motor_tx_rate_hz = 290.0
        server._motor_link_watchdog_min_ratio = 0.8
        server._motor_link_low_windows = 1
        server._motor_link_bounce_probe_until = now - 1.0
        server._motor_link_limited_hz = None
        server._bounce_motor_link = lambda: self.fail('Slow TX must not reset CAN')
        server._service_link_watchdog()
        self.assertEqual(server._motor_link_low_windows, 0)
        self.assertEqual(server._motor_link_bounce_probe_until, 0.0)
        self.assertIsNone(server._motor_link_limited_hz)

    def test_max_rate_benchmark_starts_from_idle_link(self):
        server = self.server
        server.tracking_service = SimpleNamespace(is_running=False)
        server._rate_benchmark_lock = threading.Lock()
        server._motor_link_last_targets = None
        server._motor_link_watchdog = True
        server.motor_serial = SimpleNamespace(is_open=True)
        server._motor_rx_thread = SimpleNamespace(is_alive=lambda: True)
        server._motor_tx_thread = SimpleNamespace(is_alive=lambda: True)
        server._tracking_target_lock = threading.RLock()
        server.motor_transaction_lock = threading.RLock()
        server._motor_tx_condition = threading.Condition()
        server._tracking_requested_targets = {}
        server._tracking_tx_pending = None
        events = []
        server.start_motor_link_sampling = lambda: (
            events.append('arm') or {'motor1': 0.0, 'motor2': 0.0}
        )
        server.stop_motor_link_sampling = lambda: events.append('stop')
        server._pause_link_keepalive = lambda: events.append('pause')
        server._resume_link_keepalive = lambda: events.append('resume')
        server._send_tracking_target_batch = lambda *args, **kwargs: 0.1
        server._benchmark_psd_until = lambda *args: {'sample_hz': 2000.0}
        server._benchmark_motor_until = lambda *args: {
            'tx_pair_hz': 1000.0, 'completed_pair_hz': 900.0,
        }
        server._tracking_preflight = lambda: self.fail(
            'The maximum-rate test must not require an already fast link'
        )
        benchmark_config = json.loads(
            Path(__file__).with_name('psd_calibration.json').read_text(encoding='utf-8')
        )
        benchmark_config['controller']['acquisition_backend'] = 'thread'
        with patch.object(
            Path, 'read_text', return_value=json.dumps(benchmark_config)
        ), patch.object(PsdTrackingService, '_build_reader', return_value=FakeReader()):
            result = server._run_safe_rate_benchmark(0.5)
        self.assertEqual(result['motor_only']['completed_pair_hz'], 900.0)
        self.assertEqual(result['configured']['psd_target_hz'], 1000.0)
        self.assertEqual(result['configured']['motor_target_hz'], 290.0)
        self.assertTrue(result['configured']['meets_target'])
        self.assertEqual(events, ['arm', 'pause', 'resume', 'stop'])
        self.assertTrue(server._motor_link_watchdog)

    def test_rate_benchmark_uses_process_when_configured(self):
        server = self.server
        server.tracking_service = SimpleNamespace(is_running=False)
        server._rate_benchmark_lock = threading.Lock()
        server._motor_link_last_targets = None
        server._motor_link_watchdog = True
        server.motor_serial = SimpleNamespace(is_open=True)
        server._motor_rx_thread = SimpleNamespace(is_alive=lambda: True)
        server._motor_tx_thread = SimpleNamespace(is_alive=lambda: True)
        server._tracking_target_lock = threading.RLock()
        server.motor_transaction_lock = threading.RLock()
        server._motor_tx_condition = threading.Condition()
        server._tracking_requested_targets = {}
        server._tracking_tx_pending = None
        server.start_motor_link_sampling = lambda: {'motor1': 0.0, 'motor2': 0.0}
        server.stop_motor_link_sampling = lambda: None
        server._pause_link_keepalive = lambda: None
        server._resume_link_keepalive = lambda: None
        server._send_tracking_target_batch = lambda *args, **kwargs: 0.1
        server._benchmark_motor_until = lambda *args: {
            'tx_pair_hz': 290.0, 'completed_pair_hz': 280.0,
        }

        class FakeProcess:
            def __init__(self, config, rate_hz):
                self.calls = 0

            def start(self):
                pass

            def snapshot(self):
                self.calls += 1
                return {
                    'samples': 0 if self.calls == 1 else 1000,
                    'adc_read_mean_ms': 0.2,
                    'adc_read_max_ms': 0.3,
                    'adc_schedule_overruns': 0,
                    'adc_timeouts': 0,
                }

            def error(self):
                return None

            def stop(self):
                pass

        benchmark_config = json.loads(
            Path(__file__).with_name('psd_calibration.json').read_text(encoding='utf-8')
        )
        benchmark_config['controller']['acquisition_backend'] = 'process'
        with patch.object(
            Path, 'read_text', return_value=json.dumps(benchmark_config)
        ), patch('raspberrypi_to_pyside.PsdAcquisitionProcess', FakeProcess):
            result = server._run_safe_rate_benchmark(0.5)
        self.assertEqual(result['configured']['psd']['backend'], 'process')
        self.assertEqual(result['configured']['motor']['tx_pair_hz'], 290.0)
        self.assertTrue(result['configured']['meets_target'])


if __name__ == '__main__':
    unittest.main()
