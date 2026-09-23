"""Offline regression tests using real user traces and synthetic upload timing."""

import contextlib
import io
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from canfd_upload_diagnostics import (
    UploadObservation, decode_frame, is_parameter_reply, verify_upload_cycle,
)
from raspberrypi_to_pyside import RaspberryPiDeviceServer


USER_STATUS = 'd064A0000000000FFFFFFFB007E0201002501'


def sample_phase(times, tx_count=1, frame=USER_STATUS):
    clock = [0.0]
    observation = UploadObservation(1, 1.0, clock=lambda: clock[0])
    for _ in range(tx_count):
        observation.record_tx('D00000001A000601' + '00' * 13 + '\r')
    for timestamp in times:
        clock[0] = timestamp
        observation.record_rx(frame)
    return observation.summary()


class UploadTests(unittest.TestCase):
    def test_real_temperature_probe_was_generic_status(self):
        self.assertEqual(len(decode_frame(USER_STATUS)[1]), 16)
        self.assertFalse(is_parameter_reply(USER_STATUS, 1, 0))
        observation = UploadObservation(1, 1.0, parameter_index=0)
        observation.record_rx(USER_STATUS)
        self.assertEqual(observation.summary()['parameter_replies'], 0)
        self.assertEqual(observation.summary()['status_frames'], 1)

    def test_parameter_reply_requires_header_id_and_padding(self):
        frame = 'd001A01002501' + '00' * 12
        self.assertTrue(is_parameter_reply(frame, 1, 0))
        self.assertFalse(is_parameter_reply(frame, 2, 0))
        self.assertFalse(is_parameter_reply(frame, 1, 1))
        self.assertFalse(is_parameter_reply('d064' + frame[4:], 1, 0))

    def test_reply_burst_is_not_continuous_upload(self):
        quiet = sample_phase([0.002])
        burst = sample_phase([0.001, 0.002, 0.003, 0.004])
        self.assertEqual(verify_upload_cycle(quiet, burst, quiet),
                         'status_reply_without_periodic_upload')

    def test_low_rate_upload_counts_if_on_off_transition_works(self):
        quiet = sample_phase([0.002])
        enabled = sample_phase([0.002, 0.10, 0.35, 0.6, 0.9])
        self.assertEqual(verify_upload_cycle(quiet, enabled, quiet),
                         'periodic_auto_upload')
        self.assertLess(enabled['late_status_hz'], 100)

    def test_cross_motor_and_interfering_transmissions(self):
        quiet = sample_phase([0.002])
        another_motor = sample_phase([0.1, 0.4, 0.7, 0.9], frame=USER_STATUS[:-2]+'02')
        self.assertEqual(another_motor['status_frames'], 0)
        stream = sample_phase([0.1, 0.4, 0.7, 0.9], tx_count=2)
        self.assertEqual(verify_upload_cycle(quiet, stream, quiet), 'interfering_commands')

    def test_stream_that_does_not_stop_is_not_a_verified_switch(self):
        quiet = sample_phase([0.002])
        stream = sample_phase([0.1, 0.4, 0.7, 0.9])
        self.assertEqual(verify_upload_cycle(quiet, stream, stream), 'off_state_not_quiet')

    def make_server(self):
        server = object.__new__(RaspberryPiDeviceServer)
        server.motors = {'motor1': {'canid': '01'}, 'motor2': {'canid': '02'}}
        server.tracking_service = SimpleNamespace(is_running=False)
        server._motor_rx_thread = SimpleNamespace(is_alive=lambda: True)
        server._rate_benchmark_lock = threading.Lock()
        server.motor_transaction_lock = threading.RLock()
        server._motor_tx_condition = threading.Condition()
        server._motor_auto_upload_variants = {}
        server._motor_auto_upload_enabled = {'motor1': True, 'motor2': True}
        server._motor_feedback_modes = {}
        server.writes = []
        server.write_motor_data = lambda command, **kwargs: server.writes.append(command)
        server.init_motor_serial = lambda: self.fail('Diagnostic must not reset CAN channel')
        return server

    def test_failed_probe_cleanup_does_not_reenable_or_send_motion(self):
        server = self.make_server()
        def observe(mid, command, duration, parameter_index=None):
            server.writes.append(command)
            result = sample_phase([0.002])
            result.update(adapter_nacks=0, parse_errors=0)
            return result
        server._observe_upload_phase = observe
        with patch('raspberrypi_to_pyside.time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            result = server._test_canfd_auto_upload()
        self.assertFalse(result['all_verified'])
        self.assertEqual(result['cleanup_errors'], [])
        self.assertFalse(any(server._motor_auto_upload_enabled.values()))
        for command in server.writes:
            self.assertTrue(command.startswith('D'))
            self.assertIn(decode_frame(command.strip())[1][:2], (b'\x00\x06', b'\x01\x00', b'\x01\x01'))
        self.assertTrue(all(decode_frame(command.strip())[1][2] == 0
                            for command in server.writes[-4:]))
        self.assertFalse(server._rate_benchmark_lock.locked())

    def test_exception_still_closes_upload_and_releases_lock(self):
        server = self.make_server()
        def fail(*args):
            raise RuntimeError('fake receive failure')
        server._observe_upload_phase = fail
        with patch('raspberrypi_to_pyside.time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'fake receive failure'):
                server._test_canfd_auto_upload()
        self.assertFalse(server._rate_benchmark_lock.locked())
        self.assertFalse(any(server._motor_auto_upload_enabled.values()))
        self.assertEqual(len(server.writes[-4:]), 4)


if __name__ == '__main__':
    unittest.main()
