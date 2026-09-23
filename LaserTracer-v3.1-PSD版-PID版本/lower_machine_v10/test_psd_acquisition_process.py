"""Offline checks for the spawned latest-sample acquisition path."""

import multiprocessing as mp
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from ad7606_reader import Ad7606Sample
from psd_acquisition_process import (
    BUSY_SEEN, GEN, HAS_SAMPLE, RAW, SEQUENCE, STATE_SIZE, TIMESTAMP_NS,
    VOLTAGES, PsdAcquisitionProcess, _run_acquisition,
)


def fake_process_worker(config, rate_hz, max_timeouts, state, stop, ready, errors):
    with state.get_lock():
        state[GEN] = 1
        state[HAS_SAMPLE] = 1
        state[SEQUENCE] = 7
        state[TIMESTAMP_NS] = time.monotonic_ns()
        state[VOLTAGES:VOLTAGES + 8] = (0.1, 0.2, 2.0, 0, 0, 0, 0, 0)
        state[RAW:RAW + 8] = (1, 2, 3, 0, 0, 0, 0, 0)
        state[BUSY_SEEN] = 1
    ready.set()
    stop.wait(1.0)


class FakeReader:
    def __init__(self, stop):
        self.stop = stop
        self.sequence = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_sample(self):
        self.sequence += 1
        if self.sequence == 3:
            self.stop.set()
        return Ad7606Sample(
            self.sequence, time.monotonic_ns(), (1,) * 8,
            (0.1, 0.2, 2.0) + (0.0,) * 5, True,
        )


class AcquisitionProcessTests(unittest.TestCase):
    def test_spawned_process_publishes_latest_sample(self):
        config = {'controller': {'max_consecutive_adc_timeouts': 5}}
        try:
            process = PsdAcquisitionProcess(config, 1000.0, worker=fake_process_worker)
            process.start()
        except PermissionError as exc:
            self.skipTest(f'OS sandbox blocks multiprocessing pipes: {exc}')
        try:
            snapshot = process.snapshot()
            self.assertEqual(snapshot['generation'], 1)
            self.assertEqual(snapshot['samples'], 7)
            self.assertEqual(snapshot['sample'].voltages[:3], (0.1, 0.2, 2.0))
            self.assertEqual(snapshot['sample'].raw[:3], (1, 2, 3))
            self.assertIsNone(process.error())
        finally:
            process.stop()

    def test_worker_records_samples_without_queueing_old_values(self):
        context = mp.get_context('spawn')
        state = context.Array('d', STATE_SIZE)
        stop = threading.Event()
        ready = threading.Event()
        errors = SimpleNamespace(put_nowait=lambda message: None)
        with patch('psd_acquisition_process._build_reader', return_value=FakeReader(stop)):
            _run_acquisition({}, None, 5, state, stop, ready, errors)
        self.assertTrue(ready.is_set())
        self.assertEqual(int(state[GEN]), 3)
        self.assertEqual(int(state[SEQUENCE]), 3)
        self.assertEqual(int(state[HAS_SAMPLE]), 1)


if __name__ == '__main__':
    unittest.main()
