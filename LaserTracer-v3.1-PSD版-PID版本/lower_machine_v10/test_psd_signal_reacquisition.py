"""Regression check for an optical dropout during PSD tracking."""

import json
import unittest
from pathlib import Path

from psd_tracker import PsdTrackingController, TrackerState


class SignalReacquisitionTests(unittest.TestCase):
    def test_reacquired_spot_does_not_use_position_from_before_signal_loss(self):
        config = json.loads(
            Path(__file__).with_name('psd_calibration.json').read_text(encoding='utf-8')
        )
        controller = PsdTrackingController(config)
        controller.start()
        for _ in range(config['controller']['acquire_valid_samples']):
            controller.update(0.5, -0.5, valid=True)

        lost = None
        for _ in range(config['controller']['lost_after_invalid_samples']):
            lost = controller.update(None, None, valid=False)
        self.assertEqual(lost.state, TrackerState.SIGNAL_LOST)
        self.assertIsNone(lost.filtered_norm)

        fresh = controller.update(-0.4, 0.4, valid=True)
        self.assertEqual(fresh.state, TrackerState.ACQUIRING)
        self.assertEqual(fresh.filtered_norm, (-0.4, 0.4))
        self.assertFalse(fresh.should_move)


if __name__ == '__main__':
    unittest.main()
