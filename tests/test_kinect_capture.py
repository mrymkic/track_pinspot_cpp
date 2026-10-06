import sys
from pathlib import Path
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from kinect_capture import CameraFrame, choose_pair


def frame(sequence, timestamp, received=None):
    return CameraFrame(sequence, time.monotonic() if received is None else received,
                       np.zeros((2, 2, 4), np.uint8), np.zeros((2, 2), np.uint16),
                       timestamp, timestamp, timestamp * 1000)


class PairingTests(unittest.TestCase):
    def test_does_not_accept_one_frame_lag_as_synchronized(self):
        base = frame(1, 100_000)
        aux = frame(2, 100_000 + 160 + 33_333)
        self.assertIsNone(choose_pair([base], [aux], 160))

    def test_finds_corresponding_frame_when_latest_streams_differ(self):
        base = [frame(1, 100_000, 1.0), frame(2, 133_333, 2.0)]
        aux = [frame(1, 100_160, 1.1)]
        matched = choose_pair(base, aux, 160)
        self.assertEqual((matched[0].sequence, matched[1].sequence), (1, 1))

    def test_chooses_newest_pair_with_independent_clock_origins(self):
        offset = 5_000_160
        base = [frame(1, 100_000, 1.0), frame(2, 133_333, 2.0)]
        aux = [frame(1, 100_000 + offset, 1.1), frame(2, 133_333 + offset, 2.1)]
        matched = choose_pair(base, aux, offset)
        self.assertEqual((matched[0].sequence, matched[1].sequence), (2, 2))


if __name__ == "__main__":
    unittest.main()
