import unittest

from pso_flight_demo.integrated_ros import pair_groundtruth


def pose(timestamp):
    return {'native_timestamp_us': timestamp}


class IntegratedRosPairingTests(unittest.TestCase):
    def test_uses_closest_buffered_pose_instead_of_latest_skewed_pose(self):
        px4_timestamp = 100_000
        older = pose(97_000)
        latest = pose(142_000)

        selected = pair_groundtruth(px4_timestamp, [older, latest], latest)

        self.assertIs(selected, older)

    def test_keeps_the_twenty_millisecond_pairing_gate(self):
        with self.assertRaisesRegex(ValueError, 'DDS_GROUNDTRUTH_CLOCK_PAIR'):
            pair_groundtruth(100_000, [pose(120_001)])

    def test_fails_closed_without_any_groundtruth_sample(self):
        with self.assertRaisesRegex(ValueError, 'DDS_GROUNDTRUTH_CLOCK_PAIR'):
            pair_groundtruth(100_000, [])


if __name__ == '__main__':
    unittest.main()
