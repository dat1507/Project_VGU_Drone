"""GUI clock workflow tests: synthetic timestamps only, no ROS/Gazebo calls."""
import unittest

from pso_flight_demo.dds_evidence import REQUIRED
from pso_flight_demo.integrated_flight import prepare_plan
from pso_flight_demo.integrated_ros import require_readiness_progress
from pso_flight_demo.safety import Freshness
from pso_flight_demo.tests import test_integrated_flight as fixtures


class GUIPlayTests(unittest.TestCase):
    def stamps(self, value):
        return dict.fromkeys((*REQUIRED, 'failsafe_flags', 'gazebo_groundtruth'), value)

    def test_all_normalized_dds_sources_and_groundtruth_advance(self):
        require_readiness_progress(self.stamps(15_000_000), self.stamps(16_000_000))

    def test_paused_clock_cannot_pass_readiness(self):
        first = self.stamps(15_000_000)
        with self.assertRaisesRegex(ValueError, 'WAIT_EXTERNAL_CLOCK_PROGRESS'):
            require_readiness_progress(first, first)

    def test_each_required_source_must_advance_not_just_timesync(self):
        first = self.stamps(15_000_000)
        for key in first:
            with self.subTest(key=key):
                current = self.stamps(16_000_000)
                current[key] = first[key]
                with self.assertRaisesRegex(ValueError, 'WAIT_EXTERNAL_CLOCK_PROGRESS:'+key):
                    require_readiness_progress(first, current)

    def test_missing_topic_or_groundtruth_cannot_pass(self):
        for key in self.stamps(1):
            current = self.stamps(16_000_000)
            del current[key]
            with self.subTest(key=key), self.assertRaises(KeyError):
                require_readiness_progress(self.stamps(15_000_000), current)

    def test_reversed_nonfinite_zero_native_time_rejected(self):
        for value in (14_999_999, 0, float('nan'), float('inf')):
            current = self.stamps(16_000_000)
            current['gazebo_groundtruth'] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'INVALID_OR_BACKWARDS'):
                require_readiness_progress(self.stamps(15_000_000), current)

    def test_epoch_change_requires_a_new_progressing_baseline(self):
        # The adapter clears its readiness baseline on a prestream epoch reset.
        # No epoch offset is guessed here; these are already normalized times.
        new_epoch = self.stamps(27_000_000)
        with self.assertRaises(ValueError):
            require_readiness_progress(new_epoch, new_epoch)
        require_readiness_progress(new_epoch, self.stamps(28_000_000))

    def test_gui_contract_is_same_for_both_algorithms_without_relaxing_default(self):
        for algorithm in ('standard', 'enhanced'):
            default = prepare_plan(algorithm)
            demo = prepare_plan(algorithm, demo_only=True)
            self.assertTrue(default['static_blockers'])
            self.assertEqual(demo['static_blockers'], [])
            self.assertEqual(demo['waypoint_count'], 17)
            self.assertEqual(demo['csv_sha256'], default['csv_sha256'])
            self.assertEqual(demo['physics_policy'], 'EXTERNAL_GUI_PLAY')
            self.assertTrue(demo['startup_requires_paused'])
            self.assertFalse(demo['paused_required_during_flight'])
            self.assertFalse(demo['controls_physics'] or demo['live_control_go'])

    def test_gui_play_prestream_needs_real_simulation_progress_and_mode_ack(self):
        f = fixtures.MissionTests(); f.setUp()
        mission = f.m
        mission.step(f.s())
        # Wall time alone, while the GUI is still Paused, cannot switch/Arm.
        for tick in range(1, 51):
            decision = mission.step(f.s(1+tick*.05, native_us=15_000_000))
            self.assertIsNone(decision['command'])
        self.assertEqual(mission.stage, 'PRESTREAM')
        # GUI Play advances native time; no paused flag is required by mission.
        for tick in range(1, 25):
            wall = 3.5+tick*.05
            native = 15_000_000+tick*50_000
            decision = mission.step(f.s(wall, native_us=native))
        self.assertEqual(decision['command']['command'], 176)
        self.assertEqual(mission.stage, 'MODE_PENDING')
        mission.ack(176, 0, wall, native)
        decision = mission.step(f.s(wall+.05, native_us=native+50_000, nav=4))
        self.assertIsNone(decision['command'])
        decision = mission.step(f.s(wall+.10, native_us=native+100_000, nav=14))
        self.assertEqual(mission.stage, 'ARM_READINESS')
        self.assertIsNone(decision['command'])
        decision = mission.step(f.s(wall+.15, native_us=native+150_000, nav=14))
        self.assertEqual(decision['command']['command'], 400)

    def test_pause_during_flight_expires_source_freshness_without_blind_land(self):
        fresh = Freshness()
        for key in REQUIRED:
            fresh.record(key, 20_000_000, 10.)
            fresh.record(key, 20_000_000, 12.)  # New delivery, frozen source clock.
        with self.assertRaisesRegex(ValueError, 'Stale/frozen telemetry'):
            fresh.check(REQUIRED, 12.)
        f = fixtures.MissionTests(); f.setUp()
        # Adapter has no usable paired snapshot after a freshness exception.
        decision = f.m.fail('Stale/frozen telemetry', None)
        self.assertEqual(decision['stage'], 'ABORT_OPERATOR_REQUIRED')
        self.assertIsNone(decision['command'])
        self.assertIsNone(decision['target'])


if __name__ == '__main__':
    unittest.main()
