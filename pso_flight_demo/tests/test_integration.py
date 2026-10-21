import ast
import contextlib
from dataclasses import replace
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from pso_flight_demo import artifacts
from pso_flight_demo.__main__ import capture_anchor, main, parser
from pso_flight_demo.frames import FrameAnchor, convert_yaw, enu_to_ned, first_waypoint, ned_to_enu
from pso_flight_demo.geometry import AABB, FlightEnvironment, load_world
from pso_flight_demo.planner import PlannerConfig, inertia, run, spline_polyline
from pso_flight_demo.safety import (Freshness, Observation, RouteTracker, check_observation,
                                   matching_ack, mode_confirmed, quaternion_tilt, topic_name)

MODULE = Path(__file__).resolve().parents[1]
ROOT = MODULE.parent
SDF = ROOT / "simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf"


def anchor():
    # Synthetic measured-pair fixture: never used as a flight default.
    return FrameAnchor((-11.76, 4.30, -0.013), (0.03, 0.02, 0.01),
                       396120000, 396116000, (5, 2, 5, 2, 1, 1044000),
                       "px4_groundtruth_and_local_position")


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.box = AABB("box", (1, 1, 1), (2, 2, 2))
        self.env = FlightEnvironment([self.box], [-5]*3, [5]*3, 0, 0)

    def test_segment_crossing_without_waypoint_inside(self):
        self.assertTrue(self.env.collision_mask([[0, 1.5, 1.5], [3, 1.5, 1.5]])[0])

    def test_parallel_tangent_and_stationary(self):
        for path in ([[0, 1, 1], [3, 1, 1]], [[1, 1, 1], [1, 1, 1]]):
            self.assertTrue(self.env.collision_mask(path)[0])
        self.assertFalse(self.env.collision_mask([[0, .9, 1], [3, .9, 1]])[0])

    def test_finite_segment_not_infinite_line(self):
        self.assertFalse(self.env.collision_mask([[-3, 1.5, 1.5], [0, 1.5, 1.5]])[0])

    def test_inflation_catches_point_safe_near_miss(self):
        path = [[0, .5, 1.5], [3, .5, 1.5]]
        self.assertFalse(self.env.collision_mask(path)[0])
        inflated = FlightEnvironment([self.box], [-5]*3, [5]*3)
        self.assertTrue(inflated.collision_mask(path)[0])

    def test_randomized_checker_agrees_with_original_point_geometry(self):
        from environment import Cube, Environment
        cubes = [Cube((30, 30, 30), 15), Cube((60, 70, 50), 20)]
        original = Environment(cubes, 100)
        boxes = [AABB(str(i), tuple(c.lower), tuple(c.upper)) for i, c in enumerate(cubes)]
        adapted = FlightEnvironment(boxes, [0]*3, [100]*3, 0, 0)
        rng = np.random.default_rng(81)
        paths = rng.uniform(-10, 110, (1000, 5, 3))
        np.testing.assert_array_equal(adapted.collision_mask(paths), original.collision_mask(paths))

    def test_bounds_nan_shapes(self):
        with self.assertRaises(ValueError):
            self.env.validate([[0, 0, 0], [6, 0, 0]])
        for path in ([[0, 0, 0], [np.nan, 0, 0]], [[0, 0, 0]], [[[0, 0, 0], [1, 1, 1]]]):
            with self.assertRaises(ValueError):
                self.env.validate(path)

    def test_actual_world(self):
        w = load_world(SDF)
        self.assertEqual(w.name, "warehouse_world")
        self.assertEqual(len(w.boxes), 14)
        self.assertEqual(w.floor_z, 0)
        np.testing.assert_allclose(w.physical_xy_lower, [-14.9, -11.9])
        np.testing.assert_allclose(w.physical_xy_upper, [14.9, 11.9])
        self.assertEqual(w.sdf_sha256, hashlib.sha256(SDF.read_bytes()).hexdigest())
        env = w.environment(1., 3.2)
        p = first_waypoint(anchor(), env)
        np.testing.assert_allclose(p[1] - p[0], [.5, 0, 0])
        np.testing.assert_allclose(p[0] - anchor().gazebo_enu, [0, 0, 2.5])
        np.testing.assert_array_equal(p[0], p[-1])
        with self.assertRaises(ValueError):
            w.environment(.85, 3.2)

    def test_parser_rejects_unknown_shape_frame_or_dynamic_model(self):
        text = SDF.read_text()
        variants = [text.replace("<box>", "<sphere>", 1),
                    text.replace("<pose>0 12 1", "<pose relative_to='other'>0 12 1", 1),
                    text.replace("<static>true</static>", "<static>false</static>", 1),
                    text.replace("model://x500", "model://unknown", 1)]
        with tempfile.TemporaryDirectory(dir=MODULE) as d:
            p = Path(d) / "world.sdf"
            for variant in variants:
                p.write_text(variant)
                with self.assertRaises(ValueError):
                    load_world(p)

    def test_rotated_shelf_is_conservative(self):
        text = SDF.read_text().replace("-10 8 0.5 0 0 0", "-10 8 0.5 0 0 1.5707963267948966")
        with tempfile.TemporaryDirectory(dir=MODULE) as d:
            p = Path(d) / "rotated.sdf"
            p.write_text(text)
            box = next(b for b in load_world(p).boxes if b.name == "shelf_red")
            np.testing.assert_allclose(np.array(box.upper)-box.lower, [1, 2, 1], atol=1e-12)


class FrameTests(unittest.TestCase):
    def test_axes_roundtrip_and_anchor(self):
        np.testing.assert_array_equal(enu_to_ned([1, 2, 3]), [2, 1, -3])
        rng = np.random.default_rng(4)
        p = rng.normal(size=(20, 3))
        np.testing.assert_allclose(ned_to_enu(enu_to_ned(p)), p)
        a = anchor()
        np.testing.assert_allclose(a.to_enu(a.to_ned(p)), p, atol=1e-14)
        np.testing.assert_allclose(a.to_ned(a.gazebo_enu), a.px4_ned)

    def test_yaw_and_reset(self):
        self.assertAlmostEqual(convert_yaw(0), np.pi/2)
        self.assertAlmostEqual(convert_yaw(np.pi/2), 0)
        anchor().check_resets(anchor().reset_counters)
        with self.assertRaises(ValueError):
            anchor().check_resets((6, 2, 5, 2, 1, 1044000))

    def test_no_spawn_anchor_or_unsynchronized_pair(self):
        with self.assertRaises(ValueError):
            replace(anchor(), source="spawn")
        with self.assertRaises(ValueError):
            replace(anchor(), gazebo_timestamp_us=400000000)
        with self.assertRaises(ValueError):
            replace(anchor(), px4_ned=(float("nan"), 0, 0))

    def test_native_capture_is_read_only_and_requires_grounded_validity(self):
        a = anchor()
        gt = dict(x="4.3", y="-11.76", z="0.013", timestamp_sample="396120000 (0 us)")
        lp = dict(x=".03", y=".02", z=".01", timestamp_sample="396116000 (0 us)",
                  **{k: str(v) for k, v in zip(("xy_reset_counter", "z_reset_counter", "vxy_reset_counter",
                                              "vz_reset_counter", "heading_reset_counter", "ref_timestamp"), a.reset_counters)},
                  **{k: "True" for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")})
        with patch("pso_flight_demo.__main__._listener", side_effect=[{"armed": "False"}, {"landed": "True"}, gt, lp]):
            measured = capture_anchor("unused")
        np.testing.assert_allclose(measured.gazebo_enu, a.gazebo_enu)
        with patch("pso_flight_demo.__main__._listener", side_effect=[{"armed": "True"}, {"landed": "True"}]):
            with self.assertRaises(ValueError):
                capture_anchor("unused")


class ParityTests(unittest.TestCase):
    def test_original_core_equivalence(self):
        from pso_core import run_pso
        from scenarios import make_environment, START, GOAL

        originals = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
                     for p in ("pso_core.py", "baseline_pso.py", "pso.py", "scenarios.py")}
        for algorithm in ("baseline", "enhanced"):
            for scenario, seed, cfg in [("sparse", 0, PlannerConfig()),
                                        ("dense", 7, PlannerConfig(swarm=8, iterations=9, n_waypoints=4))]:
                with self.subTest(algorithm=algorithm, scenario=scenario):
                    env = make_environment(scenario)
                    old = run_pso(env, algorithm=algorithm, seed=seed, swarm=cfg.swarm,
                                  iterations=cfg.iterations, n_waypoints=cfg.n_waypoints, jitter=cfg.jitter)
                    new = run(env, START, GOAL, [0]*3, [100]*3, algorithm, seed, cfg)
                    np.testing.assert_array_equal(new.positions, old.positions)
                    np.testing.assert_array_equal(new.path, old.path)
                    np.testing.assert_array_equal(new.best_history, old.best_history)
                    for key in ("fitness", "raw_length", "raw_collision", "convergence_iteration", "fitness_evaluations"):
                        self.assertEqual(getattr(new, key), getattr(old, key))
                    self.assertEqual(new.fitness_evaluations, cfg.swarm*(cfg.iterations+1))
        for p, digest in originals.items():
            self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(), digest)

    def test_configurable_bounds_and_warehouse_planning(self):
        w = load_world(SDF)
        env = w.environment(1., 3.2)
        p = first_waypoint(anchor(), env)
        for algorithm in ("baseline", "enhanced"):
            result = run(env, p[0], p[1], env.lower, env.upper, algorithm, 3,
                         PlannerConfig(swarm=8, iterations=10, n_waypoints=3, jitter=.15))
            self.assertFalse(result.raw_collision)
            env.validate(result.path)
            np.testing.assert_array_equal(result.path[[0, -1]], p[:2])
            self.assertTrue(np.all(np.diff(result.best_history) <= 0))

    def test_inertia_and_invalid_configuration(self):
        self.assertEqual(inertia("baseline", 199, 200), .7)
        self.assertEqual(inertia("enhanced", 0, 200), .9)
        self.assertAlmostEqual(inertia("enhanced", 199, 200), .4025)
        for kwargs in (dict(swarm=0), dict(swarm=1.5), dict(jitter=-1), dict(penalty=float("nan"))):
            with self.assertRaises(ValueError):
                PlannerConfig(**kwargs)


class ArtifactTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        self.patch = patch.object(artifacts, "MODULE_DIR", Path(self.temp.name))
        self.patch.start()
        self.world = load_world(SDF)
        self.env = self.world.environment(1., 3.2)
        self.path = first_waypoint(anchor(), self.env)

    def tearDown(self):
        self.patch.stop()
        self.temp.cleanup()

    def export(self, name="route", **kwargs):
        return artifacts.export_route(name, self.path, self.world, self.env, anchor(),
                                      algorithm="first_waypoint", seed=None, **kwargs)

    def test_exclusive_roundtrip_provenance_and_offline_default(self):
        dest = self.export()
        csv_before = (dest / "waypoints_enu.csv").read_bytes()
        with self.assertRaises(FileExistsError):
            self.export()
        self.assertEqual((dest / "waypoints_enu.csv").read_bytes(), csv_before)
        path, meta, a, _ = artifacts.load_route(dest)
        np.testing.assert_array_equal(path, self.path)
        self.assertFalse(meta["flight_tested"])
        self.assertIn("not_LUT", meta["provenance"])
        np.testing.assert_allclose(a.to_ned(path)[1] - a.to_ned(path)[0], [0, .5, 0])
        self.assertFalse(parser().parse_args(["follow", str(dest)]).execute)
        with patch("pso_flight_demo.ros_follower.run_live", side_effect=AssertionError("must not run")):
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                main(["follow", str(dest)])
        self.assertEqual(json.loads(stream.getvalue())["result"], "OFFLINE_VALIDATED_ONLY")

    def test_csv_tampering_and_frame_rejected(self):
        dest = self.export()
        (dest / "waypoints_enu.csv").write_text("x_enu_m,y_enu_m,z_enu_m\n0,0,0\n1,1,1\n")
        with self.assertRaises(ValueError):
            artifacts.load_route(dest)
        dest = self.export("other")
        meta = json.loads((dest / "manifest.json").read_text())
        meta["frame"] = "NED"
        (dest / "manifest.json").write_text(json.dumps(meta))
        with self.assertRaises(ValueError):
            artifacts.load_route(dest)

    def test_no_overwrite_traversal_and_invalid_postprocessing(self):
        for name in ("../escape", "/tmp/escape", ".", ""):
            with self.assertRaises(ValueError):
                self.export(name)
        unsafe = self.path.copy()
        unsafe[1] = [-10, 8, 1]
        with self.assertRaises(ValueError):
            artifacts.export_route("unsafe", unsafe, self.world, self.env, anchor(),
                                   algorithm="first_waypoint", seed=None, raw_path=self.path,
                                   smoothing="test")
        self.assertFalse((Path(self.temp.name)/"outputs/unsafe").exists())

    def test_spline_preserves_endpoints_but_still_requires_collision_check(self):
        p = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [2, 1, 0]], float)
        smoothed = spline_polyline(p)
        self.assertEqual(smoothed.shape, (200, 3))
        np.testing.assert_array_equal(smoothed[[0, -1]], p[[0, -1]])

    def test_real_spline_collision_is_rejected_after_raw_safe_plan(self):
        from pso_core import run_pso
        from scenarios import make_environment
        env = make_environment("sparse")
        raw = run_pso(env, algorithm="baseline", seed=0).path
        self.assertFalse(env.collides(raw))
        smoothed = spline_polyline(raw)
        self.assertTrue(env.collides(smoothed))
        checker = FlightEnvironment([AABB(str(i), tuple(c.lower), tuple(c.upper))
                                     for i, c in enumerate(env.cubes)], [0]*3, [100]*3, 0, 0)
        checker.validate(raw)
        with self.assertRaises(ValueError):
            checker.validate(smoothed)


class SafetyTests(unittest.TestCase):
    def obs(self):
        a = anchor()
        return Observation(tuple(a.to_ned(np.array(a.gazebo_enu)+[0, 0, 2.5])),
                           (1., 0., 0., 0.), (0., 0., 0.), a.reset_counters,
                           a.px4_timestamp_us+1000000, True, False, False, False, True, False, True)

    def test_observation_fault_reset_tilt_and_validity_gates(self):
        env = load_world(SDF).environment(1., 3.2)
        obs = self.obs()
        check_observation(obs, anchor(), env)
        roll = np.radians(16)
        for bad in (replace(obs, valid=False), replace(obs, failsafe=True),
                    replace(obs, gcs_lost=True), replace(obs, estimator_fault=True),
                    replace(obs, resets=(0,)*6),
                    replace(obs, quaternion=(np.cos(roll/2), np.sin(roll/2), 0, 0))):
            with self.assertRaises(ValueError):
                check_observation(bad, anchor(), env)

    def test_freshness_detects_frozen_source_even_when_callbacks_arrive(self):
        f = Freshness()
        f.record("position", 1000, 0)
        f.record("position", 1000, 2)
        with self.assertRaises(ValueError):
            f.check(["position"], 2)
        f.record("position", 2000, 2)
        f.check(["position"], 2)
        with self.assertRaises(ValueError):
            f.record("position", 500, 3)
        with self.assertRaises(ValueError):
            f.check(["status"], 2)

    def test_speed_bound_five_second_hold_and_no_waypoint_skip(self):
        t = RouteTracker([[0, 0, 1], [.5, 0, 1], [0, 0, 1]])
        actual = np.array([0., 0., 1.])
        done_at = None
        for i in range(401):
            before = t.target.copy()
            target, done = t.update(actual, i*.05)
            self.assertLessEqual(np.linalg.norm(target-before), .0100000001)
            actual = target
            if done:
                done_at = i*.05
                break
        self.assertIsNotNone(done_at)
        self.assertGreaterEqual(done_at, 15)
        with self.assertRaises(ValueError):
            t.update([1, 1, 1], 30)

    def test_versioned_topic_and_tilt(self):
        class Message:
            MESSAGE_VERSION = 4
        self.assertEqual(topic_name("/fmu/out/vehicle_status", Message), "/fmu/out/vehicle_status_v4")
        self.assertEqual(quaternion_tilt([1, 0, 0, 0]), (0, 0))
        with self.assertRaises(ValueError):
            quaternion_tilt([0, 0, 0, 0])

    def test_ack_requires_matching_source_command_time_and_mode(self):
        from types import SimpleNamespace
        ack = SimpleNamespace(command=176, target_system=245, target_component=191, result=0)
        self.assertIsNone(matching_ack([(1., ack)], 176, 2.))
        self.assertIsNone(matching_ack([(3., ack)], 21, 2.))
        self.assertEqual(matching_ack([(3., ack)], 176, 2.), 0)
        other = SimpleNamespace(command=176, target_system=1, target_component=191, result=0)
        self.assertIsNone(matching_ack([(3., other)], 176, 2.))
        self.assertFalse(mode_confirmed(None, 14, 14, 1))
        self.assertFalse(mode_confirmed(0, 4, 14, 1))
        self.assertTrue(mode_confirmed(0, 14, 14, 1))
        for ack, mode, elapsed in [(1, 14, 0), (None, 14, 4), (0, 4, 4)]:
            with self.assertRaises(ValueError):
                mode_confirmed(ack, mode, 14, elapsed)

    def test_live_opt_in_guard_and_no_arm_takeoff_commands(self):
        from pso_flight_demo.ros_follower import run_live
        with self.assertRaises(ValueError):
            run_live(None, None, None, None, "")
        tree = ast.parse((MODULE / "ros_follower.py").read_text())
        ids = [node.args[0].value for node in ast.walk(tree)
               if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
               and node.func.attr == "command" and node.args and isinstance(node.args[0], ast.Constant)]
        self.assertCountEqual(ids, [21, 176])


if __name__ == "__main__":
    unittest.main()
