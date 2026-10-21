"""Synthetic runtime fixtures are never labelled as real host measurements."""

import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from pso_flight_demo import c2
from pso_flight_demo.artifacts import load_route
from pso_flight_demo.environment_artifacts import dry_run, export_bundle, load_bundle, waypoint
from pso_flight_demo.environments import MODULE, load_environment, load_profile
from pso_flight_demo.launch_plan import launch_plan
from pso_flight_demo.runtime_readiness import inspect_runtime, pose_xyz, topic_fields, phase_problems
from pso_flight_demo.tests.test_integration import anchor

SDF_TEMPLATE = """<sdf version="1.10"><world name="test_other_world">
<spherical_coordinates><world_frame_orientation>ENU</world_frame_orientation>
<heading_deg>0</heading_deg></spherical_coordinates>
<model name="ground"><static>true</static><link name="l">
<collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size>
</plane></geometry></collision></link></model>
<model name="obstacle"><static>true</static><pose>3 3 1 0 0 0</pose><link name="l">
<collision name="c"><geometry><box><size>1 2 2</size></box></geometry></collision>
</link></model><include><uri>model://x500</uri><name>test_drone</name>
<pose>-4 -4 .24 0 0 0</pose></include></world></sdf>"""

IDENTITY = dict(boot_id="TEST_ONLY", px4_pid=10, px4_start_ticks=100,
                gazebo_server_pid=20, gazebo_start_ticks=200,
                px4_count=1, gazebo_server_count=1, agent_count=0, qgc_count=1)


class C2ConfigurationTests(unittest.TestCase):
    def test_warehouse_and_explicit_unconfigured_sparse(self):
        world, env = load_environment(load_profile())
        self.assertEqual(world.name, "warehouse_world")
        self.assertEqual(len(world.boxes), 14)
        np.testing.assert_allclose(env.lower, [-14.05, -11.05, 1.])
        planned = load_profile("sparse_lut_planned")
        self.assertIsNone(planned.sdf)
        with self.assertRaisesRegex(ValueError, "no SDF"):
            load_environment(planned)

    def test_generic_world_names_floor_spawn_and_bounds(self):
        with tempfile.TemporaryDirectory(dir=MODULE) as d:
            p = Path(d)/"test.sdf"
            p.write_text(SDF_TEMPLATE)
            cfg = replace(load_profile(), id="unit_test_world", sdf=str(p), world_name="test_other_world",
                          floor_model="ground", vehicle_name="test_drone",
                          physical_xy_bounds=[[-8, -8], [8, 8]])
            w, env = load_environment(cfg)
            self.assertEqual(w.spawn_enu, (-4., -4., .24))
            self.assertEqual(len(w.boxes), 1)
            np.testing.assert_allclose(env.upper, [7.15, 7.15, 3.2])
            cfg = replace(cfg, output_root=str(Path(d).relative_to(MODULE)/"results"))
            measured = replace(anchor(), gazebo_enu=(-4., -4., -.013))
            route = waypoint(measured, cfg, env)
            dest = export_bundle(cfg, measured, route, identity=IDENTITY)
            self.assertFalse(dry_run(dest)["c1_bundle_loader_compatible"])
            np.testing.assert_allclose(load_bundle(dest)[0], route)

    def test_mesh_nested_dynamic_and_unknown_include_fail_closed(self):
        with tempfile.TemporaryDirectory(dir=MODULE) as d:
            p = Path(d)/"test.sdf"
            cfg = replace(load_profile(), sdf=str(p), world_name="test_other_world",
                          floor_model="ground", vehicle_name="test_drone",
                          physical_xy_bounds=[[-8, -8], [8, 8]])
            variants = [SDF_TEMPLATE.replace("<box><size>1 2 2</size></box>", "<mesh><uri>file://x</uri></mesh>"),
                        SDF_TEMPLATE.replace("<static>true</static>", "<static>false</static>", 1),
                        SDF_TEMPLATE.replace("model://x500", "model://unknown"),
                        SDF_TEMPLATE.replace("<heading_deg>0", "<heading_deg>20"),
                        SDF_TEMPLATE.replace('<link name="l">', '<model name="nested"/><link name="l">', 1)]
            for text in variants:
                p.write_text(text)
                with self.assertRaises(ValueError):
                    load_environment(cfg)

    def test_configured_endpoints_are_checked_before_export(self):
        with self.assertRaises(ValueError):
            load_environment(replace(load_profile(), goal_enu=[-10, 8, 1]))
        with self.assertRaises(ValueError):
            replace(load_profile(), margin_m=.1)
        for root in ("/tmp/out", "../out", "."):
            with self.assertRaises(ValueError):
                replace(load_profile(), output_root=root)

    def test_launch_is_print_only_no_duplicate_model_spawn(self):
        with patch("subprocess.run", side_effect=AssertionError("must not execute")):
            plan = launch_plan(load_profile())
        self.assertEqual(plan["launched_processes"], 0)
        texts = "\n".join(x["command"] for x in plan["commands"])
        self.assertIn("PX4_GZ_STANDALONE=1", texts)
        self.assertIn("PX4_GZ_MODEL_NAME=x500", texts)
        self.assertNotIn(" -r ", texts)
        self.assertNotIn("multi_step", texts)
        self.assertFalse(plan["auto_unpause"])
        self.assertFalse(plan["auto_arm"])
        with self.assertRaises(SystemExit):
            with contextlib.redirect_stderr(io.StringIO()):
                c2.parser().parse_args(["launch", "--execute"])


class C2BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        relative = str(Path(self.temp.name).relative_to(MODULE)/"outputs")
        self.cfg = replace(load_profile(), output_root=relative)
        self.world, self.env = load_environment(self.cfg)
        self.path = waypoint(anchor(), self.cfg, self.env)

    def tearDown(self):
        self.temp.cleanup()

    def test_c2_warehouse_is_c1_compatible_and_non_overwriting(self):
        dest = export_bundle(self.cfg, anchor(), self.path, identity=IDENTITY, name="route")
        result = dry_run(dest, IDENTITY)
        self.assertFalse(result["publishes_commands"])
        self.assertTrue(result["reboot_identity_checked"])
        np.testing.assert_allclose(result["outward_delta_ned"], [0, .5, 0])
        legacy_path, _, _, _ = load_route(dest)
        np.testing.assert_array_equal(legacy_path, self.path)
        with self.assertRaises(FileExistsError):
            export_bundle(self.cfg, anchor(), self.path, identity=IDENTITY, name="route")

    def test_reboot_or_px4_or_gazebo_restart_invalidates_anchor(self):
        dest = export_bundle(self.cfg, anchor(), self.path, identity=IDENTITY)
        for field in ("boot_id", "px4_pid", "px4_start_ticks", "gazebo_server_pid", "gazebo_start_ticks"):
            identity = IDENTITY.copy()
            identity[field] = "DIFFERENT"
            with self.assertRaisesRegex(ValueError, "restarted"):
                load_bundle(dest, identity)

    def test_c2_policy_and_csv_tamper_detection(self):
        dest = export_bundle(self.cfg, anchor(), self.path, identity=IDENTITY)
        p = dest/"manifest.json"
        metadata = json.loads(p.read_text())
        metadata["margin_m"] = .01
        p.write_text(json.dumps(metadata))
        with self.assertRaises(ValueError):
            load_bundle(dest)

    def test_host_cli_pipeline_creates_new_anchor_waypoint_report_without_ros(self):
        report = dict(problems=[], gates={"fixture": True}, runtime_identity=IDENTITY,
                      publishes_commands=False, live_control_go=False)
        with patch("pso_flight_demo.c2.load_profile", return_value=self.cfg), \
             patch("pso_flight_demo.c2.inspect_runtime", return_value=(report, anchor(), self.env)), \
             patch("pso_flight_demo.c2.inventory", return_value={"ros_initialized": False}):
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(c2.main(["host-check", "--prepare-waypoint"]), 0)
        result = json.loads(stream.getvalue())
        self.assertFalse(result["dry_run"]["publishes_commands"])
        self.assertTrue(Path(result["anchor_path"]).is_file())
        self.assertTrue(Path(result["report_path"]).is_file())
        self.assertTrue(result["dry_run"]["c1_bundle_loader_compatible"])

    def test_failed_host_gate_writes_report_but_no_waypoint(self):
        report = dict(problems=["disarmed_landed"], gates={"disarmed_landed": False},
                      runtime_identity=IDENTITY, publishes_commands=False, live_control_go=False)
        with patch("pso_flight_demo.c2.load_profile", return_value=self.cfg), \
             patch("pso_flight_demo.c2.inspect_runtime", return_value=(report, None, self.env)), \
             patch("pso_flight_demo.c2.inventory", return_value={"ros_initialized": False}):
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(c2.main(["host-check", "--prepare-waypoint"]), 1)
        result = json.loads(stream.getvalue())
        self.assertNotIn("waypoint_bundle", result)
        self.assertEqual(result["result"], "NO_GO")
        self.assertFalse(list(self.cfg.output_directory().rglob("waypoints_enu.csv")))

    def test_dry_run_entrypoint_never_observes_or_launches(self):
        dest = export_bundle(self.cfg, anchor(), self.path, identity=IDENTITY)
        with patch("pso_flight_demo.c2.inspect_runtime", side_effect=AssertionError("no runtime")):
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(c2.main(["dry-run", str(dest)]), 0)
        self.assertEqual(json.loads(stream.getvalue())["result"], "OFFLINE_VALIDATED_ONLY")


class C2RuntimeTests(unittest.TestCase):
    def test_flight_prestart_defers_only_preflight_and_disarmed_logger_gates(self):
        failed=['commander_preflight_ready','logger_running','ekf_position_valid']
        blocking,deferred=phase_problems(failed,'flight_prestart')
        self.assertEqual(blocking,['ekf_position_valid'])
        self.assertEqual(deferred,['commander_preflight_ready','logger_running'])
        self.assertEqual(phase_problems(failed,'strict')[0],failed)
        with self.assertRaises(ValueError):phase_problems(failed,'unknown')

    def test_parsers(self):
        self.assertEqual(topic_fields(" timestamp: 123 (0 us)\n armed: False\n")["armed"], "False")
        p = "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [-11.76 4.30 -.013]\n    [0 0 0]"
        np.testing.assert_allclose(pose_xyz(p), [-11.76, 4.30, -.013])

    def test_sandbox_missing_data_is_no_go_not_fake_anchor(self):
        def denied(argv):
            return dict(argv=[str(x) for x in argv], exit=255, stdout="", stderr="Operation not permitted")
        with patch("pso_flight_demo.runtime_readiness.probe", side_effect=denied), \
             patch("pso_flight_demo.runtime_readiness.time.sleep"):
            report, measured, _ = inspect_runtime(load_profile(), 396120)
        self.assertIsNone(measured)
        self.assertTrue(report["problems"])
        self.assertFalse(report["live_control_go"])
        self.assertFalse(report["publishes_commands"])

    def test_coherent_paused_host_fixture_checks_anchor_and_drop_delta(self):
        samples = {
            "actuator_armed": " timestamp: 396000000\n armed: False\n ready_to_arm: True\n",
            "vehicle_land_detected": " timestamp: 396000000\n landed: True\n",
            "vehicle_status": " timestamp: 396000000\n pre_flight_checks_pass: True\n failsafe: False\n gcs_connection_lost: False\n",
            "vehicle_local_position": " x: .03\n y: .02\n z: .01\n timestamp_sample: 396116000\n"
                " xy_valid: True\n z_valid: True\n v_xy_valid: True\n v_z_valid: True\n"
                " xy_reset_counter: 5\n z_reset_counter: 2\n vxy_reset_counter: 5\n"
                " vz_reset_counter: 2\n heading_reset_counter: 1\n ref_timestamp: 1044000\n",
            "vehicle_local_position_groundtruth": " x: 4.3\n y: -11.76\n z: .013\n timestamp_sample: 396120000\n",
            "sensor_accel": " timestamp_sample: 396120000\n",
            "sensor_gyro": " timestamp_sample: 396120000\n",
            "estimator_status": " timestamp: 396116000\n filter_fault_flags: 0\n",
            "telemetry_status": " timestamp: 396000000\n heartbeat_type_gcs: True\n rx_message_count: 20\n",
        }
        def fixture(argv):
            argv = [str(x) for x in argv]
            if argv[:2] == ["gz", "topic"]:
                text = "sim_time {\n sec: 396\n nsec: 120000000\n}\npaused: true\niterations: 396120\n"
            elif argv[:3] == ["gz", "model", "--list"]:
                text = "Requesting state for world [warehouse_world]...\nAvailable models:\n - x500\n"
            elif argv[:2] == ["gz", "model"]:
                text = " - Pose [ XYZ (m) ] [ RPY (rad) ]:\n [-11.76 4.30 -.013]\n [0 0 0]"
            elif argv[0].endswith("px4-listener"):
                text = samples.get(argv[3], "")
            elif argv[0].endswith("px4-commander"):
                text = "INFO [commander] Disarmed"
            elif argv[0].endswith("px4-ekf2"):
                text = "attitude: 1, local position: 1, global position: 1"
            elif argv[0].endswith("px4-logger"):
                text = "Full File Logging Running:"
            elif argv[0].endswith("px4-uxrce_dds_client"):
                text = "Running, connected"
            elif argv[0] == "ss":
                text = "UNCONN 0 0 0.0.0.0:18570 0.0.0.0:*\n skmem:(r0,rb212992,d2288)\n"
            else:
                text = ""
            return dict(argv=argv, exit=0, stdout=text, stderr="")
        with patch("pso_flight_demo.runtime_readiness.probe", side_effect=fixture), \
             patch("pso_flight_demo.runtime_readiness.process_identity", return_value=IDENTITY), \
             patch("pso_flight_demo.runtime_readiness.time.sleep"):
            report, measured, _ = inspect_runtime(load_profile(), 396120)
        self.assertEqual(report["problems"], [])
        np.testing.assert_allclose(measured.gazebo_enu, anchor().gazebo_enu)
        self.assertEqual(report["new_kernel_drops"], 0)
        self.assertTrue(report["dds_client_connected"])
        self.assertFalse(report["live_control_go"])  # fixture never claims flying-clock readiness


if __name__ == "__main__":
    unittest.main()
