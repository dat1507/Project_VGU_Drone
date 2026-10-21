"""Executor tests: fake children/probes only, never real launch, signal or physics."""

import contextlib
from dataclasses import replace
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pso_flight_demo import session_launcher as sl
from pso_flight_demo import session_runtime as sr
from pso_flight_demo.environments import MODULE, load_profile
from pso_flight_demo.launch_plan import launch_plan
from pso_flight_demo.startup_checks import assess_startup


def ident(pid=800, **changes):
    return dict(dict(pid=pid, ppid=700, uid=os.getuid(), pgid=pid, sid=pid, start_ticks=100,
                     exe=f"/fake/{pid}", boot_id="offline-boot"), **changes)


class Process:
    def __init__(self, pid):
        self.pid, self.code = pid, None

    def poll(self):
        return self.code


class FakeHost:
    def __init__(self):
        self.events, self.children, self.signals = [], [], []
        self.fail_role, self.exit_role, self.conflict_at = None, None, None
        self.gate = dict(safe=True, paused=False, clock_progressing=True)
        self.bad_precheck = False
        self.shutdown_exit = 0
        self.new_identity = None
        self.paused = True

    def precheck(self, profile, with_dds):
        self.events.append("precheck")
        return dict(blockers=["existing PX4"] if self.bad_precheck else [])

    def stage_conflicts(self, profile, children, with_dds):
        if len(children) == self.conflict_at:
            raise ValueError("UDP port claimed by foreign process")

    def spawn(self, role, argv, cwd, env, directory):
        self.events.append(role)
        if role == self.fail_role:
            raise OSError("fake spawn failure")
        pid = 800+len(self.children)
        owner = ident(pid)
        if role == "PX4":
            owner["exe"] = str(Path(argv[0]).resolve())
        child = sl.Child(role, Process(pid), owner, {pid: owner})
        if role == self.exit_role:
            child.process.code = 1
        self.children.append(child)
        return child

    def members(self, child):
        return [child.owner] if child.process.poll() is None else []

    def ident(self, pid):
        if self.new_identity:
            return self.new_identity
        return next((c.owner for c in self.children if c.owner["pid"] == pid and c.process.poll() is None), None)

    def signal_owned(self, info):
        self.signals.append(info["pid"])
        child = next(c for c in self.children if c.owner["pid"] == info["pid"])
        self.events.append("stop_"+child.role)
        child.process.code = 0

    def world(self, profile):
        return dict(correct=True, paused=self.paused)

    def attached(self, profile, child, with_dds):
        self.events.append("attach")
        return True

    def bound_ports(self, profile, child):
        self.events.append("ports_"+child.role)
        return True

    def ready(self, profile, with_dds, directory):
        return dict(gates=dict(arm_on_boot_disabled=True), live_control_go=False)

    def stop_gate(self, profile):
        return self.gate

    def shutdown_px4(self, profile):
        self.events.append("shutdown_PX4")
        if self.shutdown_exit == 0:
            next(c for c in self.children if c.role == "PX4").process.code = 0
        return dict(exit=self.shutdown_exit)


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        self.addCleanup(self.temp.cleanup)
        self.profile = load_profile()
        self.host = FakeHost()
        self.plan = launch_plan(self.profile)
        self.supervisor = sl.Supervisor(self.profile, self.plan, Path(self.temp.name), host=self.host, timeout=1)

    def test_success_order_attach_one_model_no_gui_no_physics(self):
        self.assertTrue(self.supervisor.start())
        events = self.host.events
        self.assertLess(events.index("GAZEBO"), events.index("AGENT"))
        self.assertLess(events.index("ports_AGENT"), events.index("PX4"))
        self.assertIn("ports_PX4", events)
        self.assertEqual([c.role for c in self.host.children], ["GAZEBO", "AGENT", "PX4"])
        self.assertEqual(self.supervisor.state["state"], "BOOTSTRAPPED_WAIT_CLOCK")
        self.assertTrue(self.supervisor.state["new_anchor_required"])
        self.assertFalse(self.supervisor.state["live_control_go"])
        commands = sl.child_commands(self.profile, self.plan)
        px4 = commands[-1]
        self.assertIn("-d", px4[1])
        self.assertEqual(px4[3]["PX4_GZ_MODEL_NAME"], "x500")
        self.assertEqual(px4[3]["PX4_GZ_STANDALONE"], "1")
        self.assertNotIn("-r", commands[0][1])
        self.assertNotIn("QGroundControl", json.dumps([c[1] for c in commands]))

    def test_conflict_precheck_starts_nothing(self):
        self.host.bad_precheck = True
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.children, [])

    def test_failure_before_px4_rolls_back_only_owned_gazebo(self):
        self.host.fail_role = "AGENT"
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.signals, [800])
        self.assertEqual(self.supervisor.state["state"], "ROLLED_BACK")

    def test_failure_after_px4_preserves_simulator_and_agent(self):
        self.host.attached = lambda *args: (_ for _ in ()).throw(ValueError("bad attach"))
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.signals, [])
        self.assertEqual(self.supervisor.state["state"], "CLEANUP_REQUIRED")

    def test_child_exits_before_ready_stops_startup(self):
        self.host.exit_role = "GAZEBO"
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.events, ["precheck", "GAZEBO"])

    def test_timeout_no_retry_and_no_extra_stage(self):
        self.host.world = lambda profile: dict(correct=False, paused=True)
        with patch.object(sl.time, "monotonic", side_effect=[0, 2, 3]):
            self.assertFalse(self.supervisor.start())
        self.assertNotIn("PX4", self.host.events)
        self.assertIn("Timeout", self.supervisor.state["error"])

    def test_port_race_blocks_next_spawn_and_preserves_when_foreign_present(self):
        self.host.conflict_at = 1
        self.assertFalse(self.supervisor.start())
        self.assertNotIn("AGENT", self.host.events)
        self.assertFalse(self.host.signals)
        self.assertEqual(self.supervisor.state["state"], "CLEANUP_REQUIRED")

    def test_normal_shutdown_px4_then_agent_then_gazebo(self):
        self.assertTrue(self.supervisor.start())
        self.assertTrue(self.supervisor.shutdown())
        self.assertEqual(self.host.events[-3:], ["shutdown_PX4", "stop_AGENT", "stop_GAZEBO"])
        self.assertNotIn(802, self.host.signals)  # PX4 must use native logger hooks, never a signal.

    def test_paused_or_armed_shutdown_mutates_nothing(self):
        self.assertTrue(self.supervisor.start())
        for gate in (dict(safe=True, paused=True), dict(safe=False, paused=False)):
            self.host.gate = gate
            self.assertFalse(self.supervisor.shutdown())
            self.assertNotIn("shutdown_PX4", self.host.events)
            self.assertFalse(self.host.signals)
            self.assertEqual(self.supervisor.state["state"], "SHUTDOWN_BLOCKED")

    def test_native_shutdown_failure_preserves_children_and_does_not_retry(self):
        self.assertTrue(self.supervisor.start())
        self.host.shutdown_exit = None
        self.assertFalse(self.supervisor.shutdown())
        self.assertFalse(self.supervisor.shutdown())
        self.assertEqual(self.host.events.count("shutdown_PX4"), 1)
        self.assertFalse(self.host.signals)

    def test_native_shutdown_wait_timeout_never_signals_px4(self):
        self.assertTrue(self.supervisor.start())
        self.host.shutdown_px4 = lambda profile: dict(exit=0)
        with patch.object(sl.time, "monotonic", side_effect=[0, 2]):
            self.assertFalse(self.supervisor.shutdown())
        self.assertFalse(self.host.signals)

    def test_pid_reuse_blocks_cleanup(self):
        self.assertTrue(self.supervisor.start())
        self.host.new_identity = ident(800, start_ticks=200)
        self.assertFalse(self.supervisor.shutdown())
        self.assertFalse(self.host.signals)

    def test_unproven_descendant_is_never_signalled(self):
        self.assertTrue(self.supervisor.start())
        root = self.host.children[0]
        self.host.members = lambda child: [dict(root.owner, pid=999, ppid=1234)]
        with self.assertRaisesRegex(ValueError, "Unproven"):
            self.supervisor.stop_auxiliary(root)
        self.assertFalse(self.host.signals)

    def test_auxiliary_stop_cannot_signal_px4(self):
        self.assertTrue(self.supervisor.start())
        with self.assertRaisesRegex(ValueError, "auxiliary"):
            self.supervisor.stop_auxiliary(self.host.children[-1])
        self.assertFalse(self.host.signals)

    def test_pre_px4_rollback_reverse_order(self):
        self.host.fail_role = "PX4"
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.events[-2:], ["stop_AGENT", "stop_GAZEBO"])

    def test_native_only_omits_agent(self):
        self.supervisor.plan = launch_plan(self.profile, with_dds=False)
        self.supervisor.with_dds = False
        self.assertTrue(self.supervisor.start())
        self.assertNotIn("AGENT", self.host.events)

    def test_unknown_auto_arm_configuration_fails_closed(self):
        self.host.ready = lambda *args: dict(gates=dict(arm_on_boot_disabled=False))
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.supervisor.state["state"], "CLEANUP_REQUIRED")
        self.assertFalse(self.host.signals)

    def test_shutdown_request_is_handled_by_owner_in_order(self):
        self.assertTrue(self.supervisor.start())
        (self.supervisor.directory/"shutdown.request").write_text(json.dumps(
            dict(session_id=self.supervisor.directory.name)))
        with patch.object(sl.signal, "signal", return_value=None):
            self.supervisor.serve()
        self.assertEqual(self.supervisor.state["state"], "STOPPED")
        self.assertFalse((self.supervisor.directory/"shutdown.request").exists())

    def test_external_clock_change_during_readiness_is_not_reported_success(self):
        def ready(*args):
            self.host.paused = False
            return dict(gates=dict(arm_on_boot_disabled=True))
        self.host.ready = ready
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.supervisor.state["state"], "CLEANUP_REQUIRED")
        self.assertFalse(self.host.signals)

    def test_invalid_shutdown_request_preserves_processes_and_supervision(self):
        self.assertTrue(self.supervisor.start())
        (self.supervisor.directory/"shutdown.request").write_text('{"session_id":"other"}')
        with patch.object(sl.signal, "signal", return_value=None), \
             patch.object(sl.time, "sleep", side_effect=RuntimeError("stop offline loop")):
            with self.assertRaisesRegex(RuntimeError, "offline loop"):
                self.supervisor.serve()
        self.assertEqual(self.supervisor.state["state"], "REQUEST_REJECTED")
        self.assertFalse(self.host.signals)
        self.assertNotIn("shutdown_PX4", self.host.events)

    def test_unproven_spawn_identity_preserves_lock_cleanup_state(self):
        spawn = self.host.spawn
        def unproven(*args):
            child = spawn(*args)
            child.owner["identity_unproven"] = True
            return child
        self.host.spawn = unproven
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.supervisor.state["state"], "CLEANUP_REQUIRED")
        self.assertFalse(self.host.signals)

    def test_interrupted_startup_does_not_launch_next_stage(self):
        self.supervisor.stop_requested = True
        self.assertFalse(self.supervisor.start())
        self.assertEqual(self.host.children, [])

    def test_nonwarehouse_profile_propagates_world_model_and_namespace(self):
        profile = replace(self.profile, world_name="another_world", vehicle_name="demo_x500",
                          namespace="demo", instance=2, agent_port=8890)
        commands = sl.child_commands(profile, self.plan)
        env = commands[-1][3]
        self.assertEqual(env["PX4_GZ_WORLD"], "another_world")
        self.assertEqual(env["PX4_GZ_MODEL_NAME"], "demo_x500")
        self.assertEqual(env["PX4_UXRCE_DDS_PORT"], "8890")


class LockAndCLITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_singleton_and_lock_release_without_unlink(self):
        path = self.root/"lifecycle.lock"
        with sl.ExclusiveLock(path):
            with self.assertRaises(ValueError):
                with sl.ExclusiveLock(path):
                    self.fail("double owner")
        self.assertTrue(path.exists())
        with sl.ExclusiveLock(path):
            pass

    def test_symlink_lock_and_directory_rejected(self):
        (self.root/"target").touch()
        (self.root/"lock").symlink_to(self.root/"target")
        with self.assertRaises(OSError):
            with sl.ExclusiveLock(self.root/"lock"):
                self.fail("symlink lock")
        (self.root/"redirect").symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            sl.secure_directory(self.root/"redirect"/"session")

    def test_dry_run_does_not_spawn_probe_or_create_session(self):
        with patch.object(sl, "SESSION_ROOT", self.root/"sessions"), \
             patch("subprocess.Popen", side_effect=AssertionError("no spawn")), \
             patch("subprocess.run", side_effect=AssertionError("no probes")), \
             contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(sl.main(["start"]), 0)
        self.assertEqual(json.loads(out.getvalue())["result"], "DRY_RUN")
        self.assertFalse((self.root/"sessions").exists())

    def test_no_execute_no_shutdown_request(self):
        state = dict(session_id=self.root.name)
        with patch.object(sl, "read_session", return_value=(self.root, state)), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sl.main(["shutdown", str(self.root)]), 0)
        self.assertFalse((self.root/"shutdown.request").exists())

    def test_shutdown_queues_once_without_signalling_client(self):
        with patch.object(sl, "read_session", return_value=(self.root, {})), \
             patch.object(sl, "shutdown_review", return_value=dict(shutdown_request_allowed=True)), \
             patch("subprocess.run", side_effect=AssertionError("no client execution in requester")), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sl.main(["shutdown", str(self.root), "--execute"]), 0)
            self.assertEqual(sl.main(["shutdown", str(self.root), "--execute"]), 2)

    def test_stale_supervisor_refused(self):
        state = dict(schema=1, session_id=self.root.name, supervisor=ident())
        (self.root/"session.json").write_text(json.dumps(state))
        with patch.object(sl, "SESSION_ROOT", self.root.parent), patch.object(sl, "identity", return_value=None):
            with self.assertRaisesRegex(ValueError, "Supervisor absent"):
                sl.read_session(self.root)
            self.assertEqual(sl.read_session(self.root, require_owner=False)[1], state)

    def test_pidfd_identity_rechecked_after_open_before_signal(self):
        with patch.object(sl.os, "pidfd_open", return_value=90), patch.object(sl.os, "close"), \
             patch.object(sl, "identity", return_value=ident(start_ticks=101)), \
             patch.object(sl.signal, "pidfd_send_signal") as send:
            with self.assertRaisesRegex(ValueError, "identity changed"):
                sl.Host().signal_owned(ident())
            send.assert_not_called()

    def test_identity_match_requires_boot_start_uid_executable(self):
        for key, value in (("boot_id", "old"), ("start_ticks", 999), ("uid", 9999), ("exe", "/other")):
            self.assertFalse(sl.same_process(ident(), ident(**{key: value})))
        self.assertFalse(sl.same_process(ident(), None))

    def test_foreign_udp_port_blocks_and_owned_ports_must_really_bind(self):
        child = sl.Child("PX4", Process(800), ident(), {800: ident()})
        host = sl.Host()
        with patch.object(host, "members", return_value=[ident()]):
            def output(ports, owner=800):
                return dict(exit=0, stderr="", stdout="\n".join(
                    f'UNCONN 0 0 0.0.0.0:{p} 0.0.0.0:* users:(("px4",pid={owner}))' for p in ports))
            for ports, owner, expected in (([18570, 14580, 14280, 13030], 800, True),
                                           ([18570], 800, False),
                                           ([18570, 14580, 14280, 13030], 900, False)):
                with patch("pso_flight_demo.runtime_readiness.probe", return_value=output(ports, owner)):
                    self.assertEqual(host.bound_ports(load_profile(), child), expected)

    def test_stage_filter_only_excludes_wholly_owned_ports(self):
        host = sl.Host()
        child = sl.Child("GAZEBO", Process(800), ident(), {800: ident()})
        port = 'UNCONN 0 0 0.0.0.0:18570 0.0.0.0:* users:(("other",pid=999),("owned",pid=800))'
        results = [dict(exit=0, stderr="", stdout="800 ruby gz sim server\n"),
                   dict(exit=0, stderr="", stdout=port)]
        with patch.object(host, "members", return_value=[ident()]), \
             patch("pso_flight_demo.runtime_readiness.probe", side_effect=results):
            with self.assertRaisesRegex(ValueError, "Stage conflict"):
                host.stage_conflicts(load_profile(), [child], True)

    def test_live_follower_blocks_new_session(self):
        self.assertTrue(assess_startup(load_profile(),
            "999 python3 python3 -m pso_flight_demo follow route --confirm-live CONTROL_APPROVED", "")["blockers"])

    def test_old_boot_anchor_refused_by_existing_bundle_loader(self):
        from pso_flight_demo.frames import FrameAnchor
        profile = load_profile()
        state = dict(profile_sha256=sl.digest(profile.snapshot()), sdf_sha256="sdf", children=[
            dict(role="PX4", owner=ident()), dict(role="GAZEBO", members=[ident(900)])])
        anchor = FrameAnchor((0, 0, 0), (0, 0, 0), 100, 100, (0, 0, 0, 0, 0, 0),
                             "px4_groundtruth_and_local_position")
        bundle = (None, dict(sdf_sha256="sdf"), anchor, profile)
        with patch("pso_flight_demo.environment_artifacts.load_bundle", return_value=bundle):
            with self.assertRaisesRegex(ValueError, "owned runtime"):
                sl.check_anchor("unused", state, dict(px4_pid=800, px4_start_ticks=100, boot_id="old"), [0]*6)
            with self.assertRaisesRegex(ValueError, "EKF reset"):
                sl.check_anchor("unused", state, dict(px4_pid=800, px4_start_ticks=100,
                                                     boot_id="offline-boot", gazebo_server_pid=900,
                                                     gazebo_start_ticks=100), [1]*6)

    def test_shutdown_freshness_uses_sim_time_not_iteration_assumption(self):
        world = dict(correct=True, paused=False, iterations=500,
                     evidence=dict(stats=dict(stdout="sim_time {\n sec: 10\n}\n")))
        def native(profile, module, *args):
            if module == "logger":
                return dict(exit=0, stdout="Log file: current.ulg")
            field = "armed: False" if args[0] == "actuator_armed" else "landed: True"
            return dict(exit=0, stdout="    timestamp: 9900000\n    "+field)
        with patch.object(sr, "native_probe", side_effect=native), patch.object(sr, "world_snapshot", return_value=world):
            self.assertTrue(sr.shutdown_gate(load_profile())["safe"])

    def test_cold_world_handles_omitted_zero_protobuf_scalar(self):
        replies = [dict(exit=0, stdout="sim_time {\n}\npaused: true\n"),
                   dict(exit=0, stdout="Requesting state for world [warehouse_world]\n - x500\n")]
        with patch.object(sr, "probe", side_effect=replies):
            result = sr.world_snapshot(load_profile())
        self.assertTrue(result["correct"])
        self.assertEqual(result["iterations"], 0)

    def test_readiness_preserves_health_timestamp_and_decodes_actual_mode_bits(self):
        profile = load_profile()
        topics = dict(
            actuator_armed="armed: False\n ready_to_arm: True",
            vehicle_status="pre_flight_checks_pass: True\n failsafe: False\n gcs_connection_lost: False",
            vehicle_land_detected="landed: True", sensor_accel="timestamp_sample: 10000000",
            sensor_gyro="timestamp_sample: 10000000",
            vehicle_local_position="xy_valid: True\n z_valid: True\n v_xy_valid: True\n v_z_valid: True",
            vehicle_global_position="lat_lon_valid: True\n alt_valid: True", estimator_status="filter_fault_flags: 0",
            telemetry_status="heartbeat_type_gcs: True",
            health_report="can_arm_mode_flags: 393232\n can_run_mode_flags: 393232")
        def native(profile, module, *args):
            if module == "param":
                text = "x COM_ARM_ON_BOOT [1,2] : 0\n"
            elif module == "listener":
                stamp = 5_000_000 if args[0] == "health_report" else 10_000_000
                text = f" timestamp: {stamp}\n "+topics.get(args[0], "")
            else:
                text = dict(ekf2="attitude: 1, local position: 1, global position: 1",
                            logger="Full File Logging Running:").get(module, "")
            return dict(exit=0, stdout=text, stderr="")
        world = dict(correct=True, paused=True, evidence=dict(stats=dict(stdout="sim_time { sec: 10 }")))
        with patch.object(sr, "native_probe", side_effect=native), \
             patch.object(sr, "world_snapshot", return_value=world), \
             patch.object(sr, "probe", return_value=dict(exit=0, stdout="", stderr="")):
            report = sr.readiness(profile, False, self.root)
        self.assertEqual(report["modes"]["takeoff"]["source_bit"], 17)
        self.assertTrue(report["modes"]["takeoff"]["can_arm_mode_flags"])
        self.assertEqual(report["health_timestamp"], "5000000")
        self.assertTrue(report["gates"]["arm_on_boot_disabled"])
        self.assertTrue(report["gates"]["vehicle_local_position_fresh_paused"])
        self.assertFalse(report["live_control_go"])


if __name__ == "__main__":
    unittest.main()
