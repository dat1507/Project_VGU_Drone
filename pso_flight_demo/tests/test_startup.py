"""Dry-run/reboot design tests. No real process startup/shutdown or physics."""

import contextlib
from dataclasses import replace
import io
import json
import shlex
import unittest
from unittest.mock import patch

from pso_flight_demo import c2
from pso_flight_demo.environments import load_profile
from pso_flight_demo.launch_plan import launch_plan, shutdown_plan
from pso_flight_demo.startup_checks import assess_startup, expected_ports, host_startup_check


class StartupChecksTests(unittest.TestCase):
    def test_free_host_is_candidate_not_flight_go(self):
        r = assess_startup(load_profile(), "", "")
        self.assertEqual(r["result"], "NEW_SESSION_CANDIDATE")
        self.assertFalse(r["startup_executed"])
        self.assertFalse(r["live_control_go"])

    def test_process_duplicates_and_existing_live_controller_block(self):
        for process in ("10 px4 /path/px4 -i 0",
                        "11 ruby gz sim server",
                        "12 gz-sim-server /usr/bin/gz-sim-server",
                        "13 MicroXRCEAgent MicroXRCEAgent udp4 -p 8888",
                        "14 python3 python3 -m pso_flight_demo follow route --execute"):
            with self.subTest(process=process):
                self.assertEqual(assess_startup(load_profile(), process, "")["result"], "NEW_SESSION_BLOCKED")

    def test_port_checks_use_local_port_not_peer(self):
        for port in (18570, 14580, 14280, 13030, 8888):
            text = f'UNCONN 0 0 0.0.0.0:{port} 0.0.0.0:* users:(("other",pid=8))'
            self.assertEqual(assess_startup(load_profile(), "", text)["result"], "NEW_SESSION_BLOCKED")
        peer_only = "ESTAB 0 0 127.0.0.1:40000 127.0.0.1:18570"
        self.assertEqual(assess_startup(load_profile(), "", peer_only)["result"], "NEW_SESSION_CANDIDATE")

    def test_qgc_can_remain_open_but_unknown_owner_of_14550_blocks(self):
        qgc = 'UNCONN 0 0 0.0.0.0:14550 0.0.0.0:* users:(("QGroundControl",pid=22))'
        result = assess_startup(load_profile(), "22 QGroundControl /app/QGroundControl", qgc)
        self.assertFalse(result["blockers"])
        self.assertTrue(result["warnings"])
        self.assertTrue(assess_startup(load_profile(), "", qgc.replace("QGroundControl", "other"))["blockers"])

    def test_native_only_blocks_existing_agent_since_rcs_still_starts_client(self):
        result = assess_startup(load_profile(), "2 MicroXRCEAgent /Agent", "", with_dds=False)
        self.assertTrue(result["blockers"])
        result = assess_startup(load_profile(), "", "UNCONN 0 0 0.0.0.0:8888 0.0.0.0:*", with_dds=False)
        self.assertTrue(result["blockers"])

    def test_namespace_and_probe_failures_are_not_free_host(self):
        self.assertTrue(assess_startup(load_profile(), "", "", visibility="isolated_sandbox")["blockers"])
        with patch("pso_flight_demo.startup_checks.probe",
                   return_value=dict(exit=255, stdout="", stderr="Operation not permitted")):
            self.assertEqual(host_startup_check(load_profile())["result"], "NEW_SESSION_BLOCKED")

    def test_ports_follow_instance_formulas(self):
        ports = expected_ports(replace(load_profile(), instance=2, agent_port=8890))
        self.assertEqual(ports["gcs"]["local"], 18572)
        self.assertEqual(ports["gcs"]["remote"], 14550)
        self.assertEqual(ports["mavlink_offboard"]["remote"], 14542)
        self.assertEqual(ports["dds"]["agent"], 8890)

    def test_ss_zero_exit_with_permission_error_is_still_blocked(self):
        with patch("pso_flight_demo.startup_checks.probe",
                   return_value=dict(exit=0, stdout="", stderr="Cannot open netlink socket: Operation not permitted")), \
             patch("pathlib.Path.read_bytes", return_value=b"/sbin/init\0"):
            self.assertEqual(host_startup_check(load_profile())["result"], "NEW_SESSION_BLOCKED")


class LaunchAndStopPlanTests(unittest.TestCase):
    def test_default_never_executes_subprocess_and_preserves_resources(self):
        with patch("subprocess.run", side_effect=AssertionError("no execution")), \
             patch.dict("os.environ", {"GZ_SIM_RESOURCE_PATH": "/extra models:/second"}):
            plan = launch_plan(load_profile())
            stop = shutdown_plan(load_profile())
        self.assertEqual(plan["launched_processes"], 0)
        self.assertEqual(stop["stopped_processes"], 0)
        order = plan["startup_order"]
        self.assertLess(order.index("AGENT"), order.index("PX4"))
        self.assertEqual(order[-1], "READINESS")
        gz = next(c for c in plan["commands"] if c["terminal"] == "GAZEBO")
        args = shlex.split(gz["command"])
        self.assertNotIn("-r", args)
        self.assertIn("/extra models:/second", args[1])
        readiness = plan["commands"][-1]["command"]
        self.assertIn("source /opt/ros/jazzy/setup.bash", readiness)
        self.assertIn("--observe-dds 3", readiness)
        self.assertFalse(stop["automatic_physics"])

    def test_custom_profile_selector_is_shell_quoted(self):
        custom = "/tmp/profile with spaces.yaml"
        plan = launch_plan(load_profile(), profile_source=custom)
        for key in ("PRECHECK", "READINESS"):
            text = next(c["command"] for c in plan["commands"] if c["terminal"] == key)
            self.assertIn(shlex.quote(custom), text)

    def test_native_only_plan_does_not_require_or_start_agent(self):
        plan = launch_plan(load_profile(), with_dds=False)
        self.assertNotIn("AGENT", plan["startup_order"])
        self.assertNotIn("--observe-dds", plan["commands"][-1]["command"])

    def test_host_precheck_is_opt_in_and_cli_propagates_blocked_status(self):
        result = dict(result="NEW_SESSION_BLOCKED", blockers=["existing simulation"])
        with patch("pso_flight_demo.launch_plan.host_startup_check", return_value=result) as check:
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(c2.main(["launch", "--check-host"]), 1)
            check.assert_called_once()
        self.assertEqual(json.loads(out.getvalue())["launched_processes"], 0)

    def test_shutdown_command_is_only_text_and_no_execution_option_exists(self):
        plan = shutdown_plan(load_profile())
        text = json.dumps(plan)
        self.assertIn("px4-shutdown --instance 0", text)
        self.assertIn("Paused", plan["paused_shutdown_policy"])
        self.assertTrue(plan["session_identity_required_for_future_execution"])
        for command in ("launch", "shutdown-plan"):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                c2.parser().parse_args([command, "--execute"])
        with patch("subprocess.run", side_effect=AssertionError("no shutdown execution")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(c2.main(["shutdown-plan"]), 0)
        self.assertEqual(json.loads(out.getvalue())["stopped_processes"], 0)
