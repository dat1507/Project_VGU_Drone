"""One-shot host-side coordinator for an already-owned SITL session shutdown.

This module never edits session files or signals PX4/processes directly. It can
only call session_launcher's public CLI and one bounded Gazebo WorldControl
request after two separate, exact operator confirmations.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass

from .environments import MODULE, load_profile


CLOCK_APPROVAL = "APPROVE FINITE CLOCK 5000"
SHUTDOWN_APPROVAL = "APPROVE SESSION SHUTDOWN"
MAX_STEPS = 5000
POLL_SECONDS = 0.05
GATE_HEADROOM_FACTOR = 3.0
GATE_FIXED_RESERVE_SECONDS = 1.0


@dataclass(frozen=True)
class Stats:
    iterations: int
    sim_us: int
    paused: bool


def parse_stats(text: str) -> Stats:
    it = re.search(r"^iterations:\s*(\d+)\s*$", text, re.M)
    tm = re.search(r"sim_time\s*\{([^}]*)\}", text, re.S)
    if not it or not tm:
        raise ValueError("Gazebo stats lack iterations or sim_time")
    sec = re.search(r"\bsec:\s*(\d+)", tm[1])
    nsec = re.search(r"\bnsec:\s*(\d+)", tm[1])
    sim_us = (int(sec[1]) if sec else 0) * 1_000_000 + (int(nsec[1]) // 1000 if nsec else 0)
    return Stats(int(it[1]), sim_us, "paused: true" in text)


def validate_review(review: dict, expected_iterations: int, *, max_steps: int = MAX_STEPS) -> dict:
    """Fail closed unless the fresh official review proposes exactly this window."""
    if review.get("result") != "READY_FOR_SEPARATE_CLOCK_APPROVAL":
        raise ValueError("Fresh review is not READY_FOR_SEPARATE_CLOCK_APPROVAL")
    if review.get("blockers"):
        raise ValueError("Fresh review has blockers")
    if review.get("shutdown_request_allowed") is not False:
        raise ValueError("Paused preflight must not already allow shutdown")
    gate = review.get("shutdown_gate") or {}
    if gate.get("safe") is not True or gate.get("paused") is not True:
        raise ValueError("Fresh gate is not safe and Paused")
    world = gate.get("world") or {}
    if world.get("correct") is not True or world.get("iterations") != expected_iterations:
        raise ValueError("World or baseline iterations differ from operator-reviewed values")
    if review.get("auto_arm_disabled") is not True:
        raise ValueError("COM_ARM_ON_BOOT=0 was not confirmed")
    inv = review.get("inventory") or {}
    if inv.get("identity_mismatches") or inv.get("unrecorded_child_tasks"):
        raise ValueError("Process identity/ownership inventory is not clean")
    proposal = review.get("clock_proposal") or {}
    if (max_steps != MAX_STEPS
            or proposal.get("baseline_iterations") != expected_iterations
            or proposal.get("additional_steps") != max_steps
            or proposal.get("maximum_end_iterations") != expected_iterations + max_steps
            or proposal.get("requires_separate_operator_clock_approval") is not True):
        raise ValueError("Clock proposal differs from separately approved finite budget")
    world_name = load_profile("warehouse").world_name
    expected_clock = ["gz", "service", "-s", f"/world/{world_name}/control",
                      "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                      "--timeout", "1000", "--req", "pause: true, multi_step: 5000"]
    expected_rollback = ["gz", "service", "-s", f"/world/{world_name}/control",
                         "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                         "--timeout", "1000", "--req", "pause: true"]
    if shlex.split(proposal.get("command", "")) != expected_clock:
        raise ValueError("Clock proposal is not the bounded pause:true 5000-step request")
    if shlex.split(proposal.get("rollback_pause_command", "")) != expected_rollback:
        raise ValueError("Rollback proposal is not the exact pause-only request")
    if proposal.get("baseline_clock_us") != gate.get("clock_after_us"):
        raise ValueError("Clock proposal timestamp differs from reviewed native stats")
    return proposal


def remaining_window_seconds(current: int, max_end: int, progress_delta: int,
                             elapsed: float) -> float:
    """Conservative live estimate from observed iteration throughput.

    Returns -1 until a measurable rate exists. No default rate is assumed.
    """
    if progress_delta <= 0 or elapsed <= 0:
        return -1.0
    rate = progress_delta / elapsed
    remaining = max(0, max_end - current)
    return remaining / rate


def enough_headroom(remaining_seconds: float, gate_seconds: float) -> bool:
    if remaining_seconds < 0 or gate_seconds <= 0:
        return False
    return remaining_seconds >= GATE_HEADROOM_FACTOR * gate_seconds + GATE_FIXED_RESERVE_SECONDS


class HostAdapter:
    """Thin adapter; all runtime behavior is through existing public CLIs."""
    def __init__(self, session: str):
        self.session = session

    def review(self):
        return self._json([sys.executable, "-B", "-m", "pso_flight_demo.session_launcher",
                           "shutdown-review", self.session], timeout=20)

    def stats(self):
        profile = load_profile("warehouse")
        from .runtime_readiness import probe
        result = probe(["gz", "topic", "-e", "-t", f"/world/{profile.world_name}/stats", "-n", "1"], 1)
        if result.get("exit") != 0 or result.get("stderr", "").strip():
            raise ValueError("Gazebo stats probe failed")
        return parse_stats(result["stdout"])

    def start_clock(self, command: str):
        return subprocess.Popen(shlex.split(command), cwd=MODULE.parent)

    def start_shutdown(self):
        return subprocess.Popen([sys.executable, "-B", "-m", "pso_flight_demo.session_launcher",
                                 "shutdown", self.session, "--execute"], cwd=MODULE.parent,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def rollback_pause(self, command: str):
        return subprocess.run(shlex.split(command), cwd=MODULE.parent,
                              capture_output=True, text=True, timeout=3)

    def verify_stopped(self):
        return self._json([sys.executable, "-B", "-m", "pso_flight_demo.session_launcher",
                           "verify-stopped", self.session, "--timeout", "120"], timeout=125)

    @staticmethod
    def _json(argv, timeout):
        result = subprocess.run(argv, cwd=MODULE.parent, capture_output=True,
                                text=True, timeout=timeout)
        try:
            value = json.loads(result.stdout)
        except (json.JSONDecodeError, TypeError):
            raise ValueError(f"Official CLI returned no JSON (exit={result.returncode})") from None
        return value


class ShutdownCoordinator:
    def __init__(self, adapter, *, confirm=input, monotonic=time.monotonic, sleep=time.sleep):
        self.adapter, self.confirm = adapter, confirm
        self.monotonic, self.sleep = monotonic, sleep

    def run(self, expected_iterations: int, *, max_steps: int = MAX_STEPS) -> dict:
        # Initial review is read-only and gives operator the actual proposal.
        t0 = self.monotonic()
        initial = self.adapter.review()
        proposal = validate_review(initial, expected_iterations, max_steps=max_steps)
        initial_gate_s = max(0.01, self.monotonic() - t0)

        if self.confirm(f"Type exactly '{CLOCK_APPROVAL}' to allow only the finite clock step: ") != CLOCK_APPROVAL:
            return dict(result="BLOCKED", stage="CLOCK_APPROVAL", runtime_changed=False)
        if self.confirm(f"Type exactly '{SHUTDOWN_APPROVAL}' to allow one launcher shutdown request: ") != SHUTDOWN_APPROVAL:
            return dict(result="BLOCKED", stage="SHUTDOWN_APPROVAL", runtime_changed=False)

        # Re-review after approvals; compare the safety-relevant snapshot and proposal.
        t0 = self.monotonic()
        fresh = self.adapter.review()
        try:
            fresh_proposal = validate_review(fresh, expected_iterations, max_steps=max_steps)
        except ValueError:
            return dict(result="BLOCKED", stage="FRESH_REVIEW_CHANGED", runtime_changed=False)
        fresh_gate_s = max(0.01, self.monotonic() - t0)
        if _review_fingerprint(initial, proposal) != _review_fingerprint(fresh, fresh_proposal):
            return dict(result="BLOCKED", stage="FRESH_REVIEW_CHANGED", runtime_changed=False)
        gate_s = max(initial_gate_s, fresh_gate_s)

        clock = self.adapter.start_clock(proposal["command"])
        started = self.monotonic()
        previous = None
        progress_samples = []
        shutdown = None
        try:
            while self.monotonic() - started <= 10:
                now = self.monotonic()
                clock_exit = clock.poll()
                if clock_exit not in (None, 0):
                    return self._rollback(proposal, "CLOCK_SERVICE_FAILED_NO_RETRY",
                                          clock_service_exit=clock_exit)
                stats = self.adapter.stats()
                if stats.iterations < expected_iterations:
                    return self._rollback(proposal, "ITERATION_BASELINE_REGRESSED")
                if stats.iterations > expected_iterations + max_steps:
                    return self._rollback(proposal, "ITERATION_CAP_EXCEEDED")
                if stats.paused:
                    if stats.iterations == expected_iterations:
                        # The service may still be connecting; wait only within the
                        # same bounded window/deadline. Once its client exits, do not
                        # issue a second clock request.
                        if clock.poll() is None:
                            self.sleep(POLL_SECONDS)
                            continue
                        return dict(result="BLOCKED", stage="NO_CLOCK_PROGRESS", runtime_changed=False,
                                    clock_service_exit=clock.poll())
                    break
                if previous and (stats.iterations < previous.iterations or stats.sim_us < previous.sim_us):
                    return self._rollback(proposal, "CLOCK_PROGRESS_REGRESSED")
                if (stats.iterations > expected_iterations and stats.sim_us >
                        (previous.sim_us if previous else 0)):
                    progress_samples.append((now, stats.iterations, stats.sim_us))
                previous = stats
                # Need a rate estimate, then enough predicted time for both official
                # shutdown_gate passes plus fixed reserve. No assumed simulation rate.
                if len(progress_samples) >= 2:
                    t_first, i_first, us_first = progress_samples[-2]
                    t_last, i_last, us_last = progress_samples[-1]
                    rate_elapsed = t_last - t_first
                    progress_delta = i_last - i_first
                    if us_last <= us_first:
                        return self._rollback(proposal, "SIM_TIME_NOT_PROGRESSING")
                    remaining_s = remaining_window_seconds(stats.iterations,
                        expected_iterations + max_steps, progress_delta, max(rate_elapsed, 1e-6))
                    # Conservatively require room for both full official gate passes.
                    if enough_headroom(remaining_s, gate_s):
                        shutdown = self.adapter.start_shutdown()
                        break
                    if stats.iterations >= expected_iterations + max_steps:
                        return self._rollback(proposal, "INSUFFICIENT_GATE_HEADROOM")
                self.sleep(POLL_SECONDS)
            if shutdown is None:
                return self._rollback(proposal, "CLOCK_WINDOW_ENDED_BEFORE_SHUTDOWN_QUEUE")

            # Exactly one shutdown CLI invocation. Its own prequeue review and the
            # existing supervisor's independent gate remain authoritative.
            deadline = self.monotonic() + max(5.0, gate_s * GATE_HEADROOM_FACTOR + 2.0)
            while shutdown.poll() is None and self.monotonic() < deadline:
                self.sleep(POLL_SECONDS)
            if shutdown.poll() is None:
                return dict(result="OPERATOR_INTERVENTION_REQUIRED",
                            stage="SHUTDOWN_CLI_TIMEOUT_NO_RETRY",
                            note="Queue/native shutdown state is ambiguous; no pause rollback or retry was issued")
            stdout, stderr = shutdown.communicate()
            if shutdown.returncode != 0 or "REQUEST_QUEUED_NOT_PROCESS_EXIT" not in stdout:
                return self._rollback(proposal, "SHUTDOWN_NOT_QUEUED_NO_RETRY", stdout=stdout, stderr=stderr)

            verified = self.adapter.verify_stopped()
            passed = verified.get("restart_allowed") is True and verified.get("result") == "ALL_CHILDREN_EXITED_START_CANDIDATE"
            return dict(result="COMPLETE" if passed else "VERIFY_STOPPED_FAIL",
                        shutdown_request="QUEUED", verify_stopped=verified,
                        clock_service_exit=clock.poll())
        except Exception as exc:
            if shutdown is not None:
                return dict(result="OPERATOR_INTERVENTION_REQUIRED",
                            stage="ERROR_AFTER_SHUTDOWN_CLI_STARTED_NO_RETRY", error=str(exc),
                            note="Shutdown may be queued; do not alter clock or retry until host review")
            # Never retry. Pause rollback is only a fail-safe for this already
            # approved window; it does not grant another clock window.
            return self._rollback(proposal, "COORDINATOR_ERROR_NO_RETRY", error=str(exc))

    def _rollback(self, proposal, reason, **extra):
        try:
            result = self.adapter.rollback_pause(proposal["rollback_pause_command"])
            if result.returncode != 0:
                return dict(result="OPERATOR_INTERVENTION_REQUIRED", stage=reason,
                            clock_step_requested=True, rollback_exit=result.returncode,
                            rollback_stdout=result.stdout, rollback_stderr=result.stderr, **extra)
            paused = self.adapter.stats().paused
            outcome = "BLOCKED" if paused else "OPERATOR_INTERVENTION_REQUIRED"
            return dict(result=outcome, stage=reason, clock_step_requested=True,
                        rollback_exit=result.returncode, rollback_verified_paused=paused,
                        rollback_stdout=result.stdout, rollback_stderr=result.stderr, **extra)
        except Exception as exc:
            return dict(result="OPERATOR_INTERVENTION_REQUIRED", stage=reason,
                        rollback_error=str(exc), **extra)


def _review_fingerprint(review, proposal):
    inventory = review.get("inventory") or {}
    gate = review.get("shutdown_gate") or {}
    world = gate.get("world") or {}
    return (proposal.get("baseline_iterations"), proposal.get("baseline_clock_us"),
            proposal.get("maximum_end_iterations"),
            tuple(inventory.get("alive", [])), tuple(inventory.get("identity_mismatches", [])),
            tuple(inventory.get("unrecorded_child_tasks", [])), world.get("correct"),
            gate.get("safe"), gate.get("paused"), review.get("auto_arm_disabled"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--expected-iterations", type=int, required=True)
    args = parser.parse_args(argv)
    try:
        result = ShutdownCoordinator(HostAdapter(args.session)).run(args.expected_iterations)
        print(json.dumps(result, indent=2))
        return 0 if result.get("result") == "COMPLETE" else 1
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        print(json.dumps(dict(result="BLOCKED", error=str(exc), runtime_changed=False), indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
