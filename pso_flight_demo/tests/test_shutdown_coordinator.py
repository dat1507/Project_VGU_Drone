"""Offline shutdown coordinator tests; all runtime endpoints are fake."""

import unittest
from types import SimpleNamespace

from pso_flight_demo.shutdown_coordinator import (
    CLOCK_APPROVAL, SHUTDOWN_APPROVAL, ShutdownCoordinator, Stats,
    enough_headroom, remaining_window_seconds, validate_review,
)


BASE = 5_943_682


def review(iterations=BASE, *, identity=(700, 800, 801, 802)):
    return {
        "result": "READY_FOR_SEPARATE_CLOCK_APPROVAL", "blockers": [],
        "shutdown_request_allowed": False, "auto_arm_disabled": True,
        "inventory": {"alive": list(identity), "identity_mismatches": [], "unrecorded_child_tasks": []},
        "shutdown_gate": {"safe": True, "paused": True, "clock_after_us": 5_943_682_000,
                          "world": {"correct": True, "iterations": iterations}},
        "clock_proposal": {
            "baseline_iterations": iterations, "baseline_clock_us": 5_943_682_000,
            "additional_steps": 5000, "maximum_end_iterations": iterations + 5000,
            "requires_separate_operator_clock_approval": True,
            "command": "gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl "
                       "--reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true, multi_step: 5000'",
            "rollback_pause_command": "gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl "
                                      "--reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true'",
        },
    }


class FakeProc:
    returncode = 0
    def poll(self): return 0
    def communicate(self): return ("REQUEST_QUEUED_NOT_PROCESS_EXIT", "")


class FakeAdapter:
    def __init__(self, stats=None, reviews=None, verify=None):
        self.reviews = list(reviews or [review(), review()])
        self.samples = list(stats or [])
        self.last_stats = Stats(BASE, 5_943_682_000, True)
        self.verify = verify or {"result": "ALL_CHILDREN_EXITED_START_CANDIDATE", "restart_allowed": True}
        self.calls = []
    def review(self):
        self.calls.append("review")
        return self.reviews.pop(0)
    def start_clock(self, command):
        self.calls.append(("clock", command))
        return FakeProc()
    def stats(self):
        self.calls.append("stats")
        if self.samples:
            self.last_stats = self.samples.pop(0)
        return self.last_stats
    def start_shutdown(self):
        self.calls.append("shutdown")
        return FakeProc()
    def rollback_pause(self, command):
        self.calls.append(("rollback", command))
        self.last_stats = Stats(self.last_stats.iterations, self.last_stats.sim_us, True)
        return SimpleNamespace(returncode=0, stdout="paused", stderr="")
    def verify_stopped(self):
        self.calls.append("verify")
        return self.verify


class FakeTime:
    def __init__(self): self.value = 0.0
    def monotonic(self):
        self.value += .01
        return self.value
    def sleep(self, seconds): self.value += seconds


class CoordinatorTests(unittest.TestCase):
    def test_review_rejects_baseline_identity_and_unbounded_proposal_changes(self):
        for data in (review(BASE + 1),
                     dict(review(), auto_arm_disabled=False),
                     dict(review(), result="NO_GO"),
                     dict(review(), inventory={"alive": [1], "identity_mismatches": [2],
                                               "unrecorded_child_tasks": []})):
            with self.subTest(data=data):
                with self.assertRaises(ValueError): validate_review(data, BASE)
        malformed = review()
        malformed["clock_proposal"]["command"] = "gz service --req 'pause: false'"
        with self.assertRaises(ValueError): validate_review(malformed, BASE)

    def test_headroom_uses_observed_rate_and_requires_both_gate_passes(self):
        seconds = remaining_window_seconds(100, 5000, 100, 1.0)
        self.assertAlmostEqual(seconds, 49.0)
        self.assertTrue(enough_headroom(seconds, 2.0))
        self.assertFalse(enough_headroom(.5, 2.0))
        self.assertFalse(enough_headroom(-1, 1.0))

    def test_approvals_are_separate_and_wrong_phrase_causes_no_runtime_calls(self):
        adapter = FakeAdapter()
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, "no"])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["stage"], "SHUTDOWN_APPROVAL")
        self.assertEqual(adapter.calls, ["review"])

    def test_changed_fresh_review_stops_before_clock(self):
        adapter = FakeAdapter(reviews=[review(), review(BASE + 1)])
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, SHUTDOWN_APPROVAL])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["stage"], "FRESH_REVIEW_CHANGED")
        self.assertFalse(any(isinstance(call, tuple) and call[0] == "clock" for call in adapter.calls))

    def test_still_paused_no_clock_progress_never_queues_or_retries(self):
        adapter = FakeAdapter(stats=[Stats(BASE, 5_943_682_000, True)])
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, SHUTDOWN_APPROVAL])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["stage"], "NO_CLOCK_PROGRESS")
        self.assertEqual(sum(call == "shutdown" for call in adapter.calls), 0)
        self.assertEqual(sum(isinstance(call, tuple) and call[0] == "rollback" for call in adapter.calls), 0)

    def test_window_timeout_uses_one_pause_rollback_and_verifies_it(self):
        adapter = FakeAdapter(stats=[Stats(BASE, 5_943_682_000, False)])
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, SHUTDOWN_APPROVAL])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["stage"], "CLOCK_WINDOW_ENDED_BEFORE_SHUTDOWN_QUEUE")
        self.assertEqual(result["rollback_verified_paused"], True)
        self.assertEqual(sum(call == "shutdown" for call in adapter.calls), 0)
        self.assertEqual(sum(isinstance(call, tuple) and call[0] == "rollback" for call in adapter.calls), 1)

    def test_one_finite_window_queues_once_then_verifies_stopped(self):
        adapter = FakeAdapter(stats=[
            Stats(BASE, 5_943_682_000, False),
            Stats(BASE + 5, 5_943_687_000, False),
            Stats(BASE + 10, 5_943_692_000, False),
        ])
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, SHUTDOWN_APPROVAL])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["result"], "COMPLETE")
        self.assertEqual(sum(call == "shutdown" for call in adapter.calls), 1)
        self.assertEqual(adapter.calls.count("verify"), 1)
        self.assertIn("multi_step: 5000", adapter.calls[2][1])

    def test_shutdown_cli_rejection_never_retries(self):
        class RejectAdapter(FakeAdapter):
            def start_shutdown(self):
                self.calls.append("shutdown")
                p = FakeProc(); p.returncode = 2
                p.communicate = lambda: ("BLOCKED", "clock ended")
                return p
        adapter = RejectAdapter(stats=[Stats(BASE, 5_943_682_000, False),
                                       Stats(BASE + 5, 5_943_687_000, False),
                                       Stats(BASE + 10, 5_943_692_000, False)])
        time = FakeTime()
        answers = iter([CLOCK_APPROVAL, SHUTDOWN_APPROVAL])
        result = ShutdownCoordinator(adapter, confirm=lambda _: next(answers),
                                      monotonic=time.monotonic, sleep=time.sleep).run(BASE)
        self.assertEqual(result["stage"], "SHUTDOWN_NOT_QUEUED_NO_RETRY")
        self.assertEqual(sum(call == "shutdown" for call in adapter.calls), 1)
        self.assertNotIn("verify", adapter.calls)


if __name__ == "__main__":
    unittest.main()
