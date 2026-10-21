"""C2 single entry point: read-only host checks, offline routes and launch plans."""

import argparse
from dataclasses import replace
import json
from pathlib import Path

from .artifacts import write_json_exclusive
from .environment_artifacts import dry_run, export_bundle, new_output, waypoint
from .environments import load_environment, load_profile
from .frames import FrameAnchor
from .launch_plan import launch_plan, shutdown_plan
from .planner import PlannerConfig, run, spline_polyline
from .runtime_readiness import inspect_runtime
from .software_inventory import inventory


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", default="warehouse", help="Builtin name or YAML file")
    p.add_argument("--sdf", help="Read a different SDF; never modifies it")
    p.add_argument("--world-name")
    p.add_argument("--floor-model")
    p.add_argument("--vehicle-name")
    p.add_argument("--namespace")
    p.add_argument("--output-root", help="Relative to pso_flight_demo; cannot escape module")
    commands = p.add_subparsers(dest="command", required=True)
    commands.add_parser("inspect", help="Offline profile/SDF/geometry validation")
    check = commands.add_parser("host-check", help="Native read-only probes; optional subscriber-only DDS")
    check.add_argument("--expect-iterations", type=int)
    check.add_argument("--prepare-waypoint", action="store_true")
    check.add_argument("--observe-dds", type=float, default=0)
    check.add_argument("--name", help="Optional NEW report directory; otherwise UUID")
    dry = commands.add_parser("dry-run")
    dry.add_argument("bundle", type=Path)
    dry.add_argument("--runtime-report", type=Path, help="Fresh host report to check restart identity")
    launch = commands.add_parser("launch", help="Print commands only; there is deliberately no --execute")
    launch.add_argument("--without-dds", action="store_true")
    launch.add_argument("--check-host", action="store_true", help="Read-only conflict checks; still never starts processes")
    commands.add_parser("shutdown-plan", help="Print manual stop sequence only; never stops a process")
    plan = commands.add_parser("plan", help="Generate a NEW PSO bundle; never starts a follower")
    plan.add_argument("--anchor", type=Path, required=True)
    plan.add_argument("--runtime-report", type=Path, required=True)
    plan.add_argument("--start", nargs=3, type=float)
    plan.add_argument("--goal", nargs=3, type=float)
    plan.add_argument("--algorithm", choices=("baseline", "enhanced"), required=True)
    plan.add_argument("--seed", type=int, required=True)
    plan.add_argument("--jitter", type=float, required=True, help="Explicit warehouse initialization jitter in metres")
    plan.add_argument("--smooth", action="store_true")
    plan.add_argument("--name")
    return p


def execute(args):
    overrides = {k: getattr(args, k) for k in ("sdf", "world_name", "floor_model", "vehicle_name", "namespace", "output_root")}
    profile = load_profile(args.profile, overrides)
    if args.command == "inspect":
        world, env = load_environment(profile)
        return dict(result="OFFLINE_ENVIRONMENT_VALIDATED", profile=profile.snapshot(),
                    sdf_sha256=world.sdf_sha256, obstacle_boxes=len(world.boxes), floor_z=world.floor_z,
                    spawn_enu=list(world.spawn_enu), spawn_is_anchor=False,
                    flight_lower=env.lower.tolist(), flight_upper=env.upper.tolist(),
                    start_goal_status="UNSET" if profile.start_enu is None or profile.goal_enu is None else "VALIDATED",
                    publishes_commands=False, live_control_go=False)
    if args.command == "launch":
        return launch_plan(profile, not args.without_dds, args.profile, args.check_host)
    if args.command == "shutdown-plan":
        return shutdown_plan(profile)
    if args.command == "dry-run":
        identity = None
        if args.runtime_report:
            report = json.loads(args.runtime_report.read_text())
            identity = report["runtime_identity"]
        result = dry_run(args.bundle, identity)
        result["identity_evidence_note"] = "Supplied report identity checked, not a new live probe"
        return result
    if args.command == "host-check":
        if not 0 <= args.observe_dds <= 10:
            raise ValueError("DDS observation must be between 0 and 10 seconds")
        report, anchor, env = inspect_runtime(profile, args.expect_iterations)
        report["software_inventory"] = inventory(profile)
        dest = new_output(profile, args.name)
        if args.observe_dds:
            from .dds_observer import observe
            report["dds_observer"] = observe(profile, args.observe_dds, dest/"ros_logs")
        if args.prepare_waypoint and not report["problems"] and anchor is not None:
            try:
                path = waypoint(anchor, profile, env)
                bundle = export_bundle(profile, anchor, path, identity=report["runtime_identity"])
                report["anchor_path"] = str(bundle/"anchor.json")
                report["waypoint_bundle"] = str(bundle)
                report["dry_run"] = dry_run(bundle, report["runtime_identity"])
            except ValueError as exc:
                report["problems"].append("waypoint_validation_failed: "+str(exc))
        elif args.prepare_waypoint:
            report["waypoint_status"] = "NOT_CREATED: native readiness/anchor checks failed"
        report["result"] = "NO_GO" if report["problems"] else "PAUSED_NATIVE_VALIDATED"
        observed = report.get("dds_observer", {})
        report["dds_evidence_level"] = observed.get('dds_evidence_level') or ("MESSAGE_RECEIVED" if observed.get("dds_messages_received")
                                        else "GRAPH_ONLY" if observed.get("dds_entities_seen")
                                        else "NO_DATA_OR_NOT_PROBED")
        report["report_path"] = str(dest/"runtime_report.json")
        write_json_exclusive(dest/"runtime_report.json", report)
        # Full native stdout/stderr is preserved in the exclusive report file.
        return {k: v for k, v in report.items() if k not in {"evidence", "world", "profile"}}
    if args.command == "plan":
        report = json.loads(args.runtime_report.read_text())
        if report.get("problems") or report["profile"]["id"] != profile.id:
            raise ValueError("Need matching successful native report")
        if report["world"]["sdf_sha256"] != load_environment(profile)[0].sdf_sha256:
            raise ValueError("Report/SDF mismatch")
        anchor = FrameAnchor.from_dict(json.loads(args.anchor.read_text()))
        # Anchor must be the measured pair actually present in the supplied report.
        lp = report["evidence"]["vehicle_local_position"]["stdout"]
        from .runtime_readiness import topic_fields, number
        fields = topic_fields(lp)
        if int(number(fields, "timestamp_sample")) != anchor.px4_timestamp_us:
            raise ValueError("Anchor/report timestamp mismatch")
        from .frames import RESET_FIELDS
        if tuple(int(number(fields, k)) for k in RESET_FIELDS) != tuple(anchor.reset_counters):
            raise ValueError("Anchor/report reset counters mismatch")
        if any(number(fields, k) != anchor.px4_ned[i] for i, k in enumerate(("x", "y", "z"))):
            raise ValueError("Anchor/report local pose mismatch")
        from .frames import ned_to_enu
        gt = topic_fields(report["evidence"]["vehicle_local_position_groundtruth"]["stdout"])
        import numpy as np
        if int(number(gt, "timestamp_sample")) != anchor.gazebo_timestamp_us or not np.array_equal(
                ned_to_enu([number(gt, k) for k in ("x", "y", "z")]), anchor.gazebo_enu):
            raise ValueError("Anchor/report ground truth mismatch")
        start = args.start if args.start is not None else profile.start_enu
        goal = args.goal if args.goal is not None else profile.goal_enu
        if start is None or goal is None:
            raise ValueError("Explicit, geometry-validated Start and Goal required")
        profile = replace(profile, start_enu=start, goal_enu=goal)
        _, env = load_environment(profile)
        cfg = PlannerConfig(jitter=args.jitter)
        plan = run(env, start, goal, env.lower, env.upper, args.algorithm, args.seed, cfg)
        raw = env.validate(plan.path)
        final = env.validate(spline_polyline(raw)) if args.smooth else raw
        bundle = export_bundle(profile, anchor, final, identity=report["runtime_identity"], name=args.name,
                               algorithm=args.algorithm, seed=args.seed, config=cfg,
                               fitness=plan.fitness, raw_path=raw)
        return dict(bundle=str(bundle), dry_run=dry_run(bundle), publishes_commands=False)
    raise ValueError("Unsupported command")


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        result = execute(args)
    except (ValueError, OSError, KeyError) as exc:
        print(json.dumps(dict(result="NO_GO", error=str(exc), publishes_commands=False)))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    blocked_start = result.get("host_precheck", {}).get("result") == "NEW_SESSION_BLOCKED"
    return 1 if result.get("result") == "NO_GO" or blocked_start else 0


if __name__ == "__main__":
    raise SystemExit(main())
