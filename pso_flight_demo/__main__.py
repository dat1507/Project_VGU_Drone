"""Offline-first command line. No ROS initialization without --execute."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import re
import subprocess
import sys

from .artifacts import export_route, load_route, new_directory, write_json_exclusive
from .frames import FrameAnchor, RESET_FIELDS, first_waypoint, ned_to_enu
from .geometry import load_world
from .planner import PlannerConfig, run, spline_polyline

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SDF = ROOT / "simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf"


def _listener(bin_dir, topic):
    result = subprocess.run([str(Path(bin_dir) / "px4-listener"), "--instance", "0", topic, "-n", "1"],
                            text=True, capture_output=True, timeout=4, check=True)
    return dict(re.findall(r"^\s+(\w+):\s+([^\n]+)", result.stdout, re.M))


def capture_anchor(bin_dir):
    """Only native read-only listeners; no simulator control or ROS participant."""
    armed = _listener(bin_dir, "actuator_armed")
    land = _listener(bin_dir, "vehicle_land_detected")
    if armed.get("armed") != "False" or land.get("landed") != "True":
        raise ValueError("Anchor capture requires Disarmed + Landed")
    gt = _listener(bin_dir, "vehicle_local_position_groundtruth")
    lp = _listener(bin_dir, "vehicle_local_position")
    if not all(lp.get(k) == "True" for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")):
        raise ValueError("EKF local position invalid")
    def number(d, k):
        return float(d[k].split()[0])
    return FrameAnchor(tuple(ned_to_enu([number(gt, k) for k in ("x", "y", "z")])),
                       tuple(number(lp, k) for k in ("x", "y", "z")),
                       int(number(gt, "timestamp_sample")), int(number(lp, "timestamp_sample")),
                       tuple(int(number(lp, k)) for k in RESET_FIELDS),
                       "px4_groundtruth_and_local_position")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    commands = p.add_subparsers(dest="command", required=True)
    geometry = commands.add_parser("geometry", help="Read SDF and print conservative static geometry")
    geometry.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    capture = commands.add_parser("capture", help="Read native telemetry, create new measured anchor bundle")
    capture.add_argument("--name", required=True)
    capture.add_argument("--px4-bin", default="/home/phuocdat/PX4-Autopilot/build/px4_sitl_default/bin")
    for name in ("waypoint", "plan"):
        cmd = commands.add_parser(name)
        cmd.add_argument("--name", required=True, help="New outputs/ subdirectory; existing names are refused")
        cmd.add_argument("--anchor", required=True, type=Path)
        cmd.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
        cmd.add_argument("--z-min", required=True, type=float, help="Absolute ENU cruise model-origin minimum")
        cmd.add_argument("--z-max", required=True, type=float, help="Absolute ENU cruise model-origin maximum")
        if name == "plan":
            cmd.add_argument("--algorithm", choices=("baseline", "enhanced"), required=True)
            cmd.add_argument("--start", nargs=3, type=float, required=True, help="Explicit ENU metres; no spawn default")
            cmd.add_argument("--goal", nargs=3, type=float, required=True)
            cmd.add_argument("--seed", type=int, required=True)
            cmd.add_argument("--jitter", type=float, default=2.5, help="Metres, independent warehouse adaptation")
            cmd.add_argument("--smooth", action="store_true", help="Sample cubic, then validate entire output polyline")
    follow = commands.add_parser("follow", help="Offline dry-run by default: no ROS publishers or commands")
    follow.add_argument("bundle", type=Path)
    follow.add_argument("--execute", action="store_true")
    follow.add_argument("--confirm-live", default="")
    follow.add_argument("--namespace", default="")
    return p


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and (arguments[0] in {"doctor", "demo", "compare", "--help", "-h"} or
                      (arguments[0] == "plan" and "--anchor" not in arguments)):
        from .e2e import main as e2e_main
        result = e2e_main(arguments)
        if result: raise SystemExit(result)
        return
    args = parser().parse_args(arguments)
    if args.command == "geometry":
        world = load_world(args.sdf)
        print(json.dumps(asdict(world), indent=2))
    elif args.command == "capture":
        anchor = capture_anchor(args.px4_bin)
        dest = new_directory(args.name)
        write_json_exclusive(dest / "anchor.json", anchor.as_dict())
        print(dest / "anchor.json")
    elif args.command in {"waypoint", "plan"}:
        world = load_world(args.sdf)
        env = world.environment(args.z_min, args.z_max)
        anchor = FrameAnchor.from_dict(json.loads(args.anchor.read_text()))
        if args.command == "waypoint":
            path = first_waypoint(anchor, env)
            dest = export_route(args.name, path, world, env, anchor, algorithm="first_waypoint", seed=None)
        else:
            config = PlannerConfig(jitter=args.jitter)
            env.validate([args.start, args.start])
            env.validate([args.goal, args.goal])
            result = run(env, args.start, args.goal, env.lower, env.upper, args.algorithm, args.seed, config)
            raw = env.validate(result.path)
            final = env.validate(spline_polyline(raw)) if args.smooth else raw
            dest = export_route(args.name, final, world, env, anchor, algorithm=args.algorithm, seed=args.seed,
                                config=config, fitness=result.fitness, raw_path=raw,
                                smoothing="sampled_cubic_executed_as_polyline" if args.smooth else "none")
        print(dest)
    elif args.command == "follow":
        path, metadata, anchor, env = load_route(args.bundle)
        print(json.dumps(dict(result="OFFLINE_VALIDATED_ONLY", live_readiness="NOT_CHECKED",
                              publishes_commands=False, algorithm=metadata["algorithm"],
                              waypoint_count=len(path), first_ned=anchor.to_ned(path[0]).tolist(),
                              last_ned=anchor.to_ned(path[-1]).tolist()), indent=2))
        if args.execute:
            from .ros_follower import run_live
            run_live(path, metadata, anchor, env, args.confirm_live, args.namespace)


if __name__ == "__main__":
    main()
