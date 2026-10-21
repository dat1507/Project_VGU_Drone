"""C2 bundles: environment snapshot, measured anchor and restart identity."""

import csv
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
from uuid import uuid4

import numpy as np

from .artifacts import PROVENANCE, write_json_exclusive
from .environments import Profile, load_environment
from .frames import FrameAnchor


def new_output(profile, name=None):
    name = name or ("session_" + uuid4().hex)
    if not name or Path(name).name != name or name in {".", ".."}:
        raise ValueError("Output name must be a new basename")
    root = profile.output_directory()
    root.mkdir(parents=True, exist_ok=True)
    dest = root / name
    dest.mkdir(exist_ok=False)
    return dest


def waypoint(anchor, profile, env):
    start = np.asarray(anchor.gazebo_enu) + [0, 0, profile.demo_height_m]
    end = start + [profile.demo_east_m, 0, 0]
    return env.validate([start, end, start])


def validate_path(path, anchor, profile, env):
    path = env.validate(path)
    if np.any(path[:, 2] - anchor.gazebo_enu[2] > profile.max_relative_height_m):
        raise ValueError("Route exceeds configured relative height cap")
    return path


def export_bundle(profile, anchor, path, *, identity, name=None, algorithm="first_waypoint",
                  seed=None, config=None, fitness=None, raw_path=None):
    world, env = load_environment(profile)
    FrameAnchor.from_dict(anchor.as_dict())
    path = validate_path(path, anchor, profile, env)
    raw = validate_path(path if raw_path is None else raw_path, anchor, profile, env)
    if not np.array_equal(raw[[0, -1]], path[[0, -1]]):
        raise ValueError("Postprocessing changed endpoints")
    if algorithm not in {"first_waypoint", "baseline", "enhanced"}:
        raise ValueError("Unknown algorithm")
    if algorithm != "first_waypoint" and (config is None or type(seed) is not int or fitness is None):
        raise ValueError("PSO export requires seed, config and fitness")
    if algorithm != "first_waypoint":
        if profile.start_enu is None or profile.goal_enu is None:
            raise ValueError("PSO profile requires explicit Start/Goal")
        if not np.array_equal(raw[[0, -1]], [profile.start_enu, profile.goal_enu]):
            raise ValueError("Raw route endpoints differ from configured Start/Goal")
    elif not np.array_equal(path, waypoint(anchor, profile, env)):
        raise ValueError("First-waypoint route must match the measured configured demo")
    text = io.StringIO(newline="")
    writer = csv.writer(text)
    writer.writerow(["x_enu_m", "y_enu_m", "z_enu_m"])
    writer.writerows(path.tolist())
    encoded = text.getvalue().encode()
    # Keep schema/provenance compatible with C1's warehouse loader. New worlds
    # must use the C2 loader, and cannot invoke the warehouse-only C1 live code.
    meta = dict(schema_version=1, provenance=PROVENANCE, c2_environment=profile.snapshot(),
                runtime_identity=identity, live_readiness="NOT_PROVEN", algorithm=algorithm,
                seed=seed, planner_config=asdict(config) if config else None, fitness=fitness,
                fitness_definition="raw_polyline_length + penalty * binary_collision",
                world_name=world.name, sdf_path=world.sdf_path, sdf_sha256=world.sdf_sha256,
                frame="GAZEBO_WORLD_ENU", units="m", execution_interpolation="piecewise_linear",
                csv_sha256=hashlib.sha256(encoded).hexdigest(), waypoint_count=len(path),
                start=path[0].tolist(), goal=path[-1].tolist(), anchor=anchor.as_dict(),
                flight_lower=env.lower.tolist(), flight_upper=env.upper.tolist(),
                radius_m=profile.radius_m, margin_m=profile.margin_m,
                raw_collision=False, execution_collision=False, flight_tested=False,
                takeoff_landing_validated=False,
                raw_length_m=float(np.linalg.norm(np.diff(raw, axis=0), axis=1).sum()),
                execution_length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()))
    json.dumps(meta, allow_nan=False)
    dest = new_output(profile, name)
    with (dest / "waypoints_enu.csv").open("xb") as f:
        f.write(encoded)
    write_json_exclusive(dest / "manifest.json", meta)
    write_json_exclusive(dest / "anchor.json", anchor.as_dict())
    return dest


def load_bundle(directory, current_identity=None):
    directory = Path(directory)
    m = json.loads((directory / "manifest.json").read_text())
    if m.get("schema_version") != 1 or m.get("provenance") != PROVENANCE:
        raise ValueError("Unknown bundle provenance/schema")
    if m.get("frame") != "GAZEBO_WORLD_ENU" or m.get("units") != "m" or m.get("execution_interpolation") != "piecewise_linear":
        raise ValueError("Unsupported frame/units/interpolation")
    profile = Profile(**m["c2_environment"])
    world, env = load_environment(profile)
    if world.sdf_sha256 != m["sdf_sha256"] or world.name != m["world_name"] or world.sdf_path != m["sdf_path"]:
        raise ValueError("Environment changed since export")
    if not np.array_equal(env.lower, m["flight_lower"]) or not np.array_equal(env.upper, m["flight_upper"]):
        raise ValueError("Bounds changed")
    if profile.radius_m != m["radius_m"] or profile.margin_m != m["margin_m"]:
        raise ValueError("Safety policy mismatch")
    content = (directory / "waypoints_enu.csv").read_bytes()
    if hashlib.sha256(content).hexdigest() != m["csv_sha256"]:
        raise ValueError("CSV hash mismatch")
    reader = csv.reader(io.StringIO(content.decode()))
    if next(reader) != ["x_enu_m", "y_enu_m", "z_enu_m"]:
        raise ValueError("Unexpected CSV header")
    path = np.asarray([[float(x) for x in row] for row in reader])
    anchor = FrameAnchor.from_dict(m["anchor"])
    validate_path(path, anchor, profile, env)
    if len(path) != m["waypoint_count"] or not np.array_equal(path[[0, -1]], [m["start"], m["goal"]]):
        raise ValueError("Route count/endpoints mismatch")
    if current_identity is not None:
        saved = m.get("runtime_identity")
        required = ("boot_id", "px4_pid", "px4_start_ticks", "gazebo_server_pid", "gazebo_start_ticks")
        if not saved or any(saved.get(k) is None or current_identity.get(k) != saved[k] for k in required):
            raise ValueError("Host/PX4/Gazebo restarted or identity missing; recapture anchor")
    return path, m, anchor, profile


def dry_run(directory, current_identity=None):
    path, m, anchor, profile = load_bundle(directory, current_identity)
    compatible = False
    if profile.world_name == "warehouse_world" and profile.vehicle_name == "x500" and profile.instance == 0:
        from .artifacts import load_route
        try:
            load_route(directory)
            compatible = True
        except (ValueError, OSError, KeyError):
            pass
    return dict(result="OFFLINE_VALIDATED_ONLY", publishes_commands=False,
                environment=profile.id, world=profile.world_name, waypoint_count=len(path),
                start_ned=anchor.to_ned(path[0]).tolist(),
                outward_delta_ned=(anchor.to_ned(path[1])-anchor.to_ned(path[0])).tolist(),
                live_readiness="NOT_PROVEN", reboot_identity_checked=current_identity is not None,
                c1_bundle_loader_compatible=compatible,
                live_controller_support="C1_WAREHOUSE_ONLY_NOT_LIVE_REVALIDATED" if compatible else "NOT_IMPLEMENTED")
