"""Exclusive-create route bundles, strict manifests and read-only validation."""

import csv
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from .frames import FrameAnchor
from .geometry import load_world

MODULE_DIR = Path(__file__).resolve().parent
PROVENANCE = "warehouse_integration_adaptation_not_LUT_reproduction"


def new_directory(name):
    if not name or Path(name).name != name or name in {".", ".."}:
        raise ValueError("Output name must be a single new directory name")
    outputs = MODULE_DIR / "outputs"
    if outputs.is_symlink():
        raise ValueError("Symlink output directories are forbidden")
    outputs.mkdir(exist_ok=True)
    dest = outputs / name
    dest.mkdir(exist_ok=False)
    return dest


def write_json_exclusive(path, value):
    content = json.dumps(value, indent=2, allow_nan=False) + "\n"
    with Path(path).open("x", encoding="utf-8") as f:
        f.write(content)


def export_route(name, path, world, environment, anchor, *, algorithm, seed, config=None,
                 fitness=None, raw_path=None, smoothing="none"):
    if algorithm not in {"baseline", "enhanced", "first_waypoint"}:
        raise ValueError("Unknown algorithm/profile")
    anchor = FrameAnchor.from_dict(anchor.as_dict())
    if environment.radius < 0.60 or environment.margin < 0.25:
        raise ValueError("Protective envelope below approved minimum")
    raw = environment.validate(path if raw_path is None else raw_path)
    path = environment.validate(path)  # mandatory validation after any smoothing
    if np.any(path[:, 2] - anchor.gazebo_enu[2] > 3.2):
        raise ValueError("Route exceeds first-demo relative height cap of 3.2 m")
    if not np.array_equal(raw[[0, -1]], path[[0, -1]]):
        raise ValueError("Smoothing changed route endpoints")
    if fitness is not None and not np.isfinite(fitness):
        raise ValueError("Non-finite fitness")
    text = io.StringIO(newline="")
    writer = csv.writer(text)
    writer.writerow(["x_enu_m", "y_enu_m", "z_enu_m"])
    writer.writerows(path.tolist())
    encoded = text.getvalue().encode("utf-8")
    metadata = dict(schema_version=1, provenance=PROVENANCE, algorithm=algorithm, seed=seed,
                    frame="GAZEBO_WORLD_ENU", units="m", world_name=world.name,
                    sdf_path=world.sdf_path, sdf_sha256=world.sdf_sha256,
                    csv_sha256=hashlib.sha256(encoded).hexdigest(), waypoint_count=len(path),
                    start=path[0].tolist(), goal=path[-1].tolist(),
                    flight_lower=environment.lower.tolist(), flight_upper=environment.upper.tolist(),
                    radius_m=environment.radius, margin_m=environment.margin,
                    fitness=fitness, fitness_definition="raw_polyline_length + penalty * binary_collision",
                    planner_config=asdict(config) if config else None,
                    raw_length_m=float(np.linalg.norm(np.diff(raw, axis=0), axis=1).sum()),
                    execution_length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                    raw_collision=False, execution_collision=False, smoothing=smoothing,
                    execution_interpolation="piecewise_linear", anchor=anchor.as_dict(),
                    flight_tested=False, takeoff_landing_validated=False)
    # Complete all validation/serialization BEFORE claiming a fresh output name.
    json.dumps(metadata, allow_nan=False)
    dest = new_directory(name)
    with (dest / "waypoints_enu.csv").open("xb") as f:
        f.write(encoded)
    write_json_exclusive(dest / "manifest.json", metadata)
    return dest


def load_route(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "manifest.json").read_text())
    if metadata.get("schema_version") != 1 or metadata.get("provenance") != PROVENANCE:
        raise ValueError("Unrecognized route provenance/schema")
    if metadata.get("frame") != "GAZEBO_WORLD_ENU" or metadata.get("units") != "m":
        raise ValueError("Unexpected frame/units")
    if metadata.get("execution_interpolation") != "piecewise_linear":
        raise ValueError("Only validated piecewise-linear execution is supported")
    world = load_world(metadata["sdf_path"])
    if world.sdf_sha256 != metadata["sdf_sha256"]:
        raise ValueError("SDF changed since export")
    encoded = (directory / "waypoints_enu.csv").read_bytes()
    if hashlib.sha256(encoded).hexdigest() != metadata["csv_sha256"]:
        raise ValueError("CSV changed since validation")
    reader = csv.reader(io.StringIO(encoded.decode()))
    if next(reader) != ["x_enu_m", "y_enu_m", "z_enu_m"]:
        raise ValueError("Invalid CSV header")
    path = np.asarray([[float(v) for v in row] for row in reader])
    if len(path) != metadata["waypoint_count"]:
        raise ValueError("Waypoint count mismatch")
    if not np.array_equal(path[[0, -1]], [metadata["start"], metadata["goal"]]):
        raise ValueError("Endpoint mismatch")
    radius, margin = metadata["radius_m"], metadata["margin_m"]
    if radius < 0.60 or margin < 0.25:
        raise ValueError("Warehouse route below approved protective envelope")
    environment = world.environment(metadata["flight_lower"][2], metadata["flight_upper"][2], radius, margin)
    if not np.allclose(environment.lower, metadata["flight_lower"], rtol=0, atol=1e-12) or not np.allclose(environment.upper, metadata["flight_upper"], rtol=0, atol=1e-12):
        raise ValueError("Manifest bounds differ from geometry-derived bounds")
    environment.validate(path)
    anchor = FrameAnchor.from_dict(metadata["anchor"])
    if np.any(path[:, 2] - anchor.gazebo_enu[2] > 3.2):
        raise ValueError("Route exceeds relative height cap")
    return path, metadata, anchor, environment
