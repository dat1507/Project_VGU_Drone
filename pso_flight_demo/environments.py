"""C2 configuration and multi-world geometry; does not modify the C1 parser."""

from dataclasses import asdict, dataclass, fields
import hashlib
from itertools import product
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import AABB, World, _pose, vector

MODULE = Path(__file__).resolve().parent
PROJECT = MODULE.parent


@dataclass(frozen=True)
class Profile:
    id: str
    state: str
    sdf: str | None
    world_name: str | None
    vehicle_name: str
    vehicle_uri: str
    floor_model: str | None
    frame: str
    physical_xy_bounds: list | None
    cruise_z: list | None
    radius_m: float
    margin_m: float
    demo_height_m: float
    demo_east_m: float
    max_relative_height_m: float
    start_enu: list | None
    goal_enu: list | None
    output_root: str
    px4_root: str
    ros_setup: str
    px4_ros_setup: str
    qgc: str
    instance: int
    airframe: int
    namespace: str
    dds_domain: int
    agent_port: int

    def __post_init__(self):
        if not self.id or Path(self.id).name != self.id or self.id in {".", ".."}:
            raise ValueError("Invalid profile id")
        if self.state not in {"configured", "planned"} or self.frame != "ENU":
            raise ValueError("Only explicit ENU profiles are supported")
        if not self.vehicle_name or self.vehicle_uri != "model://x500":
            raise ValueError("Only the reviewed x500 protective envelope is currently supported")
        for name in (self.vehicle_name, self.world_name, self.floor_model):
            if name is not None and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
                raise ValueError("World/model names must be safe topic identifiers")
        if self.namespace.strip("/") and any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", x)
                                              for x in self.namespace.strip("/").split("/")):
            raise ValueError("Invalid ROS namespace")
        numbers = [self.radius_m, self.margin_m, self.demo_height_m, self.demo_east_m,
                   self.max_relative_height_m]
        if not np.isfinite(numbers).all() or self.radius_m < .60 or self.margin_m < .25:
            raise ValueError("Envelope must be at least radius .60 + margin .25 m")
        if min(self.demo_height_m, self.demo_east_m) <= 0 or self.demo_height_m > self.max_relative_height_m:
            raise ValueError("Invalid demo dimensions")
        for n in (self.instance, self.airframe, self.dds_domain, self.agent_port):
            if type(n) is not int or n < 0:
                raise ValueError("Invalid integer launch configuration")
        if self.instance > 9 or self.dds_domain > 232 or not 1 <= self.agent_port <= 65535:
            raise ValueError("Instance/domain/port out of range")
        self.output_directory()  # validate scope without creating directories

    def sdf_path(self):
        if not self.sdf:
            raise ValueError(f"Profile {self.id}: planned environment has no SDF configured")
        p = Path(self.sdf).expanduser()
        return (PROJECT / p).resolve() if not p.is_absolute() else p.resolve()

    def output_directory(self):
        p = Path(self.output_root)
        if p.is_absolute() or ".." in p.parts or not p.parts:
            raise ValueError("Output root must be relative inside pso_flight_demo")
        dest = MODULE / p
        if not dest.resolve().is_relative_to(MODULE.resolve()) or dest.resolve() == MODULE.resolve():
            raise ValueError("Output escapes module")
        cursor = MODULE
        for part in p.parts:
            cursor = cursor / part
            if cursor.is_symlink():
                raise ValueError("Output symlinks are forbidden")
        return dest

    def snapshot(self):
        return asdict(self)

    def validate_ready(self):
        p = self.sdf_path()
        if self.state != "configured" or not p.is_file():
            raise ValueError(f"Environment not configured or SDF missing: {p}")
        if not self.world_name or not self.floor_model or self.physical_xy_bounds is None or self.cruise_z is None:
            raise ValueError("World, floor, physical bounds and cruise heights must be configured")
        bounds = np.asarray(self.physical_xy_bounds, float)
        z = np.asarray(self.cruise_z, float)
        if bounds.shape != (2, 2) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]):
            raise ValueError("Invalid physical XY bounds")
        if z.shape != (2,) or not np.isfinite(z).all() or z[0] >= z[1]:
            raise ValueError("Invalid absolute cruise heights")


def load_profile(value="warehouse", overrides=None):
    import yaml

    p = Path(value)
    if not p.is_file():
        p = MODULE / "profiles" / f"{value}.yaml"
    if p.stat().st_size > 65536:
        raise ValueError("Profile too large")
    data = yaml.safe_load(p.read_text())
    if not isinstance(data, dict) or set(data) != {f.name for f in fields(Profile)}:
        raise ValueError("Profile keys do not match schema")
    data.update({k: v for k, v in (overrides or {}).items() if v is not None})
    return Profile(**data)


def load_environment(profile):
    profile.validate_ready()
    path = profile.sdf_path()
    raw = path.read_bytes()
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ValueError("Malformed SDF XML") from exc
    worlds = root.findall("world")
    if len(worlds) != 1 or worlds[0].get("name") != profile.world_name:
        raise ValueError("SDF world name/count does not match profile")
    world = worlds[0]
    if world.findall("frame") or world.findall("actor"):
        raise ValueError("Unsupported world frames/actors")
    if world.findtext("spherical_coordinates/world_frame_orientation") != profile.frame:
        raise ValueError("SDF/profile frame mismatch; explicit spherical ENU is required")
    if float(world.findtext("spherical_coordinates/heading_deg", "0")) != 0:
        raise ValueError("Nonzero spherical heading is not supported by ENU/NED anchor conversion")
    spawns, boxes, floors = [], [], []
    for include in world.findall("include"):
        if include.findtext("name") != profile.vehicle_name or include.findtext("uri") != profile.vehicle_uri:
            raise ValueError("Unresolved non-vehicle include: collision geometry cannot be ignored")
        spawns.append(tuple(_pose(include)[:3, 3]))
    if len(spawns) != 1:
        raise ValueError("SDF must include exactly one configured vehicle")
    for model in world.findall("model"):
        name = model.get("name")
        if name == profile.vehicle_name or model.findtext("static") not in {"true", "1"}:
            raise ValueError("Unsupported inline vehicle or moving obstacle")
        if any(model.findall(tag) for tag in ("model", "include", "frame", "joint")):
            raise ValueError("Nested models/includes/frames/joints require an explicit geometry resolver")
        for link in model.findall("link"):
            if link.findall("frame"):
                raise ValueError("Unsupported link frame")
            for c in link.findall("collision"):
                t = _pose(model) @ _pose(link) @ _pose(c)
                geometry = c.find("geometry")
                if geometry is None or len(geometry) != 1:
                    raise ValueError("Missing or ambiguous collision geometry")
                shape = geometry[0]
                if shape.tag == "box":
                    size = vector([float(x) for x in shape.findtext("size", "").split()])
                    if np.any(size <= 0):
                        raise ValueError("Invalid box dimensions")
                    vertices = np.array(list(product(*[[-s/2, s/2] for s in size])))
                    positions = (t @ np.c_[vertices, np.ones(8)].T).T[:, :3]
                    boxes.append(AABB(name, tuple(positions.min(0)), tuple(positions.max(0))))
                elif shape.tag == "plane" and name == profile.floor_model:
                    normal = vector([float(x) for x in shape.findtext("normal", "").split()])
                    if not np.allclose(t[:3, :3] @ normal, [0, 0, 1]):
                        raise ValueError("Non-horizontal floor")
                    floors.append(float(t[2, 3]))
                else:
                    raise ValueError(f"Unsupported {shape.tag} collision in {name}; fail closed")
    if len(floors) != 1:
        raise ValueError("Expected exactly one configured floor")
    bounds = profile.physical_xy_bounds
    result = World(profile.world_name, str(path), hashlib.sha256(raw).hexdigest(), tuple(boxes),
                   floors[0], tuple(bounds[0]), tuple(bounds[1]), spawns[0])
    env = result.environment(*profile.cruise_z, profile.radius_m, profile.margin_m)
    # Configured planning endpoints are checked independently, without claiming
    # that the direct segment between them must be feasible for a planner.
    for p in (profile.start_enu, profile.goal_enu):
        if p is not None:
            env.validate([vector(p), vector(p)])
    env.validate([[*spawns[0][:2], sum(profile.cruise_z)/2]]*2)
    return result, env
