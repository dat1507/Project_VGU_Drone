"""Conservative static SDF geometry and continuous segment/AABB checks."""

from dataclasses import dataclass
import hashlib
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def vector(value, size=3):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError(f"Expected {size} finite coordinates")
    return out


@dataclass(frozen=True)
class AABB:
    name: str
    lower: tuple
    upper: tuple

    def __post_init__(self):
        low, high = vector(self.lower), vector(self.upper)
        if np.any(low >= high):
            raise ValueError("Degenerate AABB")


class FlightEnvironment:
    """Bounds constrain the model origin; boxes are inflated by radius + margin.

    Inflation is an intentionally conservative AABB approximation of a swept
    sphere. This is a warehouse adaptation, NOT the LUT point-drone geometry.
    The caller supplies center bounds already inset from physical boundaries.
    """

    def __init__(self, boxes, lower, upper, radius=0.60, margin=0.25):
        self.lower, self.upper = vector(lower), vector(upper)
        if np.any(self.lower >= self.upper):
            raise ValueError("Invalid flight bounds")
        if not np.isfinite([radius, margin]).all() or min(radius, margin) < 0:
            raise ValueError("Invalid radius/margin")
        self.radius, self.margin = float(radius), float(margin)
        self.boxes = tuple(boxes)
        inflation = radius + margin
        self.low = np.array([b.lower for b in self.boxes], float).reshape(-1, 3) - inflation
        self.high = np.array([b.upper for b in self.boxes], float).reshape(-1, 3) + inflation

    def collision_mask(self, paths):
        p = np.asarray(paths, dtype=float)
        if p.ndim == 2:
            p = p[None]
        if p.ndim != 3 or p.shape[-1] != 3 or p.shape[1] < 2:
            raise ValueError("Expected (batch, >=2, 3) paths")
        if not np.isfinite(p).all():
            raise ValueError("Non-finite path")
        outside = ((p < self.lower) | (p > self.upper)).any(axis=(1, 2))
        if not len(self.boxes):
            return outside
        start = p[:, :-1, None, :]
        delta = p[:, 1:, None, :] - start
        parallel = abs(delta) < 1e-12
        safe_delta = np.where(parallel, 1.0, delta)
        t1 = (self.low[None, None] - start) / safe_delta
        t2 = (self.high[None, None] - start) / safe_delta
        inside = (start >= self.low) & (start <= self.high)
        lo = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
        hi = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
        near = np.maximum(0.0, lo.max(axis=-1))
        far = np.minimum(1.0, hi.min(axis=-1))
        return outside | (near <= far).any(axis=(1, 2))

    def validate(self, path):
        p = np.asarray(path, dtype=float)
        if p.ndim != 2:
            raise ValueError("Validation requires one (N,3) route")
        if self.collision_mask(p)[0]:
            raise ValueError("Path collides with inflated geometry or leaves flight bounds")
        return p.copy()


def _pose(element):
    pose = element.find("pose")
    if pose is not None and (pose.attrib or not pose.text):
        raise ValueError("Pose frame attributes/empty poses are unsupported")
    xyzrpy = vector([float(x) for x in pose.text.split()], 6) if pose is not None else np.zeros(6)
    x, y, z, r, p, yaw = xyzrpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(yaw), np.sin(yaw)
    rot = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                    [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                    [-sp, cp*sr, cp*cr]])
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rot, [x, y, z]
    return transform


@dataclass(frozen=True)
class World:
    name: str
    sdf_path: str
    sdf_sha256: str
    boxes: tuple
    floor_z: float
    physical_xy_lower: tuple
    physical_xy_upper: tuple
    spawn_enu: tuple

    def environment(self, z_min, z_max, radius=0.60, margin=0.25):
        """z_min/z_max are absolute ENU model-origin heights, not AGL."""
        pad = radius + margin
        if not np.isfinite([z_min, z_max]).all() or z_min <= self.floor_z + pad:
            raise ValueError("Cruise lower bound must clear floor plus envelope")
        low = [*(np.array(self.physical_xy_lower) + pad), z_min]
        high = [*(np.array(self.physical_xy_upper) - pad), z_max]
        return FlightEnvironment(self.boxes, low, high, radius, margin)


def load_world(path):
    """Parse the approved static warehouse; reject unmodelled collision shapes.

    Arbitrary relative_to frames, nested models, moving obstacles and unknown
    includes are rejected. Rotated boxes use enclosing world AABBs (conservative).
    x500 is excluded as the vehicle, not silently treated as an obstacle.
    """
    path = Path(path).resolve()
    raw = path.read_bytes()
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ValueError("Malformed SDF XML") from exc
    worlds = root.findall("world")
    if len(worlds) != 1 or worlds[0].get("name") != "warehouse_world":
        raise ValueError("Expected one warehouse_world")
    world = worlds[0]
    if world.findall("frame") or world.findall("actor"):
        raise ValueError("Explicit frames/actors are unsupported")
    orientation = world.findtext("spherical_coordinates/world_frame_orientation")
    heading = float(world.findtext("spherical_coordinates/heading_deg", "0"))
    if orientation != "ENU" or heading != 0:
        raise ValueError("Expected zero-heading ENU world")
    boxes, floors, spawn = [], [], []
    for inc in world.findall("include"):
        if inc.findtext("name") != "x500" or inc.findtext("uri") != "model://x500":
            raise ValueError("Unknown included collision geometry")
        spawn.append(tuple(_pose(inc)[:3, 3]))
    if len(spawn) != 1:
        raise ValueError("Expected exactly one included x500")
    for model in world.findall("model"):
        if model.findtext("static") not in {"true", "1"} or model.findall("model") or model.findall("include"):
            raise ValueError("Only flat static obstacle models are supported")
        if model.findall("frame") or model.findall("joint"):
            raise ValueError("Obstacle frames/joints are unsupported")
        for link in model.findall("link"):
            if link.findall("frame"):
                raise ValueError("Link frames are unsupported")
            for collision in link.findall("collision"):
                transform = _pose(model) @ _pose(link) @ _pose(collision)
                geometry = collision.find("geometry")
                if geometry is None or len(geometry) != 1:
                    raise ValueError("Missing/ambiguous collision geometry")
                shape = geometry[0]
                if shape.tag == "box":
                    size = vector([float(x) for x in shape.findtext("size", "").split()])
                    if np.any(size <= 0):
                        raise ValueError("Invalid box size")
                    corners = np.array(list(product(*[[-s/2, s/2] for s in size])))
                    coords = (transform @ np.c_[corners, np.ones(8)].T).T[:, :3]
                    boxes.append(AABB(model.get("name"), tuple(coords.min(0)), tuple(coords.max(0))))
                elif shape.tag == "plane" and model.get("name") == "white_floor":
                    normal = vector([float(x) for x in shape.findtext("normal", "").split()])
                    if not np.allclose(transform[:3, :3] @ normal, [0, 0, 1]):
                        raise ValueError("Non-horizontal floor")
                    floors.append(float(transform[2, 3]))
                else:
                    raise ValueError(f"Unsupported collision geometry: {shape.tag}")
    if len(floors) != 1:
        raise ValueError("Expected exactly one floor collision")
    walls = {b.name: b for b in boxes if b.name.startswith("wall_")}
    if set(walls) != {"wall_north", "wall_south", "wall_east", "wall_west"}:
        raise ValueError("Cannot infer enclosing warehouse bounds")
    low = (walls["wall_west"].upper[0], walls["wall_south"].upper[1])
    high = (walls["wall_east"].lower[0], walls["wall_north"].lower[1])
    return World(world.get("name"), str(path), hashlib.sha256(raw).hexdigest(), tuple(boxes),
                 floors[0], low, high, spawn[0])
