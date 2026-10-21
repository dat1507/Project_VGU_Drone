"""Measured translation between world ENU and estimator-local NED."""

from dataclasses import asdict, dataclass
import numpy as np

from .geometry import vector

RESET_FIELDS = ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter",
                "vz_reset_counter", "heading_reset_counter", "ref_timestamp")


def enu_to_ned(value):
    p = np.asarray(value, dtype=float)
    if p.ndim < 1 or p.shape[-1] != 3 or not np.isfinite(p).all():
        raise ValueError("Expected finite (...,3) vectors")
    return p[..., [1, 0, 2]] * [1, 1, -1]


def ned_to_enu(value):
    return enu_to_ned(value)


def convert_yaw(yaw):
    if not np.isfinite(yaw):
        raise ValueError("Non-finite yaw")
    return float((np.pi / 2 - yaw + np.pi) % (2 * np.pi) - np.pi)


@dataclass(frozen=True)
class FrameAnchor:
    gazebo_enu: tuple
    px4_ned: tuple
    gazebo_timestamp_us: int
    px4_timestamp_us: int
    reset_counters: tuple
    source: str = "measured"

    def __post_init__(self):
        vector(self.gazebo_enu)
        vector(self.px4_ned)
        if min(self.gazebo_timestamp_us, self.px4_timestamp_us) <= 0:
            raise ValueError("Missing measurement timestamps")
        if abs(self.gazebo_timestamp_us - self.px4_timestamp_us) > 20_000:
            raise ValueError("Anchor samples differ by more than 20 ms simulation time")
        if len(self.reset_counters) != len(RESET_FIELDS) or any(int(x) != x or x < 0 for x in self.reset_counters):
            raise ValueError("Invalid reset counters/reference timestamp")
        if self.source != "px4_groundtruth_and_local_position":
            raise ValueError("Anchor must come from measured ground truth and EKF position, not spawn")

    def to_ned(self, enu):
        return vector(self.px4_ned) + enu_to_ned(np.asarray(enu) - vector(self.gazebo_enu))

    def to_enu(self, ned):
        return vector(self.gazebo_enu) + ned_to_enu(np.asarray(ned) - vector(self.px4_ned))

    def check_resets(self, counters):
        if tuple(counters) != tuple(self.reset_counters):
            raise ValueError("EKF reset/reference changed: recapture and revalidate the route")

    def as_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        return cls(**data)


def first_waypoint(anchor, environment, distance=0.5, height=2.5):
    """Measured landed model origin -> cruise start -> East waypoint.

    Validates the cruise segment, not ground-contact takeoff/landing dynamics.
    """
    if distance != 0.5 or height != 2.5:
        raise ValueError("First-demo profile is fixed to 0.5 m East / 2.5 m relative height")
    start = vector(anchor.gazebo_enu) + [0, 0, height]
    end = start + [distance, 0, 0]
    return environment.validate([start, end, start])
