"""ROS-independent follower state, freshness gates and bounded-speed targets."""

from dataclasses import dataclass
import math
import numpy as np

from .geometry import vector


def quaternion_tilt(q):
    q = vector(q, 4)
    if abs(np.linalg.norm(q) - 1) > 0.02:
        raise ValueError("Invalid attitude quaternion")
    w, x, y, z = q
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(float(np.clip(2 * (w*y-z*x), -1, 1)))
    return roll, pitch


@dataclass(frozen=True)
class Observation:
    position_ned: tuple
    quaternion: tuple
    velocity_ned: tuple
    resets: tuple
    timestamp_us: int
    armed: bool
    landed: bool
    failsafe: bool
    gcs_lost: bool
    valid: bool
    estimator_fault: bool
    preflight: bool


class Freshness:
    def __init__(self):
        self.samples = {}

    def record(self, topic, timestamp_us, monotonic):
        if timestamp_us <= 0:
            raise ValueError("Invalid source timestamp")
        previous = self.samples.get(topic)
        if previous and timestamp_us < previous[0]:
            raise ValueError("Source time moved backwards")
        progressed = monotonic if not previous or timestamp_us > previous[0] else previous[2]
        self.samples[topic] = (timestamp_us, monotonic, progressed)

    def check(self, topics, now, received_timeout=1.5, progress_timeout=1.5):
        for topic in topics:
            if topic not in self.samples:
                raise ValueError(f"Missing telemetry: {topic}")
            _, received, progressed = self.samples[topic]
            if now - received > received_timeout or now - progressed > progress_timeout:
                raise ValueError(f"Stale/frozen telemetry: {topic}")


def check_observation(obs, anchor, environment):
    anchor.check_resets(obs.resets)
    if obs.timestamp_us < anchor.px4_timestamp_us:
        raise ValueError("Observation predates frame anchor")
    if not obs.valid or obs.estimator_fault or obs.failsafe or obs.gcs_lost:
        raise ValueError("Invalid estimator, failsafe, or GCS loss")
    vector(obs.velocity_ned)
    roll, pitch = quaternion_tilt(obs.quaternion)
    if max(abs(roll), abs(pitch)) > math.radians(15):
        raise ValueError("Tilt exceeds 15 degrees")
    position = anchor.to_enu(vector(obs.position_ned))
    environment.validate([position, position])
    return position


class RouteTracker:
    """Advance only after actual arrival, bounded commands and 5 simulated seconds hold.

    This is a slow stop-at-waypoint follower. Its speed limit applies to target
    motion; actual aircraft velocity is also checked independently by the node.
    No spline reconstruction or nearest-waypoint shortcut is allowed.
    """

    def __init__(self, path, speed=0.2, tolerance=0.10, tracking_limit=0.25):
        self.path = np.asarray(path, float)
        if self.path.ndim != 2 or self.path.shape[1] != 3 or len(self.path) < 2 or not np.isfinite(self.path).all():
            raise ValueError("Invalid route")
        if not 0 < speed <= 0.2 or not 0 < tolerance < tracking_limit <= 0.25:
            raise ValueError("Invalid first-demo tracking limits")
        self.speed, self.tolerance, self.tracking_limit = speed, tolerance, tracking_limit
        self.target = self.path[0].copy()
        self.index = 1
        self.last_time = None
        self.arrival_time = None

    def update(self, actual, sim_time):
        actual = vector(actual)
        if not np.isfinite(sim_time):
            raise ValueError("Non-finite simulation time")
        if self.last_time is not None and sim_time < self.last_time:
            raise ValueError("Simulation time moved backwards")
        if np.linalg.norm(actual - self.target) > self.tracking_limit:
            raise ValueError("Tracking error exceeds 0.25 m")
        dt = 0.0 if self.last_time is None else min(sim_time - self.last_time, 0.1)
        self.last_time = sim_time
        goal = self.path[self.index]
        delta = goal - self.target
        distance = np.linalg.norm(delta)
        if distance > 0:
            self.target += delta * min(1.0, self.speed * dt / distance)
        if np.linalg.norm(actual - goal) <= self.tolerance and np.linalg.norm(self.target - goal) < 1e-9:
            if self.arrival_time is None:
                self.arrival_time = sim_time
            if sim_time - self.arrival_time >= 5.0:
                if self.index == len(self.path) - 1:
                    return self.target.copy(), True
                self.index += 1
                self.arrival_time = None
        else:
            self.arrival_time = None
        return self.target.copy(), False


def topic_name(base, message_type):
    version = getattr(message_type, "MESSAGE_VERSION", 0)
    return base + (f"_v{version}" if version else "")


def mode_confirmed(ack, actual_mode, expected_mode, elapsed):
    """An ACK alone or a mode alone is insufficient; do not retry commands."""
    if ack is not None and ack != 0:
        raise ValueError("Command rejected")
    confirmed = ack == 0 and actual_mode == expected_mode
    if not confirmed and elapsed > 3:
        raise ValueError("Command ACK/mode timeout")
    return confirmed


def matching_ack(messages, command, sent_monotonic):
    matches = [a.result for received, a in messages if received >= sent_monotonic
               and a.command == command and a.target_system == 245 and a.target_component == 191]
    return matches[-1] if matches else None
