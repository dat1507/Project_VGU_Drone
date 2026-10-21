"""Configurable independent PSO runner; update ordering matches pso_core.py.

    Geometry inflation is supplied externally. Fitness remains path length plus
    one binary collision penalty; no clearance or smoothing term is introduced.
"""

from dataclasses import dataclass
import numpy as np

from .geometry import vector


@dataclass(frozen=True)
class PlannerConfig:
    swarm: int = 100
    iterations: int = 200
    n_waypoints: int = 15
    jitter: float = 2.5
    penalty: float = 10_000.0
    c1: float = 2.0
    c2: float = 2.0

    def __post_init__(self):
        if any(not isinstance(n, int) or isinstance(n, bool) or n < 1
               for n in (self.swarm, self.iterations, self.n_waypoints)):
            raise ValueError("Positive integer budgets required")
        if not np.isfinite([self.jitter, self.penalty, self.c1, self.c2]).all():
            raise ValueError("Non-finite planner configuration")
        if self.jitter < 0 or min(self.penalty, self.c1, self.c2) <= 0:
            raise ValueError("Invalid planner configuration")


def inertia(algorithm, iteration, total):
    if algorithm == "baseline":
        return 0.7
    if algorithm == "enhanced":
        return 0.9 - (0.9 - 0.4) * iteration / total
    raise ValueError("Expected baseline or enhanced")


def complete(mids, start, goal):
    mids = np.asarray(mids, float)
    if mids.ndim == 2:
        mids = mids[None]
    return np.concatenate((np.broadcast_to(start, (len(mids), 1, 3)), mids,
                           np.broadcast_to(goal, (len(mids), 1, 3))), axis=1)


@dataclass
class Plan:
    path: np.ndarray
    positions: np.ndarray
    fitness: float
    raw_length: float
    raw_collision: bool
    best_history: np.ndarray
    convergence_iteration: int
    fitness_evaluations: int
    algorithm: str
    seed: int


def run(environment, start, goal, lower, upper, algorithm="baseline", seed=0, config=None):
    config = config or PlannerConfig()
    inertia(algorithm, 0, config.iterations)
    start, goal, lower, upper = map(vector, (start, goal, lower, upper))
    if np.any(lower >= upper) or np.any(start < lower) or np.any(start > upper) or np.any(goal < lower) or np.any(goal > upper):
        raise ValueError("Invalid bounds/endpoints")
    if np.linalg.norm(goal - start) < 1e-9:
        raise ValueError("Start and Goal must differ")
    rng = np.random.default_rng(seed)
    straight = np.linspace(start, goal, config.n_waypoints + 2)[1:-1]
    positions = np.clip(straight + rng.uniform(-config.jitter, config.jitter,
                        size=(config.swarm, config.n_waypoints, 3)), lower, upper)
    velocities = np.zeros_like(positions)

    def evaluate(mids):
        paths = complete(mids, start, goal)
        distance = np.linalg.norm(np.diff(paths, axis=1), axis=-1).sum(axis=1)
        collided = environment.collision_mask(paths)
        return distance + config.penalty * collided.astype(float), distance, collided

    pbest = positions.copy()
    best_values, _, _ = evaluate(positions)
    best_id = int(np.argmin(best_values))
    gbest, gbest_f = pbest[best_id].copy(), float(best_values[best_id])
    history = [gbest_f]
    for t in range(config.iterations):
        w = inertia(algorithm, t, config.iterations)
        r1, r2 = rng.random(size=positions.shape), rng.random(size=positions.shape)
        velocities = (w * velocities + config.c1 * r1 * (pbest - positions)
                      + config.c2 * r2 * (gbest[None, :, :] - positions))
        positions = np.clip(positions + velocities, lower, upper)
        scores, _, _ = evaluate(positions)
        improved = scores < best_values
        pbest[improved], best_values[improved] = positions[improved], scores[improved]
        current = int(np.argmin(best_values))
        if best_values[current] < gbest_f:
            gbest_f, gbest = float(best_values[current]), pbest[current].copy()
        history.append(gbest_f)
    final, length, collision = evaluate(gbest)
    convergence = next(i for i, v in enumerate(history) if abs(v - history[-1]) <= 1e-8)
    # Matches original accounting: initial swarm + each update; final validation
    # is excluded in both runners, although it evaluates one additional path.
    return Plan(complete(gbest, start, goal)[0], gbest, float(final[0]), float(length[0]),
                bool(collision[0]), np.asarray(history), convergence,
                config.swarm * (config.iterations + 1), algorithm, seed)


def spline_polyline(path, samples=200):
    """Optional cubic sampling. Only the resulting straight segments are exported.

    A collision certificate for this polyline does NOT certify the continuous
    underlying cubic. The follower must not reconstruct the cubic.
    """
    path = np.asarray(path, float)
    if path.ndim != 2 or path.shape[1] != 3 or not np.isfinite(path).all():
        raise ValueError("Expected finite (N,3) path")
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-10]]
    if len(path) < 4 or samples < 4:
        raise ValueError("Spline needs >=4 distinct points/samples")
    knots = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    # Solve the clamped cubic second-derivative system with zero endpoint
    # tangents. NumPy only: ROS system Python need not load SciPy binary wheels.
    h = np.diff(knots)
    slopes = np.diff(path, axis=0) / h[:, None]
    matrix = np.zeros((len(path), len(path)))
    rhs = np.zeros_like(path)
    matrix[0, :2] = [2*h[0], h[0]]
    rhs[0] = 6*slopes[0]
    matrix[-1, -2:] = [h[-1], 2*h[-1]]
    rhs[-1] = -6*slopes[-1]
    for i in range(1, len(path)-1):
        matrix[i, i-1:i+2] = [h[i-1], 2*(h[i-1]+h[i]), h[i]]
        rhs[i] = 6*(slopes[i]-slopes[i-1])
    second = np.linalg.solve(matrix, rhs)
    locations = np.linspace(0, knots[-1], samples)
    indices = np.clip(np.searchsorted(knots, locations, side="right")-1, 0, len(path)-2)
    a = ((knots[indices+1]-locations)/h[indices])[:, None]
    b = 1-a
    out = (a*path[indices] + b*path[indices+1]
           + ((a**3-a)*second[indices] + (b**3-b)*second[indices+1])*h[indices, None]**2/6)
    out[0], out[-1] = path[0], path[-1]
    return out
