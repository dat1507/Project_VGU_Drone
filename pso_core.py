"""Thesis §3.3-3.5: 45-dim PSO with 15 intermediate waypoints.

Ambiguities not fixed by the thesis: zero initialization of velocity and independent
NumPy RNG stream by seed. These conventions are explicit in README_VI.md.
"""

# Công thức cập nhật PSO, fitness, waypoint

from dataclasses import dataclass
from time import perf_counter
import numpy as np
from scenarios import START, GOAL

N_WAYPOINTS = 15
POPULATION = 100
ITERATIONS = 200
C1 = C2 = 2.0
PENALTY = 10_000.0
JITTER = 2.5


def complete_paths(intermediates):
    """(B,15,3) -> (B,17,3); accepts single (15,3) as well."""
    mids = np.asarray(intermediates, dtype=float)
    if mids.ndim == 2:
        mids = mids[None, :, :]
    if mids.ndim != 3 or mids.shape[-1] != 3:
        raise ValueError('Expected (particles, waypoints, 3)')
    return np.concatenate((np.broadcast_to(START, (len(mids), 1, 3)), mids,
                           np.broadcast_to(GOAL, (len(mids), 1, 3))), axis=1)


def evaluate(mids, environment):
    """Equation (9)-(11): raw polyline Euclidean length + binary Pcol penalty."""
    paths = complete_paths(mids)
    dist = np.linalg.norm(np.diff(paths, axis=1), axis=-1).sum(axis=1)
    collided = environment.collision_mask(paths)
    fitness = dist + PENALTY * collided.astype(float)
    return fitness, dist, collided


@dataclass
class Run:
    positions: np.ndarray
    fitness: float
    raw_length: float
    raw_collision: bool
    best_history: np.ndarray
    convergence_iteration: int
    runtime_seconds: float
    fitness_evaluations: int
    algorithm: str
    seed: int

    @property
    def path(self):
        return complete_paths(self.positions)[0]


def inertia_at(algorithm, iteration, max_iterations):
    if algorithm == 'baseline':
        return 0.7
    if algorithm == 'enhanced':
        return 0.9 - (0.9 - 0.4) * iteration / max_iterations
    raise ValueError(algorithm)


def run_pso(environment, algorithm='baseline', seed=0, swarm=POPULATION,
            iterations=ITERATIONS, n_waypoints=N_WAYPOINTS, jitter=JITTER):
    if algorithm not in {'baseline', 'enhanced'}:
        raise ValueError(algorithm)
    if min(swarm, iterations, n_waypoints) < 1:
        raise ValueError('positive swarm, iterations, n_waypoints needed')
    tic = perf_counter()
    rng = np.random.default_rng(seed)
    straight = np.linspace(START, GOAL, n_waypoints + 2)[1:-1]
    positions = np.clip(straight + rng.uniform(-jitter, jitter,
                           size=(swarm, n_waypoints, 3)), 0.0, environment.bound)
    velocities = np.zeros_like(positions)  # assumption: thesis did not specify initial velocities
    pbest = positions.copy()
    best_values, lengths, collided = evaluate(positions, environment)
    evals = swarm
    best_id = int(np.argmin(best_values))
    gbest = pbest[best_id].copy()
    gbest_f = float(best_values[best_id])
    history = [gbest_f]
    for t in range(iterations):
        # Thesis eq 14: t / T_max with t = 0,...,T_max-1.
        w = inertia_at(algorithm, t, iterations)
        r1 = rng.random(size=positions.shape)
        r2 = rng.random(size=positions.shape)
        velocities = (w * velocities + C1 * r1 * (pbest - positions)
                      + C2 * r2 * (gbest[None, :, :] - positions))
        positions = np.clip(positions + velocities, 0, environment.bound)
        scores, _, _ = evaluate(positions, environment)
        evals += swarm
        improved = scores < best_values
        pbest[improved] = positions[improved]
        best_values[improved] = scores[improved]
        current_id = int(np.argmin(best_values))
        if best_values[current_id] < gbest_f:
            gbest_f = float(best_values[current_id])
            gbest = pbest[current_id].copy()
        history.append(gbest_f)
    final_f, final_len, final_coll = evaluate(gbest, environment)
    # Log history includes iteration 0 initial score, and after 1..T PSO updates.
    conv = next(i for i, val in enumerate(history) if abs(val - history[-1]) <= 1e-8)
    return Run(gbest, float(final_f[0]), float(final_len[0]), bool(final_coll[0]),
               np.asarray(history), conv, perf_counter()-tic, evals, algorithm, seed)
