"""LUT §3.4: baseline PSO, fixed inertia 0.7, c1=c2=2.0."""

# Baseline PSO với \(w=0.7\)

from pso_core import run_pso


def optimize(environment, **kwargs):
    return run_pso(environment, algorithm='baseline', **kwargs)
