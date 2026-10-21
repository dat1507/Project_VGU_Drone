"""LUT §3.5: enhanced PSO, inertia = 0.9 - (0.9-0.4)*t/200."""
# Enhanced PSO với \(w\) thay đổi tuyến tính

from pso_core import run_pso


def optimize(environment, **kwargs):
    return run_pso(environment, algorithm='enhanced', **kwargs)
