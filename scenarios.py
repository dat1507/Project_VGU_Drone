"""LUT thesis §4.1.1 (page 27): exact cube centers, side lengths and endpoints."""

# Tạo các kịch bản môi trường 3D với các hình lập phương, xác định điểm bắt đầu và điểm đến
# Khai báo chính xác ba môi trường LUT

import numpy as np
from environment import Cube, Environment

START = np.array((10., 10., 10.))
GOAL = np.array((90., 90., 90.))
BOUND = 100.0
SCENARIOS = {
    'sparse': [(30, 30, 30, 15), (60, 70, 50, 20)],
    'moderate': [(25, 30, 40, 20), (50, 50, 50, 25),
                 (75, 70, 60, 20), (40, 80, 30, 15)],
    'dense': [(20, 25, 30, 15), (35, 40, 50, 20),
              (50, 20, 60, 15), (65, 50, 40, 25),
              (80, 75, 70, 18), (30, 60, 80, 22),
              (55, 85, 20, 15), (70, 15, 45, 20)],
}


def make_environment(name):
    if name not in SCENARIOS:
        raise KeyError(name)
    return Environment([Cube(tuple(row[:3]), row[3]) for row in SCENARIOS[name]], BOUND)
