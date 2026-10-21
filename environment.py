"""LUT §3.2: 100^3 environment, axis-aligned cubic obstacles, slab collision checks.

The drone is a dimensionless point exactly as in the thesis's geometric model.
Additional safety inflation is NOT applied in the primary reproduction.
"""

# Tạo môi trường 3D, kiểm tra va chạm với hình lập phương

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Cube:
    center: tuple[float, float, float]
    size: float

    @property
    def lower(self):
        return np.asarray(self.center, dtype=float) - self.size / 2

    @property
    def upper(self):
        return np.asarray(self.center, dtype=float) + self.size / 2


class Environment:
    def __init__(self, cubes=(), bound=100.0):
        self.cubes = tuple(cubes)
        self.bound = float(bound)
        self.low = np.array([cube.lower for cube in cubes], dtype=float).reshape(-1, 3)
        self.high = np.array([cube.upper for cube in cubes], dtype=float).reshape(-1, 3)

    def segment_hits_cube(self, a, b, cube):
        """Exact finite segment–AABB intersection, including tangency, slab method."""
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        if a.shape != (3,) or b.shape != (3,):
            raise ValueError('Expected 3D points')
        d = b - a
        tnear, tfar = 0.0, 1.0
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if a[k] < cube.lower[k] or a[k] > cube.upper[k]:
                    return False
                continue
            t1 = (cube.lower[k] - a[k]) / d[k]
            t2 = (cube.upper[k] - a[k]) / d[k]
            tnear = max(tnear, min(t1, t2))
            tfar = min(tfar, max(t1, t2))
            if tnear > tfar:
                return False
        return tnear <= tfar

    def collision_mask(self, points_batch):
        """Return boolean for each polyline, vectorized across segments/obstacles.

        points_batch shape (B, M, 3) or (M, 3).
        """
        points_batch = np.asarray(points_batch, dtype=float)
        if points_batch.ndim == 2:
            points_batch = points_batch[None, :, :]
        if points_batch.ndim != 3 or points_batch.shape[2] != 3 or points_batch.shape[1] < 2:
            raise ValueError('Paths must have shape (batch, >=2, 3)')
        if not np.isfinite(points_batch).all():
            raise ValueError('Non-finite waypoints are not allowed')
        outside = np.any((points_batch < 0.0) | (points_batch > self.bound), axis=(1, 2))
        if not self.cubes:
            return outside
        start = points_batch[:, :-1, None, :]  # (B,S,1,3)
        direction = (points_batch[:, 1:, None, :] - start)
        mins = self.low[None, None, :, :]
        maxs = self.high[None, None, :, :]
        # Parallel segments: a slab contains the point -> unrestricted interval.
        parallel = np.abs(direction) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            d_safe = np.where(parallel, 1.0, direction)
            t1 = (mins - start) / d_safe
            t2 = (maxs - start) / d_safe
        in_slab = (start >= mins) & (start <= maxs)
        lower = np.where(parallel, np.where(in_slab, -np.inf, np.inf), np.minimum(t1, t2))
        upper = np.where(parallel, np.where(in_slab, np.inf, -np.inf), np.maximum(t1, t2))
        t_min = np.maximum(0.0, np.max(lower, axis=-1))
        t_max = np.minimum(1.0, np.min(upper, axis=-1))
        hits = (t_min <= t_max).any(axis=(1, 2))
        return outside | hits

    def collides(self, path):
        return bool(self.collision_mask(path)[0])
