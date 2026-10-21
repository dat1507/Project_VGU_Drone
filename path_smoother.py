"""LUT §3.6: executable reading of Eq.(15)-(16), plus a distinct true Catmull-Rom option.

IMPORTANT PAPER AMBIGUITY: Paper calls it Catmull-Rom but specifies
scipy.interpolate.make_interp_spline(...,k=3,bc_type='clamped'), which is a
clamped cubic spline, not a Catmull-Rom implementation. It also asks to
pad with duplicate endpoints AND use chord-length knots; those knots repeat,
which SciPy rejects. The default uses clamped cubic interpolation on the
nonduplicated, consecutive-distinct nodes: executable interpretation of Eq16.
The optional catmull_rom variant is a separate sensitivity check, NOT claimed
as original LUT code. Both output exactly 200 samples and preserve endpoints.
"""

# Làm mượt đường bằng spline, đo chiều dài và góc rẽ

import numpy as np
from scipy.interpolate import make_interp_spline


def unique_consecutive(points, eps=1e-10):
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError('Expected (M,3) coordinates')
    if len(pts) < 2:
        raise ValueError('At least two points required')
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > eps]
    return pts[keep]


def smooth_path(points, samples=200, method='thesis_spline'):
    points = unique_consecutive(points)
    if samples < 2:
        raise ValueError('samples >= 2 required')
    if len(points) < 2:
        return np.repeat(points[:1], samples, axis=0)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    u = np.r_[0.0, np.cumsum(distances)]
    locations = np.linspace(u[0], u[-1], samples)
    if method == 'thesis_spline':
        if len(points) < 4:
            result = np.column_stack([np.interp(locations, u, points[:, k]) for k in range(3)])
        else:
            result = make_interp_spline(u, points, k=3, bc_type='clamped')(locations)
    elif method == 'catmull_rom':
        # Chord-length cubic Hermite with central-difference derivatives; a
        # Catmull-Rom-family interpolation, not SciPy's clamped cubic spline.
        slopes = np.empty_like(points)
        slopes[0] = (points[1]-points[0])/(u[1]-u[0])
        slopes[-1] = (points[-1]-points[-2])/(u[-1]-u[-2])
        slopes[1:-1] = ((points[2:]-points[:-2]) /
                         (u[2:, None]-u[:-2, None]))
        idx = np.clip(np.searchsorted(u, locations, side='right')-1, 0, len(u)-2)
        h = u[idx+1]-u[idx]
        t = (locations-u[idx])/h
        tt = t[:, None]
        h00 = 2*tt**3 - 3*tt**2 + 1
        h10 = tt**3 - 2*tt**2 + tt
        h01 = -2*tt**3 + 3*tt**2
        h11 = tt**3 - tt**2
        result = (h00*points[idx] + h10*h[:, None]*slopes[idx]
                  + h01*points[idx+1] + h11*h[:, None]*slopes[idx+1])
    else:
        raise ValueError('method must be thesis_spline or catmull_rom')
    result[0] = points[0]
    result[-1] = points[-1]
    return result


def path_length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def turn_angle_sum(points):
    """LUT eq.18-19: sum arccos of successive segment direction dot products (rad)."""
    segments = np.diff(np.asarray(points), axis=0)
    norms = np.linalg.norm(segments, axis=1)
    denom = norms[:-1] * norms[1:]
    good = denom > 1e-12
    cos = np.ones_like(denom)
    cos[good] = np.einsum('ij,ij->i', segments[:-1][good],
                          segments[1:][good]) / denom[good]
    return float(np.arccos(np.clip(cos, -1, 1)).sum())
