# Example weekly report — LUT 3D PSO method-level recreation

**Important:** This report describes the *included sample run* of the independently implemented code, not results executed on the student's ThinkPad. Replace the numbers with your local `results/recreation_local/summary.csv` before submitting it as your own experiments.

## Research objective
Recreate in Python the 3D method reported by Tasnuba Nourin Oyshe, *Path Planning-Based Drone Delivery System* (LUT, 2026, Chapters 3–4). Compare constant-inertia baseline PSO against linearly decreasing inertia PSO using matched 3D environments, and identify repeatable failure modes prior to new algorithm design.

## Method and software validation
The recreation uses a 100×100×100 point-robot workspace and cube obstacles with an axis-aligned bounding box (AABB) representation. Collision tests use a segment–AABB slab intersection routine. Paths comprise 15 adjustable 3D waypoints (45 optimization dimensions); endpoints are fixed at (10,10,10) and (90,90,90). The objective is raw polyline length plus a 10,000 penalty when any segment intersects a cube or violates the workspace boundary. Baseline inertia is 0.7 and enhanced inertia decreases according to the publication's equation 14, with c1=c2=2. Each run uses 100 particles for 200 updates. Ten trials were performed for each of three cases, totaling 60 optimizations. Ten unit tests passed.

## Example observed results from the packaged run

| Environment | Algorithm | Mean spline length | Mean total turn angle (rad) | Raw collision rate | Spline collision rate |
|---|---|---:|---:|---:|---:|
| Sparse | Baseline | 184.97 | 16.02 | 0% | 30% |
| Sparse | Enhanced | 163.53 | 10.51 | 0% | 40% |
| Moderate | Baseline | 141.70 | 6.07 | 100% | 100% |
| Moderate | Enhanced | 139.28 | 2.41 | 100% | 100% |
| Dense | Baseline | 158.66 | 10.69 | 70% | 70% |
| Dense | Enhanced | 188.28 | 14.59 | 10% | 60% |

All length and turn-angle means in this table follow the thesis-style aggregation **including unsuccessful paths**, so they are not automatically comparable as usable route quality. For feasible-only length results, use `feasible_smoothed_length_mean` from `summary.csv`. The spline-collision column is an extra independent check not included in thesis Table 1.

## Comparison to published reference
The publication reports Moderate raw collision rate 100% for both algorithms; the recreation sample exhibits the same failure (10/10 runs each). Published Dense raw collision rates are 50% for baseline and 10% for enhanced; this sample produced 70% and 10%. Published Sparse raw collision rates are both 0%, which the sample matched. Individual mean path lengths and smoothness are not all identical; the source paper does not specify the original random seeds, detailed velocity initialization, or a fully unambiguous smoothing routine. The code therefore supports a **method-level** reconstruction rather than an exact source-code or numerical replay.

## Limitations and proposed research hypotheses
1. **Central obstacle trapping:** In Moderate, both configurations remained infeasible; this is consistent with the thesis's discussion of straight-line initialization near the central obstacle (page 31). A future experiment could seed a portion of the swarm with collision-free corridors while holding PSO parameters fixed.
2. **Smoothing-induced collisions:** For Dense/Enhanced, raw collision rate was 10% while 60% of sampled spline trajectories intersected cubes. Smoothing must be checked and repaired before a path can be considered operationally safe. A collision-aware interpolation or repair stage is a separate proposed modification.
3. **Binary penalty plateau:** The publication's fitness assigns the same 10,000 penalty to any colliding path regardless of penetration depth. The possibility that this causes poor guidance is a testable mechanism; controlled experiments with a graded collision cost are needed before claiming causality.

## Next steps
Re-run the exact parameter set locally, compare `thesis_comparison.csv` to published Table 1, check per-seed failures, and preserve this baseline unchanged before testing individual initialization, penalty and smoothing changes. Adaptive PSO and Edge AI control should be explored after these controlled experiments.
