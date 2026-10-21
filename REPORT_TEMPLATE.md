# Weekly progress — LUT 3D PSO Method-Level Recreation

**Tham chiếu:** Tasnuba Nourin Oyshe, *Path Planning-Based Drone Delivery System*, LUT, 2026.  
**Phạm vi:** Tái hiện độc lập (không phải source gốc), đối chiếu, phát hiện giới hạn.

## 1. Objective
Explain recreating the thesis's 3D path planning setup in Python; not a deployment on a flight controller.

## 2. Implementation
- AABB 3D obstacles in [0,100]^3, point-drone geometric model, 15 waypoints.
- Slab collision method, penalty fitness f(P)=D(P)+10000 I(collision).
- Baseline w=0.7; enhanced w(t)=0.9-0.5t/200; 100 particles, 200 iterations, 10 trials per scenario.
- Smoothing: executable interpretation of SciPy clamped cubic in Eq16; disclose knot duplication conflict.

## 3. Software validation
Paste local output of `python -m unittest discover -s tests -v`.

## 4. Experimental setup
Paste command and `config.json`. Attach generated 3D screenshots and seed0 convergence figures; do not treat seed0 as multi-trial statistics.

## 5. Results & comparisons
Paste `results/recreation_local/thesis_comparison.csv` with table comparisons. Distinguish raw collision rate from spline collision rate. Report mean ± sample SD. If methods differ, do not claim exact numerical reproduction.

## 6. Limitations
1. Moderate scenario fails because diagonal initialization intersects central block (as reported by LUT, page 31); explain the evidence from local trials.
2. Spline interpolation sometimes introduces collisions in paths that passed raw geometric checks; show raw vs spline rates and figures.
3. Objective's binary penalty has limited feedback among colliding paths; identify as a testable hypothesis rather than proven causal mechanism.

## 7. Proposed future research
- Guided feasible initialization (A*/RRT* seeded swarm).
- Continuous feasibility information / graded obstacle penalties.
- Smoothing with collision repair.
- Adaptive state-driven parameter control using feasible ratio/stagnation/diversity.

**Claim boundary:** no flight dynamics, PX4, Gazebo, drone radius or physical validation in this reproduction. The code was independently written from the publication.
