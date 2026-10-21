# Project continuation notes

- The old `lut_pso_week1` 2D scaffold has already passed 5 unit tests and completed a local run of 4 environments × 2 control algorithms × 30 seeds at 35 particles and 120 iterations. It remains useful as a simple learning reference but is not faithful to LUT 3D.
- The active research direction is **3D method-level recreation of Tasnuba Nourin Oyshe's 2026 LUT thesis**, then careful validation and characterization of limits before proposing genuinely new Adaptive PSO / Edge AI control mechanisms.
- Thesis method: 100³ workspace; 3D AABB obstacles; start (10,10,10), goal (90,90,90); N=15 intermediate waypoints; population 100; 200 iterations; c1=c2=2; baseline fixed w=0.7; enhanced linearly decaying w=0.9→0.4; path distance + 10,000 collision penalty; SciPy spline postprocess (with documented textual ambiguity); 3 scenarios × 10 seeds.
- The 3D source here is independently written and should never be attributed as code from LUT authors. Important unknown implementation details and smoothing ambiguity are documented in `REPRODUCTION_AUDIT.md`.
- Next tasks: read `results/recreation_local/summary.csv`, compare Table 1 LUT, inspect raw vs spline safety, then carry out controlled experiments. Keep baseline unchanged and branch proposed Adaptive PSO modifications separately.
