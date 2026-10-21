# Preserved research baseline (2026-10-22)

Original source is preserved without algorithm edits. `pso_core.py` uses 15 intermediate 3D waypoints, 100 particles, 200 iterations, zero initial velocity, independent seeded NumPy RNG, Euclidean raw polyline length plus a binary 10000 collision penalty, c1=c2=2. Standard inertia is 0.7; Enhanced inertia decreases from 0.9 toward 0.4 using t/T. Initialization uses straight-line waypoints plus uniform jitter 2.5 m, clipped to bounds. `pso_flight_demo/planner.py` already generalizes this ordering to configurable endpoints and bounds. Spline safety must be evaluated independently.

No existing Git history or nested repositories were found. The empty `.git` sandbox placeholder required approval for initialization. Remote HEAD has no references. Existing author identity is used. Generated results, historical runtime evidence and source snapshots remain on disk, excluded from public source history; original root scripts, package source, profiles, tests, docs and warehouse SDF are retained.

Python 3.12.3; Gazebo Sim 8.15.0; ROS Jazzy; PX4 v1.18.0-beta1-834-gb3e343c153. Both X11 and Wayland socket connection checks pass; xdpyinfo passes. Workspace writes and network access are permitted; .git and host installation writes are restricted. No physical hardware touched.

Original root suite: 10 tests pass in `.venv/bin/python`. System Python has NumPy 2.5.3 with incompatible distribution SciPy; original root suite fails import there. New runtime should avoid requiring SciPy. Existing flight package test output is preserved at `/tmp/pso_baseline_tests.log` during this session. Generated data totals are dominated by the local venv and archived runtime outputs (project total approximately 735 MB).
