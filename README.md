# PSO drone simulation

Plan Standard and Enhanced PSO routes from arbitrary Gazebo world ENU coordinates, then demonstrate them sequentially as visible Gazebo replay or measured PX4 SITL flight. Original LUT research scripts and legacy flight commands remain available. The planner has no dependency on ROS or a flight controller.

Run commands from the repository root. `python3 -m pso_flight_demo doctor` reports the current installation and desktop socket connectivity. Tested with Ubuntu 24.04, Python 3.12, Gazebo Harmonic 8.15, ROS 2 Jazzy and the installed PX4 1.18 development build. The new planner/replay uses NumPy and installed Gazebo `sdformat14`, `transport13`, `msgs10` Python bindings; SciPy is only needed by the original research scripts. The existing `.venv/bin/python` runs those original scripts and tests. The host's system SciPy and user NumPy are incompatible; this application does not import that SciPy.

## Run the visible demonstration

The warehouse contains walls up to 2 m. These verified endpoints are model-origin coordinates in metres, East/North/Up:

```bash
python3 -m pso_flight_demo demo \
  --world "$PWD/simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf" \
  --start -6 -4 3.2 --goal 6 4 3.2 \
  --algorithms both --path auto --mode replay --gui --seed 42
```

Use `--algorithms standard` or `--algorithms enhanced` independently. Standard replay is orange, Enhanced replay is blue. Terminal progress labels each algorithm. `--markers` adds the selected polyline; `--screenshots` captures the Gazebo window through XWayland. `--speed 3 --fps 8 --hold 2` controls interpolation and endpoint holds. Replay includes every original vertex and executes sampled cubic output as straight segments. Its quadrotor is a kinematic visual with no collision geometry; it is intentionally able to pass through walls. It never represents PX4 flight.

Each run starts an owned, isolated Gazebo server, runs one algorithm, shuts it down, and starts a fresh simulation for the next algorithm. Existing Gazebo/PX4/DDS sessions are left running. It does not attach to an arbitrary existing instance. Readiness uses service discovery; every replay pose is confirmed using the simulator pose stream. A persistent request-only client avoids blocking transport callback threads. Missing Physics/UserCommands/SceneBroadcaster plugins are added to a generated runtime copy; original SDF is preserved.

## Plan and compare

```bash
python3 -m pso_flight_demo plan \
  --world worlds/empty.sdf --start 0 0 2 --goal 3 2 3 \
  --algorithms both --path auto --seed 42 --output runs/my_plan
python3 -m pso_flight_demo compare runs/my_plan
```

`--output` must name a new directory. Without it a unique UTC/UUID run directory is created beneath ignored `runs/`. Plan and demo accept the same world, endpoint, seed, particle and collision options. Full help: `python3 -m pso_flight_demo demo --help` or `plan --help`. Legacy `plan --anchor ... --sdf ...` continues to use the original anchor-based interface.

Default comparable budgets are 100 particles, 200 iterations, 15 intermediate waypoints, c1=c2=2, binary collision penalty 10000, and jitter 2.5 m. Standard inertia is 0.7; Enhanced is `0.9 - 0.5*t/T`. Zero velocities, RNG ordering, best-update ordering and clipping match the preserved research implementation exactly; regression tests compare both RNG histories and particle coordinates. No new search heuristic replaces either algorithm. Warehouse bounds, safety inflation and jitter scaling are explicitly adaptations of the research environment. Use `--jitter`, `--swarm`, `--iterations`, `--waypoints` to set an explicit comparable experiment.

`--path auto` prefers a feasible smoothed polyline, then feasible raw, and records failure if neither is feasible. `--path raw` and `--path smoothed` select explicitly and still validate. Raw and smoothed collisions and lengths are always saved independently. With the verified warehouse endpoints and seed 42 at default jitter, Enhanced's raw route is feasible and its spline is not; auto correctly falls back to raw. A sampled-polyline certificate does not certify a reconstructed continuous cubic.

## Collision-tolerant demonstration

This diagnostic deliberately initializes an unchanging straight line through a wall (`--jitter 0`). It is exported as infeasible and replay still completes:

```bash
python3 -m pso_flight_demo demo \
  --world simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf \
  --start -3 -3 1.5 --goal 3 -3 1.5 \
  --algorithms both --jitter 0 --path raw \
  --mode replay --allow-collisions --gui --seed 42
```

PX4 refuses an infeasible route or the collision-tolerant option. `--mode auto --fallback replay` explicitly permits fallback and retains `px4_attempt` diagnostics beside the actual replay result. The default fallback is `none`; a replay never changes `px4_flight` to true.

## PX4 physics flight

The installed PX4 build, X500 models, MicroXRCEAgent and matching workspace `px4_msgs` are reused. The adapter sources the ROS workspaces in its own child process, so the outer terminal need not be sourced. Verified warehouse command:

```bash
python3 -m pso_flight_demo demo \
  --world simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf \
  --start -6 -4 3.2 --goal 6 4 3.2 \
  --algorithms both --jitter .4 --path raw \
  --mode px4 --gui --screenshots --px4-timeout 130 --seed 42
```

Jitter .4 is an explicit flight demonstration parameter; both algorithms still use the same full computational budget. It produces shorter routes than the default 2.5 m initialization. A separate fully measured Enhanced flight was also verified in the empty world with default jitter, A=(0,0,2), B=(2,1,2).

The adapter adds sensor plugins and a navigation reference only to `sitl_world.sdf`, removes the passive source X500 from that copy, and spawns its own physics X500 at A's XY coordinates. It validates the spawn/takeoff column. It launches its own PX4 executable with a separate writable rootfs, a nonzero instance, loopback GCS heartbeat, an ephemeral DDS agent port, a random DDS domain, and namespace `pso_sitl`. No preflight checks are disabled. All owned PIDs and process exit codes are recorded; cleanup signals only owned process groups.

Readiness requires fresh valid estimator position and odometry, a disarmed/landed vehicle, preflight checks, synchronized DDS timestamps, advancing messages, fresh Gazebo truth and the expected SITL system ID. A measured ground-truth/estimator anchor accounts for ENU→NED orientation and local translation. EKF resets invalidate it. Heartbeat/setpoints precede Offboard request; command acknowledgements and observed mode/arming must agree. Actual position confirms waypoint arrival. Watchdogs catch stalls, timeouts, stale odometry, unexpected landing, mode exit, failsafe and conservative obstacle-envelope intersection. Goal hover precedes accepted landing, landed detection and disarming. Target error and path cross-track error are different metrics; full flight distance includes takeoff/landing, while route distance is reported separately.

`--px4-root` overrides `~/PX4-Autopilot`. Environment overrides: `PSO_PX4_ROOT`, `PSO_ROS_SETUP` (default `/opt/ros/jazzy/setup.bash`) and `PSO_PX4_ROS_SETUP` (default `~/ros2_px4_ws/install/setup.bash`). Gazebo resource search uses `--resource-path` (repeatable), source world directory, `GZ_SIM_RESOURCE_PATH` and the selected PX4 model directory. Headless replay does not need ROS, DDS or PX4 executable startup.

## SDF support and bounds

One named SDF world, zero-heading ENU and downward Z gravity are required. Installed libsdformat resolves nested models, explicit `relative_to` frames, link/collision poses and local included resources. Static boxes, spheres and cylinders become conservative world AABBs; rotations are retained when enclosing transformed geometry. OBJ and ASCII/binary STL collision meshes are enclosed after scale/pose; submesh selection and other mesh formats fail with diagnostics. Horizontal upward planes define an infinite floor. Sloping planes, actors, dynamic obstacles other than the execution X500, heightmaps and unsupported shapes fail closed. Unsupported geometry is never assumed free. Remote Fuel downloading and animated obstacle planning are not implemented.

The warehouse's four walls determine XY limits, inset by radius+margin. Other worlds default to the A/B bounding box plus 5 m on each axis, clamped above the floor. This is a search domain, not a discovered property boundary. Override with `--bounds XMIN YMIN ZMIN XMAX YMAX ZMAX`. Defaults are vehicle radius .60 m and extra margin .25 m. Start/goal must be finite, distinct, within bounds and clear of inflated obstacles; intentional obstacle intersection is allowed only with `--allow-collisions`. Placement through a floor remains invalid. All straight segments use continuous slab intersection, including touching. AABBs can reject valid routes around curved or rotated geometry; diagnostics explicitly record approximations.

## Headless and manual desktop verification

```bash
python3 -m pso_flight_demo demo \
  --world worlds/empty.sdf --start 0 0 2 --goal 3 2 3 \
  --algorithms both --mode replay --headless --seed 42
PSO_GAZEBO_TEST=1 python3 -m unittest pso_flight_demo.tests.test_end_to_end.RealGazeboTests -v
python3 -m unittest discover -s pso_flight_demo/tests -v
.venv/bin/python -m unittest discover -s tests -v
```

`--headless` is also the default if `--gui` is absent. If an agent cannot access desktop sockets, run the same demo command with `--gui` in an unrestricted Ubuntu Desktop terminal. No planner or controller logic changes are required. This session's X11/Wayland socket checks and Gazebo GUI window captures succeeded. GUI uses XWayland (`QT_QPA_PLATFORM=xcb`) because whole-desktop screenshots under Wayland may be black. Headless state measurements establish simulation movement, while actual window captures establish rendering; keep those claims separate.

## Scientific artifacts and evidence

Each run includes arguments, environment doctor output, original SDF/hash, resolved resource hashes, conservative geometry, bounds, seed/configuration, convergence history, raw and smoothed CSVs, selected CSV, fitness, independent collision certificates and lengths, planning time and actual execution status. Replay writes only measured Gazebo pose telemetry. PX4 writes estimator/Gazebo telemetry, origin anchor, command/ACK history, goal error, cross-track metrics and separate actual distances. Simulator, DDS and PX4 logs and GUI screenshots are retained beneath each algorithm directory. Failures stay available in their original directories; results are never overwritten. Relevant successful session evidence: `runs/warehouse_gui_verified`, `runs/empty_both`, `runs/colliding_both`, `runs/px4_attempt2`, `runs/warehouse_px4_gui`. Raw data and screenshots are ignored by Git.

## Hardware boundary

Planning and ENU routes are independent of the flight executor. Future CubeBlack integration should run planning/control on a companion computer and implement a separately reviewed MAVLink/MAVSDK or firmware-supported XRCE transport adapter. Check the actual board firmware's module availability and matching message definitions before selecting ROS 2 DDS. This task neither checked nor changed CubeBlack firmware, MAIN OUT 1–4, Skywalker ESC configuration or the power system. No physical serial link, physical arm or actuator command was used. SITL success establishes no physical readiness. Future ArduPilot execution can implement the same route interface.

Primary references: [SDFormat pose semantics](https://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/), [PX4 ROS 2 Offboard](https://docs.px4.io/main/en/ros2/offboard_control), [PX4 DDS and message matching](https://docs.px4.io/main/en/middleware/uxrce_dds), [PX4 companion computers](https://docs.px4.io/v1.15/en/companion_computer/index).

## Version control

Original source is preserved in baseline commit `6eb66de7a2e9c242bd0ee571c49fb2ba334f6b81`; implementation is on `feature/pso-gazebo-end-to-end`. The designated remote is `https://github.com/dat1507/Project_VGU_Drone.git`. No history was removed or force-pushed. Publication status and exact remaining blockers are recorded in the local acceptance report. Generated investigations, credentials, environments and simulation output remain outside tracked source.
