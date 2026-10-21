# PSO warehouse integration (C1)

This module is an independent warehouse integration, **not a new LUT reproduction
result**. It does not modify or import `export_flight_path.py`. Original PSO
sources, results, SDF, PX4 and ROS workspaces remain unchanged.

## What is implemented

* `geometry.py`: approved warehouse SDF reader, world AABBs, floor and enclosing
  XY bounds. Rejects unknown collision shapes/includes, moving/nested models,
  explicit frames and non-horizontal floor. Rotated static boxes are enclosed
  conservatively. A radius of **0.60 m + margin 0.25 m** inflates each obstacle;
  world boundaries are inset by the same amount. Segment/slab tests include
  tangencies; finite endpoints alone are insufficient.
* `planner.py`: configurable Start/Goal, per-axis bounds, same particle updates,
  zero velocity initialization, RNG ordering, pbest/gbest strict comparisons,
  inertia and distance-plus-binary-penalty fitness as `pso_core.py`. Defaults:
  100 particles, 200 updates, 15 intermediate points, c1=c2=2, penalty=10000.
  Baseline w=0.7; enhanced w(t)=0.9-0.5*t/T for t=0..T-1 (last w=0.4025 for
  T=200). Inflation is a separate warehouse geometry adaptation; penalty/jitter
  are explicit configuration, and warehouse units are metres.
* `frames.py`: measured ENU/local-NED anchor; timestamp separation <=20 ms.
  Watches XY/Z, velocity, heading reset counters and reference timestamp. Any
  change requires abort, a new measured anchor and revalidation. Spawn is never
  used as a measurement. Heading accuracy/initialization is not assumed from
  position alone.
* `artifacts.py`: CSV/manifest bundle created with exclusive mode in a **new**
  directory under `pso_flight_demo/outputs/`. Never overwrites existing names.
  Stores algorithm, seed, Start/Goal, bounds, frame, SDF hash, CSV hash, fitness,
  config, safety policy and collision results. Loader repeats geometry checks.
  Hashes detect accidental changes; manifests are not cryptographic signatures.
* `safety.py`, `ros_follower.py`: offline-testable safety/command gates and an
  opt-in ROS 2 position follower. Default CLI dry-run is offline and initializes
  **no ROS node and no publishers**. The live implementation has **no Arm,
  Takeoff or force-disarm command**; it requires an approved native takeoff to
  an already-armed hover first.
* `__main__.py`: geometry, read-only telemetry capture, new route generation,
  and default offline dry-run entry points.

## Coordinate contract

World frame is ENU with spherical heading zero. All route CSVs contain
`x_enu_m,y_enu_m,z_enu_m`: absolute world model-origin positions in metres.
They are **not PX4 local positions**.

For a simultaneous measured pair g0 (Gazebo ENU) and n0 (EKF local NED):

```
n = n0 + [gY-g0Y, gX-g0X, -(gZ-g0Z)]
g = g0 + [nE-n0E, nN-n0N, -(nD-n0D)]
yaw_NED = wrap(pi/2 - yaw_ENU)
```

`capture` reads PX4 ground truth (converted back to Gazebo ENU) and EKF local
position. Both are model-origin positions in the current GZBridge. It requires
Landed, Disarmed and valid local position. No GNSS latitude/longitude or spawn
approximation is used. The source label is provenance, not protection against
someone manually fabricating an anchor: only genuine native captures are valid.

Flight lower/upper Z arguments are **absolute ENU origin heights**. Relative
demo height is measured from captured landed origin, which may be slightly
below floor Z. Do not substitute floor height or the include spawn Z. Enforced
relative route cap is 3.2 m. The floor plus inflated sphere restricts cruise;
native takeoff/landing ground contact is intentionally outside this cruise
certificate. No ceiling is inferred from wall tops.

## Read-only and dry-run workflow (no flight approval implied)

From the project root, use `PYTHONDONTWRITEBYTECODE=1` to avoid bytecode writes
outside this module when comparison tests import the original sources.

1. Inspect geometry (no simulator/network connection):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo geometry
```

2. On Ubuntu host, with Gazebo still Paused, PX4 Disarmed and Landed, capture
   a **new measured anchor**. This command reads native topics and writes only
   the new anchor bundle; it sends no flight or simulator-control command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo capture --name c1_anchor_01
```

3. Prepare the fixed profile: current landed XY -> 2.5 m relative hover ->
   0.5 m East -> return. The following 1.0..3.2 m absolute cruise bounds are
   appropriate only for the inspected floor-at-zero SDF. Actual waypoints are
   generated from the captured pose and must pass full geometry checks:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo waypoint \
  --name c1_waypoint_01 \
  --anchor pso_flight_demo/outputs/c1_anchor_01/anchor.json \
  --z-min 1.0 --z-max 3.2
```

4. Offline dry-run (does not contact ROS or PX4):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo follow \
  pso_flight_demo/outputs/c1_waypoint_01
```

Expected result: `OFFLINE_VALIDATED_ONLY`, `live_readiness=NOT_CHECKED`,
`publishes_commands=false`. This is **not** flight authorization, live telemetry
verification, a Gazebo dynamics test or proof of DDS connectivity.

Names above are examples of new artifact names, not permission to overwrite
them. If a name exists, choose another; never delete or reuse previous results.
No real anchor or flight route is bundled with the source.

## Standard/Enhanced planning API

```python
from pso_flight_demo.geometry import load_world
from pso_flight_demo.planner import PlannerConfig, run

world = load_world(approved_sdf_path)
env = world.environment(z_min, z_max)  # explicitly approved cruise heights
cfg = PlannerConfig(jitter=initialization_jitter_metres)
result = run(env, measured_validated_start, approved_goal,
             env.lower, env.upper, algorithm="baseline", seed=seed, config=cfg)
env.validate(result.path)  # a finite penalty is NOT a feasibility guarantee
```

Repeat with `algorithm="enhanced"` and equal seeds, geometry and budgets. CLI
`plan --help` describes required explicit inputs; there is no default Goal or
warehouse route. `--start` means the airborne route start, not the ground pose.
Use `export_route` or CLI `plan` to create separate new route bundles. `--smooth`
samples a clamped cubic using NumPy only and **revalidates every output straight
segment**. The exported execution path is a polyline; the follower must never
reconstruct an unchecked continuous spline. Smoothing can fail validation,
which must not trigger an automatic retry, safety reduction or fallback export.

Current policy allows flying above 2 m walls if envelope clearance and height
limits permit. An experiment claiming corridor avoidance must explicitly forbid
overflight in its planning scenario. Start/Goal, height policy, margin, speed
and planner evaluation budget must be equal when comparing algorithms.

## Live follower contract — NOT RUN OR FLIGHT-TESTED IN C1

Live entry requires **both** `--execute` and `--confirm-live CONTROL_APPROVED`.
Do not supply them until a separate live/flight approval. No launch command for
automatic arming or takeoff exists here.

Before approval, verify Agent UDP endpoint, DDS domain/namespace, timesync and
Offboard-loss parameters from runtime without changing them. Source ROS Jazzy
and the existing `ros2_px4_ws/install/setup.bash`. The installed client/agent
and message compatibility must be checked on the actual host. This module does
not start an Agent, daemon, bridge or logger or modify any parameter.

Live policy:

* Fixed sysid/component 1/1, measured anchor, matching SDF, active GCS and valid
  estimator. Begin within 0.15 m of route start in an already-airborne native
  hover. No mode command is sent if the initial gate fails.
* Version-derived topic suffixes, BEST_EFFORT volatile telemetry subscriptions,
  reliable publishers, 20 Hz heartbeat/position setpoints, 1.2 simulated seconds
  prestream. ACK **and** actual Offboard mode must agree within 3 wall seconds.
  Commands are sent once, without automatic retry.
* Wall-clock ROS node + automatic DDS time synchronization; assumes RTF near 1.
  Message receipt/source progression must both remain fresh within 1.5 wall
  seconds. Pause/frozen clock is treated as a stop condition, not continued GO.
* Position-only targets have NaN velocity/acceleration feedforward. Target speed
  <=0.2 m/s, actual speed <=0.7 m/s, tracking error <=0.25 m, tilt <=15 degrees,
  relative height <=3.2 m; cruise position remains inside validated geometry.
* Stop at each waypoint, require 5 simulated seconds within 0.10 m before the
  next segment. The entire route has a conservative 120 simulated-second limit.
  Long PSO routes may need a separately reviewed execution profile.
* Completion or a recoverable tracking/mode error requests native Land once.
  LAND ACK and mode must confirm; waits up to 30 wall seconds for Landed and
  Disarmed. Unsafe/stale estimator, reset, rejected Land or exceptions request
  Gazebo Pause. Failed Pause prints an explicit manual-GUI-Pause instruction.
  No force-disarm. Final landed/disarmed state requests Pause as well.

Limitations: ACK/mode and safety logic have offline unit coverage, but no live
ROS/DDS, Gazebo timing, motor dynamics or landing integration validation. No
independent live Gazebo pose monitor is implemented; external observation is
mandatory for the first flight to detect estimator-vs-world disagreement. DDS
estimator fault flags are checked, but the full `estimator_status.filter_fault_flags`
topic is not in the default DDS map: keep the native EKF observer available.
Heartbeat publication is not proof that middleware delivered it. Python process
crash, SIGKILL or a failed Gazebo service cannot be guaranteed to execute cleanup;
PX4's existing Offboard-loss failsafe and an operator with GUI Pause remain
required. Dynamic obstacles, mesh collision, arbitrary SDF frames, PX4 estimator
frame resets during a route and autonomous takeoff are not supported.

## Verification

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s pso_flight_demo/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.static_checks
```

Tests use temporary files **inside this module**, never existing result CSVs.
They include bit-for-bit original/new PSO comparisons for both algorithms with
full default budget on sparse and a second dense/seed/budget combination,
warehouse planning fixtures, geometry, frame/reset/freshness/ACK safety and
no-overwrite/dry-run protections. Synthetic test anchors are never flight data.
