# C2: runtime, environments and reboot

C2 additions/updates stay inside this module. C1/LUT/PX4/SDF/ROS source and existing outputs are unchanged.
C2 has no live-controller, process-start, kill, restart, arm, takeoff, Land,
unpause or multi-step entry point. The launch command only prints a future
startup plan. Do not execute that plan on the currently running session.

The separate session_launcher entry point now implements an opt-in foreground
executor. Its default remains dry-run; --execute is required for startup or a
shutdown request. Implementation has only been tested offline with fake
children/probes. See REBOOT_GUIDE_VI.md for ownership, paused shutdown, new-anchor
checks and the separately approved first host test. No current process has been
started/stopped and no clock/control action is part of that executor.

## Host measurements and waypoint preparation

The sandbox cannot access PX4 sockets or Gazebo interfaces. Keep Gazebo Paused,
PX4 Disarmed/Landed, and run this single group in Ubuntu Terminal from the
project root:

    source /opt/ros/jazzy/setup.bash
    source /home/phuocdat/ros2_px4_ws/install/setup.bash
    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse host-check --expect-iterations 396120 --prepare-waypoint --observe-dds 3

This does not start an Agent. It performs bounded native read-only probes,
creates a NEW UUID report directory, optionally creates a validated waypoint
bundle, and subscribes/inspects DDS for three seconds. It publishes no control
messages. Normal ROS discovery/metadata traffic is expected. ROS logs stay in
the newly allocated module output directory. No ROS_DOMAIN_ID, DDS, network or
PX4 configuration is changed.

Native checks cover:

* Paused iterations before/after probes; correct world and one vehicle; one
  PX4/Gazebo server; measured pose against ground truth.
* Disarmed/Landed, preflight/ready-to-arm, EKF local/global validity, filter
  faults, sensors, logger and native GCS-heartbeat snapshot.
* Position/sensor sample age <=50 ms of the paused simulation clock; estimator
  status age <=100 ms; Commander/armed/land status age <=1 second. An older
  change-only health_report is preserved, not called fresh.
* Before/after UDP queues and NEW kernel drop difference, separately from uORB
  MAVLink loss counters. Drops can increase while PX4 clock is frozen.
* uXRCE client status, actual UXRCE_DDS*/COM_O*/NAV_DLL_ACT parameters.
* ROS distro/imports; installed px4_msgs wire schema vs source and build message
  versions; DDS topic suffixes, publisher/input-reader graph and received samples.

Each native subprocess has a four-second timeout; unavailable interfaces are
reported and never silently replaced with spawn/historical ULog data.
Full stdout/stderr/exit codes are kept in a new runtime_report.json.

All native gates and geometry must pass before anchor.json and the new route
are written. If native gates fail, no waypoint is created. Collision failure
preserves a NO_GO report. Report and route bundles are distinct NEW UUID
directories under the selected output_root.

The default warehouse route is measured landed XY -> 2.5 m relative cruise ->
0.5 m East -> return. Every cruise segment is checked with radius 0.60 m plus
0.25 m margin. Native takeoff/landing ground contact and dynamics are not covered
by that cruise collision certificate.

Recheck an actual returned route without ROS or network:

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 dry-run BUNDLE_DIRECTORY

Optional --runtime-report NEW_REPORT_PATH compares saved boot/PX4/Gazebo
identity. It compares a supplied report, not a new live query.
OFFLINE_VALIDATED_ONLY and publishes_commands=false do not prove flight
readiness. Native heartbeat flags can remain frozen while QGC times out by wall
clock. DDS graph discovery is not data flow; no new DDS samples during Pause
does not establish a broken network. A running-clock check remains separate.

## Environment configuration and audit

Choose a builtin name or new YAML file with --profile. PyYAML safe_load is used;
the module does not install dependencies.

* profiles/warehouse.yaml: actual approved SDF, floor/world/model identities,
  operating geofence, cruise heights, envelope, output/install paths and DDS
  expectations. It contains no default PSO Start/Goal.
* profiles/sparse_lut_planned.yaml: intentionally planned/unconfigured. No SDF,
  actual world/floor/geofence/heights or Start/Goal is fabricated. Inspect/export
  returns NO_GO until a real SDF and separately reviewed complete profile exist.
  Temporary unit-test worlds are not the anticipated LUT sparse deployment.

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse inspect
    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile sparse_lut_planned inspect

The second command must reject the missing SDF.

For another environment, create a NEW YAML with this schema and state=configured.
CLI overrides exist for SDF, world/floor/vehicle name, namespace and output root.
Bounds, cruise/relative heights, demo offset, radius/margin, Start/Goal, install
paths, instance/airframe and DDS domain/port are configurable in YAML. No original
source changes are needed. SDF paths resolve relative to the project. Output
roots must remain inside this module; existing names and output symlinks fail.

Supported geometry: static flat box collision models, one horizontal plane
floor, one included model://x500 with a configured entity name and the reviewed
x500 envelope. Rotated boxes use conservative enclosing AABBs.
Unknown collision shapes, meshes/cylinders/heightmaps, non-vehicle includes,
nested/dynamic models, explicit frames and nonzero spherical heading fail
closed. Visual meshes with supported box collisions are valid physical geometry.

Physical XY bounds are an explicit operating geofence, not a visual-floor
inference. Cruise bounds are inset by the envelope; heights refer to absolute
ENU model-origin Z. Relative height uses the measured landed origin, not spawn.
No ceiling is inferred from wall height. A planner can fly above 2 m walls if
the selected policy allows it; a corridor-comparison experiment must explicitly
exclude overflight when that is the intended experiment.

Before export, C2 checks SDF/world/floor/spawn/frame and endpoint feasibility,
then every raw and postprocessed segment. Start/Goal are required for PSO export
and must match the actual route. Use plan --help: it requires anchor/report,
algorithm, seed and explicit initialization jitter in metres. Inspect does not
run PSO. Finite penalty never permits exporting a colliding result.

C2 bundles retain the C1 warehouse CSV/schema and add an environment snapshot
and runtime identity. They are marked NOT LUT reproduction results. Warehouse
C1-loader roundtrip and generic C2-world roundtrip have unit coverage.
Hashes detect accidental changes, not malicious fabrication.

| Former fixed value | C2 offline/readiness/launch |
|---|---|
| SDF, world/model/floor names, bounds, spawn | YAML + SDF; anchor is measured |
| Start/Goal | YAML/API/CLI; explicit before PSO export |
| Height, offset, safety radius/margin, height cap | YAML; approved minima enforced |
| Output root | YAML/CLI, module-confined and exclusive |
| Install paths, instance/airframe, namespace/domain/port | YAML |
| Frame | Explicit zero-heading ENU; unsupported frames fail |
| Live follower | Existing C1 remains warehouse/instance-0/sysid-1 specific |

C1's hardcoded live Pause target and some flight limits are NOT patched under
the new-files-only constraint. C2 has no --execute flag. Do not invoke the
C1 live follower in another world. Generalized live control and automated
process lifecycle supervision remain future implementation work.

## Reboot and launcher dry-run

The current authoritative startup/shutdown procedure and future executor design
are in [REBOOT_GUIDE_VI.md](REBOOT_GUIDE_VI.md). No executable launcher has been enabled.

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse launch

Returns PRINT_COMMANDS_ONLY, launched_processes=0. There is no --execute mode.
It checks the selected SDF and installed build/Agent paths; prints singleton
preconditions and separate-terminal commands. Optional launch --check-host
performs read-only process/UDP conflict checks, refuses known sandbox visibility,
and returns nonzero when blocked. It does not reserve ports, acquire lifecycle
locks, start/stop anything or guarantee exclusive ownership.

Future approved order:

1. Check current processes/occupied ports. Refuse duplicate PX4/Gazebo/Agent/QGC.
   The read-only conflict guard is implemented; atomic singleton locks and a
   process supervisor remain future work.
2. Start the selected SDF once WITHOUT -r; SDF already includes one x500.
3. Start the selected Agent first if DDS is needed and its port is free.
4. Start PX4 standalone and attach via PX4_GZ_MODEL_NAME, not model spawning.
   World/model/paths/instance and expected DDS startup environment come from YAML.
   Those environment assignments concern a future new process only.
   Let the user open QGC once. The printed readiness command sources the selected
   ROS setup files before subscriber-only DDS observation.
5. Run host-check without replaying --expect-iterations 396120. Initial
   sensors/EKF/MAVLink/logger may not initialize at paused time zero. Request a
   separately approved physics warmup; the launcher never advances physics.
6. Establish a new initialized paused/disarmed session, take a NEW anchor and
   validate the small waypoint again. Never claim old EKF/time/pose was restored.

Print the shutdown procedure without executing it:

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse shutdown-plan

Native PX4 shutdown uses a work queue, shutdown hooks and HRT time; its logger
hook waits for closure. A request while simulation is Paused may stall. The
shutdown plan never advances physics or force-kills to work around this. See
the reboot guide for the normal running-clock sequence and already-Paused case.

Persisted on disk: source, SDF/config, compiled binaries, CSV/manifests, already
written ULogs and parameters actually saved in PX4 rootfs. Unsaved runtime
parameters are not guaranteed to survive.
Lost: process identities, physics time/pose/velocity, EKF origin/covariance/
alignment, arming/readiness state, MAVLink/DDS sessions/counters/queues.
A route or ULog is not a checkpoint.

C2 records kernel boot_id and PX4/Gazebo PID + process-start ticks. A mismatch
against a NEW report invalidates the anchor even if counters/timestamps repeat.
Direct legacy C1 live entry does not implement this restart guard: do not bypass
the fresh C2 gate.

## First Offboard flight GO checklist

Require all of the following:

* Fresh native evidence: correct world/one vehicle, Disarmed/Landed, sensors,
  logger, EKF validity, no faults/failsafe, preflight and ready-to-arm.
* New synchronized measured anchor, current process identities, unchanged SDF,
  correct ENU/NED axes, validated 0.5 m route and safety envelope.
* Agent/client endpoint/domain/namespace and installed/built schemas match;
  actual DDS data, timesync and Offboard input readers are verified.
* Separately approved DISARMED running-clock observation proves fresh continuous
  telemetry, GCS connection and acceptable new drop/queue trend.
* Runtime Offboard-loss policy reviewed; independent Gazebo pose observer,
  normal Land and operator GUI Pause available.
* Separate explicit approval before native takeoff, mode switch or live follower.

Every C2 report keeps live_control_go=false. No live flight was tested.
