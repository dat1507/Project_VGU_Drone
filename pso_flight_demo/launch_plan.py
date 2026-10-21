"""Reboot startup planner: returns commands only; no launch implementation."""

import os
from pathlib import Path
import shlex
import shutil

from .environments import PROJECT, load_environment
from .startup_checks import expected_ports, host_startup_check


def launch_plan(profile, with_dds=True, profile_source=None, check_host=False):
    world, _ = load_environment(profile)
    selector = str(profile_source or profile.id)
    root = Path(profile.px4_root).expanduser()
    build = root / "build/px4_sitl_default"
    binary, rootfs = build/"bin/px4", build/"rootfs"
    if not binary.is_file() or not rootfs.is_dir() or not os.access(binary, os.X_OK):
        raise ValueError("PX4 build/rootfs missing; launcher never builds or installs it")
    models = root/"Tools/simulation/gz/models"
    if any(not (models/name/"model.sdf").is_file() for name in ("x500", "x500_base")):
        raise ValueError("Reviewed x500/x500_base model resources are missing")
    ros = Path(profile.ros_setup).expanduser()
    overlay = Path(profile.px4_ros_setup).expanduser()
    workspace_agent = overlay.parent/"microxrcedds_agent/bin/MicroXRCEAgent"
    agent = str(workspace_agent) if os.access(workspace_agent, os.X_OK) else shutil.which("MicroXRCEAgent")
    if with_dds and not agent:
        raise ValueError("MicroXRCEAgent is not on PATH; no automatic install")
    if with_dds and (not ros.is_file() or not overlay.is_file()):
        raise ValueError("Selected ROS/px4_msgs setup scripts are missing")
    resource_paths = list(dict.fromkeys([str(root/"Tools/simulation/gz/models"),
                                        str(Path(world.sdf_path).parent)] +
                                       [p for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if p]))
    inspect = ["python3", "-B", "-m", "pso_flight_demo.c2", "--profile", selector]
    host_command = ["host-check"] + (["--observe-dds", "3"] if with_dds else [])
    ros_prefix = (f"source {shlex.quote(str(ros))} && source {shlex.quote(str(overlay))} && "
                  if with_dds else "")
    commands = [
        dict(terminal="PRECHECK", guard="No existing PX4/Gazebo server before new session",
             command="cd "+shlex.quote(str(PROJECT))+" && "+
             shlex.join(inspect+["launch", "--check-host"]+([] if with_dds else ["--without-dds"]))),
        dict(terminal="GAZEBO", guard="Exactly one world process; SDF already includes exactly one configured vehicle",
             command=shlex.join(["env", "GZ_SIM_RESOURCE_PATH="+":".join(resource_paths),
                                 "gz", "sim", "-v", "4", world.sdf_path])),
    ]
    if with_dds:
        commands.append(dict(terminal="AGENT", guard=f"No existing Agent or listener on UDP {profile.agent_port}",
                             command=shlex.join([agent, "udp4", "-p", str(profile.agent_port)])))
    commands += [
        dict(terminal="PX4", guard="No existing px4 process; attach to the included model, never spawn another",
             command="cd "+shlex.quote(str(rootfs))+" && "+
             shlex.join(["env", "-u", "PX4_HOME_LAT", "-u", "PX4_HOME_LON", "-u", "PX4_HOME_ALT",
                         f"PX4_SYS_AUTOSTART={profile.airframe}", "PX4_SIMULATOR=gz",
                         "PX4_GZ_STANDALONE=1", f"PX4_GZ_WORLD={world.name}",
                         f"PX4_GZ_MODEL_NAME={profile.vehicle_name}",
                         f"PX4_UXRCE_DDS_NS={profile.namespace}",
                         f"PX4_UXRCE_DDS_PORT={profile.agent_port}",
                         f"ROS_DOMAIN_ID={profile.dds_domain}", str(binary), "-d",
                         "-i", str(profile.instance), "-w", str(rootfs), "-s", "etc/init.d-posix/rcS"])),
    ]
    commands += [
        dict(terminal="QGC_OPTIONAL", guard="Open once only, after checking existing QGC",
             command=shlex.quote(str(Path(profile.qgc).expanduser()))),
        dict(terminal="READINESS", guard="Read-only; no automatic physics warmup",
             command="cd "+shlex.quote(str(PROJECT))+" && "+
             ros_prefix+"PYTHONDONTWRITEBYTECODE=1 "+
             shlex.join(inspect+host_command)),
    ]
    warnings = ["Do not paste startup commands into the currently running session.",
                "Read-only host checks are not atomic locks or proof of flight readiness.",
                "Keep the same GZ_PARTITION in Gazebo and PX4 shells; do not invent a GZ_IP.",
                "Verify COM_ARM_ON_BOOT=0 before any approved clock warmup; do not change it automatically."]
    if not with_dds:
        warnings.append("Native-only omits Agent/observer, but PX4 rcS still starts its DDS client; "
                        "this plan does not disable it or adopt an existing Agent.")
    for key in ("GZ_IP", "GZ_PARTITION", "GZ_DISCOVERY_MSG_PORT", "GZ_DISCOVERY_SRV_PORT", "PX4_NET_INTERFACE"):
        if os.environ.get(key):
            warnings.append(f"Inherited {key}={os.environ[key]} must be reviewed on host; not changed")
    if not Path(profile.qgc).expanduser().is_file():
        warnings.append("QGC AppImage path is missing; opening GUI remains manual")
    result = dict(mode="PRINT_COMMANDS_ONLY", launched_processes=0, publishes_commands=False,
                  auto_arm=False, auto_takeoff=False, auto_unpause=False, auto_follower=False,
                  world=world.name, vehicle=profile.vehicle_name, sdf_sha256=world.sdf_sha256,
                  with_dds=with_dds,
                  startup_order=[c["terminal"] for c in commands], ports=expected_ports(profile),
                  qgc_policy="manual GUI open once; reuse existing QGC only after checking its UDP 14550 owner",
                  commands=commands, warnings=warnings,
                  readiness_phases=["FILES_AND_CONFLICT_CHECK", "WORLD_PAUSED", "PX4_ATTACHED",
                                    "WAIT_FOR_EXPLICIT_CLOCK_WARMUP", "INITIALIZED_PAUSED",
                                    "NEW_ANCHOR_REQUIRED", "READY_FOR_SEPARATE_FLIGHT_REVIEW"],
                  singleton_enforcement="This print-only entry point holds no locks; session_launcher --execute "
                                        "implements foreground lifecycle/instance flock locks",
                  important="After reboot physics/EKF start from a new session. Paused zero-time startup "
                            "cannot initialize sensors/EKF; a separate approved warmup is required. "
                            "Never replay old anchors or claim iteration/estimator restoration.")
    if check_host:
        result["host_precheck"] = host_startup_check(profile, with_dds)
    return result


def shutdown_plan(profile):
    """Describe future manual shutdown; never run the commands."""
    if not profile.world_name:
        raise ValueError("Select a configured world profile before planning shutdown")
    bin_dir = Path(profile.px4_root).expanduser()/"build/px4_sitl_default/bin"
    instance = str(profile.instance)
    return dict(mode="SHUTDOWN_PLAN_ONLY", stopped_processes=0, publishes_commands=False,
                target_world=profile.world_name, target_instance=profile.instance,
                session_identity_required_for_future_execution=True,
                automatic_kill=False, automatic_physics=False,
                prerequisites=["Landed=True", "Armed=False", "No active Offboard follower"],
                steps=[
                    dict(action="Inspect landed/disarmed and logger before shutdown",
                         commands=[shlex.join([str(bin_dir/"px4-listener"), "--instance", instance, topic, "-n", "1"])
                                   for topic in ("vehicle_land_detected", "actuator_armed")] +
                                  [shlex.join([str(bin_dir/"px4-logger"), "--instance", instance, "status"])]),
                    dict(action="Request PX4 graceful shutdown only after explicit shutdown approval",
                         command=shlex.join([str(bin_dir/"px4-shutdown"), "--instance", instance]),
                         caveat="Request accepted is not process exit. Work queue/timers/logger may wait "
                                "for simulation clock. If already Paused, do not unpause automatically."),
                    dict(action="Wait for owned PX4 process exit and logger closure; no force-kill/retry"),
                    dict(action="Close QGC using its GUI; stop only the owned Agent/observer in their terminals"),
                    dict(action="Close the owned Gazebo GUI/server last; preserve logs and configuration"),
                    dict(action="Verify processes and ports released before starting another world",
                         command="ps -eo pid,comm,args"),
                ],
                paused_shutdown_policy="If a graceful request stalls while clock is Paused, stop and "
                                       "request a separately approved disarmed clock/cleanup procedure.",
                future_executor_policy="Only session-owned PID/process-group with matching start identity; "
                                       "never global killall, pkill, stale-PID signalling or automatic SIGKILL.")
