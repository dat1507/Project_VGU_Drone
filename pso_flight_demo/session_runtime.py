"""Read-only runtime gates for the session executor. No clock/control commands."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import shlex

from .runtime_readiness import probe, process_identity, topic_fields
from .startup_checks import expected_ports
from .telemetry import gcs_snapshot


def simulation_us(stats):
    body = re.search(r"sim_time\s*\{([^}]*)\}", stats, re.S)
    if not body:
        return None
    sec = re.search(r"\bsec: (\d+)", body[1])
    nsec = re.search(r"\bnsec: (\d+)", body[1])
    return (int(sec[1]) if sec else 0)*1_000_000 + (int(nsec[1])//1000 if nsec else 0)


def world_snapshot(profile):
    stats = probe(["gz", "topic", "-e", "-t", f"/world/{profile.world_name}/stats", "-n", "1"], 2)
    models = probe(["gz", "model", "--list"], 2)
    names = re.findall(r"^\s*-\s+(\S+)\s*$", models["stdout"], re.M)
    correct = (stats["exit"] == models["exit"] == 0
               and f"world [{profile.world_name}]" in models["stdout"]
               and names.count(profile.vehicle_name) == 1
               and not any(n.startswith("x500") and n != profile.vehicle_name for n in names))
    iteration = re.search(r"^iterations: (\d+)$", stats["stdout"], re.M)
    # Protobuf text omits scalar defaults: a newly paused world may omit iterations=0.
    iterations = int(iteration[1]) if iteration else (0 if simulation_us(stats["stdout"]) == 0 else None)
    readable = stats['exit'] == 0 and not stats.get('stderr','').strip() and simulation_us(stats['stdout']) is not None
    # A timeout/empty reply cannot establish paused=False. Unknown stays unknown.
    paused = ('paused: true' in stats['stdout']) if readable else None
    return dict(correct=bool(correct and readable and iterations is not None), paused=paused,
                iterations=iterations, evidence=dict(stats=stats, models=models))


def native_probe(profile, module, *args):
    binary = Path(profile.px4_root).expanduser()/"build/px4_sitl_default/bin"/f"px4-{module}"
    return probe([str(binary), "--instance", str(profile.instance), *args], 2)


def shutdown_gate(profile):
    before = world_snapshot(profile)
    armed = native_probe(profile, "listener", "actuator_armed", "-n", "1")
    landed = native_probe(profile, "listener", "vehicle_land_detected", "-n", "1")
    logger = native_probe(profile, "logger", "status")
    world = world_snapshot(profile)
    af, lf = topic_fields(armed["stdout"]), topic_fields(landed["stdout"])
    # Use sim_time, not an assumed 1 ms physics timestep.
    text = world["evidence"]["stats"]["stdout"]
    clock_us = simulation_us(text)
    def fresh(fields):
        try:
            stamp = int(fields["timestamp"].split()[0])
            return clock_us is not None and stamp > 0 and 0 <= clock_us-stamp <= 1_000_000
        except (KeyError, TypeError, ValueError):
            return False
    safe = (before["correct"] and world["correct"]
            and armed["exit"] == landed["exit"] == logger["exit"] == 0
            and af.get("armed") == "False" and lf.get("landed") == "True"
            and fresh(af) and fresh(lf))
    before_us = simulation_us(before["evidence"]["stats"]["stdout"])
    progressing = (before["correct"] and world["correct"] and before["paused"] is False and world["paused"] is False
                   and before_us is not None and clock_us is not None and clock_us > before_us
                   and world["iterations"] > before["iterations"])
    blockers = []
    if not safe:
        blockers.append("Need correct world/model and fresh Disarmed/Landed (age <= 1 simulation second), logger responsive")
    if before_us is None or clock_us is None or before['paused'] is None or world['paused'] is None:
        blockers.append('CLOCK_UNKNOWN_STATS_UNAVAILABLE: cannot infer paused/running state; no clock or shutdown proposal')
    elif before["paused"] or world["paused"]:
        blockers.append("CLOCK_PAUSED: native shutdown work queue/HRT/logger may wait; no automatic unpause")
    elif not progressing:
        blockers.append("CLOCK_NOT_PROGRESSING: not-paused flag alone is not running-clock evidence")
    return dict(safe=bool(safe), paused=world["paused"], world=world, world_before=before,
                clock_progressing=bool(progressing), can_shutdown=bool(safe and progressing),
                clock_before_us=before_us, clock_after_us=clock_us, blockers=blockers,
                evidence=dict(armed=armed, landed=landed, logger=logger))


def readiness(profile, with_dds, directory):
    """Bounded parallel native probes; cold-paused missing samples remain NOT READY."""
    world = world_snapshot(profile)
    calls = {name: ("listener", name, "-n", "1") for name in (
        "actuator_armed", "vehicle_status", "vehicle_land_detected", "sensor_accel", "sensor_gyro",
        "vehicle_local_position", "vehicle_global_position", "estimator_status", "health_report",
        "failsafe_flags", "telemetry_status")}
    calls.update({name: (name, "status") for name in ("commander", "ekf2", "logger", "uxrce_dds_client")})
    calls["arm_on_boot"] = ("param", "show", "COM_ARM_ON_BOOT")
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = {name: pool.submit(native_probe, profile, *args) for name, args in calls.items()}
        evidence = {name: future.result() for name, future in futures.items()}
    fields = {name: topic_fields(value["stdout"]) for name, value in evidence.items()}
    a, v, lp, gp, es = [fields[name] for name in (
        "actuator_armed", "vehicle_status", "vehicle_local_position", "vehicle_global_position", "estimator_status")]
    def ok(name):
        return evidence[name]["exit"] == 0
    gcs = gcs_snapshot(evidence['telemetry_status']['stdout'], v,
                       simulation_us(world['evidence']['stats']['stdout']))
    gates = dict(world_model=world["correct"], physics_paused=world["paused"],
                 disarmed=ok("actuator_armed") and a.get("armed") == "False",
                 landed=ok("vehicle_land_detected") and fields["vehicle_land_detected"].get("landed") == "True",
                 arm_on_boot_disabled=ok("arm_on_boot") and bool(re.search(
                     r"\bCOM_ARM_ON_BOOT\s+\[[^]]+\]\s*:\s*0\s*$", evidence["arm_on_boot"]["stdout"], re.M)),
                 ekf_valid=ok("ekf2") and "attitude: 1, local position: 1, global position: 1" in evidence["ekf2"]["stdout"],
                 local_valid=ok("vehicle_local_position") and all(lp.get(k) == "True" for k in (
                     "xy_valid", "z_valid", "v_xy_valid", "v_z_valid")),
                 global_valid=ok("vehicle_global_position") and gp.get("lat_lon_valid") == gp.get("alt_valid") == "True",
                 no_filter_fault=ok("estimator_status") and es.get("filter_fault_flags") == "0",
                 preflight=ok("vehicle_status") and v.get("pre_flight_checks_pass") == "True" and v.get("failsafe") == "False",
                 ready_to_arm=ok("actuator_armed") and a.get("ready_to_arm") == "True",
                 logger=ok("logger") and "Full File Logging Running:" in evidence["logger"]["stdout"],
                 gcs_snapshot=ok("telemetry_status") and ok('vehicle_status') and gcs['snapshot_pass'])
    for sensor in ("sensor_accel", "sensor_gyro"):
        gates[sensor] = ok(sensor) and fields[sensor].get("timestamp_sample", "0").split()[0] != "0"
    clock_us = simulation_us(world["evidence"]["stats"]["stdout"])
    for name, budget in (("vehicle_local_position", 50_000), ("vehicle_global_position", 50_000),
                         ("sensor_accel", 50_000), ("sensor_gyro", 50_000), ("estimator_status", 100_000),
                         ("vehicle_status", 1_000_000), ("actuator_armed", 1_000_000)):
        try:
            stamp = int(fields[name]["timestamp"].split()[0])
            gates[name+"_fresh_paused"] = bool(world["paused"] and clock_us is not None
                                               and 0 <= clock_us-stamp <= budget)
        except (KeyError, ValueError):
            gates[name+"_fresh_paused"] = False
    health = fields["health_report"]
    modes = {}
    status_source = Path(profile.px4_root).expanduser()/"msg/versioned/VehicleStatus.msg"
    try:
        source = status_source.read_text()
    except OSError:
        source = ""
    for name, symbol in (("hold", "AUTO_LOITER"), ("takeoff", "AUTO_TAKEOFF"), ("land", "AUTO_LAND")):
        try:
            bit = int(re.search(r"NAVIGATION_STATE_"+symbol+r"\s*=\s*(\d+)", source)[1])
            modes[name] = {key: bool(int(health[key].split()[0]) & (1 << bit))
                           for key in ("can_arm_mode_flags", "can_run_mode_flags")}
            modes[name]["source_bit"] = bit
        except (KeyError, ValueError, TypeError):
            modes[name] = None
    ps = probe(["ps", "-eo", "pid=,comm=,args="], 2)
    sockets = probe(["ss", "-H", "-uanp"], 2)
    evidence.update(processes=ps, sockets=sockets)
    dds = dict(enabled=with_dds, status="DISABLED_BY_OPTION")
    if with_dds:
        # A subscriber-only observer, in a sourced ROS subprocess. No controller is imported.
        code = ("import json; from pso_flight_demo.environments import Profile; "
                "from pso_flight_demo.dds_observer import observe; "
                f"print(json.dumps(observe(Profile(**json.loads({json.dumps(profile.snapshot())!r})),"
                f"3,{str(directory/'ros_logs')!r})))")
        command = (f"source {shlex.quote(str(Path(profile.ros_setup).expanduser()))} && "
                   f"source {shlex.quote(str(Path(profile.px4_ros_setup).expanduser()))} && "
                   f"exec env ROS_DOMAIN_ID={profile.dds_domain} PYTHONDONTWRITEBYTECODE=1 python3 -B -c "+shlex.quote(code))
        observation = probe(["bash", "-c", command], 12)
        evidence["dds_observer"] = observation
        try:
            dds = json.loads(observation["stdout"].splitlines()[-1])
        except (ValueError, IndexError):
            dds = dict(errors=["DDS observer failed"], enabled=True)
        dds["client_connected"] = "Running, connected" in evidence["uxrce_dds_client"]["stdout"]
        dds["status"] = "OBSERVED" if not dds.get("errors") and dds.get("dds_messages_received") else "NOT_PROVEN_WHILE_PAUSED"
    return dict(result="WAIT_FOR_SEPARATELY_APPROVED_CLOCK_AND_NEW_ANCHOR", gates=gates,
                modes=modes, health_timestamp=health.get("timestamp"), fields=fields, world=world, gcs_evidence=gcs,
                runtime_identity=process_identity(ps["stdout"]), expected_ports=expected_ports(profile),
                dds=dds, evidence=evidence, live_control_go=False, publishes_commands=False,
                note="Change-only health timestamps are preserved; no clock progress or live GCS claim.")
