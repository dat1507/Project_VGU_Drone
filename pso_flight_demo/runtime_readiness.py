"""Host read-only probes. No start, mode, parameter set, setpoint or world control."""

from dataclasses import asdict
from pathlib import Path
import re
import subprocess
import time

import numpy as np

from .environments import load_environment
from .frames import FrameAnchor, RESET_FIELDS, ned_to_enu
from .telemetry import single_fields, gcs_snapshot, simulation_us


def phase_problems(problems, gate_phase):
    if gate_phase not in {"strict", "flight_prestart"}:
        raise ValueError("Unknown runtime gate phase: "+str(gate_phase))
    deferred = {"commander_preflight_ready", "logger_running"} if gate_phase == "flight_prestart" else set()
    return [p for p in problems if p not in deferred], [p for p in problems if p in deferred]


def probe(argv, timeout=4):
    try:
        p = subprocess.run([str(x) for x in argv], capture_output=True, text=True, timeout=timeout)
        return dict(argv=[str(x) for x in argv], exit=p.returncode, stdout=p.stdout, stderr=p.stderr)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(argv=[str(x) for x in argv], exit=None, stdout="", stderr=str(exc))


def topic_fields(output):
    return single_fields(output)


def number(fields, key):
    return float(fields[key].split()[0])


def pose_xyz(text):
    match = re.search(r"Pose \[ XYZ.*?\n\s*\[([^]]+)\]", text)
    if not match:
        raise ValueError("Cannot read Gazebo model pose")
    return np.asarray([float(x) for x in match.group(1).split()])


def process_identity(text):
    processes = []
    for line in text.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3:
            processes.append((int(parts[0]), parts[1], parts[2]))
    px4 = [p[0] for p in processes if p[1] == "px4"]
    gz = [p[0] for p in processes if p[1] in {"ruby", "gz", "gz-sim-server"}
          and ("gz sim server" in p[2] or p[1] == "gz-sim-server")]
    def ticks(pid):
        try:
            return int(Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19])
        except (OSError, ValueError, IndexError):
            return None
    try:
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        boot = None
    return dict(boot_id=boot, px4_pid=px4[0] if len(px4) == 1 else None,
                px4_start_ticks=ticks(px4[0]) if len(px4) == 1 else None,
                gazebo_server_pid=gz[0] if len(gz) == 1 else None,
                gazebo_start_ticks=ticks(gz[0]) if len(gz) == 1 else None,
                px4_count=len(px4), gazebo_server_count=len(gz),
                agent_count=sum(comm == "MicroXRCEAgent" for _, comm, _ in processes),
                qgc_count=sum(comm == "QGroundControl" for _, comm, _ in processes))


def inspect_runtime(profile, expected_iterations=None, *, gate_phase="strict"):
    if gate_phase not in {"strict", "flight_prestart"}:
        raise ValueError("Unknown runtime gate phase: "+str(gate_phase))
    world, env = load_environment(profile)
    bin_dir = Path(profile.px4_root).expanduser() / "build/px4_sitl_default/bin"
    instance = str(profile.instance)
    commands = {
        "stats": ["gz", "topic", "-e", "-t", f"/world/{world.name}/stats", "-n", "1"],
        "models": ["gz", "model", "--list"],
        "pose": ["gz", "model", "-m", profile.vehicle_name, "--pose"],
        "processes": ["ps", "-eo", "pid=,comm=,args="],
        "sockets_before": ["ss", "-uanpm", f"( sport = :{profile.agent_port} or sport = :18570 or sport = :14550 )"],
    }
    for module in ("commander", "ekf2", "logger", "uxrce_dds_client"):
        commands[module] = [bin_dir/f"px4-{module}", "--instance", instance, "status"]
    for name in ("actuator_armed", "vehicle_land_detected", "vehicle_status", "vehicle_local_position",
                 "vehicle_local_position_groundtruth", "estimator_status", "sensor_accel", "sensor_gyro",
                 "telemetry_status", "health_report", "failsafe_flags"):
        commands[name] = [bin_dir/"px4-listener", "--instance", instance, name, "-n", "1"]
    commands['mavlink'] = [bin_dir/'px4-mavlink', '--instance', instance, 'status']
    for pattern in ("UXRCE_DDS*", "COM_O*", "NAV_DLL_ACT", "COM_ARM_ON_BOOT",
                    "COM_DL_LOSS_T", "NAV_RCL_ACT", "COM_RCL_EXCEPT", "COM_FAIL_ACT_T", "SDLOG_MODE",
                    "MPC_ACC*", "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX*", "MPC_TKO_SPEED",
                    "MPC_LAND_SPEED", "COM_DISARM_LAND", "RTL_RETURN_ALT"):
        commands["params_"+pattern] = [bin_dir/"px4-param", "--instance", instance, "show", pattern]
    evidence = {name: probe(command) for name, command in commands.items()}
    time.sleep(1)
    evidence["sockets_after"] = probe(commands["sockets_before"])
    evidence["stats_after"] = probe(commands["stats"])
    identity = process_identity(evidence["processes"]["stdout"])
    fields = {k: topic_fields(evidence[k]["stdout"]) for k in commands if k in {
        "actuator_armed", "vehicle_land_detected", "vehicle_status", "vehicle_local_position",
        "vehicle_local_position_groundtruth", "estimator_status", "sensor_accel", "sensor_gyro",
        "telemetry_status", "failsafe_flags", "health_report"}}
    gates, problems = {}, []
    def gate(name, test):
        try:
            gates[name] = bool(test())
        except (KeyError, ValueError, IndexError, TypeError):
            gates[name] = False
        if not gates[name]:
            problems.append(name)
    stats = evidence["stats"]["stdout"]
    final_stats = evidence["stats_after"]["stdout"]
    iteration = re.search(r"^iterations: (\d+)$", stats, re.M)
    final_iteration = re.search(r"^iterations: (\d+)$", final_stats, re.M)
    gate("stats_readable_paused", lambda: evidence["stats"]["exit"] == 0 and "paused: true" in stats)
    gate("clock_remained_paused", lambda: "paused: true" in final_stats and iteration and final_iteration
         and iteration[1] == final_iteration[1])
    if expected_iterations is not None:
        gate("expected_iterations", lambda: iteration and int(iteration[1]) == expected_iterations)
    models = re.findall(r"^\s*-\s+(\S+)\s*$", evidence["models"]["stdout"], re.M)
    gate("one_configured_vehicle_in_correct_world", lambda: models.count(profile.vehicle_name) == 1
         and f"world [{world.name}]" in evidence["models"]["stdout"])
    gate("single_px4_and_gazebo", lambda: identity["px4_count"] == identity["gazebo_server_count"] == 1)
    gate("disarmed_landed", lambda: fields["actuator_armed"]["armed"] == "False"
         and fields["vehicle_land_detected"]["landed"] == "True"
         and "[commander] Disarmed" in evidence["commander"]["stdout"])
    gate("commander_preflight_ready", lambda: fields["actuator_armed"]["ready_to_arm"] == "True"
         and fields["vehicle_status"]["pre_flight_checks_pass"] == "True"
         and fields["vehicle_status"]["failsafe"] == "False")
    gate("ekf_position_valid", lambda: all(fields["vehicle_local_position"][k] == "True"
         for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"))
         and "attitude: 1, local position: 1, global position: 1" in evidence["ekf2"]["stdout"])
    gate("no_filter_fault", lambda: number(fields["estimator_status"], "filter_fault_flags") == 0)
    gate("logger_running", lambda: "Full File Logging Running:" in evidence["logger"]["stdout"])
    gcs = gcs_snapshot(evidence['telemetry_status']['stdout'], fields['vehicle_status'], simulation_us(stats))
    gate("gcs_heartbeat_snapshot", lambda: evidence['telemetry_status']['exit'] == 0
         and evidence['vehicle_status']['exit'] == 0 and gcs['snapshot_pass'])
    gate("sensors_present", lambda: number(fields["sensor_accel"], "timestamp_sample") > 0
         and number(fields["sensor_gyro"], "timestamp_sample") > 0)
    def clock_us():
        body = re.search(r"sim_time\s*\{([^}]+)\}", stats, re.S)[1]
        def component(key):
            m = re.search(r"\b"+key+r": (\d+)", body)
            return int(m[1]) if m else 0
        return component("sec")*1_000_000 + component("nsec")//1000
    gate("source_samples_match_paused_clock", lambda: all(
        0 <= clock_us()-number(fields[k], "timestamp_sample") <= 50_000
        for k in ("vehicle_local_position", "vehicle_local_position_groundtruth", "sensor_accel", "sensor_gyro")))
    gate("status_timestamps_match_paused_clock", lambda: all(
        0 <= clock_us()-number(fields[k], "timestamp") <= budget
        for k, budget in (("estimator_status", 100_000), ("vehicle_status", 1_000_000),
                          ("actuator_armed", 1_000_000), ("vehicle_land_detected", 1_000_000))))
    gate("runtime_identity_complete", lambda: all(identity[k] is not None for k in
         ("boot_id", "px4_pid", "px4_start_ticks", "gazebo_server_pid", "gazebo_start_ticks")))
    anchor = None
    try:
        lp, gt = fields["vehicle_local_position"], fields["vehicle_local_position_groundtruth"]
        anchor = FrameAnchor(tuple(ned_to_enu([number(gt, k) for k in ("x", "y", "z")])),
                             tuple(number(lp, k) for k in ("x", "y", "z")),
                             int(number(gt, "timestamp_sample")), int(number(lp, "timestamp_sample")),
                             tuple(int(number(lp, k)) for k in RESET_FIELDS),
                             "px4_groundtruth_and_local_position")
        gate("gazebo_pose_matches_groundtruth", lambda: np.linalg.norm(pose_xyz(evidence["pose"]["stdout"])
                                                                       - anchor.gazebo_enu) <= .02)
    except (KeyError, ValueError, TypeError):
        problems.append("measured_synchronized_anchor_missing")
    def udp(text):
        match = re.search(r":18570\s.*?\n\s*skmem:\([^)]*\bd(\d+)\)", text, re.S)
        queue = re.search(r"UNCONN\s+(\d+)\s+\d+\s+\S+:18570\s", text)
        return dict(drops=int(match[1]) if match else None, queue=int(queue[1]) if queue else None)
    before, after = [udp(evidence[k]["stdout"]) for k in ("sockets_before", "sockets_after")]
    delta = after["drops"]-before["drops"] if before["drops"] is not None and after["drops"] is not None else None
    blocking_problems, deferred_problems = phase_problems(problems, gate_phase)
    report = dict(profile=profile.snapshot(), world=asdict(world), captured_wall_time=time.time(),
                  runtime_identity=identity, gates=gates, problems=blocking_problems,
                  all_problems=problems, deferred_gates=deferred_problems,
                  gate_phase=gate_phase,
                  udp_before=before, udp_after=after, new_kernel_drops=delta,
                  paused_snapshot_only=True, timestamp_progression="FROZEN_EXPECTED_WHILE_PAUSED",
                  live_control_go=False, publishes_commands=False, evidence=evidence)
    report["software_paths"] = {key: Path(getattr(profile, key)).expanduser().is_file()
                                for key in ("ros_setup", "px4_ros_setup", "qgc")}
    report["dds_client_connected"] = "Running, connected" in evidence["uxrce_dds_client"]["stdout"]
    report['gcs_evidence'] = gcs
    return report, anchor, env
