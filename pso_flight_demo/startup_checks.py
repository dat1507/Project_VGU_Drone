"""Read-only new-session checks. No process management or simulator services."""

from pathlib import Path
import re

from .runtime_readiness import probe


def expected_ports(profile):
    i = profile.instance
    return dict(gcs=dict(local=18570+i, remote=14550),
                mavlink_offboard=dict(local=14580+i, remote=14540+i),
                payload=dict(local=14280+i, remote=14030+i),
                gimbal=dict(local=13030+i, remote=13280+i),
                dds=dict(agent=profile.agent_port, protocol="UDP", domain=profile.dds_domain,
                         namespace=profile.namespace))


def assess_startup(profile, process_text, socket_text, *, with_dds=True, visibility="host_terminal"):
    blockers, warnings = [], []
    found = dict(px4=[], gazebo=[], agent=[], qgc=[], known_controllers=[])
    for line in process_text.splitlines():
        columns = line.split(None, 2)
        if len(columns) != 3:
            continue
        pid, comm, args = columns
        if not pid.isdigit():
            continue
        if comm == "px4":
            found["px4"].append(int(pid))
        if comm in {"gz-sim-server", "gz-sim-gui"} or (
                comm in {"ruby", "gz"} and re.search(r"\bgz\s+sim\b", args)):
            found["gazebo"].append(int(pid))
        if comm == "MicroXRCEAgent":
            found["agent"].append(int(pid))
        if comm == "QGroundControl":
            found["qgc"].append(int(pid))
        if (comm.startswith("python") and (
                "offboard_control" in args or ("pso_flight_demo" in args and
                                               ("--execute" in args or "--confirm-live" in args)))) or "offboard" in comm.lower():
            found["known_controllers"].append(int(pid))
    for kind in ("px4", "gazebo", "known_controllers"):
        if found[kind]:
            blockers.append(f"Existing {kind} processes: {found[kind]}; no automatic reuse/stop")
    if found["agent"]:
        blockers.append("Existing Agent: explicit reuse policy is not implemented")
    if found["qgc"]:
        warnings.append("QGC is already open: keep that GUI; do not open another")
    ports = expected_ports(profile)
    reserved = {ports[k]["local"] for k in ("gcs", "mavlink_offboard", "payload", "gimbal")}
    # rcS still starts its client in native-only mode. An unexpected existing
    # endpoint must not silently become the new PX4's DDS partner.
    reserved.add(profile.agent_port)
    occupied = []
    for line in socket_text.splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        local = re.search(r":(\d+)$", parts[3])
        if local is None:
            continue
        port = int(local[1])
        if port in reserved:
            occupied.append(port)
        if port == 14550 and "QGroundControl" not in line:
            blockers.append("UDP 14550 is occupied by an unknown/non-QGC owner")
    if occupied:
        blockers.append(f"Reserved local UDP ports occupied: {sorted(set(occupied))}")
    if visibility != "host_terminal":
        blockers.append("Host process visibility is not established: run checks in Ubuntu host Terminal")
    return dict(result="NEW_SESSION_BLOCKED" if blockers else "NEW_SESSION_CANDIDATE",
                blockers=blockers, warnings=warnings, processes=found, occupied_ports=sorted(set(occupied)),
                process_visibility=visibility, startup_executed=False, live_control_go=False,
                note="Candidate means no visible conflict, not flight readiness or guaranteed exclusive ownership.")


def host_startup_check(profile, with_dds=True, *, ignore_pids=()):
    results = dict(processes=probe(["ps", "-eo", "pid=,comm=,args="]),
                   udp=probe(["ss", "-H", "-uanp"]))
    try:
        init = Path("/proc/1/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        visibility = "isolated_sandbox" if "codex-linux-sandbox" in init else "host_terminal"
    except OSError:
        visibility = "unknown"
    process_text = "\n".join(line for line in results["processes"]["stdout"].splitlines()
                             if not line.split() or not line.split()[0].isdigit()
                             or int(line.split()[0]) not in ignore_pids)
    result = assess_startup(profile, process_text, results["udp"]["stdout"],
                            with_dds=with_dds, visibility=visibility)
    # ss can return 0 even when netlink access failed; empty stdout then does
    # NOT establish that the host has free ports.
    if any(item["exit"] != 0 or item["stderr"].strip() for item in results.values()):
        result["blockers"].append("ps/ss probe failed: absence of output is not proof of free ports/processes")
        result["result"] = "NEW_SESSION_BLOCKED"
    result["probe_status"] = {name: dict(exit=v["exit"], stderr=v["stderr"]) for name, v in results.items()}
    return result
