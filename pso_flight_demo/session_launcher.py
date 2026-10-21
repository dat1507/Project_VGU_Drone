"""Opt-in, foreground session supervisor. Default CLI only prints a plan.

No flight commands, physics services, automatic restart, SIGKILL or process adoption.
Runtime execution requires a separate operator approval and explicit --execute.
"""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time
from uuid import uuid4

from .environments import MODULE, PROJECT, Profile, load_environment, load_profile
from .launch_plan import launch_plan
from .session_runtime import native_probe, readiness, shutdown_gate, world_snapshot
from .startup_checks import assess_startup, host_startup_check, expected_ports


SESSION_ROOT = MODULE/"sessions"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def identity(pid):
    """Linux PID identity, robust to spaces/parentheses in comm and PID reuse."""
    try:
        root = Path(f"/proc/{pid}")
        stat = (root/"stat").read_text().rsplit(")", 1)[1].split()
        if stat[0] == "Z":
            return None
        return dict(pid=int(pid), start_ticks=int(stat[19]), ppid=int(stat[1]),
                    pgid=int(stat[2]), sid=int(stat[3]), uid=root.stat().st_uid,
                    exe=str((root/"exe").resolve(strict=True)),
                    boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip())
    except (OSError, ValueError, IndexError):
        return None


def same_process(saved, current):
    return bool(saved and current and all(saved.get(k) == current.get(k) for k in
                ("pid", "start_ticks", "boot_id", "uid", "sid", "pgid", "exe")))


def secure_directory(path):
    path = Path(path)
    if not path.resolve().is_relative_to(MODULE.resolve()) or path.resolve() == MODULE.resolve():
        raise ValueError("Session files must stay inside pso_flight_demo")
    for part in [path, *path.parents]:
        if part == MODULE.parent:
            break
        if part.is_symlink():
            raise ValueError("Symlink in session directory")
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.stat().st_uid != os.getuid() or path.stat().st_mode & 0o022:
        raise ValueError("Session directory must be owned by this user and not group/world writable")
    return path


class ExclusiveLock:
    """Persistent lock inode; never unlink a lock and create two lock universes."""
    def __init__(self, path):
        self.path, self.fd = Path(path), None

    def __enter__(self):
        secure_directory(self.path.parent)
        self.fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self.fd)
            self.fd = None
            raise ValueError("Another launcher owns the lifecycle lock") from None
        return self

    def __exit__(self, *args):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def write_state(directory, value):
    # Only this NEW session's mutable state is replaced; existing artifacts are never touched.
    tmp = directory/("state_"+uuid4().hex+".tmp")
    with tmp.open("x") as stream:
        os.chmod(tmp, 0o600)
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, directory/"session.json")


@dataclass
class Child:
    role: str
    process: object
    owner: dict
    members: dict


class Host:
    """All mutating system calls are confined here and mocked in offline tests."""
    def precheck(self, profile, with_dds):
        return host_startup_check(profile, with_dds, ignore_pids=(os.getpid(),))

    def ident(self, pid):
        return identity(pid)

    def members(self, child):
        if child.owner.get("identity_unproven"):
            raise ValueError("Cannot inspect unproven child session; manual review required")
        result = []
        for path in Path("/proc").iterdir():
            if path.name.isdigit():
                info = identity(int(path.name))
                if info and info["sid"] == child.owner["sid"]:
                    result.append(info)
        return result

    def spawn(self, role, argv, cwd, env, directory):
        with (directory/f"{role.lower()}.log").open("xb") as log:
            process = subprocess.Popen(argv, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                       close_fds=True)
        # Exec may not have completed yet: pid/start/session are recorded before executable stabilizes.
        info = identity(process.pid)
        if info is None:
            info = dict(pid=process.pid, identity_unproven=True)
        return Child(role, process, info, {process.pid: info})

    def signal_owned(self, saved):
        # pidfd ties the signal to a kernel task, closing the check/signal PID-reuse race.
        if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
            raise ValueError("pidfd support required; no unsafe PID-signal fallback")
        fd = os.pidfd_open(saved["pid"])
        try:
            if not same_process(saved, identity(saved["pid"])):
                raise ValueError("Process identity changed; refusing signal")
            signal.pidfd_send_signal(fd, signal.SIGINT)
        finally:
            os.close(fd)

    def stage_conflicts(self, profile, children, with_dds, *, supervisor=None):
        # Recheck all foreign processes and reserved ports immediately before each spawn.
        from .runtime_readiness import probe
        ps = probe(["ps", "-eo", "pid=,comm=,args="], 2)
        ss = probe(["ss", "-H", "-uanp"], 2)
        if any(r["exit"] != 0 or r["stderr"].strip() for r in (ps, ss)):
            raise ValueError("Cannot establish process/port ownership")
        owned = {os.getpid()}
        # A read-only requester is not the foreground supervisor. Exclude only
        # its fully verified PID, never its login SID or arbitrary Python tasks.
        if supervisor is not None:
            if not same_process(supervisor, identity(supervisor['pid'])):
                raise ValueError('Supervisor identity changed during conflict check')
            owned.add(supervisor['pid'])
        for child in children:
            owned.update(x["pid"] for x in self.members(child))
        filtered_ps = "\n".join(line for line in ps["stdout"].splitlines()
                                if line.split() and int(line.split()[0]) not in owned)
        # Ports with mixed, hidden or foreign owners must not be ignored.
        foreign_sockets = []
        for line in ss["stdout"].splitlines():
            owners = {int(x) for x in re.findall(r"pid=(\d+)", line)}
            if not owners or not owners.issubset(owned):
                foreign_sockets.append(line)
        check = assess_startup(profile, filtered_ps, "\n".join(foreign_sockets), with_dds=with_dds)
        if check["blockers"]:
            raise ValueError("Stage conflict: "+"; ".join(check["blockers"]))

    def world(self, profile):
        return world_snapshot(profile)

    def attached(self, profile, child, with_dds):
        result = native_probe(profile, "commander", "status")
        param = native_probe(profile, "param", "show", "COM_ARM_ON_BOOT")
        # Separate readiness may be absent at t=0. Disarmed + explicit auto-arm-off are mandatory.
        return (result["exit"] == 0 and "[commander] Disarmed" in result["stdout"]
                and param["exit"] == 0 and bool(re.search(
                    r"\bCOM_ARM_ON_BOOT\s+\[[^]]+\]\s*:\s*0\s*$", param["stdout"], re.M)))

    def bound_ports(self, profile, child):
        from .runtime_readiness import probe
        result = probe(["ss", "-H", "-uanp"], 2)
        if result["exit"] != 0 or result["stderr"].strip():
            return False
        ports = expected_ports(profile)
        required = ({profile.agent_port} if child.role == "AGENT" else
                    {ports[k]["local"] for k in ("gcs", "mavlink_offboard", "payload", "gimbal")})
        owned = {info["pid"] for info in self.members(child)}
        found = {port: [] for port in required}
        for line in result["stdout"].splitlines():
            parts = line.split()
            match = re.search(r":(\d+)$", parts[3]) if len(parts) >= 5 else None
            if match and int(match[1]) in required:
                found[int(match[1])].append({int(x) for x in re.findall(r"pid=(\d+)", line)})
        return all(len(owners) == 1 and owners[0] and owners[0].issubset(owned) for owners in found.values())

    def ready(self, profile, with_dds, directory):
        return readiness(profile, with_dds, directory)

    def stop_gate(self, profile):
        return shutdown_gate(profile)

    def shutdown_px4(self, profile):
        return native_probe(profile, "shutdown")


def child_commands(profile, plan):
    """Reuse reviewed plan argv; no eval/shell for Gazebo, Agent or PX4."""
    env = dict(os.environ)
    for key in ("PX4_HOME_LAT", "PX4_HOME_LON", "PX4_HOME_ALT"):
        env.pop(key, None)
    commands = []
    for stage in plan["commands"]:
        role = stage["terminal"]
        if role in {"GAZEBO", "AGENT"}:
            argv = shlex.split(stage["command"])
            stage_env = dict(env)
            if role == "GAZEBO":
                _, resource, *argv = argv
                stage_env["GZ_SIM_RESOURCE_PATH"] = resource.split("=", 1)[1]
                env["GZ_SIM_RESOURCE_PATH"] = stage_env["GZ_SIM_RESOURCE_PATH"]
            commands.append((role, argv, str(PROJECT), stage_env))
        elif role == "PX4":
            root = Path(profile.px4_root).expanduser()/"build/px4_sitl_default"
            stage_env = dict(env, PX4_SYS_AUTOSTART=str(profile.airframe), PX4_SIMULATOR="gz",
                             PX4_GZ_STANDALONE="1", PX4_GZ_WORLD=profile.world_name,
                             PX4_GZ_MODEL_NAME=profile.vehicle_name, PX4_UXRCE_DDS_NS=profile.namespace,
                             PX4_UXRCE_DDS_PORT=str(profile.agent_port), ROS_DOMAIN_ID=str(profile.dds_domain))
            # -d disables pxh stdin shell, but keeps the PX4 daemon/client interface alive.
            commands.append((role, [str(root/"bin/px4"), "-d", "-i", str(profile.instance),
                                    "-w", str(root/"rootfs"), "-s", "etc/init.d-posix/rcS"],
                             str(root/"rootfs"), stage_env))
    return commands


class Supervisor:
    def __init__(self, profile, plan, directory, *, with_dds=True, timeout=30, host=None):
        self.profile, self.plan, self.directory = profile, plan, directory
        self.with_dds, self.timeout, self.host = with_dds, timeout, host or Host()
        self.children = []
        self.shutdown_requested = False
        self.stop_requested = False
        self.state = dict(schema=1, session_id=directory.name, created_wall_time=time.time(),
                          supervisor=identity(os.getpid()), profile=profile.snapshot(),
                          profile_sha256=digest(profile.snapshot()), sdf_sha256=plan["sdf_sha256"],
                          with_dds=with_dds, state="ALLOCATED", children=[],
                          startup_plan=plan,
                          new_anchor_required=True, publishes_commands=False, live_control_go=False)

    def save(self, state=None, **updates):
        if state:
            self.state["state"] = state
        self.state.update(updates)
        self.state["children"] = [dict(role=c.role, owner=c.owner, members=list(c.members.values()),
                                       exit=c.process.poll()) for c in self.children]
        write_state(self.directory, self.state)

    def track(self, child):
        if child.owner.get("identity_unproven"):
            if child.process.poll() is None:
                raise ValueError("Child identity unavailable; preserve processes for manual review")
            return
        for info in sorted(self.host.members(child), key=lambda x: (x["start_ticks"], x["pid"])):
            old = child.members.get(info["pid"])
            if old:
                if not same_process(old, info):
                    # A legitimate child exec is allowed while tracking, never PID/start reuse.
                    if any(old[k] != info[k] for k in ("pid", "start_ticks", "boot_id", "uid", "sid", "pgid")):
                        raise ValueError("Owned child identity changed")
                    child.members[info["pid"]] = info
                continue
            if (info["uid"] != child.owner["uid"] or info["start_ticks"] < child.owner["start_ticks"]
                    or info["ppid"] not in child.members):
                raise ValueError("Unproven process in child session; refusing adoption")
            child.members[info["pid"]] = info
        current = self.host.ident(child.owner["pid"])
        if current:
            if current["start_ticks"] != child.owner["start_ticks"] or current["sid"] != child.owner["sid"]:
                raise ValueError("Leader identity no longer matches")
            child.owner = current

    def wait(self, predicate, label):
        until = time.monotonic()+self.timeout
        while True:
            if self.stop_requested:
                raise ValueError("Startup interrupted; applying owned-process rollback policy")
            for child in self.children:
                self.track(child)
                if child.process.poll() is not None:
                    raise ValueError(f"{child.role} exited during {label}")
            if predicate():
                return
            if time.monotonic() >= until:
                raise ValueError(f"Timeout waiting for {label}")
            time.sleep(.2)

    def start(self):
        check = self.host.precheck(self.profile, self.with_dds)
        if check["blockers"]:
            self.save("START_BLOCKED", precheck=check)
            return False
        try:
            for role, argv, cwd, env in child_commands(self.profile, self.plan):
                if self.stop_requested:
                    raise ValueError("Startup interrupted")
                world, _ = load_environment(self.profile)
                if world.sdf_sha256 != self.plan["sdf_sha256"]:
                    raise ValueError("SDF changed since startup plan; no automatic continuation")
                self.host.stage_conflicts(self.profile, self.children, self.with_dds)
                self.save("STARTING_"+role)
                child = self.host.spawn(role, argv, cwd, env, self.directory)
                self.children.append(child)
                self.save()
                self.track(child)
                if role == "PX4" and child.owner.get("exe") != str(Path(argv[0]).resolve()):
                    raise ValueError("PX4 executable identity does not match selected build")
                if role == "GAZEBO":
                    def world_ready():
                        snapshot = self.host.world(self.profile)
                        return snapshot["correct"] and snapshot["paused"]
                    self.wait(world_ready, "paused world/model")
                elif role == "AGENT":
                    self.wait(lambda: self.host.bound_ports(self.profile, child), "owned Agent UDP port")
                elif role == "PX4":
                    self.wait(lambda: self.host.attached(self.profile, child, self.with_dds), "disarmed PX4 attach/auto-arm-off")
                    self.wait(lambda: self.host.bound_ports(self.profile, child), "owned PX4 MAVLink ports")
                    self.host.stage_conflicts(self.profile, self.children, self.with_dds)
            if not self.host.world(self.profile)["paused"]:
                raise ValueError("World no longer paused; no automatic physics correction")
            report = self.host.ready(self.profile, self.with_dds, self.directory)
            if not report["gates"].get("arm_on_boot_disabled"):
                raise ValueError("COM_ARM_ON_BOOT=0 could not be verified; keep clock paused")
            final_world = self.host.world(self.profile)
            if not final_world["correct"] or not final_world["paused"]:
                raise ValueError("World/model/physics changed during readiness; manual review, no clock command")
            self.save("BOOTSTRAPPED_WAIT_CLOCK", readiness=report)
            return True
        except (OSError, ValueError, RuntimeError) as exc:
            self.save("START_FAILED", error=str(exc))
            self.rollback()
            return False

    def stop_auxiliary(self, child):
        if child.role not in {"AGENT", "GAZEBO"}:
            raise ValueError("Only auxiliary owned processes can receive graceful signals")
        self.track(child)
        members = self.host.members(child)
        for info in members:
            if not same_process(child.members.get(info["pid"]), info):
                raise ValueError("Unowned/reused PID: cleanup blocked")
        # Signals target verified tasks via pidfd, never a broad killpg/pkill.
        for info in sorted(members, key=lambda m: m["pid"] == child.owner["pid"]):
            try:
                self.host.signal_owned(info)
            except ProcessLookupError:
                pass
        until = time.monotonic()+self.timeout
        while self.host.members(child):
            if time.monotonic() >= until:
                raise ValueError(f"{child.role} did not exit gracefully; no force-kill")
            child.process.poll()
            time.sleep(.1)
        child.process.poll()

    def rollback(self):
        # Once PX4 was launched, never tear away its simulator/Agent, even on startup failure.
        if any(c.role == "PX4" for c in self.children):
            self.save("CLEANUP_REQUIRED", cleanup_reason="PX4 may be paused/initializing; explicit shutdown review required")
            return False
        try:
            self.host.stage_conflicts(self.profile, self.children, self.with_dds)
            for child in reversed(self.children):
                self.stop_auxiliary(child)
            self.host.stage_conflicts(self.profile, self.children, self.with_dds)
            self.save("ROLLED_BACK")
            return True
        except (OSError, ValueError) as exc:
            self.save("CLEANUP_REQUIRED", cleanup_reason=str(exc))
            return False

    def shutdown(self):
        try:
            world, _ = load_environment(self.profile)
            if (world.sdf_sha256 != self.state["sdf_sha256"] or
                    digest(self.profile.snapshot()) != self.state["profile_sha256"]):
                raise ValueError("Session world/profile changed; shutdown ownership review required")
            # Do not operate if a foreign PX4/Agent/world/controller has appeared.
            self.host.stage_conflicts(self.profile, self.children, self.with_dds)
            for child in self.children:
                self.track(child)
            px4 = next((c for c in self.children if c.role == "PX4"), None)
            if px4 and px4.process.poll() is None:
                gate = self.host.stop_gate(self.profile)
                self.save(shutdown_gate=gate)
                if (not gate.get("safe") or gate.get("paused") is not False or
                        gate.get("clock_progressing") is not True):
                    self.save("SHUTDOWN_BLOCKED", error="; ".join(gate.get("blockers") or
                              ["Fresh Disarmed/Landed and demonstrated clock progression required; no automatic unpause"]))
                    return False
                if self.shutdown_requested:
                    raise ValueError("Shutdown already requested; no automatic retry")
                if not same_process(px4.owner, self.host.ident(px4.owner["pid"])):
                    raise ValueError("PX4 process identity changed")
                for child in self.children:
                    if child.process.poll() is None and not same_process(child.owner, self.host.ident(child.owner["pid"])):
                        raise ValueError(f"{child.role} identity changed during shutdown probes")
                    self.track(child)
                self.shutdown_requested = True
                result = self.host.shutdown_px4(self.profile)
                self.save("STOPPING_PX4", shutdown_result=result)
                if result["exit"] != 0:
                    raise ValueError("Native shutdown request failed/timed out; no retry")
                until = time.monotonic()+self.timeout
                while px4.process.poll() is None or self.host.members(px4):
                    if time.monotonic() >= until:
                        raise ValueError("PX4/descendants did not exit; preserve Gazebo/Agent, no force-kill")
                    time.sleep(.1)
            # Native PX4 exit runs logger shutdown hooks; preserve its logs on disk.
            for role in ("AGENT", "GAZEBO"):
                for child in self.children:
                    if child.role == role:
                        self.stop_auxiliary(child)
            self.host.stage_conflicts(self.profile, self.children, self.with_dds)
            self.save("STOPPED")
            return True
        except (OSError, ValueError, RuntimeError) as exc:
            self.save("CLEANUP_REQUIRED", error=str(exc))
            return False

    def serve(self):
        """Foreground owner remains alive holding locks; requests are explicit operator actions."""
        pending_signal = [signal.SIGINT] if self.stop_requested else []
        def interrupted(signum, frame):
            pending_signal.append(signum)
        old = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
        try:
            while True:
                request = self.directory/"shutdown.request"
                if request.exists() or pending_signal:
                    if request.exists():
                        try:
                            if request.is_symlink() or request.stat().st_uid != os.getuid():
                                raise ValueError("Unsafe shutdown request file")
                            value = json.loads(request.read_text())
                            if value.get("session_id") != self.directory.name:
                                raise ValueError("Wrong session shutdown request")
                        except (OSError, ValueError) as exc:
                            request.rename(self.directory/("shutdown_rejected_"+uuid4().hex+".json"))
                            self.save("REQUEST_REJECTED", error=str(exc))
                            continue
                        request.rename(self.directory/("shutdown_processed_"+uuid4().hex+".json"))
                    pending_signal.clear()
                    if self.shutdown():
                        return
                    print(json.dumps(dict(state=self.state["state"], error=self.state.get("error"))), flush=True)
                for child in self.children:
                    try:
                        self.track(child)
                    except (OSError, ValueError) as exc:
                        self.save("CLEANUP_REQUIRED", error=str(exc))
                if all(not c.owner.get("identity_unproven") and c.process.poll() is not None
                       and not self.host.members(c) for c in self.children):
                    self.save("EXITED")
                    return
                if (any(c.process.poll() is not None for c in self.children)
                        and self.state["state"] != "CHILD_EXITED_REVIEW_REQUIRED"):
                    self.save("CHILD_EXITED_REVIEW_REQUIRED")
                time.sleep(.5)
        finally:
            for s, handler in old.items():
                signal.signal(s, handler)


def read_session(value, *, require_owner=True):
    path = Path(value).expanduser().resolve()
    if not path.is_relative_to(SESSION_ROOT.resolve()) or path == SESSION_ROOT.resolve():
        raise ValueError("Session must be an existing directory inside module sessions/")
    state = json.loads((path/"session.json").read_text())
    if state.get("session_id") != path.name or state.get("schema") != 1:
        raise ValueError("Invalid session state")
    if require_owner and not same_process(state["supervisor"], identity(state["supervisor"]["pid"])):
        raise ValueError("Supervisor absent/restarted: no adoption/signalling; manual ownership review needed")
    return path, state


def session_inventory(state):
    """Read-only /proc inventory, including unrecorded tasks in owned child SIDs.

    A manifest is evidence to compare, never permission to adopt or signal a task.
    """
    live = {int(p.name): info for p in Path('/proc').iterdir() if p.name.isdigit()
            and (info := identity(int(p.name))) is not None}
    saved = {state['supervisor']['pid']: state['supervisor']}
    sessions = set()
    for child in state['children']:
        owner = child['owner']
        if owner.get('identity_unproven') or 'sid' not in owner:
            raise ValueError('Unproven child identity; manual review required')
        sessions.add(owner['sid'])
        for record in [owner, *child['members']]:
            if record['pid'] in saved and not same_process(saved[record['pid']],record):
                raise ValueError('Ambiguous recorded process identity')
            saved[record['pid']] = record
    return dict(alive=[pid for pid,old in saved.items() if same_process(old,live.get(pid))],
        identity_mismatches=[pid for pid,old in saved.items() if pid in live and not same_process(old,live[pid])],
        unrecorded_child_tasks=[pid for pid,info in live.items() if info['sid'] in sessions and pid not in saved],
        absent=[pid for pid in saved if pid not in live])


def shutdown_review(path, state):
    """Current read-only checks; paused approval proposal is NOT shutdown approval.

    Also used by the new CLI before queuing to an already-running old supervisor.
    No DDS, anchor, flight-readiness gate, physics request or process mutation.
    """
    profile = Profile(**state['profile'])
    world, _ = load_environment(profile)
    if (digest(profile.snapshot()) != state['profile_sha256'] or world.sdf_sha256 != state['sdf_sha256']):
        raise ValueError('Session profile/SDF mismatch')
    expected_exe = str((Path(profile.px4_root).expanduser()/'build/px4_sitl_default/bin/px4').resolve())
    roots = {c['role']:c['owner'] for c in state['children']}
    if (set(roots) != ({'GAZEBO','PX4','AGENT'} if state['with_dds'] else {'GAZEBO','PX4'}) or
            roots['PX4'].get('exe') != expected_exe):
        raise ValueError('Unexpected session children / PX4 build identity')
    inventory = session_inventory(state)
    required = {state['supervisor']['pid'], *(r['pid'] for r in roots.values())}
    if (not required.issubset(inventory['alive']) or inventory['identity_mismatches'] or
            inventory['unrecorded_child_tasks']):
        raise ValueError('Session process identity/ownership mismatch')
    host = Host()
    # These are read-only registry records; no Process/Popen or adoption occurs.
    records = [Child(c['role'],None,c['owner'],{m['pid']:m for m in c['members']}) for c in state['children']]
    host.stage_conflicts(profile,records,state['with_dds'],supervisor=state['supervisor'])
    gate = host.stop_gate(profile)
    arm_on_boot = native_probe(profile,'param','show','COM_ARM_ON_BOOT')
    auto_arm_off = arm_on_boot['exit'] == 0 and bool(re.search(
        r'\bCOM_ARM_ON_BOOT\s+\[[^]]+\]\s*:\s*0\s*$',arm_on_boot['stdout'],re.M))
    if session_inventory(state) != inventory:
        raise ValueError('Process inventory changed during probes; no automatic retry')
    blockers = []
    if state['state'] not in {'SHUTDOWN_BLOCKED','BOOTSTRAPPED_WAIT_CLOCK'} or 'shutdown_result' in state:
        blockers.append('Native shutdown may already have been attempted; do not retry')
    if (path/'shutdown.request').exists():
        blockers.append('Shutdown request already pending; no duplicate queue')
    if not gate['safe']:
        blockers.extend(gate['blockers'])
    if not auto_arm_off:
        blockers.append('COM_ARM_ON_BOOT=0 not verified; no clock proposal')
    proposal = None
    # A finite cleanup window is an upper bound, not the old DDS 20000->25000 test.
    if not blockers and gate['world_before']['paused'] and gate['paused']:
        if (gate['clock_before_us'] != gate['clock_after_us'] or
                gate['world_before']['iterations'] != gate['world']['iterations']):
            blockers.append('Paused baseline changed during checks')
        else:
            proposal = dict(baseline_iterations=gate['world']['iterations'],
                baseline_clock_us=gate['clock_after_us'],additional_steps=5000,
                maximum_end_iterations=gate['world']['iterations']+5000,
                requires_separate_operator_clock_approval=True,
                command=shlex.join(['gz','service','-s',f'/world/{profile.world_name}/control',
                    '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000',
                    '--req','pause: true, multi_step: 5000']),
                rollback_pause_command=shlex.join(['gz','service','-s',f'/world/{profile.world_name}/control',
                    '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000',
                    '--req','pause: true']),
                note='Queue shutdown WHILE clock progresses, not after it auto-pauses. ACK is not clock evidence. '
                     'PX4 exit may stall lockstep before budget completion. No automatic second window.')
    can_queue = not blockers and gate['can_shutdown']
    return dict(result=('READY_FOR_NATIVE_SHUTDOWN_REQUEST' if can_queue else
                        'READY_FOR_SEPARATE_CLOCK_APPROVAL' if proposal and not blockers else 'NO_GO'),
        blockers=blockers,shutdown_gate=gate,inventory=inventory,auto_arm_disabled=bool(auto_arm_off),
        clock_proposal=proposal,shutdown_request_allowed=bool(can_queue),
        running_supervisor_code='Already imported; editing source does not hot-reload its policy',
        publishes_commands=False,controls_physics=False,live_control_go=False)


def stopped_check(state):
    """Do not infer process exit from cached STOPPED or native client ACK alone."""
    inventory = session_inventory(state)
    profile = Profile(**state['profile'])
    check = host_startup_check(profile,state['with_dds'],ignore_pids=(os.getpid(),))
    gone = (state['state'] in {'STOPPED','EXITED'} and not inventory['alive'] and
            not inventory['identity_mismatches'] and not inventory['unrecorded_child_tasks'] and
            not check['blockers'])
    return dict(result='ALL_CHILDREN_EXITED_START_CANDIDATE' if gone else 'NO_GO',
        session_state=state['state'],inventory=inventory,host_precheck=check,
        restart_allowed=bool(gone),launches_processes=False,live_control_go=False)


def check_anchor(bundle, state, current_identity, current_resets):
    """Read-only identity + world/profile check; never enables a follower."""
    from .environment_artifacts import load_bundle
    _, manifest, anchor, profile = load_bundle(bundle, current_identity)
    if (digest(profile.snapshot()) != state["profile_sha256"]
            or manifest["sdf_sha256"] != state["sdf_sha256"]):
        raise ValueError("Different session profile/world; recapture anchor")
    owned_px4 = next((c["owner"] for c in state["children"] if c["role"] == "PX4"), None)
    if (not owned_px4 or owned_px4["pid"] != current_identity.get("px4_pid")
            or owned_px4["start_ticks"] != current_identity.get("px4_start_ticks")
            or owned_px4["boot_id"] != current_identity.get("boot_id")):
        raise ValueError("Anchor not measured in this owned runtime session")
    gazebo = next((c for c in state["children"] if c["role"] == "GAZEBO"), {})
    if not any(m["pid"] == current_identity.get("gazebo_server_pid")
               and m["start_ticks"] == current_identity.get("gazebo_start_ticks")
               and m["boot_id"] == current_identity.get("boot_id") for m in gazebo.get("members", [])):
        raise ValueError("Gazebo server not owned by this session; recapture/review anchor")
    anchor.check_resets(current_resets)
    return dict(result="ANCHOR_IDENTITY_MATCH_ONLY", live_control_go=False,
                reset_counters=list(anchor.reset_counters), publishes_commands=False)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", default="warehouse", help="Builtin profile name or YAML file")
    sub = p.add_subparsers(dest="action", required=True)
    start = sub.add_parser("start")
    start.add_argument("--execute", action="store_true")
    start.add_argument("--without-dds", action="store_true")
    start.add_argument("--check-host", action="store_true", help="Read-only conflict check in dry-run")
    start.add_argument("--timeout", type=float, default=30, help="Per-stage deadline; probes have their own short timeouts")
    for action in ("shutdown", "shutdown-review", "verify-stopped", "status", "readiness", "check-anchor"):
        command = sub.add_parser(action)
        command.add_argument("session", help="Path printed by start --execute")
        if action == "shutdown":
            command.add_argument("--execute", action="store_true")
        if action == "verify-stopped":
            command.add_argument("--timeout",type=float,default=35,help="Read-only bounded wait, 1..120 wall seconds")
        if action == "check-anchor":
            command.add_argument("bundle")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.action == "start":
            if not 1 <= args.timeout <= 120:
                raise ValueError("Timeout must be 1..120 seconds")
            profile = load_profile(args.profile)
            plan = launch_plan(profile, not args.without_dds, args.profile, args.check_host)
            if not args.execute:
                print(json.dumps(dict(result="DRY_RUN", plan=plan, executor="FOREGROUND_EXPLICIT_EXECUTE_ONLY",
                                      writes_files=False, launches_processes=False), indent=2))
                return int(bool(plan.get("host_precheck", {}).get("blockers")))
            # Explicit startup only. Never call this branch during offline verification.
            if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
                raise ValueError("Linux pidfd APIs are required for safe ownership cleanup")
            with ExitStack() as locks:
                locks.enter_context(ExclusiveLock(SESSION_ROOT/".locks/lifecycle.lock"))
                locks.enter_context(ExclusiveLock(SESSION_ROOT/".locks"/f"instance_{profile.instance}.lock"))
                directory = secure_directory(SESSION_ROOT/uuid4().hex)
                supervisor = Supervisor(profile, plan, directory, with_dds=not args.without_dds, timeout=args.timeout)
                def stop_requested(signum, frame):
                    supervisor.stop_requested = True
                old_handlers = {s: signal.signal(s, stop_requested) for s in (signal.SIGINT, signal.SIGTERM)}
                try:
                    success = supervisor.start()
                    print(json.dumps(dict(session=str(directory), state=supervisor.state["state"],
                                          error=supervisor.state.get("error"), flight_go=False)), flush=True)
                    if success or supervisor.state["state"] == "CLEANUP_REQUIRED":
                        supervisor.serve()
                    return 0 if success else 1
                finally:
                    for s, handler in old_handlers.items():
                        signal.signal(s, handler)
        path, state = read_session(args.session, require_owner=args.action not in {"status","verify-stopped"})
        if args.action == "status":
            print(json.dumps(state, indent=2))
        elif args.action == "shutdown-review":
            report = shutdown_review(path,state)
            print(json.dumps(report,indent=2))
            return int(report['result'] == 'NO_GO')
        elif args.action == "verify-stopped":
            if not 1 <= args.timeout <= 120:
                raise ValueError('Timeout must be 1..120 seconds')
            until = time.monotonic()+args.timeout
            while True:
                report = stopped_check(state)
                if report['restart_allowed'] or time.monotonic() >= until:
                    print(json.dumps(report,indent=2))
                    return int(not report['restart_allowed'])
                time.sleep(.5)
                _,state = read_session(path,require_owner=False)
        elif args.action == "shutdown":
            if not args.execute:
                print(json.dumps(dict(result="DRY_RUN", session=str(path), would_request_shutdown=True,
                                      paused_policy="BLOCK_NO_UNPAUSE_NO_FORCE", stopped_processes=0)))
            else:
                review = shutdown_review(path,state)
                if not review['shutdown_request_allowed']:
                    raise ValueError('Shutdown not queued: '+review['result']+'; '+
                                     '; '.join(review['blockers'])+'; see shutdown-review. No automatic unpause/retry.')
                with (path/"shutdown.request").open("x") as stream:
                    json.dump(dict(session_id=path.name, requested_wall_time=time.time()), stream)
                print(json.dumps(dict(result="REQUEST_QUEUED_NOT_PROCESS_EXIT", session=str(path),
                                      next="Run verify-stopped before any start; supervisor independently rechecks safety")))
        else:
            profile = Profile(**state["profile"])
            world, _ = load_environment(profile)
            if world.sdf_sha256 != state["sdf_sha256"]:
                raise ValueError("SDF changed during session")
            report = readiness(profile, state["with_dds"], path)
            if args.action == "check-anchor":
                from .frames import RESET_FIELDS
                lp = report["fields"]["vehicle_local_position"]
                resets = [int(lp[k].split()[0]) for k in RESET_FIELDS]
                report = check_anchor(args.bundle, state, report["runtime_identity"], resets)
            print(json.dumps(report, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(json.dumps(dict(result="BLOCKED", error=str(exc), live_control_go=False)))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
