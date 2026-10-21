"""Demo preparation only: frozen PSO routes and one bounded read-only host check.

There is deliberately no execute, physics, model mutation or flight command here.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import uuid

import numpy as np

from .artifacts import write_json_exclusive
from .environments import MODULE, load_environment, load_profile
from .frames import enu_to_ned
from .geometry import FlightEnvironment
from .visualization.raw_pso_gazebo import SOURCE, load_routes

OUTPUT = MODULE / 'outputs_demo_first'
DEFAULT_SESSION = MODULE / 'sessions/0d3aeccae53a4a01a6096ccbf44661ac'
# Audit the existing controller; these values do not alter its limits.
CONTROLLER = dict(target_speed_mps=.2, actual_speed_limit_mps=.7,
                  waypoint_tolerance_m=.10, tracking_limit_m=.25,
                  waypoint_hold_sim_s=5., prestream_sim_s=1.2,
                  publish_hz=20, ack_timeout_wall_s=3.,
                  route_timeout_sim_s=120., land_timeout_wall_s=30.)


def new_output(kind):
    OUTPUT.mkdir(exist_ok=True)
    if OUTPUT.is_symlink():
        raise ValueError('Demo output must not be a symlink')
    path = OUTPUT / (kind+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    path.mkdir(exist_ok=False)
    return path


def topic_schema(profile):
    """Read installed message source, do not guess versioned topic suffixes."""
    root = Path(profile.px4_ros_setup).expanduser().parents[1] / 'src/px4_msgs/msg'
    result = {}
    for name, base in [('VehicleLocalPosition', '/fmu/out/vehicle_local_position'),
                       ('VehicleStatus', '/fmu/out/vehicle_status'),
                       ('VehicleCommandAck', '/fmu/out/vehicle_command_ack'),
                       ('TimesyncStatus', '/fmu/out/timesync_status'),
                       ('OffboardControlMode', '/fmu/in/offboard_control_mode'),
                       ('TrajectorySetpoint', '/fmu/in/trajectory_setpoint'),
                       ('VehicleCommand', '/fmu/in/vehicle_command')]:
        file = root / (name+'.msg')
        try:
            version = re.search(r'^uint32\s+MESSAGE_VERSION\s*=\s*(\d+)\b', file.read_text(), re.M)
            version = int(version[1]) if version else 0
            result[name] = dict(topic=base+(f'_v{version}' if version else ''),
                                message_version=version, source=str(file),
                                binding_and_graph_match='REQUIRES_FRESH_HOST_CHECK')
        except OSError:
            result[name] = dict(source=str(file), status='SOURCE_UNAVAILABLE')
    return result


def route_audit(route, world, env, *, validate_geometry=True):
    points = np.asarray(route.points, float)
    if validate_geometry:
        env.validate(points)
    cfg = CONTROLLER
    travel = route.length/cfg['target_speed_mps']
    holds = (len(points)-1)*cfg['waypoint_hold_sim_s']
    minimum = cfg['prestream_sim_s']+travel+holds
    t = cfg['tracking_limit_m']
    # Same safety envelope plus the controller's allowed tracking error.
    tube = FlightEnvironment(world.boxes, env.lower+t, env.upper-t,
                             env.radius, env.margin+t)
    failed = [i for i in range(len(points)-1)
              if tube.collision_mask(points[i:i+2])[0]]
    obstacles = []
    for i in failed:
        names = [box.name for box in world.boxes if FlightEnvironment(
            [box], [-1e6]*3, [1e6]*3, env.radius, env.margin+t
        ).collision_mask(points[i:i+2])[0]]
        obstacles.append(dict(segment=[i,i+1], obstacle_names=names,
                              bounds_violation=bool(((points[i:i+2] < tube.lower) |
                                                     (points[i:i+2] > tube.upper)).any())))
    # Static obstacle checks only. Ground contact and the actual landed pose
    # require separate native measurements; the SDF floor is NOT a frame anchor.
    columns = FlightEnvironment(world.boxes, [*env.lower[:2],world.floor_z-1.],
                                env.upper, env.radius, env.margin)
    takeoff = [[*points[0,:2], world.floor_z], points[0]]
    landing = [points[-1], [*points[-1,:2], world.floor_z]]
    return dict(algorithm=route.algorithm, seed=route.provenance['seed'],
                source_csv=route.provenance['csv'], csv_sha256=route.provenance['csv_sha256'],
                waypoint_count=len(points), length_m=route.length,
                start_enu_m=points[0].tolist(), goal_enu_m=points[-1].tolist(),
                height_enu_range_m=[float(points[:,2].min()),float(points[:,2].max())],
                minimum_landed_origin_z_for_relative_cap_m=float(points[:,2].max()-3.2),
                relative_height_cap_m=3.2, radius_m=env.radius, margin_m=env.margin,
                nominal_raw_geometry='PASS', tracking_tube_go=not failed,
                tracking_tube_failures=obstacles,
                takeoff_column_obstacle_clear=not bool(columns.collision_mask(takeoff)[0]),
                goal_land_column_obstacle_clear=not bool(columns.collision_mask(landing)[0]),
                floor_contact_and_takeoff_dynamics='NOT_PROVEN',
                minimum_route_time_sim_s=minimum,
                current_timeout_sim_s=cfg['route_timeout_sim_s'],
                current_timeout_go=minimum < cfg['route_timeout_sim_s'],
                frame_conversion='p_ned = anchor.px4_ned + R_ENU_NED * (p_enu - anchor.gazebo_enu)',
                absolute_ned_exported=False, fresh_anchor='REQUIRED_PER_FLIGHT',
                phases=['approved_native_takeoff_to_A','native_hold_at_A',
                        'offboard_prestream_and_ACK','all_17_raw_waypoints',
                        'goal_hold_5_sim_s','native_LAND_ACK_and_landed_disarmed'],
                flight_go=False, flight_tested=False)


def prepare():
    routes = load_routes()  # frozen CSV + provenance + continuous geometry, no optimizer
    profile = load_profile()
    world, env = load_environment(profile)
    if ((routes['standard'].points[0],routes['standard'].points[-1]) !=
            (routes['enhanced'].points[0],routes['enhanced'].points[-1])):
        raise ValueError('Standard/Enhanced must have identical Start/Goal')
    audits = {name:route_audit(route,world,env) for name,route in routes.items()}
    # A proposal for later controller work, not a change to the 120 s live gate.
    proposed = math.ceil((max(a['minimum_route_time_sim_s'] for a in audits.values())+30)/10)*10
    out = new_output('prepare')
    for name, route in routes.items():
        (out/(name+'_raw_enu.csv')).write_bytes(Path(route.provenance['csv']).read_bytes())
        relative = enu_to_ned(np.asarray(route.points)-route.points[0])
        with (out/(name+'_relative_ned_NOT_SETPOINTS.csv')).open('x',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['north_from_A_m','east_from_A_m','down_from_A_m'])
            writer.writerows(relative)
    report = dict(result='OFFLINE_PREPARED_PX4_BLOCKED', demo_status='BLOCKED',
                  px4_flight='NO_GO', output=str(out), source=str(SOURCE),
                  routes=audits, same_controller_config=CONTROLLER,
                  proposed_common_route_timeout_sim_s=proposed,
                  proposal_applied=False, topic_schema_from_source=topic_schema(profile),
                  blockers=['FRESH_HOST_SESSION_WORLD_GUI_NOT_VERIFIED',
                            'CURRENT_120S_TIMEOUT_TOO_SHORT_FOR_BOTH_ROUTES',
                            'FULL_ROUTE_TRACKING_TUBE_NOT_CLEAR',
                            'FRESH_DDS_HEADING_ANCHOR_AND_RELATIVE_HEIGHT_REQUIRED',
                            'EXISTING_FOLLOWER_REQUIRES_ALREADY_AIRBORNE_HOVER',
                            'TAKEOFF_LANDING_AND_FAILSAFE_NOT_LIVE_VALIDATED',
                            'GROUNDTRUTH_WATCHDOG_NOT_IN_LIVE_LOOP',
                            'SEPARATE_PHYSICS_AND_FLIGHT_APPROVAL'],
                  fallback_selected=False, live_control_go=False, publishes_commands=False)
    write_json_exclusive(out/'demo_plan.json',report)
    return report


def host_visibility():
    try:
        init = Path('/proc/1/cmdline').read_bytes()
        return 'isolated_sandbox' if b'codex-linux-sandbox' in init else 'host_terminal'
    except OSError:
        return 'unknown'


def host_check(session):
    """Stop early at the first infrastructure failure; no recovery or retry."""
    from .session_launcher import read_session, session_inventory, digest, identity, same_process
    from .runtime_preparation import owned_runtime, same_runtime
    from .runtime_readiness import probe, process_identity
    from .session_runtime import readiness
    from .visualization.raw_pso_gazebo import Gazebo, OWN_NAMES

    out = new_output('host_readonly')
    report = dict(result='BLOCKED', output=str(out), session=str(session),
                  process_visibility=host_visibility(), gui_render='OPERATOR_VISUAL_CHECK_REQUIRED',
                  live_control_go=False, publishes_commands=False, controls_physics=False,
                  model_mutations=False, fallback_selected=False, blockers=[])
    try:
        if report['process_visibility'] != 'host_terminal':
            raise ValueError('HOST_VISIBILITY_UNAVAILABLE: cannot infer host processes exited')
        _, state = read_session(session)  # validates supervisor boot/PID/start/exe
        profile = load_profile()
        world, _ = load_environment(profile)
        if state['profile_sha256'] != digest(profile.snapshot()) or state['sdf_sha256'] != world.sdf_sha256:
            raise ValueError('SESSION_PROFILE_OR_SDF_MISMATCH')
        inv = session_inventory(state)
        report['inventory'] = inv
        if (inv['identity_mismatches'] or any(not same_process(c['owner'],identity(c['owner']['pid']))
                                             for c in state['children'])):
            raise ValueError('SESSION_PROCESS_IDENTITY_NOT_CONFIRMED')
        processes = probe(['ps','-eo','pid=,comm=,args='],2)
        if processes['exit'] != 0 or processes['stderr'].strip():
            raise ValueError('PROCESS_SCAN_FAILED')
        runtime = process_identity(processes['stdout'])
        if runtime['px4_count'] != 1 or runtime['gazebo_server_count'] != 1 or not owned_runtime(state,runtime):
            raise ValueError('NEED_EXACTLY_ONE_LAUNCHER_OWNED_PX4_AND_GAZEBO')
        report.update(runtime_identity=runtime, session_id=state['session_id'])
        gz = Gazebo(read_only=True)
        models = gz.scene()
        report['scene'] = dict(world=world.name, clock=gz.baseline, services=gz.services,
                               x500_id=models['x500'].id,
                               demo_models={name:models[name].id for name in OWN_NAMES if name in models},
                               ownership_verified=False)
        native = readiness(profile,state['with_dds'],out)
        write_json_exclusive(out/'native_readiness.json',native)
        gz.guard()  # paused and unchanged clock/provider after bounded probes
        after = probe(['ps','-eo','pid=,comm=,args='],2)
        inv_after = session_inventory(state)
        if (after['exit'] != 0 or after['stderr'].strip() or
                not same_runtime(runtime,process_identity(after['stdout'])) or
                not same_process(state['supervisor'],identity(state['supervisor']['pid'])) or
                inv_after['identity_mismatches'] or
                any(not same_process(c['owner'],identity(c['owner']['pid'])) for c in state['children'])):
            raise ValueError('SESSION_CHANGED_DURING_READONLY_CHECK')
        report['native_gates'] = native['gates']
        report['heading_good_for_control'] = native['fields'].get('vehicle_local_position',{}).get('heading_good_for_control') == 'True'
        dds = native['dds']
        report['dds_evidence_level'] = dds.get('dds_evidence_level','NOT_PROBED')
        report['dds_note'] = 'No new messages while paused is not proof of DDS failure; no clock progression is requested.'
        controls = {k:v.get('control_publishers') for k,v in dds.get('graph',{}).items()
                    if k.startswith('/fmu/in/')}
        report['control_publishers'] = controls
        report['blockers'] = [k for k,v in native['gates'].items() if not v]
        expected = {'/fmu/in/offboard_control_mode','/fmu/in/trajectory_setpoint','/fmu/in/vehicle_command'}
        if set(controls) != expected or any(type(v) is not int or v != 0 for v in controls.values()):
            report['blockers'].append('CONTROL_PUBLISHERS_ZERO_NOT_PROVEN')
        report['result'] = 'READONLY_ENVIRONMENT_CANDIDATE' if not report['blockers'] else 'BLOCKED'
    except (OSError,ValueError,RuntimeError,KeyError,ImportError) as exc:
        report['blockers'].append(str(exc))
        if hasattr(exc,'code'):
            report['error_code'] = exc.code
    write_json_exclusive(out/'host_check.json',report)
    return report


def diagnose_transport(session):
    """One bounded observation across native CLI and server network settings.

    Does not require paused stats to *diagnose* a missing stats stream. It never
    grants readiness, constructs a mutation client or changes Transport settings.
    """
    from concurrent.futures import ThreadPoolExecutor
    from .session_launcher import read_session, identity, same_process
    from .runtime_readiness import probe, process_identity
    from .runtime_preparation import owned_runtime
    from .visualization.raw_pso_gazebo import TRANSPORT_CLI, TRANSPORT_ENV, WORLD

    out = new_output('transport_diagnostic')
    report = dict(result='READONLY_DIAGNOSTICS_ONLY', output=str(out), session=str(session),
                  live_control_go=False, visualization_go=False, controls_physics=False,
                  model_mutations=False, blockers=[])
    try:
        if host_visibility() != 'host_terminal':
            raise ValueError('HOST_VISIBILITY_UNAVAILABLE')
        _,state = read_session(session)
        scan = probe(['ps','-eo','pid=,comm=,args='],2)
        if scan['exit'] != 0 or scan['stderr'].strip():
            raise ValueError('PROCESS_SCAN_FAILED')
        runtime = process_identity(scan['stdout'])
        if runtime['px4_count'] != 1 or runtime['gazebo_server_count'] != 1 or not owned_runtime(state,runtime):
            raise ValueError('SESSION_RUNTIME_IDENTITY_MISMATCH')
        server = identity(runtime['gazebo_server_pid'])
        if server is None:
            raise ValueError('GAZEBO_SERVER_IDENTITY_UNAVAILABLE')
        report['runtime_identity'] = runtime
        keys = (*TRANSPORT_ENV,'IGN_PARTITION','IGN_IP')
        shell = {k:os.environ.get(k) for k in keys}
        report['caller_transport_env'] = shell
        report['process_transport_env'] = {}
        for name,pid in [('gazebo_server',server['pid']),('px4',runtime['px4_pid'])]:
            try:
                # Deliberately never capture the full environment or credentials.
                env = dict(row.split(b'=',1) for row in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0') if b'=' in row)
                filtered = {k:env.get(k.encode(),b'').decode(errors='replace') or None for k in keys}
                report['process_transport_env'][name] = dict(values=filtered,
                    differs_from_caller=[k for k in keys if filtered[k] != shell[k]])
            except OSError as exc:
                report['process_transport_env'][name] = dict(error=str(exc))
        topic = str(TRANSPORT_CLI.with_name('gz-transport-topic'))
        commands = dict(
            interfaces=['ip','-brief','address'],
            routes=['ip','-4','route','show'],
            stats_discovery=[topic,'-i','-t',f'/world/{WORLD}/stats'],
            # Natural duration permits CLI buffers to flush; outer deadline is
            # still finite. No message-count wait, retry or physics dependency.
            stats_samples=[topic,'-e','-t',f'/world/{WORLD}/stats','-d','3'],
            scene=[str(TRANSPORT_CLI),'-s',f'/world/{WORLD}/scene/info',
                   '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene',
                   '--timeout','5000','--req',''])
        with ThreadPoolExecutor(max_workers=len(commands)) as pool:
            futures = {name:pool.submit(probe,argv,6) for name,argv in commands.items()}
            evidence = {name:future.result() for name,future in futures.items()}
        write_json_exclusive(out/'transport_probes.json',evidence)
        report['probes'] = {k:dict(exit=v['exit'],stdout_bytes=len(v['stdout'].encode()),
                                 stderr=v['stderr'][:600]) for k,v in evidence.items()}
        for name in ('interfaces','routes','stats_discovery','stats_samples'):
            report[name] = evidence[name]['stdout'][:6000]
        scene = evidence['scene']
        if scene['exit'] == 0 and not scene['stderr'].strip():
            from google.protobuf import text_format
            from gz.msgs10.scene_pb2 import Scene
            try:
                msg = Scene(); text_format.Parse(scene['stdout'],msg)
                report['scene_response'] = dict(name=msg.name,model_count=len(msg.model),
                    x500_ids=[m.id for m in msg.model if m.name == 'x500'],
                    demo_models=[dict(name=m.name,id=m.id) for m in msg.model if m.name.startswith('pso_demo_visual_')],
                    ownership_verified=False)
                if not msg.model:
                    report['blockers'].append('SCENE_EMPTY_RESPONSE')
            except (ValueError,text_format.ParseError) as exc:
                report['blockers'].append('SCENE_INVALID_RESPONSE: '+str(exc)[:300])
        else:
            report['blockers'].append('SCENE_REQUEST_NOT_SUCCESSFUL')
        if (not same_process(server,identity(server['pid'])) or
                not same_process(state['supervisor'],identity(state['supervisor']['pid'])) or
                not owned_runtime(state,process_identity(probe(['ps','-eo','pid=,comm=,args='],2)['stdout']))):
            raise ValueError('SESSION_CHANGED_DURING_DIAGNOSTIC')
        report['note'] = 'Diagnostics only; exit=0 and GUI rendering alone do not prove fresh stats or flight readiness. No settings changed.'
    except (OSError,ValueError,RuntimeError,ImportError,KeyError) as exc:
        report['result'] = 'BLOCKED'
        report['blockers'].append(str(exc))
    write_json_exclusive(out/'transport_diagnostic.json',report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    sub.add_parser('prepare',help='Offline: copy both frozen raw paths and audit the existing flight controller')
    host = sub.add_parser('check-host',help='Read-only: launcher ownership, Scene, native PX4, short DDS subscribe')
    host.add_argument('--session',type=Path,default=DEFAULT_SESSION)
    diag = sub.add_parser('diagnose-transport',help='Read-only: bounded native stats/Scene discovery, network and filtered process environments')
    diag.add_argument('--session',type=Path,default=DEFAULT_SESSION)
    args = parser.parse_args(argv)
    try:
        if args.command == 'prepare':
            report = prepare()
        elif args.command == 'check-host':
            report = host_check(args.session)
        else:
            report = diagnose_transport(args.session)
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        print(json.dumps(dict(result='BLOCKED',error=str(exc),live_control_go=False)))
        return 2
    print(json.dumps(report,indent=2,allow_nan=False))
    return int(report['result'] == 'BLOCKED')


if __name__ == '__main__':
    raise SystemExit(main())
