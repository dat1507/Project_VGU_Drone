"""Read-only host preparation CLI. No execute, command, physics or flight action.

snapshot/check-anchor validate the current launcher owner BEFORE native probes.
observe only subscribes; clock-plan only prints a conditional operator procedure.
"""

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import time
from uuid import uuid4

import numpy as np

from .environments import MODULE, load_profile, load_environment
from .frames import FrameAnchor, RESET_FIELDS, enu_to_ned, convert_yaw
from .geometry import FlightEnvironment
from .json_utils import json_safe
from .runtime_readiness import inspect_runtime, probe, process_identity, topic_fields, number, pose_xyz
from .session_launcher import read_session, identity, same_process, digest
from .software_inventory import inventory
from .telemetry import simulation_us, gcs_progress

OFFLINE = MODULE/'outputs_c2/warehouse/offline_pso_20261019T021207Z/validated_run'
DEFAULT_SESSION = MODULE/'sessions/f9dc3f80b86541ceac91f65d435a9876'
IDENTITY_FIELDS = ('boot_id', 'px4_pid', 'px4_start_ticks', 'gazebo_server_pid', 'gazebo_start_ticks')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x') as f:
        json.dump(json_safe(value), f, indent=2, allow_nan=False)


def same_runtime(a, b):
    return all(a.get(k) is not None and a.get(k) == b.get(k) for k in IDENTITY_FIELDS)


def owned_runtime(state, current):
    px4 = [c for c in state['children'] if c['role'] == 'PX4']
    gz = [c for c in state['children'] if c['role'] == 'GAZEBO']
    if len(px4) != 1 or len(gz) != 1:
        return False
    owner = px4[0]['owner']
    return (owner['pid'] == current.get('px4_pid') and owner['start_ticks'] == current.get('px4_start_ticks')
            and owner['boot_id'] == current.get('boot_id') and
            any(m['pid'] == current.get('gazebo_server_pid') and
                m['start_ticks'] == current.get('gazebo_start_ticks') and
                m['boot_id'] == current.get('boot_id') for m in gz[0].get('members', [])))


def host_snapshot(session, expected_iterations, *, gate_phase="strict"):
    _, state = read_session(session)  # fails before IPC when owner is absent
    profile = load_profile()
    if state['profile_sha256'] != digest(profile.snapshot()):
        raise ValueError('Session/profile hash changed')
    for child in state['children']:
        if not same_process(child['owner'], identity(child['owner']['pid'])):
            raise ValueError('Launcher-owned child absent/restarted: '+child['role'])
    report, anchor, env = inspect_runtime(profile, expected_iterations, gate_phase=gate_phase)
    after = process_identity(probe(['ps', '-eo', 'pid=,comm=,args='])['stdout'])
    if not same_runtime(report['runtime_identity'], after) or not owned_runtime(state, after):
        raise ValueError('Runtime changed during capture or is not owned by launcher')
    if not same_process(state['supervisor'], identity(state['supervisor']['pid'])):
        raise ValueError('Launcher owner changed during capture')
    if state['sdf_sha256'] != report['world']['sdf_sha256']:
        raise ValueError('Session/SDF mismatch')
    report.update(session_id=state['session_id'], session_owner_verified=True,
                  profile_sha256=digest(profile.snapshot()), dds_evidence_level='NOT_PROBED',
                  live_control_go=False)
    return report, anchor, env


def anchor_packet(report, anchor):
    if anchor is None or not report.get('session_owner_verified'):
        raise ValueError('No measured, session-bound anchor')
    pose = report['evidence']['pose']['stdout']
    lp = topic_fields(report['evidence']['vehicle_local_position']['stdout'])
    gt = topic_fields(report['evidence']['vehicle_local_position_groundtruth']['stdout'])
    match = re.search(r'Pose \[ XYZ.*?\n\s*\[[^]]+\]\n\s*\[([^]]+)\]', pose)
    if not match:
        raise ValueError('Missing Gazebo RPY frame evidence')
    rpy = np.array([float(x) for x in match[1].split()])
    expected_heading = convert_yaw(rpy[2])
    gt_heading, ekf_heading = number(gt, 'heading'), number(lp, 'heading')
    angle = lambda x: abs((x+math.pi) % (2*math.pi)-math.pi)
    frame_gates = dict(gazebo_gt_position=float(np.linalg.norm(pose_xyz(pose)-anchor.gazebo_enu)) <= .02,
        gazebo_gt_heading=angle(gt_heading-expected_heading) <= .02,
        ekf_gt_heading=angle(ekf_heading-gt_heading) <= .15,
        heading_good_for_control=lp.get('heading_good_for_control') == 'True',
        synchronized_native_samples=abs(anchor.gazebo_timestamp_us-anchor.px4_timestamp_us) <= 20000)
    return dict(schema='runtime_anchor_candidate_v1', anchor=anchor.as_dict(),
        runtime_identity=report['runtime_identity'], session_id=report['session_id'],
        sdf_sha256=report['world']['sdf_sha256'], profile_sha256=report['profile_sha256'],
        world_name=report['world']['name'], captured_wall_time=report['captured_wall_time'],
        native_gates=report['gates'], frame_gates=frame_gates,
        frame_evidence=dict(gazebo_rpy=rpy.tolist(), gazebo_expected_ned_heading=expected_heading,
                            gt_ned_heading=gt_heading, ekf_ned_heading=ekf_heading),
        candidate_only=True, flight_ready=False, dds_proven=False, live_control_go=False)


def check_anchor_packet(packet, fresh, current_anchor, now=None):
    now = time.time() if now is None else now
    reasons = []
    if packet.get('schema') != 'runtime_anchor_candidate_v1':
        reasons.append('ANCHOR_SCHEMA')
    if not fresh.get('session_owner_verified') or not same_runtime(packet['runtime_identity'], fresh['runtime_identity']):
        reasons.append('RUNTIME_IDENTITY')
    for k in ('session_id', 'profile_sha256'):
        if packet[k] != fresh[k]:
            reasons.append(k.upper())
    if packet['sdf_sha256'] != fresh['world']['sdf_sha256'] or packet['world_name'] != fresh['world']['name']:
        reasons.append('SDF_OR_WORLD')
    if not 0 <= now-fresh['captured_wall_time'] <= 30 or not 0 <= now-packet['captured_wall_time'] <= 30:
        reasons.append('WALL_AGE')
    try:
        stored = FrameAnchor.from_dict(packet['anchor'])
        if current_anchor is None:
            raise ValueError('Missing current anchor')
        stored.check_resets(current_anchor.reset_counters)
        delta_world = np.array(current_anchor.gazebo_enu)-stored.gazebo_enu
        delta_ekf = np.array(current_anchor.px4_ned)-stored.px4_ned
        if np.linalg.norm(enu_to_ned(delta_world)-delta_ekf) > .15:
            reasons.append('ANCHOR_TRANSLATION_DRIFT')
        current_packet = anchor_packet(fresh, current_anchor)
        if not all(current_packet['frame_gates'].values()) or not all(packet['frame_gates'].values()):
            reasons.append('FRAME_OR_HEADING')
        if fresh.get('problems'):
            reasons.append('NATIVE_GATES')
    except (ValueError, KeyError, TypeError):
        reasons.append('ANCHOR_RESET_OR_MEASUREMENT')
    return dict(result='ANCHOR_MATCH_ONLY' if not reasons else 'NO_GO', anchor_valid=not reasons,
                problems=reasons, dds_proven=False, live_control_go=False, publishes_commands=False)


def candidate_assessment(anchor=None, anchor_valid=False):
    m = json.loads((OFFLINE/'selected_enhanced/execution_candidate.json').read_text())
    index = json.loads((OFFLINE/'artifact_sha256.json').read_text())
    assert sha(OFFLINE/'selected_enhanced/execution_candidate.json') == index['selected_enhanced/execution_candidate.json']
    root = OFFLINE/'selected_enhanced'
    def load(name):
        with (root/name).open() as f:
            reader = csv.reader(f)
            if next(reader) != ['x_enu_m', 'y_enu_m', 'z_enu_m']:
                raise ValueError('CSV frame/schema changed')
            return np.array([[float(x) for x in row] for row in reader])
    if sha(root/m['parent_raw_csv']) != m['parent_raw_csv_sha256'] or sha(root/m['segment_csv']) != m['segment_csv_sha256']:
        raise ValueError('Candidate CSV hash changed')
    raw, leg = load(m['parent_raw_csv']), load(m['segment_csv'])
    if (m['algorithm'] != 'enhanced' or m['seed'] != 8 or m['segment_indices'] != [0, 1] or
            not np.array_equal(leg, raw[:2]) or not np.array_equal(raw[[0,-1]], [m['parent_start_enu'],m['parent_goal_enu']]) or
            m['radius_m'] != .6 or m['margin_m'] != .25):
        raise ValueError('Parent/segment provenance or envelope changed')
    world, env = load_environment(load_profile())
    old_geometry = json.loads((OFFLINE/'geometry.json').read_text())
    if world.sdf_sha256 != old_geometry['world']['sdf_sha256']:
        raise ValueError('Offline SDF changed')
    env.validate(leg)
    length = float(np.linalg.norm(leg[1]-leg[0]))
    tube = world.environment(1.25, 2.95, radius=.6, margin=.5)
    tube_ok = not bool(tube.collision_mask(leg)[0])
    result = dict(result='NO_GO', algorithm='enhanced', seed=8, parent_csv_sha256=m['parent_raw_csv_sha256'],
        segment_csv_sha256=m['segment_csv_sha256'], start_enu=leg[0].tolist(), waypoint_enu=leg[1].tolist(),
        length_m=length, segment_delta_ned=enu_to_ned(leg[1]-leg[0]).tolist(),
        nominal_segment_collision_free=True, tracking_tube_025m_clear=tube_ok,
        radius_m=.6, margin_m=.25, target_speed_limit_mps=.2, actual_speed_limit_mps=.7,
        arrival_tolerance_m=.1, tracking_limit_m=.25, hover_seconds=5,
        publish_hz=20, prestream_seconds=1.2, ack_timeout_wall_s=3,
        route_timeout_s=120, land_timeout_wall_s=30,
        segment_min_time_s=1.2+length/.2+5, acceleration_bounded_by_current_follower=False,
        proposed_start_native_hold_s=3, proposed_native_takeoff_budget_s=30,
        estimated_total_mission_budget_s=30+3+1.2+length/.2+5+3+30+20,
        timing_estimate_is_not_dynamics_certificate=True,
        current_follower_auto_takeoff=False, first_waypoint_stop='two-point execution prefix, hold 5 s then request LAND; no next waypoint',
        ground_contact_dynamics_proven=False, live_control_go=False, anchor_valid=anchor_valid,
        blockers=['DDS_SESSION_WINDOW', 'BOUNDED_ACCELERATION_NOT_IMPLEMENTED',
                  'GROUND_CONTACT_DYNAMICS_UNPROVEN', 'FAILSAFE_POLICY', 'WATCHDOG_NOT_IN_LIVE_CONTROL_LOOP',
                  'SEPARATE_FLIGHT_APPROVAL'])
    if anchor is not None:
        ground = np.array(anchor.gazebo_enu)
        land = np.array([*leg[-1,:2], ground[2]])
        transitions = np.array([ground, leg[0], leg[0], leg[1], leg[1], land])
        # All collision boxes and XY envelope retained for takeoff/landing.
        # Floor-envelope violation near contact is explicitly NOT waived.
        columns = FlightEnvironment(world.boxes, [*env.lower[:2], min(ground[2], world.floor_z)-1],
                                    env.upper, .6, .25)
        result['transitions'] = dict(phase_order=['takeoff','start_hold','pso_leg','waypoint_hold','land'],
            obstacle_box_segments_clear=not bool(columns.collision_mask(transitions)[0]),
            floor_envelope_all_segments_clear=bool(transitions[:,2].min() > world.floor_z+.85),
            relative_height_max_m=float(transitions[:,2].max()-ground[2]),
            relative_height_go=bool(transitions[:,2].max()-ground[2] <= 3.2),
            start_xy_error_m=float(np.linalg.norm(ground[:2]-leg[0,:2])),
            enu=transitions.tolist())
        if anchor_valid:
            result['transitions']['ned'] = [anchor.to_ned(p).tolist() for p in transitions]
            result['transitions']['round_trip_error_m'] = max(float(np.linalg.norm(anchor.to_enu(anchor.to_ned(p))-p)) for p in transitions)
            result['ned_is_diagnostic_only'] = True
        else:
            result['blockers'].append('FRESH_ANCHOR_CHECK_REQUIRED_BEFORE_NED_CONVERSION')
    else:
        result['blockers'].append('FRESH_ANCHOR_AND_TRANSITION_POSES_MISSING')
    return result


def clock_plan(report):
    problems = list(report.get('problems', []))
    if not report.get('session_owner_verified'):
        problems.append('CURRENT_SESSION_NOT_VERIFIED')
    param = report['evidence'].get('params_COM_ARM_ON_BOOT', {}).get('stdout', '')
    if not re.search(r'COM_ARM_ON_BOOT\s+\[[^]]+\]\s*:\s*0\s*$', param, re.M):
        problems.append('AUTO_ARM_NOT_CONFIRMED_DISABLED')
    iteration = re.search(r'^iterations: (\d+)$', report['evidence']['stats']['stdout'], re.M)
    if not iteration:
        problems.append('ITERATIONS_MISSING')
    return dict(result='NO_GO' if problems else 'CLOCK_WINDOW_PLAN_ONLY', problems=problems,
        start_iterations=int(iteration[1]) if iteration else None, additional_steps=5000,
        expected_end_iterations=int(iteration[1])+5000 if iteration else None,
        native_start_us=simulation_us(report['evidence']['stats']['stdout']),
        physics_seconds=5, requires_separate_approval=True, executed=False,
        preconditions=['fresh native checks, unchanged owner/profile/SDF',
                       'no control publishers; armed=false; landed=true; COM_ARM_ON_BOOT=0',
                       'observer SUBSCRIPTIONS_READY; max_step_size in unchanged SDF = 0.001'],
        operator_command="gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true, multi_step: 5000'",
        rollback_command="gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'",
        rollback='Operator pauses on unexpected motion/arming, wrong iteration/identity, or >20 s wall; no retry. Require exact end iterations, +5 native seconds and paused=true. Observer cannot control physics.',
        live_control_go=False)


def window_native_state(session):
    """Bounded lightweight checks on a worker; DDS executor keeps spinning."""
    started = time.monotonic()
    _, state = read_session(session)
    for child in state['children']:
        if not same_process(child['owner'], identity(child['owner']['pid'])):
            raise ValueError('Launcher-owned process changed during clock handshake')
    profile = load_profile()
    world, _ = load_environment(profile)
    current = process_identity(probe(['ps', '-eo', 'pid=,comm=,args='], 1)['stdout'])
    if (not owned_runtime(state, current) or state['profile_sha256'] != digest(profile.snapshot()) or
            state['sdf_sha256'] != world.sdf_sha256):
        raise ValueError('Owned session/profile/SDF changed')
    binary = Path(profile.px4_root).expanduser()/'build/px4_sitl_default/bin/px4-listener'
    evidence = {}
    for key, argv in {
        'armed': [binary, '--instance', '0', 'actuator_armed', '-n', '1'],
        'landed': [binary, '--instance', '0', 'vehicle_land_detected', '-n', '1'],
        'position': [binary, '--instance', '0', 'vehicle_local_position', '-n', '1'],
        'auto_arm': [binary.parent/'px4-param', '--instance', '0', 'show', 'COM_ARM_ON_BOOT'],
        'stats': ['gz','topic','-e','-t','/world/warehouse_world/stats','-n','1'],
    }.items():
        evidence[key] = probe(argv, 1)
        if evidence[key]['exit'] != 0:
            raise ValueError('Read-only window probe failed: '+key)
    stats = evidence['stats']['stdout']
    iteration = re.search(r'^iterations: (\d+)$', stats, re.M)
    position = topic_fields(evidence['position']['stdout'])
    if (not math.isfinite(number(position, 'heading')) or
            not all(position.get(k) == 'True' for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')) or
            not all(math.isfinite(number(position,k)) for k in ('x','y','z'))):
        raise ValueError('Invalid native heading or EKF local position')
    return dict(runtime_identity=current, session_id=state['session_id'],
        profile_sha256=state['profile_sha256'], sdf_sha256=world.sdf_sha256,
        iterations=int(iteration[1]) if iteration else None, clock_us=simulation_us(stats),
        paused='paused: true' in stats,
        disarmed=topic_fields(evidence['armed']['stdout']).get('armed') == 'False',
        landed=topic_fields(evidence['landed']['stdout']).get('landed') == 'True',
        auto_arm_disabled=bool(re.search(r'COM_ARM_ON_BOOT\s+\[[^]]+\]\s*:\s*0\s*$', evidence['auto_arm']['stdout'], re.M)),
        reset_counters=[int(number(position,k)) for k in RESET_FIELDS],
        heading=number(position,'heading'), heading_good_for_control=position.get('heading_good_for_control'),
        errors=[], probe_started_monotonic=started, captured_monotonic=time.monotonic())


def finite_window_plan(report, software):
    """This task is pinned to 20000 -> 25000, never rebase silently."""
    plan = clock_plan(report)
    if plan['start_iterations'] != 20000 or plan['native_start_us'] != 20_000_000:
        plan['problems'].append('EXPECTED_20000_ITERATIONS_AND_20_SECONDS')
    reference = MODULE/'outputs_c2/warehouse/runtime_check_ad242a1bffaf4931bf02fbf8edf61752/native_after.json'
    previous = json.loads(reference.read_text())
    if not same_runtime(report['runtime_identity'], previous['runtime_identity']) or report['session_id'] != DEFAULT_SESSION.name:
        plan['problems'].append('REFERENCE_SESSION_IDENTITY_CHANGED')
    if (software.get('errors') or not software.get('schemas') or not all(
            s['serialized_fields_match'] and s['built_version_matches'] for s in software['schemas'].values())):
        plan['problems'].append('INSTALLED_SCHEMA_CHECK')
    plan.update(result='NO_GO' if plan['problems'] else 'CONDITIONAL_20000_TO_25000_PLAN',
                expected_end_iterations=25000, expected_end_sim_time_us=25_000_000,
                graph_preconditions_verified=False, live_control_go=False)
    if plan['problems']:
        plan.update(operator_command=None, expected_end_iterations=None, expected_end_sim_time_us=None,
                    needs_new_plan_from_actual_data=True)
    return plan


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, default=DEFAULT_SESSION)
    p.add_argument('--expect-iterations', type=int, default=15000)
    p.add_argument('--output', type=Path)
    sub = p.add_subparsers(dest='action', required=True)
    sub.add_parser('snapshot')
    sub.add_parser('candidate')
    chk = sub.add_parser('check-anchor')
    chk.add_argument('anchor_packet', type=Path)
    obs = sub.add_parser('observe')
    obs.add_argument('--wall-seconds', type=float, default=40)
    coordinated = sub.add_parser('observe-window', help='Read-only handshake; operator controls physics separately')
    coordinated.add_argument('--wall-seconds', type=float, default=60)
    operator = sub.add_parser('check-window', help='Live observer challenge + print command only, never execute physics')
    operator.add_argument('directory', type=Path)
    operator.add_argument('--operator-approval', required=True)
    args = p.parse_args(argv)
    if args.action in ('observe','observe-window') and not 5 <= args.wall_seconds <= 60:
        p.error('observe requires 5..60 bounded wall seconds')
    dest = args.output or MODULE/'outputs_c2/warehouse'/('runtime_check_'+uuid4().hex)
    dest.mkdir(parents=True, exist_ok=False)
    try:
        if args.action == 'check-window':
            from .clock_window import check_operator_window
            result = check_operator_window(args.directory, args.operator_approval,
                                           lambda: window_native_state(args.session))
        elif args.action == 'candidate':
            result = candidate_assessment()
        else:
            report, anchor, _ = host_snapshot(args.session, args.expect_iterations)
            write(dest/'native_snapshot.json', report)
            packet = anchor_packet(report, anchor)
            write(dest/'anchor_candidate.json', packet)
            write(dest/'candidate.json', candidate_assessment(anchor, False))
            software = inventory(load_profile())
            write(dest/'software_inventory.json', software)
            if args.action == 'check-anchor':
                result = check_anchor_packet(json.loads(args.anchor_packet.read_text()), report, anchor)
                if result['anchor_valid']:
                    write(dest/'candidate_checked_anchor.json', candidate_assessment(anchor, True))
            elif args.action in ('observe', 'observe-window'):
                from .dds_observer import observe
                from .groundtruth import GazeboPoseObserver
                entity = re.search(r'Model: \[(\d+)\]', report['evidence']['pose']['stdout'])
                if not entity:
                    raise ValueError('Missing Gazebo x500 entity ID')
                gt = GazeboPoseObserver('warehouse_world', 'x500', int(entity[1]))
                monitor = None
                if args.action == 'observe-window':
                    from .clock_window import WindowMonitor
                    plan = finite_window_plan(report, software)
                    write(dest/'clock_window_plan.json', plan)
                    if plan['problems']:
                        raise ValueError('Finite window preconditions: '+', '.join(plan['problems']))
                    baseline = window_native_state(args.session)
                    if (not same_runtime(report['runtime_identity'], baseline['runtime_identity']) or
                            baseline['session_id'] != report['session_id'] or
                            baseline['profile_sha256'] != report['profile_sha256'] or
                            baseline['sdf_sha256'] != report['world']['sdf_sha256']):
                        raise ValueError('Session changed between preconditions and observer startup')
                    from .clock_window import WindowProtocol
                    preliminary = WindowProtocol(baseline)
                    preliminary.native(baseline, time.monotonic())
                    if preliminary.problems:
                        raise ValueError('Clock baseline changed before observers started')
                    write(dest/'window_baseline.json', baseline)
                    monitor = WindowMonitor(dest, baseline, lambda: window_native_state(args.session), gt, plan)
                try:
                    if monitor is None:
                        gt.start()
                    observed = observe(load_profile(), args.wall_seconds, dest/'ros_logs', monitor=monitor,
                        ready=None if monitor else lambda: print('SUBSCRIPTIONS_READY; NO PHYSICS REQUEST; operator approval still required', flush=True))
                finally:
                    groundtruth = gt.finish()
                    write(dest/'groundtruth.json', groundtruth)
                write(dest/'dds_raw_observation.json', observed)
                after, _, _ = host_snapshot(args.session, None)
                write(dest/'native_after.json', after)
                observed['runtime_identity_unchanged'] = same_runtime(report['runtime_identity'], after['runtime_identity'])
                observed['native_clock_delta_us'] = simulation_us(after['evidence']['stats']['stdout'])-simulation_us(report['evidence']['stats']['stdout'])
                observed['gcs_rx_and_snapshot_progress'] = gcs_progress(report['gcs_evidence'], after['gcs_evidence'])
                from .dds_evidence import assess_window
                window = assess_window(report, after, observed, groundtruth, software)
                if monitor is not None and observed.get('coordination', {}).get('problems'):
                    window.update(result='NO_GO', session_data_bound=False,
                                  problems=window['problems']+observed['coordination']['problems'])
                if monitor is not None:
                    monitor.protocol.transition('ASSESS_DDS_AND_SESSION', time.monotonic())
                    monitor.protocol.transition('PASS' if window['result'] == 'PASS_FINITE_CLOCK_EVIDENCE_ONLY' else 'NO_GO', time.monotonic())
                    monitor.status(time.monotonic())
                    observed['coordination']['state'] = monitor.protocol.state
                write(dest/'window_assessment.json', window)
                observed['session_data_bound'] = window['session_data_bound']
                observed['native_clock_progression_proven'] = window['result'] == 'PASS_FINITE_CLOCK_EVIDENCE_ONLY'
                observed['result'] = ('DATA_OBSERVED_SEE_WINDOW_ASSESSMENT' if observed.get('dds_messages_received') else
                    'PAUSED_NO_NEW_MESSAGES_NOT_A_DDS_FAILURE' if observed['native_clock_delta_us'] == 0 else
                    'NO_GO_NO_DDS_MESSAGES_WHILE_CLOCK_ADVANCED')
                write(dest/'dds_observation.json', observed)
                result = dict(result=window['result'], dds_evidence_level=observed.get('dds_evidence_level', 'NO_DATA'),
                              live_control_go=False, reason='Window/session correlation and flight gates must be reviewed')
            else:
                write(dest/'clock_window_plan.json', clock_plan(report))
                result = dict(result='GO_FOR_NEXT_READONLY_CHECK' if not report['problems'] else 'NO_GO',
                    problems=report['problems'], anchor_candidate_only=True,
                    frame_gates=packet['frame_gates'], dds_evidence_level='NOT_PROBED', live_control_go=False)
        result.update(output=str(dest), publishes_commands=False, controls_physics=False)
        write(dest/'result.json', result)
        print(json.dumps(result, indent=2))
        return 2 if result['result'] == 'NO_GO' else 0
    except (ValueError, OSError, KeyError, RuntimeError, ImportError, AssertionError) as exc:
        result = dict(result='NO_GO', error=str(exc), output=str(dest), publishes_commands=False,
                      controls_physics=False, live_control_go=False)
        write(dest/'error.json', result)
        print(json.dumps(result, indent=2))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
