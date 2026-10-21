"""Opt-in ROS adapter for integrated_flight. No physics/model API calls.

Importing this file does not initialize ROS or create publishers. All controllers
remain absent until an owned native snapshot and fresh measured anchor pass.
"""
import json
import hashlib
import math
import os
import re
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from .json_utils import json_safe as _json_safe
from .dds_evidence import LiveEpochClock, REQUIRED
from .frames import FrameAnchor, RESET_FIELDS, convert_yaw
from .geometry import vector
from .integrated_flight import (CONFIG, CONFIRMATION, Mission, Sample,
                                contact_environment, transition_checks)
from .safety import Freshness, quaternion_tilt, topic_name
from .demo_policy import (DEMO_CONFIRMATION,RISK_EVENT,CEILING_EVENT,ObservedCollision,localhost_ros,
                          position_environment,verify_sitl)
from .runtime_safety import WatchSample


def validate_graph(graph, own_publishers=False, *, require_sitl_readers=False, required_outputs=7):
    outputs = [v for k,v in graph.items() if k.startswith('/fmu/out/')]
    if len(outputs) != required_outputs:
        raise ValueError('PX4_OUTPUT_GRAPH_INCOMPLETE:expected='+str(required_outputs)+',found='+str(len(outputs)))
    for topic, endpoint in ((k,v) for k,v in graph.items() if k.startswith('/fmu/out/')):
        if endpoint['publishers'] != 1:
            raise ValueError('PX4_TOPIC_PUBLISHER_COUNT:'+topic+':'+str(endpoint['publishers']))
        if len(endpoint['gids']) != 1:
            raise ValueError('PX4_TOPIC_GID_COUNT:'+topic+':'+str(len(endpoint['gids'])))
        if endpoint['types'] != [endpoint['expected_type']]:
            raise ValueError('PX4_TOPIC_SCHEMA_MISMATCH:'+topic+':'+str(endpoint['types']))
    prefixes = {tuple(v['gids'][0][:12]) for v in outputs}
    if len(prefixes) != 1 or not any(next(iter(prefixes))):
        raise ValueError('DDS_PARTICIPANT_MISMATCH')
    inputs = [v for k,v in graph.items() if k.startswith('/fmu/in/')]
    if len(inputs) != 3 or any(v['publishers'] != int(own_publishers) or v['readers'] != 1 for v in inputs):
        raise ValueError('UNEXPECTED_CONTROL_PUBLISHER_OR_PX4_READER')
    if require_sitl_readers:
        prefix=next(iter(prefixes))
        if any(len(v.get('reader_gids',[]))!=1 or tuple(v['reader_gids'][0][:12])!=prefix
               or v.get('reader_types')!=[v.get('expected_type')] for v in inputs):
            raise ValueError('CONTROL_READER_NOT_THE_VERIFIED_SITL_PARTICIPANT')
    return next(iter(prefixes))


class NativeStatusMonitor:
    """Read-only uORB status bridge for PX4 topics omitted from dds_topics.yaml."""
    def __init__(self, profile, *, poll_interval=.4, command_timeout=.8):
        self.bin_dir=Path(profile.px4_root).expanduser()/'build/px4_sitl_default/bin'
        self.instance=str(profile.instance)
        self.poll_interval=float(poll_interval)
        self.command_timeout=float(command_timeout)
        self.lock=threading.Lock()
        self.stop_event=threading.Event()
        self.records={}
        self.thread=None

    def _probe(self, key, argv):
        try:
            result=subprocess.run([str(x) for x in argv],capture_output=True,text=True,
                                  timeout=self.command_timeout)
            record=dict(exit=result.returncode,stdout=result.stdout,stderr=result.stderr,
                        received=time.monotonic())
        except (OSError,subprocess.TimeoutExpired) as exc:
            record=dict(exit=None,stdout='',stderr=str(exc),received=time.monotonic())
        with self.lock:
            old=self.records.get(key,{})
            match=re.search(r'^\s*timestamp:\s*(\d+)',record['stdout'],re.M)
            stamp=int(match[1]) if match else None
            record['timestamp_us']=stamp
            record['progressed']=record['received'] if stamp is not None and stamp != old.get('timestamp_us') else old.get('progressed')
            self.records[key]=record

    def _run(self):
        while not self.stop_event.is_set():
            self._probe('actuator_armed',[self.bin_dir/'px4-listener','--instance',self.instance,'actuator_armed','-n','1'])
            self._probe('health_report',[self.bin_dir/'px4-listener','--instance',self.instance,'health_report','-n','1'])
            self._probe('logger',[self.bin_dir/'px4-logger','--instance',self.instance,'status'])
            self.stop_event.wait(self.poll_interval)

    def start(self):
        if self.thread is not None: raise RuntimeError('Native status monitor already started')
        self.thread=threading.Thread(target=self._run,name='px4-readonly-status',daemon=True)
        self.thread.start()

    def snapshot(self, native_us, now=None):
        now=time.monotonic() if now is None else now
        with self.lock: records={k:dict(v) for k,v in self.records.items()}
        result={}
        for key in ('actuator_armed',):
            item=records.get(key)
            if (item is None or item['exit'] != 0 or item['timestamp_us'] is None or
                    now-item['received'] > 1.5 or now-(item['progressed'] or 0.) > 1.5 or
                    abs(int(native_us)-item['timestamp_us']) > 1_500_000):
                raise ValueError('NATIVE_PX4_STATUS_STALE:'+key)
            from .runtime_readiness import topic_fields
            result[key]=topic_fields(item['stdout'])
        logger=records.get('logger')
        health=records.get('health_report')
        health_fresh=bool(health and health['exit'] == 0 and health['timestamp_us'] is not None and
            now-health['received'] <= 1.5 and now-(health['progressed'] or 0.) <= 1.5 and
            abs(int(native_us)-health['timestamp_us']) <= 1_500_000)
        result['health_report']=None
        if health_fresh:
            from .runtime_readiness import topic_fields
            result['health_report']=topic_fields(health['stdout'])
        result['health_report_fresh']=health_fresh
        result['logger_active']=bool(logger and logger['exit'] == 0 and now-logger['received'] <= 1.5 and
                                     'Full File Logging Running:' in logger['stdout'])
        if health_fresh:
            result['health_reason']=('health_error_flags='+result['health_report'].get('health_error_flags','unknown')+
                ',arming_check_error_flags='+result['health_report'].get('arming_check_error_flags','unknown')+
                ',arming_check_warning_flags='+result['health_report'].get('arming_check_warning_flags','unknown'))
        else:
            result['health_reason']='health_report_stale_or_not_published'
        return result

    def finish(self):
        self.stop_event.set()
        if self.thread is not None: self.thread.join(timeout=3.)


def measured_anchor(native_anchor, pose, position, native_us):
    """Translation may be confirmed/refreshed only by paired actual samples."""
    resets = tuple(int(getattr(position,k)) for k in RESET_FIELDS)
    native_anchor.check_resets(resets)
    if native_us < native_anchor.px4_timestamp_us or pose['native_timestamp_us'] < native_anchor.gazebo_timestamp_us:
        raise ValueError('ANCHOR_MEASUREMENT_PREDATES_NATIVE_PROOF')
    if abs(native_us-pose['native_timestamp_us']) > 20_000:
        raise ValueError('ANCHOR_PAIR_EXCEEDS_20MS')
    q = pose['quaternion_enu']
    quaternion_tilt(q)
    w,x,y,z = q
    yaw_enu = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    error = abs((position.heading-convert_yaw(yaw_enu)+math.pi) % (2*math.pi)-math.pi)
    if not math.isfinite(error) or error > .15:
        raise ValueError('EKF_GAZEBO_HEADING_FRAME')
    ned = vector([position.x,position.y,position.z])
    if np.linalg.norm(native_anchor.to_enu(ned)-pose['position_enu']) > .15:
        raise ValueError('NATIVE_ANCHOR_TRANSLATION_DRIFT')
    return FrameAnchor(tuple(pose['position_enu']),tuple(ned),pose['native_timestamp_us'],
                       native_us,resets,'px4_groundtruth_and_local_position')


def require_readiness_progress(first, current):
    """Observe external clock progression; never control Gazebo physics.

    Discovery or one fresh packet per topic cannot authorize activation. Every
    required normalized DDS source and Gazebo GT must advance in this epoch.
    """
    for key in (*REQUIRED, 'failsafe_flags', 'gazebo_groundtruth'):
        before, after = first[key], current[key]
        if not (math.isfinite(before) and math.isfinite(after) and 0 < before <= after):
            raise ValueError('READINESS_CLOCK_INVALID_OR_BACKWARDS:'+key)
        if after == before:
            raise ValueError('WAIT_EXTERNAL_CLOCK_PROGRESS:'+key)


def pair_groundtruth(position_timestamp_us, buffered_poses, previous_pose=None,
                     tolerance_us=20_000):
    """Pair a PX4 position sample with the closest fresh-cycle Gazebo sample.

    DDS position and Gazebo pose callbacks are asynchronous. Comparing only
    each stream's newest sample can reject a valid nearby pair when their
    publication periods differ, so search the bounded pose buffer first.
    The existing 20 ms coherence threshold remains a hard gate.
    """
    candidates = list(buffered_poses)
    if previous_pose is not None:
        candidates.append(previous_pose)
    if not candidates:
        raise ValueError('DDS_GROUNDTRUTH_CLOCK_PAIR')
    pose = min(candidates, key=lambda item: abs(
        int(item['native_timestamp_us'])-int(position_timestamp_us)))
    if abs(int(position_timestamp_us)-int(pose['native_timestamp_us'])) > tolerance_us:
        raise ValueError('DDS_GROUNDTRUTH_CLOCK_PAIR')
    return pose


def paired_watch_observation(sample, ekf_ned, target_enu, dds_received_wall, setpoint_wall):
    """Build watchdog input from the exact GT pose paired in snapshot().

    The independently advancing latest GT sample can be outside the timestamp
    tolerance even when snapshot() selected a valid nearby sample.
    """
    gt_us=int(sample.groundtruth_native_us)
    ekf_us=int(sample.native_us)
    if gt_us <= 0 or ekf_us <= 0 or abs(gt_us-ekf_us) > 20_000:
        raise ValueError('NATIVE_TIME_PAIR_INVALID')
    return WatchSample(tuple(sample.actual),tuple(ekf_ned),tuple(target_enu),gt_us,ekf_us,
        float(sample.groundtruth_received_wall),float(dds_received_wall),float(setpoint_wall),
        tuple(sample.resets),estimator_valid=bool(sample.valid),dds_connected=True,
        armed=bool(sample.armed),landed=bool(sample.landed),gcs_lost=bool(sample.gcs_lost))


def parameter(evidence, name):
    import re
    text = '\n'.join(v['stdout'] for k,v in evidence.items() if k.startswith('params_'))
    values = re.findall(r'\b'+re.escape(name)+r'\s+\[[^]]+\]\s*:\s*([-+\d.eE]+)',text)
    # The same parameter may occur in two reviewed wildcard probes.
    if not values or len(set(values)) != 1 or not math.isfinite(float(values[0])):
        raise ValueError('MISSING_OR_CONFLICTING_PARAMETER:'+name)
    return float(values[0])


class GraphCheckMixin:
    """Graph inspection shared by the live executor and ROS-free regression fakes."""
    def graph_check(self):
        graph = {}
        for key, cls in self.types.items():
            name = topic_name('/fmu/out/'+key, cls)
            endpoints = self.get_publishers_info_by_topic(name)
            graph[name] = dict(publishers=self.count_publishers(name),gids=[list(p.endpoint_gid) for p in endpoints],
                types=[p.topic_type for p in endpoints],expected_type='px4_msgs/msg/'+cls.__name__)
        for key, cls in self.inputs.items():
            name = topic_name('/fmu/in/'+key, cls)
            readers = self.get_subscriptions_info_by_topic(name) if self.demo_only else []
            graph[name] = dict(publishers=self.count_publishers(name),readers=self.count_subscribers(name),
                reader_gids=[list(r.endpoint_gid) for r in readers],reader_types=[r.topic_type for r in readers],
                expected_type='px4_msgs/msg/'+cls.__name__)
        participant = validate_graph(graph,bool(self.publishers_control),require_sitl_readers=self.demo_only,
                                     required_outputs=len(self.types))
        if self.expected_participant is not None and participant != self.expected_participant:
            raise ValueError('DDS_PARTICIPANT_CHANGED')
        self.expected_participant = participant
        self.log('GRAPH',graph=graph)


def run(args, plan, out):
    with localhost_ros(bool(getattr(args,'demo_only',False))):
        return _run(args,plan,out)


def _run(args,plan,out):
    demo_only=bool(getattr(args,'demo_only',False))
    approval=DEMO_CONFIRMATION if demo_only else CONFIRMATION
    if (not args.execute_flight or args.confirm_flight != approval or plan['static_blockers'] or
            bool(plan.get('demo_only',False))!=demo_only):
        raise ValueError('SEPARATE_APPROVAL_OR_STATIC_GATES_MISSING')
    from .environments import load_profile, load_environment
    from .groundtruth import GazeboPoseObserver
    from .runtime_preparation import host_snapshot, anchor_packet
    from .runtime_safety import GroundTruthWatchdog, WatchSample
    from .session_launcher import read_session, identity, same_process
    from .software_inventory import inventory
    from .visualization.raw_pso_gazebo import Gazebo, load_routes, OWN_NAMES

    profile = load_profile()
    if (profile.instance != 0 or profile.namespace or profile.dds_domain != 0 or
            int(os.environ.get('ROS_DOMAIN_ID','0')) != 0 or os.environ.get('ROS_DISTRO') != 'jazzy'):
        raise ValueError('Reviewed warehouse runtime requires instance/domain 0 and empty namespace')
    native, initial_anchor, env = host_snapshot(args.session,args.expect_iterations,gate_phase='flight_prestart')
    if native['problems'] or not initial_anchor:
        raise ValueError('OWNED_NATIVE_READINESS_OR_MEASURED_ANCHOR_MISSING')
    packet = anchor_packet(native,initial_anchor)
    # Final in-flight mag alignment is NOT PX4's initial arm/offboard condition.
    # Keep all other existing anchor/frame gates and verify initial yaw explicitly.
    if any(not value for key,value in packet['frame_gates'].items() if key != 'heading_good_for_control'):
        raise ValueError('NATIVE_FRAME_PROOF_MISSING')
    from .runtime_readiness import topic_fields
    est = topic_fields(native['evidence']['estimator_status']['stdout'])
    bits = int(est['control_mode_flags'].split()[0])
    if not (bits & 1 and bits & 2) or est.get('pre_flt_fail_innov_heading') != 'False':
        raise ValueError('INITIAL_YAW_OR_TILT_NOT_ALIGNED')
    if (parameter(native['evidence'],'COM_ARM_ON_BOOT') != 0 or
            parameter(native['evidence'],'COM_DISARM_LAND') <= 0 or
            parameter(native['evidence'],'COM_OF_LOSS_T') <= 0):
        raise ValueError('AUTO_ARM_DISARM_OR_OFFBOARD_LOSS_CONFIGURATION')
    for key, minimum in [('MPC_XY_VEL_MAX',CONFIG.speed),('MPC_Z_VEL_MAX_UP',CONFIG.speed),
                         ('MPC_Z_VEL_MAX_DN',CONFIG.speed),('MPC_ACC_HOR',CONFIG.acceleration),
                         ('MPC_ACC_UP_MAX',CONFIG.acceleration),('MPC_ACC_DOWN_MAX',CONFIG.acceleration)]:
        if parameter(native['evidence'],key) < minimum:
            raise ValueError('PX4_CONTROLLER_LIMIT:'+key)
    software = inventory(profile)
    if software['errors'] or not software['schemas'] or not all(s['serialized_fields_match'] and
            s['built_version_matches'] for s in software['schemas'].values()):
        raise ValueError('ROS_PX4_BUILT_SCHEMA_MISMATCH')
    world,_ = load_environment(profile)
    route = load_routes(validate_geometry=not demo_only)[args.algorithm]
    points = np.asarray(route.points,float)
    columns = contact_environment(world,env)
    transitions = transition_checks(points,initial_anchor,world,env,demo_only=demo_only)
    if not transitions['go']:
        raise ValueError('MEASURED_TAKEOFF_LAND_OR_RELATIVE_HEIGHT_NO_GO')
    gz = Gazebo(read_only=True)
    models = gz.scene()
    if any(name in models for name in OWN_NAMES):
        raise ValueError('VISUALIZATION_MODELS_PRESENT_REVIEW_CLEANUP_SEPARATELY')
    entity_id = models['x500'].id
    gz.guard()
    def sitl_proof():
        _,state=read_session(args.session)
        pid=native['runtime_identity']['px4_pid']
        entries=Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        process_env={k.decode():v.decode() for k,v in (x.split(b'=',1) for x in entries if b'=' in x)}
        return verify_sitl(profile,state,native,process_env)
    if demo_only:
        proof=sitl_proof()
        with (out/'sitl_identity.json').open('x') as f:
            json.dump(dict(proof=proof,collision_risk_policy=RISK_EVENT),f,indent=2)
        env,columns=position_environment(env),position_environment(columns)
    with (out/'native_preflight.json').open('x') as f:
        json.dump(dict(native=native,anchor_candidate=packet,software=software,
                       transitions=transitions,entity_id=entity_id),f,indent=2)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import (VehicleLocalPosition, VehicleStatus, VehicleAttitude,
        VehicleLandDetected, EstimatorStatusFlags, TimesyncStatus, VehicleCommandAck,
        VehicleCommand, TrajectorySetpoint, OffboardControlMode, FailsafeFlags)

    class Executor(GraphCheckMixin, Node):
        def __init__(self):
            super().__init__('pso_integrated_flight', enable_rosout=False, start_parameter_services=False)
            self.demo_only = demo_only
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Use host UTC with verified XRCE timesync, not ROS /clock timestamps')
            self.clock_map = LiveEpochClock()
            self.samples, self.acks, self.fresh = {}, [], Freshness()
            self.mission = self.anchor = self.watch = None
            self.publishers_control = {}
            self.started = time.monotonic()
            self.next_graph = self.next_identity = 0.
            self.last_stream = None
            self.last_gt = None
            self.last_watch_stamp = None
            self.input_error = None
            self.readiness_first_stamps = None
            self.last_wait_report = 0.
            self.collision_monitor=ObservedCollision(world) if demo_only else None
            self.expected_participant = None
            self.log_file = (out/'telemetry.jsonl').open('x',buffering=1)
            if demo_only:
                self.log(RISK_EVENT,sitl_verified=True,physics_collisions='UNCHANGED',
                         collision_detection='CONSERVATIVE_OBSERVED_BODY_PROXY_NOT_CONTACT_SENSOR')
                self.log(CEILING_EVENT,route_project_cap_enforced=False,
                         takeoff_relative_height_cap_m=CONFIG.relative_height,
                         takeoff_cap_enforced=True,px4_limits_and_failsafes='UNCHANGED')
            self.types = dict(vehicle_local_position=VehicleLocalPosition,vehicle_status=VehicleStatus,
                vehicle_attitude=VehicleAttitude,vehicle_land_detected=VehicleLandDetected,
                estimator_status_flags=EstimatorStatusFlags,timesync_status=TimesyncStatus,
                vehicle_command_ack=VehicleCommandAck,failsafe_flags=FailsafeFlags)
            self.inputs = dict(offboard_control_mode=OffboardControlMode,
                               trajectory_setpoint=TrajectorySetpoint,vehicle_command=VehicleCommand)
            qos = QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.subs = [self.create_subscription(cls,topic_name('/fmu/out/'+key,cls),
                        lambda msg,k=key:self.receive(k,msg),qos) for key,cls in self.types.items()]
            self.gt = GazeboPoseObserver(world.name,'x500',entity_id)
            self.gt.start()
            self.native_monitor=NativeStatusMonitor(profile)
            self.native_monitor.start()
            self.log('SUBSCRIPTIONS_READY',physics_policy='EXTERNAL_GUI_PLAY',
                     requires_operator_physics_approval=True)
            remaining = max(0., 30-(time.time()-native['captured_wall_time']))
            print('SUBSCRIPTIONS_READY — nhấn Play trong Gazebo GUI NGAY nếu đã phê duyệt bay. '
                  'Giữ Play suốt chuyến; executor tự Arm sau khi live gates PASS. '
                  f'Native proof còn {remaining:.1f}s. Không có lệnh unpause từ executor.',flush=True)

        def log(self,event,**fields):
            self.log_file.write(json.dumps(
                _json_safe(dict(event=event,wall_monotonic=time.monotonic(),**fields)),
                allow_nan=False)+'\n')

        def receive(self,key,msg):
            now = time.monotonic()
            try:
                if key == 'timesync_status':
                    previous_epoch = self.clock_map.epoch
                    self.clock_map.update(msg.timestamp,msg.estimated_offset,msg.observed_offset,
                                          msg.round_trip_time,msg.source_protocol,now)
                    if self.clock_map.epoch != previous_epoch:
                        if self.mission is not None:
                            raise ValueError('TIMESYNC_RESET_AFTER_PRESTREAM')
                        self.samples.clear()
                        self.fresh = Freshness()
                        self.readiness_first_stamps = None
                self.samples[key] = (now,msg)
                fields = {k:getattr(msg,k) for k in msg.get_fields_and_field_types()
                    if k in ('timestamp','timestamp_sample','heading','heading_good_for_control','x','y','z',
                        'vx','vy','vz','xy_valid','z_valid','v_xy_valid','v_z_valid','arming_state','nav_state',
                        'pre_flight_checks_pass','latest_arming_reason','latest_disarming_reason',
                        'failsafe','gcs_connection_lost','landed','system_id','component_id',
                        'estimated_offset','observed_offset','remote_timestamp','round_trip_time','source_protocol',
                        'command','result','result_param1','result_param2','target_system','target_component','ref_timestamp',
                        'ready_to_arm','offboard_control_signal_lost','is_logging','health_error_flags',
                        'arming_check_error_flags','arming_check_warning_flags') or
                    k in RESET_FIELDS or k.startswith(('cs_','fs_'))}
                if key == 'vehicle_attitude': fields['q'] = list(msg.q)
                self.log('DDS',topic=key,epoch=self.clock_map.epoch,fields=fields)
                if key == 'vehicle_command_ack' and self.mission is not None:
                    if msg.target_system == 245 and msg.target_component == 191:
                        stamp = self.clock_map.normalize(key,msg.timestamp,now)
                        self.mission.ack(msg.command,msg.result,now,stamp,
                            dict(result_param1=msg.result_param1,result_param2=msg.result_param2))
            except (ValueError,TypeError) as exc:
                if self.mission is not None: self.input_error = str(exc)
                self.log('INPUT_REJECTED',reason=str(exc))

        def identity_check(self):
            _,state = read_session(args.session)
            from .session_launcher import digest
            if (state['session_id'] != native['session_id'] or state['sdf_sha256'] != world.sdf_sha256 or
                    state['profile_sha256'] != native['profile_sha256'] or
                    digest(load_profile().snapshot()) != native['profile_sha256'] or
                    hashlib.sha256(Path(world.sdf_path).read_bytes()).hexdigest() != world.sdf_sha256):
                raise ValueError('SESSION_OR_SDF_CHANGED')
            for child in state['children']:
                if not same_process(child['owner'],identity(child['owner']['pid'])):
                    raise ValueError('OWNED_PROCESS_CHANGED')
            expected = native['runtime_identity']
            current = identity(expected['gazebo_server_pid'])
            if (current is None or current['start_ticks'] != expected['gazebo_start_ticks'] or
                    current['boot_id'] != expected['boot_id']):
                raise ValueError('GAZEBO_SERVER_RESTARTED')

        def snapshot(self,now):
            if self.input_error:
                raise ValueError(self.input_error)
            normalized = {}
            live_required=(*REQUIRED,'failsafe_flags')
            for key in live_required:
                received,msg = self.samples[key]
                if not 0 <= now-received <= 1.5:
                    raise ValueError('STALE_DDS:'+key)
                stamp = self.clock_map.normalize(key,msg.timestamp,now)
                self.fresh.record(key,stamp,received)
                normalized[key] = stamp
            self.fresh.check(live_required,now)
            native_us = normalized['vehicle_local_position']
            native_status=self.native_monitor.snapshot(native_us,now)
            failsafe_msg=self.samples['failsafe_flags'][1]
            p,s,a,l,e = [self.samples[k][1] for k in ('vehicle_local_position','vehicle_status',
                'vehicle_attitude','vehicle_land_detected','estimator_status_flags')]
            if s.system_id != 1 or s.component_id != 1:
                raise ValueError('PX4_SYSID_COMPONENT')
            valid = (all((p.xy_valid,p.z_valid,p.v_xy_valid,p.v_z_valid,e.cs_tilt_align,e.cs_yaw_align))
                and not (e.cs_mag_fault or e.cs_gnss_fault) and
                not any(getattr(e,k) for k in e.get_fields_and_field_types() if k.startswith('fs_')))
            roll,pitch = quaternion_tilt(a.q)
            valid = valid and max(abs(roll),abs(pitch)) <= math.radians(15)
            with self.gt.lock:
                if self.gt.errors: raise ValueError('GAZEBO_GROUNDTRUTH:'+str(self.gt.errors))
                if self.gt.samples: self.last_gt = self.gt.samples[-1]
                poses=list(self.gt.samples) if demo_only else []
                # The flight journal samples latest GT at 20 Hz. Keep the existing
                # observer's bounded buffer drained; do not accumulate an entire flight in RAM.
                self.gt.samples.clear()
            collision_suspected=bool(self.collision_monitor and self.collision_monitor.update(poses))
            pose = (pair_groundtruth(native_us,poses,self.last_gt) if demo_only else self.last_gt)
            if not pose or not 0 <= now-pose['received_monotonic'] <= .25:
                raise ValueError('GROUNDTRUTH_STALE')
            if now-self.samples['vehicle_local_position'][0] > .5:
                raise ValueError('POSITION_STALE')
            if abs(native_us-pose['native_timestamp_us']) > 20_000:
                raise ValueError('DDS_GROUNDTRUTH_CLOCK_PAIR')
            if self.anchor is None:
                if time.time()-native['captured_wall_time'] > 30:
                    raise ValueError('NATIVE_ANCHOR_PROOF_EXPIRED_RECAPTURE_READONLY')
                candidate = measured_anchor(initial_anchor,pose,p,native_us)
                if not transition_checks(points,candidate,world,env,demo_only=demo_only)['go']:
                    raise ValueError('LIVE_TRANSITION_OR_HEIGHT_NO_GO')
            else:
                candidate = self.anchor
                candidate.check_resets(tuple(int(getattr(p,k)) for k in RESET_FIELDS))
            delta = np.linalg.norm(candidate.to_enu([p.x,p.y,p.z])-pose['position_enu'])
            w,x,y,z = pose['quaternion_enu']
            gt_yaw = convert_yaw(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
            heading_error = abs((p.heading-gt_yaw+math.pi)%(2*math.pi)-math.pi)
            frame_good = math.isfinite(heading_error) and heading_error <= .15 and delta <= .15
            health_reason = ('pre_flight_checks_pass='+str(bool(s.pre_flight_checks_pass))+
                ',ready_to_arm='+str(native_status['actuator_armed'].get('ready_to_arm'))+
                ',offboard_control_signal_lost='+str(bool(failsafe_msg.offboard_control_signal_lost))+
                ',latest_arming_reason='+str(getattr(s,'latest_arming_reason','unknown'))+
                ','+native_status['health_reason'])
            sample = Sample(now,native_us,tuple(pose['position_enu']),tuple([p.vx,p.vy,p.vz]),
                tuple(int(getattr(p,k)) for k in RESET_FIELDS),s.arming_state == s.ARMING_STATE_ARMED,
                bool(l.landed),int(s.nav_state),bool(s.pre_flight_checks_pass),bool(valid),
                bool(e.cs_yaw_align),bool(p.heading_good_for_control),bool(frame_good),True,
                bool(s.failsafe),bool(s.gcs_connection_lost),collision_suspected,
                ready_to_arm=native_status['actuator_armed'].get('ready_to_arm') == 'True',
                offboard_signal_lost=bool(failsafe_msg.offboard_control_signal_lost),
                logger_active=native_status['logger_active'],
                arming_status_fresh=True,arming_reason=health_reason,
                groundtruth_native_us=int(pose['native_timestamp_us']),
                groundtruth_received_wall=float(pose['received_monotonic']))
            if self.anchor is None:
                if not (sample.valid and sample.initial_yaw and sample.frame_good
                        and not sample.armed and sample.landed and not sample.failsafe and not sample.gcs_lost
                        and not sample.collision_suspected):
                    raise ValueError('LIVE_INITIAL_READINESS')
                stamps = dict(normalized, gazebo_groundtruth=pose['native_timestamp_us'])
                if self.readiness_first_stamps is None:
                    self.readiness_first_stamps = stamps.copy()
                require_readiness_progress(self.readiness_first_stamps, stamps)
                self.anchor = candidate
                self.log('ANCHOR_CONFIRMED',anchor=self.anchor.as_dict(),runtime_identity=native['runtime_identity'])
                with (out/'confirmed_anchor.json').open('x') as f:
                    json.dump(dict(anchor=self.anchor.as_dict(),runtime_identity=native['runtime_identity'],
                        session_id=native['session_id'],profile_sha256=native['profile_sha256'],
                        sdf_sha256=world.sdf_sha256,confirmed_wall=time.time(),epoch=self.clock_map.epoch,
                        scope='INITIAL_YAW_FOR_OFFBOARD_VERTICAL_TAKEOFF_FINAL_HEADING_REQUIRED_BEFORE_PSO'),f,indent=2)
                print('LIVE_READINESS_PASS — DDS/clock/frame/anchor đã xác minh; '
                      'bắt đầu prestream và chuỗi bay được duyệt. Giữ Gazebo Play.',flush=True)
            self.log('PAIRED_STATE',sample=sample.__dict__,groundtruth=pose,
                     ekf_position_ned=[p.x,p.y,p.z],epoch=self.clock_map.epoch)
            return sample

        def activate(self,s):
            self.graph_check()  # zero control publishers BEFORE creating ours
            self.identity_check()
            if demo_only: sitl_proof()
            qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
            self.publishers_control = {k:self.create_publisher(cls,topic_name('/fmu/in/'+k,cls),qos)
                                       for k,cls in self.inputs.items()}
            logger_required = parameter(native['evidence'],'SDLOG_MODE') == 1
            self.mission = Mission(points,self.anchor,env,columns,demo_only=demo_only,
                                   logger_required=logger_required)
            self.watch = GroundTruthWatchdog(self.anchor,columns,project_ceiling_required=not demo_only)
            self.yaw = float(self.samples['vehicle_local_position'][1].heading)
            if not math.isfinite(self.yaw): raise ValueError('INVALID_YAW_HOLD')
            self.log('CONTROL_PUBLISHERS_CREATED',approval=approval,demo_only=demo_only)

        def publish(self,decision):
            if demo_only and (decision['target'] is not None or decision.get('command')):
                self.graph_check()  # verify local SITL readers before every publication batch
            if decision['target'] is not None:
                if self.anchor is None: raise ValueError('NO_CONFIRMED_ANCHOR')
                stamp = time.time_ns()//1000
                mode = OffboardControlMode()
                mode.timestamp,mode.position = stamp,True
                sp = TrajectorySetpoint()
                sp.timestamp = stamp
                sp.position = self.anchor.to_ned(decision['target']).tolist()
                sp.velocity = sp.acceleration = sp.jerk = [float('nan')]*3
                sp.yaw,sp.yawspeed = self.yaw,float('nan')
                self.publishers_control['offboard_control_mode'].publish(mode)
                self.publishers_control['trajectory_setpoint'].publish(sp)
                self.last_stream = time.monotonic()
                details=dict(position_ned=list(sp.position),target_enu=decision['target'],
                    timestamp=stamp,velocity_acceleration='NAN_POSITION_ONLY')
                self.log('SETPOINT',**details)
            command = decision.get('command')
            if command:
                msg = VehicleCommand()
                msg.timestamp = time.time_ns()//1000
                msg.command,msg.param1,msg.param2 = command['command'],command['param1'],command['param2']
                msg.target_system,msg.target_component = 1,1
                msg.source_system,msg.source_component = 245,191
                msg.from_external = True
                self.publishers_control['vehicle_command'].publish(msg)
                self.log('COMMAND_SENT_ONCE',**command)
            self.log('MISSION',stage=decision['stage'],index=self.mission.index,
                     goal_confirmed=self.mission.goal_confirmed,reason=decision.get('reason'))
            if decision.get('event'):
                event = decision['event']
                self.log(event['name'], **{k:v for k,v in event.items() if k != 'name'})

        def tick(self):
            now = time.monotonic()
            s = None
            try:
                if now >= self.next_identity:
                    self.identity_check()
                    self.next_identity = now+1
                if now >= self.next_graph:
                    self.graph_check()
                    self.next_graph = now+1
                s = self.snapshot(now)
                if self.mission is None: self.activate(s)
                self.watch.environment = env if self.mission.stage in ('HOLD_A','FOLLOW') else columns
                if self.last_watch_stamp != s.groundtruth_native_us:
                    p = self.samples['vehicle_local_position'][1]
                    # After Land ACK+mode the native controller owns descent;
                    # the final cruise target no longer defines tracking error.
                    watch_target = s.actual if self.mission.land_mode_accepted else self.mission.target
                    observation = paired_watch_observation(s,(p.x,p.y,p.z),watch_target,
                        self.samples['vehicle_local_position'][0],self.last_stream or now)
                    decision = self.watch.evaluate(observation,now,landing_column_clear=False,
                        offboard_stream_required=not self.mission.land_mode_accepted,
                        tracking_required=self.mission.stage != 'TAKEOFF',
                        landing_mode=(self.mission.stage == 'LANDING' and
                                      self.mission.land_mode_accepted),
                        landing_vertical_speed_limit=self.mission.cfg.landing_vertical_speed)
                    self.last_watch_stamp = s.groundtruth_native_us
                    if decision['reasons']:
                        raise ValueError('GROUNDTRUTH_WATCHDOG:'+','.join(decision['reasons']))
                self.publish(self.mission.step(s))
            except (ValueError,KeyError,TypeError) as exc:
                if self.mission is None:
                    self.log('WAIT_READINESS',reason=str(exc))
                    if now-self.last_wait_report >= 2.:
                        print('WAIT_READINESS — '+str(exc)+'; chưa tạo control publisher.',flush=True)
                        self.last_wait_report = now
                    if now-self.started > 60:
                        raise RuntimeError('READINESS_TIMEOUT_NO_CONTROL_PUBLISHED: '+str(exc))
                else:
                    self.publish(self.mission.fail(str(exc),s))

        def interrupt(self):
            if self.mission:
                try: s = self.snapshot(time.monotonic())
                except (ValueError,KeyError,TypeError): s = None
                self.publish(self.mission.fail('OPERATOR_INTERRUPT',s))

    rclpy.init(args=[])
    node = None
    outcome = dict(flight_success=False,goal_confirmed=False,stage='INITIALIZING',failure=None,
                   demo_only=demo_only,collision_risk_policy=RISK_EVENT if demo_only else None,
                   project_ceiling_policy=CEILING_EVENT if demo_only else 'ENFORCED',
                   controls_physics=False,operator_pause_required=True,
                   ulog_path=native['evidence']['logger']['stdout'])
    try:
        node = Executor()
        next_tick = time.monotonic()
        interrupted = False
        while rclpy.ok():
            try:
                now = time.monotonic()
                rclpy.spin_once(node,timeout_sec=max(0.,min(.01,next_tick-now)))
                if time.monotonic() >= next_tick:
                    node.tick()
                    next_tick = time.monotonic()+1/CONFIG.publish_hz
                if node.mission and node.mission.stage in ('DONE','ABORT_OPERATOR_REQUIRED'):
                    break
            except KeyboardInterrupt:
                if interrupted: break
                interrupted = True
                node.interrupt()
        success = bool(node.mission and node.mission.success)
        outcome.update(dict(flight_success=success,goal_confirmed=bool(node.mission and node.mission.goal_confirmed),
                collision_suspected=bool(node.collision_monitor and node.collision_monitor.suspected),
                collision_free_claimed=False,
                stage=node.mission.stage if node.mission else 'READINESS',
                failure=node.mission.failure if node.mission else None,controls_physics=False,
                operator_pause_required=not success))
        if not success:
            raise RuntimeError('MISSION_NOT_CONFIRMED: operator must inspect/Pause; no automatic retry')
    except BaseException as exc:
        outcome['failure'] = str(exc)
        outcome['flight_success'] = False
        outcome['operator_pause_required'] = True
        if node and node.mission:
            outcome['stage'] = node.mission.stage
            outcome['goal_confirmed'] = node.mission.goal_confirmed
            outcome['collision_suspected']=bool(node.collision_monitor and node.collision_monitor.suspected)
            outcome['collision_free_claimed']=False
        print('EXECUTOR STOPPED: operator must inspect/Pause; landing is not guaranteed after link/API failure.',flush=True)
        raise
    finally:
        with (out/'result.json').open('x') as f:
            json.dump(outcome,f,indent=2)
        if node:
            node.gt.finish()
            node.native_monitor.finish()
            node.log_file.close()
            node.destroy_node()
        rclpy.shutdown()
