"""Integrated PX4 mission preparation. Default is offline; no ROS imports.

Live use requires separate operator flight/physics approval. This program never
changes Gazebo physics, models, parameters or the frozen PSO routes.
"""
import argparse
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

import numpy as np

from .demo_first import new_output, route_audit
from .environments import load_environment, load_profile
from .geometry import FlightEnvironment, vector
from .visualization.raw_pso_gazebo import load_routes
from .demo_policy import RISK_EVENT, CEILING_EVENT, DEMO_CONFIRMATION, position_environment


@dataclass(frozen=True)
class Config:
    speed: float = .2
    follow_speed: float = .15
    acceleration: float = .1
    actual_speed: float = .7
    tracking: float = .25
    arrival: float = .10
    hold: float = 5.
    publish_hz: float = 20.
    prestream: float = 1.2
    ack_wall: float = 3.
    arming_ready_wall: float = 12.
    post_arm_logger_wall: float = 5.
    takeoff_sim: float = 45.
    takeoff_liftoff_response_sim: float = 8.
    takeoff_liftoff_min_m: float = .15
    route_sim: float = 300.
    mission_sim: float = 400.
    mission_wall: float = 800.
    land_wall: float = 45.
    relative_height: float = 3.2
    landing_vertical_speed: float = .9


CONFIG = Config()  # Same immutable controller for both algorithms.
CONFIRMATION = 'PX4_PSO_FLIGHT_APPROVED'


class LinearLeg:
    """Analytic trapezoid on ONE raw segment; zero speed at both ends.

    Changes timing only. No spatial smoothing, shortcut or waypoint replacement.
    """
    def __init__(self, start, end, cfg=CONFIG):
        self.start, self.end = vector(start), vector(end)
        self.length = float(np.linalg.norm(self.end-self.start))
        self.a = cfg.acceleration
        self.ta = min(cfg.speed/self.a, math.sqrt(self.length/self.a))
        self.peak = self.a*self.ta
        self.cruise = max(0., self.length/self.peak-self.ta) if self.peak else 0.
        self.duration = 2*self.ta+self.cruise

    def position(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Invalid leg time')
        if not self.length or elapsed >= self.duration:
            return self.end.copy()
        if elapsed < self.ta:
            distance = .5*self.a*elapsed**2
        elif elapsed < self.ta+self.cruise:
            distance = .5*self.a*self.ta**2+self.peak*(elapsed-self.ta)
        else:
            distance = self.length-.5*self.a*(self.duration-elapsed)**2
        return self.start+(self.end-self.start)*(distance/self.length)


def contact_environment(world, env, tracking=0.):
    """Keep obstacle envelope and XY bounds during ground-contact transitions.

    Cruise floor clearance cannot apply at touchdown. Only the floor contact is
    phase-specific; other obstacles, XY bounds and the height cap stay enforced.
    """
    return FlightEnvironment(world.boxes, [*(env.lower[:2]+tracking), world.floor_z-1],
                             [*(env.upper[:2]-tracking), env.upper[2]],
                             env.radius, env.margin+tracking)


def transition_checks(points, anchor, world, env, *, demo_only=False):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Expected finite ENU route')
    ground = vector(anchor.gazebo_enu)
    end_ground = np.array([*points[-1][:2], ground[2]])
    columns = contact_environment(world, env, CONFIG.tracking)
    result = dict(start_xy_error_m=float(np.linalg.norm(ground[:2]-points[0][:2])),
        relative_height_max_m=float(np.max(points[:,2])-ground[2]),
        takeoff_clear=not bool(columns.collision_mask([ground, points[0]])[0]),
        landing_clear=not bool(columns.collision_mask([points[-1],end_ground])[0]),
        floor_contact='ONLY_TAKEOFF_LAND_PHASE_NOT_A_CRUISE_CLEARANCE_CERTIFICATE')
    result['go'] = (result['start_xy_error_m'] <= CONFIG.arrival and result['takeoff_clear']
        and result['landing_clear'] and 0 < points[0,2]-ground[2]
        and result['relative_height_max_m']+CONFIG.tracking <= CONFIG.relative_height)
    if demo_only:
        # The operator waived project ceiling, collision and EXTRA tracking
        # reserve only. Measured start/frame and positive takeoff remain required.
        result['collision_risk_policy']=RISK_EVENT
        result['project_ceiling_policy']=CEILING_EVENT
        result['go']=bool(result['start_xy_error_m'] <= CONFIG.arrival and
            0 < points[0,2]-ground[2])
    return result


def prepare_plan(algorithm='standard', *, demo_only=False):
    routes = load_routes(validate_geometry=not demo_only)
    route = routes[algorithm]
    world, env = load_environment(load_profile())
    audit = route_audit(route, world, env,validate_geometry=not demo_only)
    follow_cfg = replace(CONFIG,speed=CONFIG.follow_speed)
    legs = [LinearLeg(a,b,follow_cfg) for a,b in zip(route.points,route.points[1:])]
    minimum = sum(l.duration+CONFIG.hold for l in legs)
    blockers = []
    if not audit['tracking_tube_go'] and not demo_only:
        blockers.append('FULL_RAW_ROUTE_TRACKING_TUBE_OUTSIDE_APPROVED_BOUNDS_OR_OBSTACLES')
    if minimum >= CONFIG.route_sim:
        blockers.append('ROUTE_TIME_BUDGET')
    return dict(result='OFFLINE_PREPARED', algorithm=algorithm,demo_only=demo_only,
        collision_risk_policy=RISK_EVENT if demo_only else 'DEFAULT_COLLISION_GATES_ENFORCED',
        bypassed_project_gates=['STATIC_COLLISION_FEASIBILITY','TRACKING_MARGIN_ENVELOPE',
            'PROJECT_CEILING_3_2M_ABSOLUTE_AND_RELATIVE'] if demo_only else [],
        project_ceiling_policy=CEILING_EVENT if demo_only else 'ENFORCED',
        gazebo_collision_physics='UNCHANGED',px4_limits_and_failsafes='UNCHANGED',
        runtime_position_limits=('FINITE_XY_LOWER_BOUND_TRACKING_RETAINED_TAKEOFF_HEIGHT_CAP_3_2M_ROUTE_CEILING_WAIVED'
            if demo_only else 'UNCHANGED'),
        required_sitl_proof=demo_only,
        source_csv=route.provenance['csv'], csv_sha256=route.provenance['csv_sha256'],
        waypoint_count=len(route.points), length_m=route.length,
        start_enu=route.points[0], goal_enu=route.points[-1], controller=asdict(CONFIG),
        minimum_route_sim_s=minimum, raw_geometry='OFFLINE_CERTIFICATE_RETAINED_NOT_LIVE_CERTIFICATION' if demo_only else 'PASS',
        static_blockers=blockers, tracking_tube_failures=audit['tracking_tube_failures'],
        required_live_evidence=['CURRENT_OWNED_SESSION_SCHEMA_AND_ZERO_OTHER_CONTROLLERS',
            'FRESH_STABLE_DDS_EPOCH_NATIVE_CLOCK_AND_GAZEBO_GT',
            'MEASURED_CONFIRMED_ANCHOR_AND_TRANSITION_COLUMNS',
            'INITIAL_YAW_FRAME_PREFLIGHT_AND_NATIVE_ARM_MODE_ACK',
            'FINAL_HEADING_ALIGNMENT_AT_A_BEFORE_PSO', 'SEPARATE_FLIGHT_AND_PHYSICS_APPROVAL'],
        phases=['READINESS','PRESTREAM','OFFBOARD_ACK','ARM_READINESS','ARM_ACK','POST_ARM_LOGGER','VERTICAL_TAKEOFF',
                'HOLD_A_FINAL_HEADING','ALL_17_RAW_WAYPOINTS','GOAL_CONFIRMED','LAND_ACK','LANDED_AND_DISARMED'],
        physics_policy='EXTERNAL_GUI_PLAY',startup_requires_paused=True,
        operator_play_after='SUBSCRIPTIONS_READY',paused_required_during_flight=False,
        live_control_go=False, flight_tested=False, controls_physics=False,
        model_mutations=False, fallback_selected=False)


@dataclass
class Sample:
    wall: float
    native_us: int
    actual: tuple
    velocity: tuple
    resets: tuple
    armed: bool = False
    landed: bool = True
    nav: int = 4
    preflight: bool = True
    valid: bool = True
    initial_yaw: bool = True
    final_heading: bool = False
    frame_good: bool = True
    coherent: bool = True
    failsafe: bool = False
    gcs_lost: bool = False
    collision_suspected: bool = False
    ready_to_arm: bool = True
    offboard_signal_lost: bool = False
    logger_active: bool = True
    arming_status_fresh: bool = True
    arming_reason: str = ''
    groundtruth_native_us: int = 0
    groundtruth_received_wall: float = 0.


class Mission:
    """Pure mission state machine. Commands are returned, never executed here."""
    def __init__(self, points, anchor, env, contact_env, cfg=CONFIG, *, demo_only=False,
                 logger_required=True):
        if demo_only:
            env,contact_env=position_environment(env),position_environment(contact_env)
        self.path = env.validate(points)
        self.demo_only=demo_only
        self.logger_required=bool(logger_required)
        self.collision_detected=False
        self.anchor, self.env, self.contact, self.cfg = anchor, env, contact_env, cfg
        if len(self.path) != 17:
            raise ValueError('Expected all 17 validated raw waypoints')
        self.stage = 'READINESS'
        self.target = vector(anchor.gazebo_enu)
        self.started_wall = self.started_native = self.phase_wall = self.phase_native = None
        self.sent = None
        self.acks = []
        self.index = 0
        self.arrived = None
        self.motion = None
        self.last_wall = self.last_native = None
        self.route_started = None
        self.goal_confirmed = self.success = self.aborted = False
        self.failure = None
        self.land_sent = False
        self.land_mode_accepted = False
        self.takeoff_groundtruth_start_z = None
        self.takeoff_started_native = None
        self.takeoff_liftoff_confirmed = False

    def ack(self, command, result, received, native_us, details=None):
        self.acks.append((command, result, received, native_us, details or {}))
        self.acks = self.acks[-100:]

    def enter(self, stage, s):
        self.stage, self.phase_wall, self.phase_native = stage, s.wall, s.native_us

    def command(self, number, p1, p2, s):
        self.sent = (number,s.wall,s.native_us)
        return dict(command=number, param1=p1, param2=p2)

    def accepted(self, s, state):
        command, wall, native = self.sent
        matches = [item for item in self.acks if item[0] == command and item[2] >= wall and item[3] >= native]
        if matches and matches[-1][1] != 0:
            raise ValueError('COMMAND_REJECTED:'+str(command)+':result='+str(matches[-1][1])+':'+str(matches[-1][4]))
        ok = bool(matches and matches[-1][1] == 0 and state)
        if not ok and s.wall-wall > self.cfg.ack_wall:
            raise ValueError('ACK_STATE_TIMEOUT:'+str(command))
        return ok

    def begin_leg(self, a, b, s, *, follow=False):
        leg_cfg = replace(self.cfg,speed=self.cfg.follow_speed) if follow else self.cfg
        self.motion = LinearLeg(a,b,leg_cfg)
        self.phase_native = s.native_us
        self.arrived = None

    def follow_leg(self, s):
        elapsed = (s.native_us-self.phase_native)/1e6
        self.target = self.motion.position(elapsed)
        reached = (elapsed >= self.motion.duration and
                   np.linalg.norm(vector(s.actual)-self.motion.end) <= self.cfg.arrival and
                   np.linalg.norm(vector(s.velocity)) <= .10)
        if reached:
            if self.arrived is None:
                self.arrived = s.native_us
            return (s.native_us-self.arrived)/1e6 >= self.cfg.hold
        self.arrived = None
        return False

    def fail(self, reason, s=None):
        # Preserve the initiating fault. Later faults during an already
        # requested Land may change the safe-stop state, but must not hide why
        # the mission was first aborted.
        if self.failure is None:
            self.failure = reason
        self.aborted = True
        # No blind Land on lost/ambiguous position, stale link or estimator.
        reliable = bool(s and s.coherent and s.valid and s.frame_good and s.initial_yaw
                        and not s.failsafe and not s.gcs_lost)
        if reliable:
            try:
                self.anchor.check_resets(s.resets)
                self.contact.validate([s.actual,[*s.actual[:2],self.anchor.gazebo_enu[2]]])
            except ValueError:
                reliable = False
        if reliable and s.armed and not self.land_sent:
            self.enter('LANDING',s)
            self.land_sent = True
            self.target = vector(s.actual)
            return dict(stage=self.stage, target=self.target.tolist(),
                        command=self.command(21,0.,0.,s), reason=reason)
        self.stage = 'ABORT_OPERATOR_REQUIRED'
        return dict(stage=self.stage,target=None,command=None,reason=reason)

    def step(self, s):
        command = None
        event = None
        try:
            if self.stage in ('DONE','ABORT_OPERATOR_REQUIRED'):
                return dict(stage=self.stage,target=None,command=None)
            if not all(math.isfinite(x) for x in (s.wall,s.native_us,*s.actual,*s.velocity)):
                raise ValueError('NONFINITE_TELEMETRY')
            if s.native_us < self.anchor.px4_timestamp_us:
                raise ValueError('OBSERVATION_PREDATES_ANCHOR')
            if self.started_wall is None:
                self.started_wall,self.started_native = s.wall,s.native_us
            if (s.wall-self.started_wall > self.cfg.mission_wall or
                    (s.native_us-self.started_native)/1e6 > self.cfg.mission_sim):
                raise ValueError('MISSION_TIMEOUT')
            if self.last_wall is not None and (s.wall < self.last_wall or s.native_us < self.last_native
                    or (self.stage != 'READINESS' and (s.wall-self.last_wall > .15 or
                        s.native_us-self.last_native > 150_000))):
                raise ValueError('CLOCK_OR_SETPOINT_SCHEDULING')
            self.last_wall,self.last_native = s.wall,s.native_us
            self.anchor.check_resets(s.resets)
            self.collision_detected |= s.collision_suspected
            if s.collision_suspected and not self.aborted:
                raise ValueError('OBSERVED_COLLISION_SUSPECTED')
            if not (s.valid and s.initial_yaw and s.frame_good and s.coherent) or s.failsafe or s.gcs_lost:
                raise ValueError('STATE_FRAME_TIME_OR_ESTIMATOR')
            actual = vector(s.actual)
            speed=vector(s.velocity)
            if self.stage == 'LANDING' and self.land_mode_accepted:
                speed_exceeded=(np.linalg.norm(speed[:2]) > self.cfg.actual_speed or
                                abs(float(speed[2])) > self.cfg.landing_vertical_speed)
            else:
                speed_exceeded=np.linalg.norm(speed) > self.cfg.actual_speed
            if speed_exceeded:
                raise ValueError('ACTUAL_SPEED')
            # DEMO_ONLY waives the route's project ceiling, but takeoff still
            # has a finite local height cap to prevent a climbing setpoint.
            if (not self.demo_only or self.stage == 'TAKEOFF') and actual[2]-self.anchor.gazebo_enu[2] > self.cfg.relative_height:
                raise ValueError('RELATIVE_HEIGHT')
            airborne_phase = self.stage in ('TAKEOFF','HOLD_A','FOLLOW','LANDING')
            # A PX4 Land transition may disarm before the landed detector's
            # next sample. During LANDING, wait for explicit landed evidence;
            # never turn the intermediate disarmed state into success or abort.
            if airborne_phase and not s.armed and self.stage != 'LANDING':
                raise ValueError('UNEXPECTED_DISARM')
            active_env = self.env if self.stage in ('HOLD_A','FOLLOW') else self.contact
            active_env.validate([actual,actual])
            if (self.stage not in ('LANDING','TAKEOFF') and
                    np.linalg.norm(actual-self.target) > self.cfg.tracking):
                raise ValueError('TRACKING_LIMIT')
            if self.stage in ('ARM_READINESS','POST_ARM_LOGGER','TAKEOFF','HOLD_A','FOLLOW') and s.nav != 14:
                raise ValueError('OFFBOARD_EXIT')
            if self.stage == 'READINESS':
                if s.armed or not s.landed:
                    raise ValueError('INITIAL_DISARM_LAND')
                self.enter('PRESTREAM',s)
            elif self.stage == 'PRESTREAM':
                if s.armed or not s.landed:
                    raise ValueError('PRESTREAM_STATE_CHANGED')
                if (s.native_us-self.phase_native)/1e6 >= self.cfg.prestream and s.wall-self.phase_wall >= self.cfg.prestream:
                    self.enter('MODE_PENDING',s)
                    command = self.command(176,1.,6.,s)
            elif self.stage == 'MODE_PENDING':
                if s.armed or not s.landed:
                    raise ValueError('MODE_PENDING_STATE_CHANGED')
                if self.accepted(s,s.nav == 14):
                    self.enter('ARM_READINESS',s)
            elif self.stage == 'ARM_READINESS':
                if s.armed or not s.landed or s.nav != 14:
                    raise ValueError('ARM_READINESS_STATE_CHANGED')
                ready = (s.preflight and s.ready_to_arm and not s.offboard_signal_lost
                         and s.arming_status_fresh and not s.failsafe and not s.gcs_lost)
                if ready:
                    self.enter('ARM_PENDING',s)
                    command = self.command(400,1.,0.,s)  # no force-arm magic
                elif s.wall-self.phase_wall > self.cfg.arming_ready_wall:
                    raise ValueError('PX4_ARMING_READINESS_TIMEOUT:'+(s.arming_reason or
                        'preflight='+str(s.preflight)+',ready_to_arm='+str(s.ready_to_arm)+
                        ',offboard_signal_lost='+str(s.offboard_signal_lost)+
                        ',arming_status_fresh='+str(s.arming_status_fresh)))
            elif self.stage == 'ARM_PENDING':
                if s.nav != 14:
                    raise ValueError('OFFBOARD_EXIT_BEFORE_ARM')
                if self.accepted(s,s.armed):
                    self.enter('POST_ARM_LOGGER',s)
            elif self.stage == 'POST_ARM_LOGGER':
                if not s.armed or not s.landed:
                    raise ValueError('POST_ARM_STATE_UNCERTAIN')
                if s.logger_active or not self.logger_required:
                    self.enter('TAKEOFF',s)
                    self.takeoff_started_native = s.native_us
                    self.takeoff_groundtruth_start_z = actual[2]
                    event = dict(name='TAKEOFF_GROUNDTRUTH_RESPONSE_WINDOW_STARTED',
                        baseline_z_m=self.takeoff_groundtruth_start_z,
                        start_native_us=self.takeoff_started_native,
                        minimum_climb_m=self.cfg.takeoff_liftoff_min_m,
                        response_window_s=self.cfg.takeoff_liftoff_response_sim,
                        total_takeoff_timeout_s=self.cfg.takeoff_sim)
                    self.begin_leg(self.anchor.gazebo_enu,self.path[0],s)
                elif s.wall-self.phase_wall > self.cfg.post_arm_logger_wall:
                    if s.coherent and s.valid and s.frame_good and s.landed and not s.failsafe:
                        self.enter('DISARM_PENDING',s)
                        command = self.command(400,0.,0.,s)
                    else:
                        raise ValueError('POST_ARM_LOGGER_TIMEOUT_STATE_UNCERTAIN')
            elif self.stage == 'DISARM_PENDING':
                if self.accepted(s,not s.armed) and not s.armed and s.landed:
                    self.stage = 'ABORT_OPERATOR_REQUIRED'
                    self.failure = 'POST_ARM_LOGGER_INACTIVE_DISARMED'
            elif self.stage == 'TAKEOFF':
                elapsed_takeoff = (s.native_us-self.takeoff_started_native)/1e6
                if elapsed_takeoff > self.cfg.takeoff_sim:
                    raise ValueError('TAKEOFF_TIMEOUT')
                climb = actual[2]-self.takeoff_groundtruth_start_z
                liftoff_event = None
                if climb >= self.cfg.takeoff_liftoff_min_m:
                    if not self.takeoff_liftoff_confirmed:
                        self.takeoff_liftoff_confirmed = True
                        liftoff_event = dict(name='TAKEOFF_GROUNDTRUTH_LIFTOFF_CONFIRMED',
                            baseline_z_m=self.takeoff_groundtruth_start_z,groundtruth_z_m=actual[2],
                            climb_m=climb,elapsed_sim_s=elapsed_takeoff,
                            threshold_m=self.cfg.takeoff_liftoff_min_m,
                            response_window_s=self.cfg.takeoff_liftoff_response_sim)
                elif (not self.takeoff_liftoff_confirmed and
                        elapsed_takeoff >= self.cfg.takeoff_liftoff_response_sim):
                    raise ValueError('TAKEOFF_NO_GROUNDTRUTH_LIFTOFF_WITHIN_8S')
                if self.follow_leg(s):
                    if not self.takeoff_liftoff_confirmed:
                        raise ValueError('TAKEOFF_POSITION_REACHED_WITHOUT_GROUNDTRUTH_LIFTOFF')
                    if np.linalg.norm(actual-self.target) > self.cfg.tracking:
                        raise ValueError('TAKEOFF_CRUISE_TRACKING_NOT_RECOVERED')
                    if s.landed:
                        raise ValueError('TAKEOFF_NOT_AIRBORNE')
                    self.enter('HOLD_A',s)
                if liftoff_event:
                    return dict(stage=self.stage,target=self.target.tolist(),command=None,
                                event=liftoff_event)
            elif self.stage == 'HOLD_A':
                if s.landed:
                    raise ValueError('UNEXPECTED_LANDED')
                if s.final_heading:
                    self.index = 1
                    self.route_started = s.native_us
                    self.enter('FOLLOW',s)
                    self.begin_leg(self.path[0],self.path[1],s,follow=True)
                elif (s.native_us-self.phase_native)/1e6 > 10.:
                    raise ValueError('FINAL_HEADING_ALIGNMENT_TIMEOUT')
            elif self.stage == 'FOLLOW':
                if s.landed or not s.final_heading:
                    raise ValueError('FINAL_HEADING_OR_AIRBORNE_LOST')
                if (s.native_us-self.route_started)/1e6 > self.cfg.route_sim:
                    raise ValueError('ROUTE_TIMEOUT')
                if self.follow_leg(s):
                    if self.index == len(self.path)-1:
                        self.goal_confirmed = True
                        self.enter('LANDING',s)
                        self.land_sent = True
                        command = self.command(21,0.,0.,s)
                    else:
                        self.index += 1
                        self.begin_leg(self.path[self.index-1],self.path[self.index],s,follow=True)
            elif self.stage == 'LANDING':
                # Latch ACK + observed Land mode. PX4 may leave nav_state=LAND
                # after it has landed and disarmed; that later state must not
                # retroactively invalidate an already accepted Land transition.
                self.land_mode_accepted = self.land_mode_accepted or self.accepted(s,s.nav == 18)
                if self.land_mode_accepted and s.landed and not s.armed:
                    if self.aborted:
                        self.stage = 'ABORT_OPERATOR_REQUIRED'
                    else:
                        self.stage = 'DONE'
                        self.success = self.goal_confirmed and not self.collision_detected
                elif s.wall-self.phase_wall > self.cfg.land_wall:
                    raise ValueError('LAND_OR_DISARM_TIMEOUT')
            stream = (self.stage not in ('DONE','ABORT_OPERATOR_REQUIRED') and
                      not (self.stage == 'LANDING' and self.land_mode_accepted))
            result=dict(stage=self.stage,target=self.target.tolist() if stream else None,command=command)
            if event:
                result['event']=event
            return result
        except (ValueError,TypeError,KeyError) as exc:
            return self.fail(str(exc),s)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,epilog=
        'GUI clock workflow: start this command while Paused; after SUBSCRIPTIONS_READY '
        'press Play immediately and keep physics running through Land/Disarmed. '
        'Already-running startup is rejected. No Clock Coordinator is required.')
    parser.add_argument('--algorithm',choices=('standard','enhanced'),default='standard')
    parser.add_argument('--session',type=Path)
    parser.add_argument('--expect-iterations',type=int,default=15000)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run',action='store_true')
    mode.add_argument('--execute-flight',action='store_true')
    parser.add_argument('--confirm-flight')
    parser.add_argument('--demo-only',action='store_true',help='SITL only: waive project ceiling/collision/margin gates; runtime control gates remain')
    args = parser.parse_args(argv)
    out = new_output('integrated_'+args.algorithm)
    plan = prepare_plan(args.algorithm,demo_only=args.demo_only)
    with (out/'plan.json').open('x') as f:
        json.dump(plan,f,indent=2)
    if args.demo_only:
        with (out/'risk_acceptance.json').open('x') as f:
            json.dump(dict(event=RISK_EVENT,project_ceiling_policy=CEILING_EVENT,
                bypassed_project_gates=plan['bypassed_project_gates'],scope='GAZEBO_PX4_SITL_ONLY',
                live_flight_authorized=False,physics_changes_authorized=False),f,indent=2)
    if not args.execute_flight:
        print(json.dumps(dict(plan,output=str(out)),indent=2))
        return 0
    expected=DEMO_CONFIRMATION if args.demo_only else CONFIRMATION
    if args.confirm_flight != expected or args.session is None:
        parser.error('Live flight needs --session and explicit --confirm-flight '+expected)
    print('FLIGHT_LOG_DIRECTORY: '+str(out),flush=True)
    try:
        if plan['static_blockers']:
            raise ValueError('STATIC_NO_GO: '+', '.join(plan['static_blockers']))
        from .integrated_ros import run
        run(args,plan,out)
    except (ValueError,RuntimeError,OSError,ImportError) as exc:
        with (out/'failure.json').open('x') as f:
            json.dump(dict(result='NO_GO',reason=str(exc),flight_success=False),f,indent=2)
        print('NO_GO: '+str(exc))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
