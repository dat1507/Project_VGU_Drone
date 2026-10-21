"""Pure/mocked tests: no ROS init, IPC, Gazebo physics, commands or processes."""
from dataclasses import replace
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from pso_flight_demo.dds_evidence import MessageEvidence, REQUIRED, normalize_epochs, LiveEpochClock
from pso_flight_demo.frames import FrameAnchor, RESET_FIELDS
from pso_flight_demo.geometry import FlightEnvironment
from pso_flight_demo.integrated_flight import (CONFIG, LinearLeg, Mission, Sample, prepare_plan)
from pso_flight_demo.integrated_ros import (measured_anchor, validate_graph, run,
    GraphCheckMixin, NativeStatusMonitor, paired_watch_observation)
from pso_flight_demo.safety import topic_name
from pso_flight_demo.runtime_safety import GroundTruthWatchdog, WatchSample


class EpochTests(unittest.TestCase):
    def evidence(self):
        c=MessageEvidence()
        for wall,native,offset in [(1.,1_000_000,-10_000_000),(2.,2_000_000,0)]:
            for key in REQUIRED:
                c.record(key,native-offset,wall)
                if key == 'timesync_status':
                    c.messages[key]['samples'][-1].update(estimated_offset_us=offset,source_protocol=2)
        return c

    def test_raw_reversal_is_not_native_reversal(self):
        c=self.evidence()
        self.assertTrue(c.messages['timesync_status']['backwards'])
        r=c.summarize({},[])
        self.assertEqual(r['dds_evidence_level'],'TIMESTAMP_PROGRESSING')
        self.assertFalse(r['session_data_bound'] or r['live_control_go'])
        self.assertFalse(r['epoch_assessment']['utc_convergence_proven'])

    def test_ambiguous_reset_boundary_stays_no_go(self):
        c=self.evidence()
        c.messages['vehicle_local_position']['samples'][0]['received_monotonic']=1.5
        self.assertFalse(normalize_epochs(c.messages)['coherent'])
        self.assertEqual(c.summarize({},[])['dds_evidence_level'],'MESSAGE_RECEIVED')

    def test_missing_topic_wrong_protocol_and_native_reverse(self):
        for kind in ['missing','protocol','reverse']:
            c=self.evidence()
            if kind=='missing': del c.messages['vehicle_attitude']
            if kind=='protocol':c.messages['timesync_status']['samples'][0]['source_protocol']=1
            if kind=='reverse':c.messages['timesync_status']['samples'][-1]['timestamp']=500_000
            self.assertFalse(normalize_epochs(c.messages)['coherent'])

    def test_health_errors_are_not_erased_by_epoch_correction(self):
        self.assertEqual(self.evidence().summarize({},['estimator fault'])['dds_evidence_level'],'MESSAGE_RECEIVED')

    def clock(self):
        c=LiveEpochClock()
        for i in range(1,4): c.update(i*1_000_000+10_000_000,-10_000_000,-10_000_000,1,2,float(i))
        return c

    def test_live_clock_stale_offset_and_rtt_rejected(self):
        for kind in ['stale','offset','rtt']:
            c=self.clock()
            if kind=='stale': now=5.
            else:
                now=4.
                c.update(14_000_000,-10_000_000,-10_200_000 if kind=='offset' else -10_000_000,
                         20_000 if kind=='rtt' else 1,2,now)
            with self.assertRaises(ValueError): c.normalize('p',13_000_000,now)

    def test_clock_requires_three_advanced_sync_samples(self):
        c=LiveEpochClock()
        c.update(11_000_000,-10_000_000,-10_000_000,1,2,1.)
        c.update(11_000_000,-10_000_000,-10_000_000,1,2,1.1)
        with self.assertRaises(ValueError):c.normalize('p',11_000_000,1.1)

    def test_live_epoch_change_invalidates_until_reacquired(self):
        c=self.clock(); self.assertEqual(c.normalize('p',13_100_000,3.1),3_100_000)
        c.update(4_000_000,0,0,1,2,4.)
        self.assertEqual(c.epoch,1)
        with self.assertRaises(ValueError):c.normalize('p',4_000_000,4.)

    def test_old_packet_not_reinterpreted_when_offset_adjusts(self):
        c=self.clock(); t=c.normalize('p',13_100_000,3.1)
        c.update(14_000_010,-10_000_010,-10_000_010,1,2,4.)
        self.assertEqual(c.normalize('p',13_100_000,4.),t)

    def test_reversed_native_packet_rejected(self):
        c=self.clock();c.normalize('p',13_100_000,3.1)
        with self.assertRaises(ValueError):c.normalize('p',13_000_000,3.2)

    def test_invalid_sync_invalidates_previously_usable_clock(self):
        c=self.clock()
        with self.assertRaises(ValueError):c.update(14_000_000,-10_000_000,-10_000_000,1,1,4.)
        with self.assertRaises(ValueError):c.normalize('p',13_000_000,4.)

    def test_saved_cold_warmup_remains_no_go_at_ambiguous_reset(self):
        import json
        root=Path(__file__).resolve().parents[1]
        p=root/'sessions/7ae2da20fe854e098d511d580d1ee335/finite_warmup_20261019T140814Z_415b8027/dds_raw_observation.json'
        saved=json.loads(p.read_text())
        r=normalize_epochs(saved['messages'])
        self.assertFalse(r['coherent'])
        self.assertTrue(any('AMBIGUOUS_RESET_BOUNDARY' in problem for problem in r['problems']))


class PlanningTests(unittest.TestCase):
    def test_both_frozen_routes_same_controller_and_goals(self):
        a,b=prepare_plan('standard'),prepare_plan('enhanced')
        self.assertEqual(a['controller'],b['controller'])
        self.assertEqual(a['start_enu'],b['start_enu']);self.assertEqual(a['goal_enu'],b['goal_enu'])
        self.assertEqual(CONFIG.speed,.2)
        self.assertEqual(CONFIG.follow_speed,.15)
        self.assertEqual(CONFIG.acceleration,.1)
        for p in [a,b]:
            self.assertEqual(p['waypoint_count'],17)
            self.assertEqual(p['raw_geometry'],'PASS')
            self.assertTrue(p['static_blockers'])
            self.assertLess(p['minimum_route_sim_s'],CONFIG.route_sim)
            self.assertFalse(p['live_control_go'])

    def test_trapezoid_respects_raw_segment_speed_and_acceleration(self):
        for distance in [.01,.1,1.,3.]:
            for cfg,max_speed in ((CONFIG,CONFIG.speed),
                                  (replace(CONFIG,speed=CONFIG.follow_speed),CONFIG.follow_speed)):
                leg=LinearLeg([0,0,0],[distance,0,0],cfg)
                t=np.linspace(0,leg.duration,10001)
                p=np.array([leg.position(x) for x in t]);dt=t[1]-t[0]
                v=np.diff(p,axis=0)/dt;acc=np.diff(v,axis=0)/dt
                self.assertLessEqual(float(np.linalg.norm(v,axis=1).max()),max_speed+1e-8)
                self.assertLessEqual(float(np.linalg.norm(acc,axis=1).max()),CONFIG.acceleration+1e-6)
                np.testing.assert_array_equal(p[-1],[distance,0,0])
                self.assertTrue(np.all(p[:,1:]==0))

    def test_waypoint_transition_waits_for_position_velocity_and_hold(self):
        fixture=MissionTests();fixture.setUp();s=fixture.active();m=fixture.m
        start=np.array([0.,0.,2.5]);end=np.array([.2,0.,2.5])
        m.begin_leg(start,end,s)
        end_time=m.motion.duration
        lagging=replace(s,native_us=s.native_us+int(end_time*1e6),actual=tuple(start))
        self.assertFalse(m.follow_leg(lagging))
        np.testing.assert_allclose(m.target,end,atol=1e-12)
        # Being at the waypoint is insufficient while velocity exceeds the
        # existing arrival threshold; then the configured hold is still required.
        fast=replace(lagging,native_us=lagging.native_us+50_000,actual=tuple(end),
                     velocity=(.11,0.,0.))
        self.assertFalse(m.follow_leg(fast))
        stopped=replace(fast,native_us=fast.native_us+50_000,velocity=(0.,0.,0.))
        self.assertFalse(m.follow_leg(stopped))
        held=replace(stopped,native_us=stopped.native_us+int(CONFIG.hold*1e6))
        self.assertTrue(m.follow_leg(held))

    def test_missing_approval_stops_before_ros_or_host(self):
        with self.assertRaises(ValueError):
            run(NS(execute_flight=False,confirm_flight=None),dict(static_blockers=[]),Path('/unused'))


class AnchorGraphTests(unittest.TestCase):
    def setUp(self):
        self.resets=(1,1,1,1,1,1000)
        self.anchor=FrameAnchor((1,2,.2),(7,8,9),10_000_000,10_000_000,self.resets,
                                'px4_groundtruth_and_local_position')
        self.p=NS(x=7.2,y=8.1,z=9.,heading=math.pi/2,**dict(zip(RESET_FIELDS,self.resets)))
        self.pose=dict(position_enu=[1.1,2.2,.2],quaternion_enu=[1.,0.,0.,0.],native_timestamp_us=11_000_000)

    def test_real_origin_translation_and_enu_ned(self):
        a=measured_anchor(self.anchor,self.pose,self.p,11_001_000)
        np.testing.assert_allclose(a.to_ned([2.1,2.2,1.2]),[7.2,9.1,8.])

    def test_unpaired_reset_bad_heading_and_origin_rejected(self):
        for kind in ['time','reset','heading','origin']:
            p=copy.copy(self.p);stamp=11_001_000
            if kind=='time':stamp+=30_000
            if kind=='reset':p.heading_reset_counter+=1
            if kind=='heading':p.heading=0.
            if kind=='origin':p.x+=1
            with self.assertRaises(ValueError):measured_anchor(self.anchor,self.pose,p,stamp)

    def graph(self):
        g={'/fmu/out/'+k:dict(publishers=1,gids=[[1]*12+[i]],types=['px4_msgs/msg/'+k],
            expected_type='px4_msgs/msg/'+k) for i,k in enumerate((*REQUIRED,'vehicle_command_ack'))}
        g.update({'/fmu/in/'+k:dict(publishers=0,readers=1) for k in ['vehicle_command','trajectory_setpoint','offboard_control_mode']})
        return g

    def test_zero_other_controllers_and_single_participant(self):
        validate_graph(self.graph())
        for kind in ['publisher','gid','schema','reader']:
            g=self.graph()
            if kind=='publisher':g['/fmu/in/vehicle_command']['publishers']=1
            if kind=='gid':g['/fmu/out/vehicle_status']['gids'][0][0]=2
            if kind=='schema':g['/fmu/out/vehicle_status']['types']=['wrong']
            if kind=='reader':g['/fmu/in/vehicle_command']['readers']=0
            with self.assertRaises(ValueError):validate_graph(g)

    def test_graph_check_queries_px4_message_versioned_topic_names(self):
        names=('VehicleLocalPosition','VehicleStatus','VehicleAttitude','VehicleLandDetected',
               'EstimatorStatusFlags','TimesyncStatus','VehicleCommandAck','FailsafeFlags')
        versions=dict(VehicleLocalPosition=1,VehicleStatus=4,VehicleCommandAck=1)
        classes={name:type(name,(),{'MESSAGE_VERSION':versions.get(name,0)}) for name in names}
        by_key=dict(vehicle_local_position=classes['VehicleLocalPosition'],
            vehicle_status=classes['VehicleStatus'],vehicle_attitude=classes['VehicleAttitude'],
            vehicle_land_detected=classes['VehicleLandDetected'],
            estimator_status_flags=classes['EstimatorStatusFlags'],timesync_status=classes['TimesyncStatus'],
            vehicle_command_ack=classes['VehicleCommandAck'],failsafe_flags=classes['FailsafeFlags'])
        class FakeNode(GraphCheckMixin):
            types=by_key
            inputs={k:type(k,(),{'MESSAGE_VERSION':0}) for k in
                    ('vehicle_command','trajectory_setpoint','offboard_control_mode')}
            demo_only=False
            publishers_control={}
            expected_participant=None
            def __init__(self): self.queried=[]
            def count_publishers(self,topic): return 1 if topic.startswith('/fmu/out/') else 0
            def count_subscribers(self,topic): return 1
            def get_publishers_info_by_topic(self,topic):
                self.queried.append(topic)
                cls=next(cls for key,cls in self.types.items()
                         if topic==topic_name('/fmu/out/'+key,cls))
                return [NS(endpoint_gid=[1]*12+[2],topic_type='px4_msgs/msg/'+cls.__name__)]
            def get_subscriptions_info_by_topic(self,topic): return []
            def log(self,*args,**kwargs): pass
        node=FakeNode();node.graph_check()
        self.assertIn('/fmu/out/vehicle_status_v4',node.queried)
        self.assertIn('/fmu/out/vehicle_local_position_v1',node.queried)
        self.assertIn('/fmu/out/vehicle_command_ack_v1',node.queried)
        self.assertNotIn('/fmu/out/vehicle_status',node.queried)

    def test_graph_reports_missing_versioned_endpoint_directly(self):
        graph=self.graph()
        graph['/fmu/out/vehicle_status_v4']=graph.pop('/fmu/out/vehicle_status')
        graph['/fmu/out/vehicle_status_v4']['publishers']=0
        with self.assertRaisesRegex(ValueError,'PX4_TOPIC_PUBLISHER_COUNT:/fmu/out/vehicle_status_v4:0'):
            validate_graph(graph,required_outputs=7)


class NativeStatusMonitorTests(unittest.TestCase):
    def test_native_readonly_status_requires_fresh_source_time_and_reports_readiness(self):
        monitor=NativeStatusMonitor(NS(px4_root='/unused',instance=0))
        monitor.records={
            'actuator_armed':dict(exit=0,stdout='timestamp: 1000000\n ready_to_arm: True\n',
                received=10.,timestamp_us=1_000_000,progressed=10.),
            'health_report':dict(exit=0,stdout='timestamp: 1000000\n arming_check_error_flags: 0\n'
                ' arming_check_warning_flags: 0\n health_error_flags: 0\n',received=10.,
                timestamp_us=1_000_000,progressed=10.),
            'logger':dict(exit=0,stdout='Full File Logging Running:',received=10.)}
        state=monitor.snapshot(1_000_000,10.2)
        self.assertEqual(state['actuator_armed']['ready_to_arm'],'True')
        self.assertTrue(state['logger_active'])
        monitor.records['health_report']['progressed']=8.
        state=monitor.snapshot(1_000_000,10.2)
        self.assertFalse(state['health_report_fresh'])
        self.assertEqual(state['health_reason'],'health_report_stale_or_not_published')
        monitor.records['actuator_armed']['progressed']=8.
        with self.assertRaisesRegex(ValueError,'NATIVE_PX4_STATUS_STALE:actuator_armed'):
            monitor.snapshot(1_000_000,10.2)


class PairedWatchObservationTests(unittest.TestCase):
    def test_watchdog_receives_exact_gt_pose_paired_to_px4_timestamp(self):
        sample=Sample(wall=5.,native_us=969_568_000,actual=(-11.7,4.3,.1),
            velocity=(0.,0.,0.),resets=(1,1,1,1,1,100),groundtruth_native_us=969_566_000,
            groundtruth_received_wall=4.99)
        # A newer async GT sample must not replace the valid 2 ms pair above.
        observation=paired_watch_observation(sample,(0.,0.,0.),sample.actual,4.995,5.)
        self.assertEqual(observation.native_gt_us,969_566_000)
        self.assertEqual(observation.native_ekf_us,969_568_000)
        self.assertLessEqual(abs(observation.native_gt_us-observation.native_ekf_us),20_000)

    def test_watchdog_pair_tolerance_remains_a_hard_gate(self):
        sample=Sample(wall=5.,native_us=969_600_000,actual=(0.,0.,0.),
            velocity=(0.,0.,0.),resets=(1,1,1,1,1,100),groundtruth_native_us=969_566_000,
            groundtruth_received_wall=4.99)
        with self.assertRaisesRegex(ValueError,'NATIVE_TIME_PAIR_INVALID'):
            paired_watch_observation(sample,(0.,0.,0.),sample.actual,4.995,5.)

    def test_takeoff_only_suppresses_target_tracking_not_freshness_or_frame_gates(self):
        anchor=FrameAnchor((0,0,.2),(10,20,-.1),15_000_000,15_000_000,
            (1,1,1,1,1,1000),'px4_groundtruth_and_local_position')
        env=FlightEnvironment([],[-10,-10,-1],[10,10,3.2])
        watch=GroundTruthWatchdog(anchor,env,project_ceiling_required=False)
        s=WatchSample((0,0,.2),(10,20,-.1),(0,0,1.),15_000_000,15_000_000,
            10.,10.,10.,(1,1,1,1,1,1000))
        result=watch.evaluate(s,10.,tracking_required=False)
        self.assertNotIn('GROUNDTRUTH_TRACKING',result['reasons'])
        stale=replace(s,received_gt_wall=9.)
        self.assertIn('GROUNDTRUTH_STALE',watch.evaluate(stale,10.,tracking_required=False)['reasons'])
        # The unchanged default retains the cruise threshold.
        cruise=replace(s,target_enu=(0,0,.451))
        self.assertIn('GROUNDTRUTH_TRACKING',watch.evaluate(cruise,10.,tracking_required=True)['reasons'])


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.resets=(1,1,1,1,1,1000)
        self.anchor=FrameAnchor((0,0,.2),(10,20,-.1),15_000_000,15_000_000,self.resets,
                                'px4_groundtruth_and_local_position')
        self.env=FlightEnvironment([],[-10,-10,1],[10,10,3.2])
        self.contact=FlightEnvironment([],[-10,-10,-1],[10,10,3.2])
        self.path=np.array([[i*.1,0,2.5] for i in range(17)])
        self.m=Mission(self.path,self.anchor,self.env,self.contact)

    def s(self,wall=1.,**kwargs):
        s=Sample(wall,int(15_000_000+(wall-1)*1e6),(0,0,.2),(0,0,0),self.resets)
        return replace(s,**kwargs)

    def test_no_arm_before_prestream_and_ack_plus_offboard_mode(self):
        s=self.s();self.assertIsNone(self.m.step(s)['command'])
        # Small ticks prevent a scheduler-gap fault.
        for i in range(1,25):d=self.m.step(self.s(1+i*.05))
        self.assertEqual(d['command']['command'],176)
        self.assertEqual(self.m.stage,'MODE_PENDING')
        self.m.ack(176,0,2.21,16_210_000)
        self.assertIsNone(self.m.step(self.s(2.25))['command'])
        d=self.m.step(self.s(2.30,nav=14))
        self.assertEqual(d['stage'],'ARM_READINESS')
        self.assertIsNone(d['command'])
        d=self.m.step(self.s(2.35,nav=14))
        self.assertEqual(d['command'],dict(command=400,param1=1.,param2=0.))

    def test_disarmed_offboard_lost_and_logger_inactive_do_not_block_prestream(self):
        s=self.s(preflight=False,ready_to_arm=False,offboard_signal_lost=True,logger_active=False)
        self.assertEqual(self.m.step(s)['stage'],'PRESTREAM')
        for i in range(1,25):
            d=self.m.step(self.s(1+i*.05,preflight=False,ready_to_arm=False,
                offboard_signal_lost=True,logger_active=False))
        self.assertEqual(d['command']['command'],176)
        self.m.ack(176,0,2.21,16_210_000)
        d=self.m.step(self.s(2.25,nav=14,preflight=False,ready_to_arm=False,
            offboard_signal_lost=False,logger_active=False))
        self.assertEqual(d['stage'],'ARM_READINESS')
        self.assertIsNone(d['command'])
        d=self.m.step(self.s(2.30,nav=14,preflight=True,ready_to_arm=True,
            offboard_signal_lost=False,logger_active=False))
        self.assertEqual(d['command']['command'],400)

    def test_true_preflight_failure_times_out_without_arm_and_logs_health_reason(self):
        s=self.s(nav=14);self.m.enter('ARM_READINESS',s)
        d=self.m.step(self.s(14.,nav=14,preflight=False,ready_to_arm=False,
            arming_status_fresh=True,arming_reason='arming_check_error_flags=8'))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertIn('arming_check_error_flags=8',self.m.failure)
        self.assertIsNone(d['command'])

    def test_post_arm_logger_timeout_disarms_only_while_still_landed(self):
        s=self.s(armed=True,landed=True,nav=14);self.m.enter('POST_ARM_LOGGER',s)
        command=None
        for i in range(1,103):
            d=self.m.step(self.s(1+i*.05,armed=True,landed=True,nav=14,logger_active=False))
            if d.get('command'): command=d['command']
        self.assertEqual(self.m.stage,'DISARM_PENDING')
        self.assertEqual(command,dict(command=400,param1=0.,param2=0.))
        self.assertIsNotNone(d['target'])
        self.m.ack(400,0,6.2,20_200_000)
        d=self.m.step(self.s(6.25,armed=False,landed=True,nav=14,logger_active=False))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertEqual(self.m.failure,'POST_ARM_LOGGER_INACTIVE_DISARMED')

    def test_post_arm_logger_pass_is_required_before_takeoff(self):
        s=self.s(armed=True,landed=True,nav=14);self.m.enter('POST_ARM_LOGGER',s)
        d=self.m.step(self.s(1.05,armed=True,landed=True,nav=14,logger_active=True))
        self.assertEqual(d['stage'],'TAKEOFF')

    def test_post_arm_unexpected_airborne_state_requests_land_not_takeoff(self):
        s=self.s(armed=True,landed=False,nav=14);self.m.enter('POST_ARM_LOGGER',s)
        d=self.m.step(replace(s,wall=1.05,native_us=15_050_000))
        self.assertEqual(d['stage'],'LANDING')
        self.assertEqual(d['command']['command'],21)
        self.assertNotEqual(d['stage'],'TAKEOFF')

    def test_arm_rejection_records_ack_health_details_and_never_takes_off(self):
        s=self.s(nav=14);self.m.enter('ARM_PENDING',s);self.m.command(400,1.,0.,s)
        self.m.ack(400,2,1.01,s.native_us,dict(result_param1=1,result_param2=0))
        d=self.m.step(self.s(1.05,nav=14))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertIn('result_param1',self.m.failure)
        self.assertNotIn(self.m.stage,['TAKEOFF','FOLLOW'])

    def test_observation_before_anchor_never_starts_stream(self):
        d=self.m.step(self.s(native_us=14_900_000))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertIsNone(d['target']);self.assertIsNone(d['command'])

    def active(self,stage='FOLLOW'):
        s=self.s(armed=True,landed=False,nav=14,actual=tuple(self.path[0]),final_heading=True)
        self.m.started_wall,self.m.started_native=s.wall,s.native_us
        self.m.target=self.path[0].copy();self.m.enter(stage,s)
        self.m.route_started=s.native_us;self.m.begin_leg(self.path[0],self.path[1],s,follow=True)
        return s

    def takeoff(self):
        s=self.s(armed=True,landed=True,nav=14)
        self.m.started_wall=s.wall;self.m.started_native=s.native_us
        self.m.enter('TAKEOFF',s)
        self.m.takeoff_started_native=s.native_us
        self.m.takeoff_groundtruth_start_z=s.actual[2]
        self.m.begin_leg(self.anchor.gazebo_enu,self.path[0],s)
        return s

    def test_takeoff_spoolup_can_exceed_cruise_tracking_without_early_abort(self):
        self.takeoff()
        decision=None
        # Simulate >0.25m commanded climb while GT still reports floor contact.
        for i in range(1,81):
            decision=self.m.step(self.s(1+i*.05,armed=True,landed=True,nav=14))
        self.assertEqual(self.m.stage,'TAKEOFF')
        self.assertIsNone(self.m.failure)
        self.assertGreater(float(np.linalg.norm(self.m.target-np.array([0,0,.2]))),.25)
        self.assertIsNone(decision.get('command'))

    def test_takeoff_groundtruth_liftoff_before_eight_seconds_is_logged_once(self):
        self.takeoff();event_count=0
        for i in range(1,150):
            z=.2 if i<145 else .36
            decision=self.m.step(self.s(1+i*.05,armed=True,landed=z<=.2,nav=14,actual=(0,0,z)))
            event_count += int(bool(decision.get('event')))
            if self.m.takeoff_liftoff_confirmed: break
        self.assertTrue(self.m.takeoff_liftoff_confirmed)
        self.assertEqual(event_count,1)
        self.assertGreaterEqual(decision['event']['climb_m'],.15)
        self.assertLessEqual(decision['event']['elapsed_sim_s'],8.)

    def test_takeoff_without_groundtruth_liftoff_requests_land_after_eight_sim_seconds(self):
        self.takeoff();land_decision=None
        for i in range(1,164):
            decision=self.m.step(self.s(1+i*.05,armed=True,landed=True,nav=14))
            if decision.get('command'):
                land_decision=decision
        self.assertEqual(self.m.stage,'LANDING')
        self.assertEqual(land_decision['command']['command'],21)
        self.assertEqual(self.m.failure,'TAKEOFF_NO_GROUNDTRUTH_LIFTOFF_WITHIN_8S')

    def test_takeoff_height_xy_velocity_and_failsafe_gates_remain_active(self):
        cases=(dict(actual=(0,0,3.41)),dict(actual=(11,0,.2)),
               dict(velocity=(.71,0,0)),dict(failsafe=True))
        for kwargs in cases:
            self.setUp();self.takeoff()
            d=self.m.step(self.s(1.05,armed=True,landed=False,nav=14,**kwargs))
            self.assertNotEqual(self.m.stage,'TAKEOFF')
            self.assertIsNotNone(self.m.failure)

    def test_demo_only_still_enforces_takeoff_height_cap(self):
        self.m=Mission(self.path,self.anchor,self.env,self.contact,demo_only=True)
        self.takeoff()
        d=self.m.step(self.s(1.05,armed=True,landed=False,nav=14,actual=(0,0,3.41)))
        self.assertEqual(self.m.failure,'RELATIVE_HEIGHT')

    def test_takeoff_must_recover_cruise_tracking_before_hold_a(self):
        self.takeoff();self.m.takeoff_liftoff_confirmed=True
        self.m.motion=NS(position=lambda elapsed: self.path[0].copy(),duration=0.,end=self.path[0].copy())
        self.m.phase_native=15_000_000
        for i in range(1,103):
            d=self.m.step(self.s(1+i*.05,armed=True,landed=False,nav=14,
                actual=tuple(self.path[0]),velocity=(0,0,0)))
        self.assertEqual(self.m.stage,'HOLD_A')
        self.assertLessEqual(float(np.linalg.norm(self.path[0]-np.array(d['target']))),.25)
        self.assertTrue(self.m.takeoff_liftoff_confirmed)

    def test_takeoff_hold_follow_tracking_abort_requests_land_and_never_confirms_goal(self):
        self.takeoff()
        self.m.takeoff_liftoff_confirmed=True
        self.m.motion=NS(position=lambda elapsed: self.path[0].copy(),duration=0.,end=self.path[0].copy())
        self.m.phase_native=15_000_000
        for i in range(1,103):
            self.m.step(self.s(1+i*.05,armed=True,landed=False,nav=14,
                actual=tuple(self.path[0]),velocity=(0,0,0)))
        self.assertEqual(self.m.stage,'HOLD_A')

        sample=self.s(6.2,armed=True,landed=False,nav=14,final_heading=True,
            actual=tuple(self.path[0]),velocity=(0,0,0))
        self.m.step(sample)
        self.assertEqual(self.m.stage,'FOLLOW')
        self.assertEqual(self.m.index,1)

        # Reproduce the observed independent GT tracking watchdog fault during
        # a route leg. The existing 0.25 m gate must request Land, not success.
        decision=self.m.fail('GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING',sample)
        self.assertEqual(decision['stage'],'LANDING')
        self.assertEqual(decision['command']['command'],21)
        self.assertFalse(self.m.goal_confirmed)

        self.m.ack(21,0,6.26,sample.native_us+60_000)
        self.m.step(replace(sample,wall=6.3,native_us=sample.native_us+100_000,nav=18))
        for i in range(1,5):
            decision=self.m.step(replace(sample,wall=6.3+i*.05,
                native_us=sample.native_us+100_000+i*50_000,nav=4,
                armed=False,landed=i==4))
        self.assertEqual(decision['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertEqual(self.m.failure,'GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING')
        self.assertFalse(self.m.success)
        self.assertFalse(self.m.goal_confirmed)

    def test_accepted_land_mode_uses_bounded_vertical_descent_speed(self):
        s=self.active()
        self.m.fail('GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING',s)
        self.m.ack(21,0,1.01,s.native_us)
        self.m.step(replace(s,wall=1.05,native_us=s.native_us+50_000,nav=18))
        self.assertTrue(self.m.land_mode_accepted)

        # PX4's accepted Land mode may descend slightly faster than the cruise
        # vector-speed ceiling. Horizontal speed and a finite vertical bound
        # remain enforced through confirmation.
        d=self.m.step(replace(s,wall=1.10,native_us=s.native_us+100_000,
            nav=18,velocity=(0.,0.,.715)))
        self.assertEqual(d['stage'],'LANDING')
        self.assertTrue(self.m.land_mode_accepted)
        d=self.m.step(replace(s,wall=1.15,native_us=s.native_us+150_000,
            nav=18,velocity=(0.,0.,.91)))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertEqual(self.m.failure,'GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING')

    def test_heading_false_is_allowed_for_initial_yaw_but_not_horizontal_route(self):
        self.assertEqual(self.m.step(self.s(final_heading=False))['stage'],'PRESTREAM')
        s=self.active()
        d=self.m.step(replace(s,final_heading=False))
        self.assertEqual(d['command']['command'],21)
        self.assertFalse(self.m.goal_confirmed)

    def test_hold_a_requires_final_heading(self):
        s=self.active('HOLD_A')
        d=self.m.step(replace(s,final_heading=False))
        self.assertEqual(d['stage'],'HOLD_A')
        d=self.m.step(replace(s,wall=1.05,native_us=15_050_000,final_heading=True))
        self.assertEqual(d['stage'],'FOLLOW')

    def test_missing_arm_or_offboard_ack_cannot_take_off(self):
        for stage,command in [('MODE_PENDING',176),('ARM_PENDING',400)]:
            self.setUp();s=self.s(nav=14)
            self.m.enter(stage,s);self.m.command(command,1.,0.,s)
            # Stale ACK before send is deliberately excluded.
            self.m.ack(command,0,.5,14_500_000)
            for i in range(65):d=self.m.step(self.s(1+i*.05,nav=14))
            self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')

    def test_ack_rejected_stops_without_retry(self):
        s=self.s();self.m.enter('MODE_PENDING',s);self.m.command(176,1.,6.,s)
        self.m.ack(176,2,1.01,15_010_000)
        d=self.m.step(self.s(1.05))
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED');self.assertIsNone(d['command'])

    def test_estimator_reset_or_lost_time_cannot_blind_land(self):
        for kind in ['reset','time','estimator','gcs']:
            self.setUp();s=self.active()
            if kind=='reset':s=replace(s,resets=(2,1,1,1,1,1000))
            if kind=='time':s=replace(s,coherent=False)
            if kind=='estimator':s=replace(s,valid=False)
            if kind=='gcs':s=replace(s,gcs_lost=True)
            d=self.m.step(s)
            self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED');self.assertIsNone(d['command'])

    def test_tracking_or_deadline_fault_requests_land_once_if_usable(self):
        s=self.active();s=replace(s,actual=(.3,0,2.5))
        d=self.m.step(s);self.assertEqual(d['command']['command'],21)
        d=self.m.fail('another fault',s)
        self.assertIsNone(d['command'])
        self.assertEqual(self.m.failure,'TRACKING_LIMIT')

    def test_first_abort_reason_is_latched_across_landing_faults(self):
        s=self.active()
        self.m.fail('GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING',s)
        d=self.m.fail('GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_SPEED',s)
        self.assertEqual(d['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertEqual(self.m.failure,'GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING')

    def test_wall_clock_limits_independent_of_simulation_time(self):
        s=self.active();self.m.last_wall=0.;self.m.last_native=s.native_us
        d=self.m.step(s)
        self.assertIn('SCHEDULING',self.m.failure)
        self.assertEqual(d['command']['command'],21)

    def test_land_ack_is_not_landed_and_disarmed(self):
        s=self.active();self.m.enter('LANDING',s);self.m.land_sent=True;self.m.command(21,0.,0.,s)
        self.m.ack(21,0,1.,s.native_us)
        self.m.step(replace(s,nav=18))
        self.assertFalse(self.m.success);self.assertEqual(self.m.stage,'LANDING')

    def test_land_pending_keeps_stream_until_ack_and_mode(self):
        s=self.active();decision=self.m.fail('abort with usable pose',s)
        self.assertIsNotNone(decision['target'])
        decision=self.m.step(replace(s,wall=1.05,native_us=15_050_000))
        self.assertIsNotNone(decision['target'])
        self.m.ack(21,0,1.06,15_060_000)
        decision=self.m.step(replace(s,wall=1.1,native_us=15_100_000,nav=18))
        self.assertIsNone(decision['target'])
        self.assertFalse(self.m.success)

    def test_abort_land_mode_acceptance_latches_until_landed_disarmed(self):
        s=self.active(); original='GROUNDTRUTH_WATCHDOG:GROUNDTRUTH_TRACKING'
        decision=self.m.fail(original,s)
        self.assertEqual(decision['command']['command'],21)
        self.m.ack(21,0,1.06,15_060_000)
        decision=self.m.step(replace(s,wall=1.1,native_us=15_100_000,nav=18))
        self.assertIsNone(decision['target'])
        self.assertTrue(self.m.land_mode_accepted)
        # PX4 returns to its disarmed nav state after completing landing. Keep
        # normal sample cadence long enough to cross the old ACK timeout.
        for i in range(1,62):
            landed=i==61
            decision=self.m.step(replace(s,wall=1.1+i*.05,native_us=15_100_000+i*50_000,
                nav=14,landed=landed,armed=not landed))
        self.assertEqual(decision['stage'],'ABORT_OPERATOR_REQUIRED')
        self.assertEqual(self.m.failure,original)
        self.assertFalse(self.m.success)

    def test_land_ack_mode_latch_survives_disarmed_hold_before_landed_sample(self):
        s=self.active();self.m.enter('LANDING',s);self.m.land_sent=True;self.m.command(21,0.,0.,s)
        self.m.ack(21,0,1.01,s.native_us)
        d=self.m.step(replace(s,wall=1.05,native_us=15_050_000,nav=18,armed=False,landed=False))
        self.assertEqual(self.m.stage,'LANDING')
        self.assertTrue(self.m.land_mode_accepted)
        for i in range(1,5):
            d=self.m.step(replace(s,wall=1.05+i*.05,native_us=15_050_000+i*50_000,
                nav=4,armed=False,landed=i==4))
        self.assertEqual(self.m.stage,'DONE')

    def test_native_land_watchdog_does_not_require_offboard_stream_but_keeps_gt_gate(self):
        position=(0,0,2.5)
        sample=WatchSample(position,tuple(self.anchor.to_ned(position)),position,
            17_000_000,17_000_000,10.,10.,8.,self.resets)
        result=GroundTruthWatchdog(self.anchor,self.env).evaluate(sample,10.,offboard_stream_required=False)
        self.assertEqual(result['reasons'],[])
        sample=replace(sample,received_gt_wall=9.)
        result=GroundTruthWatchdog(self.anchor,self.env).evaluate(sample,10.,offboard_stream_required=False)
        self.assertIn('GROUNDTRUTH_STALE',result['reasons'])

    def test_complete_integrated_mission_visits_all_raw_waypoints(self):
        commands=[];visited=set();nav=4;armed=False;landing=False;actual=np.array([0.,0.,.2]);old=actual.copy()
        for tick in range(8000):
            wall=1+tick*.05
            if landing:
                actual=actual.copy();actual[2]=max(.2,actual[2]-.01)
                if actual[2]<=.200001:armed=False
            else:actual=self.m.target.copy()
            velocity=(actual-old)/.05;old=actual.copy()
            sample=self.s(wall,actual=tuple(actual),velocity=tuple(velocity),nav=nav,armed=armed,
                landed=actual[2]<=.200001,final_heading=actual[2]>2.)
            decision=self.m.step(sample)
            if self.m.stage=='FOLLOW':visited.add(self.m.index)
            if decision['command']:
                c=decision['command'];commands.append(c['command'])
                self.m.ack(c['command'],0,wall,sample.native_us)
                if c['command']==176:nav=14
                elif c['command']==400:armed=True
                elif c['command']==21:nav=18;landing=True
            if self.m.stage in ['DONE','ABORT_OPERATOR_REQUIRED']:break
        self.assertTrue(self.m.success,self.m.failure)
        self.assertTrue(self.m.goal_confirmed)
        self.assertEqual(visited,set(range(1,17)))
        self.assertEqual(commands,[176,400,21])


if __name__=='__main__':unittest.main()
