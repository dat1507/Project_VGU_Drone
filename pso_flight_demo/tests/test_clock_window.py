"""Finite-window coordination tests use fake clocks/native data; no real physics."""

import ast
import copy
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import time
import sys
import unittest
from unittest.mock import patch

from pso_flight_demo.clock_window import (APPROVAL, WindowProtocol, WindowMonitor,
                                        check_operator_window, graph_problems)
from pso_flight_demo.clock_window import publish_once
from pso_flight_demo.dds_evidence import REQUIRED, assess_window
from pso_flight_demo import runtime_preparation as rp
from pso_flight_demo.tests import test_runtime_preparation as fixtures


def native(**changes):
    d=dict(runtime_identity={'boot_id':'FAKE_BOOT','px4_pid':100,'px4_start_ticks':1},
        session_id='FAKE_SESSION',profile_sha256='FAKE_PROFILE',sdf_sha256='FAKE_SDF',
        iterations=20000,clock_us=20_000_000,paused=True,disarmed=True,landed=True,
        auto_arm_disabled=True,reset_counters=[0]*6,heading=1.5,heading_good_for_control='False',
        errors=[],probe_started_monotonic=0.,captured_monotonic=0.)
    d.update(changes)
    return d


def graph():
    d={'/fmu/out/'+key:dict(publishers=1,publisher_gids=[[1]*16],
        expected_type='px4_msgs/msg/'+key,actual_types=['px4_msgs/msg/'+key]) for key in REQUIRED}
    d.update({'/fmu/in/'+key:dict(px4_readers=1,control_publishers=0)
              for key in ('vehicle_command','offboard_control_mode','trajectory_setpoint')})
    return d


def ready_protocol():
    p=WindowProtocol(native());p.deadline=60.
    assert p.ready(graph(),native(),True,list(REQUIRED),1.)
    return p


class ProtocolTests(unittest.TestCase):
    def test_ready_without_physics_times_out_once(self):
        p=ready_protocol();p.finish(60.)
        self.assertEqual(p.state,'NO_GO')
        self.assertIn('OBSERVER_TIMEOUT_NO_CONFIRMED_CLOCK',p.problems)

    def test_physics_before_ready_fails(self):
        p=WindowProtocol(native())
        self.assertFalse(p.ready(graph(),native(iterations=25000,clock_us=25_000_000),True,list(REQUIRED),1.))
        self.assertIn('PHYSICS_BEFORE_SUBSCRIPTIONS_READY',p.problems)

    def test_no_default_approval_and_early_physics_rejected(self):
        p=ready_protocol()
        with self.assertRaises(ValueError):p.approve('',native(),2.)
        p.native(native(iterations=21000,clock_us=21_000_000,paused=False),3.)
        self.assertEqual(p.state,'NO_GO')

    def test_operator_approval_does_not_advance_physics(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        self.assertEqual(p.state,'WAIT_FOR_OPERATOR_APPROVAL')
        self.assertIsNone(p.complete_at)

    def test_timeout_in_middle_of_window(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        p.native(native(iterations=22000,clock_us=22_000_000,paused=False),4.)
        p.finish(60.)
        self.assertIn('OBSERVER_TIMEOUT_MID_WINDOW',p.problems)

    def test_wrong_iteration_paused_endpoint_fails(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        p.native(native(iterations=24999,clock_us=25_000_000),8.)
        self.assertEqual(p.state,'NO_GO')

    def test_endpoint_without_pause_is_not_complete(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        p.native(native(iterations=25000,clock_us=25_000_000,paused=False),8.)
        self.assertIsNone(p.complete_at);p.finish(60.)
        self.assertEqual(p.state,'NO_GO')

    def test_world_control_ack_is_not_native_completion(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        p.native(native(world_control_ack=True),8.)
        self.assertIsNone(p.complete_at);p.finish(60.)
        self.assertEqual(p.state,'NO_GO')

    def test_exact_native_endpoint_then_save(self):
        p=ready_protocol();p.approve(APPROVAL,native(),2.)
        p.native(native(iterations=25000,clock_us=25_000_000),8.)
        self.assertEqual(p.state,'VERIFY_EXACT_CLOCK_PROGRESS')
        p.finish(8.3);self.assertEqual(p.state,'STOP_AND_SAVE_OBSERVATIONS')

    def test_identity_armed_landed_autoarm_reset_heading_faults(self):
        faults=(dict(runtime_identity={'boot_id':'OTHER'}),dict(disarmed=False),dict(landed=False),
                dict(auto_arm_disabled=False),dict(reset_counters=[1]*6),dict(heading=1.9),
                dict(heading_good_for_control='True'),dict(errors=['denied']))
        for fault in faults:
            with self.subTest(fault=fault):
                p=ready_protocol();p.native(native(**fault),2.)
                self.assertEqual(p.state,'NO_GO')
                p=ready_protocol();p.approve(APPROVAL,native(),2.)
                sample=native(iterations=22000,clock_us=22_000_000,paused=False)
                sample.update(fault);p.native(sample,3.)
                self.assertEqual(p.state,'NO_GO')

    def test_missing_subscriptions_groundtruth_or_control_publishers_not_ready(self):
        for created,gt,gr in ((list(REQUIRED[:-1]),True,graph()),(list(REQUIRED),False,graph()),
                              (list(REQUIRED),True,{**graph(),'/fmu/in/vehicle_command':{'control_publishers':1,'px4_readers':1}})):
            p=WindowProtocol(native());p.deadline=60.
            self.assertFalse(p.ready(gr,native(),gt,created,1.))
            self.assertIsNone(p.ready_at)

    def test_bounded_deadline_cannot_be_extended_by_approval(self):
        p=ready_protocol()
        with self.assertRaises(ValueError):p.approve(APPROVAL,native(),50.)
        self.assertEqual(p.deadline,60.)


class AssessmentTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.FiniteWindowTests();f.setUp()
        self.b,self.a,self.obs,self.gt,self.sw=f.b,f.a,f.obs,f.gt,f.sw

    def assess(self):return assess_window(self.b,self.a,self.obs,self.gt,self.sw)

    def test_existing_assessment_still_passes_only_finite_evidence(self):
        d=self.assess();self.assertEqual(d['result'],'PASS_FINITE_CLOCK_EVIDENCE_ONLY')
        self.assertFalse(d['live_control_go'])

    def test_missing_each_of_six_topics_fails(self):
        for key in REQUIRED:
            old=self.obs['messages'].pop(key)
            self.assertEqual(self.assess()['result'],'NO_GO')
            self.obs['messages'][key]=old

    def test_frozen_and_reversed_source_timestamps_fail(self):
        rows=self.obs['messages']['vehicle_status']['samples'];old=copy.deepcopy(rows)
        for row in rows:row['timestamp']=rows[0]['timestamp']
        self.assertEqual(self.assess()['result'],'NO_GO')
        self.obs['messages']['vehicle_status']['samples']=list(reversed(old))
        self.assertEqual(self.assess()['result'],'NO_GO')

    def test_unexpected_arm_and_control_publisher_fail(self):
        self.a['evidence']['actuator_armed']['stdout']=' timestamp: 20000000\n armed: True\n'
        self.assertIn('PX4_NOT_DISARMED_LANDED',self.assess()['problems'])
        self.obs['graph']['/fmu/in/vehicle_command']=dict(control_publishers=1)
        self.assertIn('UNEXPECTED_CONTROL_PUBLISHER',self.assess()['problems'])

    def test_missing_groundtruth_and_gcs_rx_no_progress(self):
        self.gt['samples']=[]
        self.assertIn('INDEPENDENT_GROUNDTRUTH_WINDOW',self.assess()['problems'])
        self.a['gcs_evidence']=copy.deepcopy(self.b['gcs_evidence'])
        self.a['gcs_evidence']['clock_us']=20_000_000
        self.assertIn('GCS_RX_AND_TIMESTAMP',self.assess()['problems'])

    def test_heading_and_reset_changes_fail(self):
        text=self.a['evidence']['vehicle_local_position']['stdout']
        self.a['evidence']['vehicle_local_position']['stdout']=text.replace('heading: 1.67180','heading: 2.00000')
        self.assertIn('DDS_NATIVE_TIME_POSITION_OR_RESET_CORRELATION',self.assess()['problems'])
        self.a['evidence']['vehicle_local_position']['stdout']=text.replace('xy_reset_counter: 5','xy_reset_counter: 6')
        self.assertIn('DDS_NATIVE_TIME_POSITION_OR_RESET_CORRELATION',self.assess()['problems'])


class HandshakeTests(unittest.TestCase):
    def test_live_challenge_returns_text_without_executing_physics(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);owner={'pid':123}
            status=dict(state='WAIT_FOR_OPERATOR_APPROVAL',heartbeat_monotonic=10.,deadline_monotonic=60.,
                        owner=owner,native=native(),operator_command='OPERATOR_ONLY_TEXT',rollback_command='PAUSE_TEXT')
            (p/'observer_status.json').write_text(json.dumps(status))
            def reply(path,value):
                publish_once(path,value)
                publish_once(p/('operator_response_'+value['nonce']+'.json'),
                             dict(nonce=value['nonce'],ok=True,checked_monotonic=10.))
            with patch('pso_flight_demo.clock_window.same_process',return_value=True), \
                 patch('pso_flight_demo.clock_window.identity',return_value=owner), \
                 patch('pso_flight_demo.clock_window.publish_once',side_effect=reply), \
                 patch('subprocess.run',side_effect=AssertionError('No physics')):
                result=check_operator_window(p,APPROVAL,lambda:native(),lambda:10.)
            self.assertEqual(result['result'],'READY_FOR_SEPARATE_CLOCK_APPROVAL')
            self.assertEqual(result['operator_command'],'OPERATOR_ONLY_TEXT')
            self.assertFalse(result['controls_physics'])

    def test_atomic_handshake_never_overwrites_previous_challenge(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'challenge.json';publish_once(p,{'nonce':'first'})
            with self.assertRaises(FileExistsError):publish_once(p,{'nonce':'second'})
            self.assertEqual(json.loads(p.read_text()),{'nonce':'first'})

    def test_dead_pid_rejected_even_with_fresh_heartbeat(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);(p/'observer_status.json').write_text(json.dumps(dict(state='WAIT_FOR_OPERATOR_APPROVAL',
                heartbeat_monotonic=10.,deadline_monotonic=60.,owner={'pid':123})))
            with patch('pso_flight_demo.clock_window.identity',return_value=None), self.assertRaises(ValueError):
                check_operator_window(p,APPROVAL,lambda:self.fail('No probes for dead observer'),lambda:10.)

    def test_stale_or_dead_observer_cannot_offer_command(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);status=dict(state='WAIT_FOR_OPERATOR_APPROVAL',heartbeat_monotonic=1.,
                deadline_monotonic=60.,owner={'pid':123},native=native())
            (p/'observer_status.json').write_text(json.dumps(status))
            with self.assertRaises(ValueError):check_operator_window(p,APPROVAL,lambda:native(),lambda:5.)

    def test_explicit_approval_is_required_before_native_check(self):
        with self.assertRaises(ValueError):
            check_operator_window('/unused','',lambda: self.fail('No native operation'),lambda:1.)

    def test_monitor_rejects_control_publishers_before_creating_gazebo_subscriber(self):
        with tempfile.TemporaryDirectory() as td:
            gt=NS(subscribed=False,errors=[],start=lambda:self.fail('must not subscribe'))
            with patch('pso_flight_demo.clock_window.identity',return_value={'pid':1}):
                m=WindowMonitor(td,native(),lambda:native(),gt,{'operator_command':'TEXT','rollback_command':'TEXT'})
            with self.assertRaises(ValueError):m.pre_subscriptions_check({'topic':{'control_publishers':1}})
            m.pool.shutdown()

    def test_nonblocking_monitor_does_not_stop_executor_while_native_job_waits(self):
        with tempfile.TemporaryDirectory() as td:
            gt=NS(subscribed=True,errors=[])
            with patch('pso_flight_demo.clock_window.identity',return_value={'pid':1}):
                m=WindowMonitor(td,native(),lambda:native(),gt,{'operator_command':'TEXT','rollback_command':'TEXT'})
            # A pending native task must not be synchronously awaited in update.
            m.future=NS(done=lambda:False)
            m.begin(60.);m.subscriptions_created(list(REQUIRED))
            for t in (1.,1.05,1.1):self.assertTrue(m.update(graph(),t,[]))
            m.future=None;m.finish({})

    def test_no_physics_or_publish_calls_in_coordinator(self):
        for name in ('clock_window.py','dds_observer.py','groundtruth.py'):
            tree=ast.parse((rp.MODULE/name).read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):
                    self.assertNotIn(n.func.attr,('create_publisher','publish','advertise','request','create_client'))

    def test_gazebo_subscribe_failure_is_not_ready(self):
        from pso_flight_demo.groundtruth import GazeboPoseObserver
        modules={'gz.transport13':NS(Node=lambda:NS(subscribe=lambda *args:False)),
                 'gz.msgs10.pose_v_pb2':NS(Pose_V=object)}
        g=GazeboPoseObserver('warehouse_world','x500',64)
        with patch.dict(sys.modules,modules),self.assertRaises(RuntimeError):g.start()
        self.assertFalse(g.subscribed)


class PlanTests(unittest.TestCase):
    def test_identity_race_between_preflight_and_subscriptions_is_rejected(self):
        f=fixtures.FiniteWindowTests();f.setUp();report=f.a;report['session_id']=rp.DEFAULT_SESSION.name
        report['evidence']['params_COM_ARM_ON_BOOT']={'stdout':'x COM_ARM_ON_BOOT [1,2] : 0\n'}
        changed=copy.deepcopy(report['runtime_identity']);changed['px4_start_ticks']+=1
        baseline=native(runtime_identity=changed,session_id=report['session_id'],
            profile_sha256=report['profile_sha256'],sdf_sha256=report['world']['sdf_sha256'])
        with tempfile.TemporaryDirectory() as td,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(rp,'host_snapshot',return_value=(report,f.anchor,None)), \
             patch.object(rp,'anchor_packet',return_value={}),patch.object(rp,'candidate_assessment',return_value={}), \
             patch.object(rp,'inventory',return_value=f.sw),patch.object(rp,'window_native_state',return_value=baseline), \
             patch('pso_flight_demo.groundtruth.GazeboPoseObserver.start',side_effect=AssertionError('No subscriber on changed session')):
            dest=Path(td)/'new'
            self.assertEqual(rp.main(['--expect-iterations','20000','--output',str(dest),'observe-window']),2)
            self.assertIn('Session changed',json.loads((dest/'error.json').read_text())['error'])

    def test_full_cli_saves_fake_overlapping_window_and_keeps_live_control_false(self):
        from pso_flight_demo.telemetry import gcs_snapshot
        f=fixtures.FiniteWindowTests();f.setUp()
        before=copy.deepcopy(f.a);before['session_id']=rp.DEFAULT_SESSION.name
        before['evidence']['params_COM_ARM_ON_BOOT']={'stdout':'x COM_ARM_ON_BOOT [1,2] : 0\n'}
        after=copy.deepcopy(before)
        after['evidence']['stats']['stdout']=after['evidence']['stats']['stdout'].replace('sec: 20','sec: 25').replace('iterations: 20000','iterations: 25000')
        after['gcs_evidence']=gcs_snapshot(fixtures.telemetry(0,stamp=24_800_000,rx=50000),
            {'timestamp':'25000000','gcs_connection_lost':'False'},25_000_000)
        observed=copy.deepcopy(f.obs);poses=copy.deepcopy(f.gt)
        for item in observed['messages'].values():
            for row in item['samples']:
                row['timestamp']+=5_000_000;row['timestamp_sample']+=5_000_000
        for row in poses['samples']:row['native_timestamp_us']+=5_000_000
        baseline=native(runtime_identity=before['runtime_identity'],session_id=before['session_id'],
            profile_sha256=before['profile_sha256'],sdf_sha256=before['world']['sdf_sha256'])
        class FakeGazebo:
            subscribed=False
            errors=[]
            def __init__(self,*args):pass
            def start(self):self.subscribed=True
            def finish(self):return poses
        class FakeMonitor:
            def __init__(self,directory,baseline,probe,gt,plan):self.protocol=WindowProtocol(baseline);self.gt=gt
            def status(self,*args):pass
        def fake_observe(profile,seconds,logs,ready=None,monitor=None):
            self.assertEqual(seconds,60);monitor.gt.start();self.assertTrue(monitor.gt.subscribed)
            p=monitor.protocol;p.deadline=60.
            self.assertTrue(p.ready(graph(),baseline,True,list(REQUIRED),1.))
            p.approve(APPROVAL,baseline,2.)
            p.native({**baseline,'iterations':25000,'clock_us':25_000_000},8.)
            p.finish(8.3)
            observed['coordination']=dict(state=p.state,problems=p.problems,events=p.events)
            return observed
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()), \
             patch.object(rp,'host_snapshot',side_effect=[(before,f.anchor,None),(after,f.anchor,None)]), \
             patch.object(rp,'anchor_packet',return_value={}),patch.object(rp,'candidate_assessment',return_value={}), \
             patch.object(rp,'inventory',return_value=f.sw),patch.object(rp,'window_native_state',return_value=baseline), \
             patch('pso_flight_demo.clock_window.WindowMonitor',FakeMonitor), \
             patch('pso_flight_demo.groundtruth.GazeboPoseObserver',FakeGazebo), \
             patch('pso_flight_demo.dds_observer.observe',side_effect=fake_observe), \
             patch('subprocess.run',side_effect=AssertionError('NO REAL RUNTIME OR PHYSICS')):
            dest=Path(td)/'new'
            rc=rp.main(['--expect-iterations','20000','--output',str(dest),'observe-window'])
            self.assertEqual(rc,0)
            saved=json.loads((dest/'window_assessment.json').read_text())
            self.assertEqual(saved['result'],'PASS_FINITE_CLOCK_EVIDENCE_ONLY')
            self.assertFalse(saved['live_control_go'])
            self.assertTrue((dest/'dds_raw_observation.json').exists())

    def test_plan_refuses_old_start_instead_of_rebasing(self):
        f=fixtures.FiniteWindowTests();f.setUp()
        d=rp.finite_window_plan(f.b,f.sw)
        self.assertEqual(d['result'],'NO_GO');self.assertIsNone(d['operator_command'])

    def test_conditional_exact_20000_to_25000_and_changed_identity(self):
        f=fixtures.FiniteWindowTests();f.setUp()
        report=f.a;report['session_id']=rp.DEFAULT_SESSION.name
        report['evidence']['params_COM_ARM_ON_BOOT']={'stdout':'x COM_ARM_ON_BOOT [1,2] : 0\n'}
        d=rp.finite_window_plan(report,f.sw)
        self.assertEqual(d['result'],'CONDITIONAL_20000_TO_25000_PLAN')
        self.assertEqual(d['expected_end_iterations'],25000)
        self.assertEqual(d['expected_end_sim_time_us'],25_000_000)
        self.assertFalse(d['executed']);self.assertFalse(d['live_control_go'])
        report['runtime_identity']['px4_start_ticks']+=1
        self.assertEqual(rp.finite_window_plan(report,f.sw)['result'],'NO_GO')

    def test_wall_limit_is_still_5_to_60(self):
        for seconds in ('4','61'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                rp.main(['observe-window','--wall-seconds',seconds])


if __name__ == '__main__':
    unittest.main()
