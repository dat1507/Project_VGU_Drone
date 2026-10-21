"""Synthetic/mocked tests only; no simulator/ROS participant or physics calls."""

from dataclasses import replace
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

import numpy as np

from pso_flight_demo import runtime_preparation as rp
from pso_flight_demo.dds_evidence import MessageEvidence, REQUIRED, assess_window
from pso_flight_demo.frames import FrameAnchor, RESET_FIELDS, ned_to_enu
from pso_flight_demo.groundtruth import pose_sample
from pso_flight_demo.runtime_readiness import topic_fields, number
from pso_flight_demo.runtime_safety import GroundTruthWatchdog, WatchSample, LandConfirmation
from pso_flight_demo.telemetry import topic_instances, gcs_snapshot, gcs_progress
from pso_flight_demo.environments import load_environment, load_profile


SAVED = rp.MODULE/'outputs_c2/warehouse/session_2b387491d7fc49a881acd52852d3bcc5/runtime_report.json'


def fixture():
    """Load historical evidence as an explicitly synthetic test fixture only."""
    r = json.loads(SAVED.read_text())
    r.update(session_owner_verified=True, session_id='SYNTHETIC_TEST', profile_sha256='TEST_HASH',
             captured_wall_time=time.time(), problems=[])
    r['evidence']['vehicle_local_position']['stdout'] = r['evidence']['vehicle_local_position']['stdout'].replace(
        'heading_good_for_control: False', 'heading_good_for_control: True')
    lp = topic_fields(r['evidence']['vehicle_local_position']['stdout'])
    gt = topic_fields(r['evidence']['vehicle_local_position_groundtruth']['stdout'])
    a = FrameAnchor(tuple(ned_to_enu([number(gt,k) for k in ('x','y','z')])),
        tuple(number(lp,k) for k in ('x','y','z')), int(number(gt,'timestamp_sample')),
        int(number(lp,'timestamp_sample')), tuple(int(number(lp,k)) for k in RESET_FIELDS),
        'px4_groundtruth_and_local_position')
    r['gcs_evidence'] = gcs_snapshot(r['evidence']['telemetry_status']['stdout'],
        topic_fields(r['evidence']['vehicle_status']['stdout']), 15_000_000)
    return r,a


def telemetry(index, heartbeat=True, stamp=14_800_000, rx=10):
    return f'Instance {index}:\n telemetry_status\n    timestamp: {stamp}\n    rx_message_count: {rx}\n    heartbeat_type_gcs: {heartbeat}\n    mode: 0\n    type: 0\n'


class GCSParserTests(unittest.TestCase):
    def snapshot(self, text, now=15_000_000):
        return gcs_snapshot(text, dict(timestamp=str(now),gcs_connection_lost='False'), now)

    def test_real_log_exposes_last_instance_overwrite(self):
        r = json.loads(SAVED.read_text())
        text = r['evidence']['telemetry_status']['stdout']
        # Old implementation really returned the last False, despite instance 0 True.
        import re
        self.assertEqual(dict(re.findall(r'^\s+(\w+):\s+([^\n]+)',text,re.M))['heartbeat_type_gcs'],'False')
        self.assertEqual(topic_fields(text), {})
        self.assertEqual(self.snapshot(text)['selected_instances'],[0])
        self.assertTrue(self.snapshot(text)['snapshot_pass'])

    def test_gcs_not_assumed_instance_zero(self):
        d=self.snapshot(telemetry(0,False)+telemetry(3,True))
        self.assertEqual(d['selected_instances'],[3])
        self.assertTrue(d['snapshot_pass'])

    def test_two_valid_links_are_preserved(self):
        d=self.snapshot(telemetry(2)+telemetry(5))
        self.assertEqual(d['selected_instances'],[2,5])

    def test_missing_stale_future_or_zero_rx_fails(self):
        for text in ('', ' heartbeat_type_gcs: True\n', telemetry(4,stamp=1),
                     telemetry(4,stamp=16_000_000),telemetry(4,rx=0)):
            with self.subTest(text=text):self.assertFalse(self.snapshot(text)['snapshot_pass'])

    def test_duplicate_fields_and_duplicate_instances_fail_closed(self):
        self.assertEqual(topic_fields(' timestamp: 10\n timestamp: 20\n'),{})
        self.assertEqual(topic_instances(telemetry(1)+telemetry(1)),{})
        self.assertFalse(self.snapshot(telemetry(1)+'    heartbeat_type_gcs: False\n')['snapshot_pass'])

    def test_aggregate_gcs_lost_or_stale_status_blocks(self):
        for status in ({'timestamp':'15000000','gcs_connection_lost':'True'},
                       {'timestamp':'1','gcs_connection_lost':'False'},{}):
            self.assertFalse(gcs_snapshot(telemetry(1),status,15_000_000)['snapshot_pass'])

    def test_progress_requires_native_clock_rx_and_same_link(self):
        b=self.snapshot(telemetry(2))
        self.assertFalse(gcs_progress(b,b))
        self.assertTrue(gcs_progress(b,self.snapshot(telemetry(2,stamp=19_800_000,rx=20),20_000_000)))
        self.assertFalse(gcs_progress(b,self.snapshot(telemetry(2,stamp=19_800_000,rx=10),20_000_000)))
        self.assertFalse(gcs_progress(b,self.snapshot(telemetry(3,stamp=19_800_000,rx=20),20_000_000)))


class RuntimePreparationJsonTests(unittest.TestCase):
    def test_write_round_trips_nested_numpy_values_and_nulls_nonfinite(self):
        payload = {
            'scalar_f32': np.float32(1.25),
            'scalar_f64': np.float64(2.5),
            'scalar_int': np.int64(9),
            'nested': {'vector': np.array([np.float32(3), np.float64(4)], dtype=np.float32),
                       'values': [np.int32(5), (np.float64(6), np.float32(np.nan))]},
            'positive_infinity': np.float64(np.inf),
            'negative_infinity': np.float32(-np.inf),
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'observer.json'
            rp.write(path, payload)
            loaded = json.loads(path.read_text())
        self.assertEqual(loaded['scalar_f32'], 1.25)
        self.assertEqual(loaded['scalar_f64'], 2.5)
        self.assertEqual(loaded['scalar_int'], 9)
        self.assertEqual(loaded['nested']['vector'], [3.0, 4.0])
        self.assertEqual(loaded['nested']['values'], [5, [6.0, None]])
        self.assertIsNone(loaded['positive_infinity'])
        self.assertIsNone(loaded['negative_infinity'])


class DDSEvidenceTests(unittest.TestCase):
    def test_graph_is_not_message_or_progression(self):
        c=MessageEvidence();d=c.summarize({'topic':{'publishers':1}},[])
        self.assertEqual(d['dds_evidence_level'],'GRAPH_ONLY')
        self.assertFalse(d['dds_messages_received'])

    def test_one_message_or_one_progressing_topic_is_not_all_topics(self):
        c=MessageEvidence();c.record('vehicle_local_position',10,1)
        c.record('vehicle_local_position',20,2)
        self.assertEqual(c.summarize({},[])['dds_evidence_level'],'MESSAGE_RECEIVED')

    def test_all_required_topics_progress(self):
        c=MessageEvidence()
        for k in REQUIRED:
            c.record(k,10,1);c.record(k,20,2)
        self.assertEqual(c.summarize({},[])['dds_evidence_level'],'TIMESTAMP_PROGRESSING')
        self.assertFalse(c.summarize({},[])['session_data_bound'])
        self.assertIsNone(c.messages['vehicle_status']['samples'][0]['source_sequence'])

    def test_frozen_backwards_and_errors_block_progress_level(self):
        for second in (10,9):
            c=MessageEvidence()
            for k in REQUIRED:c.record(k,10,1);c.record(k,second,2)
            self.assertEqual(c.summarize({},[])['dds_evidence_level'],'MESSAGE_RECEIVED')
        self.assertFalse(c.summarize({},['fault'])['source_clock_progressed'])

    def test_invalid_timestamp_rejected(self):
        with self.assertRaises(ValueError):MessageEvidence().record('x',0,1)

    def test_no_data_while_paused_is_not_dds_failure(self):
        self.assertEqual(MessageEvidence().summarize({},[])['dds_evidence_level'],'NO_DATA')


class FiniteWindowTests(unittest.TestCase):
    def setUp(self):
        self.b,self.anchor=fixture();self.a=copy.deepcopy(self.b)
        self.a['evidence']['stats']['stdout']=self.a['evidence']['stats']['stdout'].replace('sec: 15','sec: 20').replace('iterations: 15000','iterations: 20000')
        self.a['evidence']['telemetry_status']['stdout']=telemetry(0,stamp=19_800_000,rx=40000)
        self.a['gcs_evidence']=gcs_snapshot(self.a['evidence']['telemetry_status']['stdout'],
            dict(timestamp='20000000',gcs_connection_lost='False'),20_000_000)
        c=MessageEvidence()
        lp=topic_fields(self.b['evidence']['vehicle_local_position']['stdout'])
        self.positions=[number(lp,k) for k in ('x','y','z')]
        self.resets=[int(number(lp,k)) for k in RESET_FIELDS]
        for i in range(6):
            stamp=15_200_000+i*900_000
            for key in REQUIRED:
                c.record(key,stamp+1_000_000,i+1,timestamp_sample=stamp+1_000_000)
                row=c.messages[key]['samples'][-1]
                if key=='timesync_status':row.update(estimated_offset_us=-1_000_000,source_protocol=2)
                if key=='vehicle_local_position':row.update(position_ned=self.positions,resets=self.resets)
        c.messages['vehicle_status'].update(system_id=1,component_id=1)
        self.graph={key:dict(publishers=1,publisher_gids=[[1]*12+[i+1]*4]) for i,key in enumerate(REQUIRED)}
        self.obs=dict(messages=c.messages,graph=self.graph,errors=[],**c.summarize(self.graph,[]))
        self.gt=dict(errors=[],samples=[dict(native_timestamp_us=15_200_000+i*900_000,entity_id=64,
            position_enu=list(self.anchor.gazebo_enu)) for i in range(6)])
        self.sw=dict(errors=[],schemas={k:dict(serialized_fields_match=True,built_version_matches=True) for k in REQUIRED})

    def check(self):return assess_window(self.b,self.a,self.obs,self.gt,self.sw)

    def test_correct_native_time_offset_session_and_data_pass_only_window(self):
        d=self.check()
        self.assertEqual(d['result'],'PASS_FINITE_CLOCK_EVIDENCE_ONLY')
        self.assertTrue(d['session_data_bound']);self.assertFalse(d['flight_ready'])

    def test_frozen_clock_wrong_steps_or_other_boot_blocks(self):
        for change in ('clock','steps','boot'):
            old=copy.deepcopy(self.a)
            if change=='clock':self.a['evidence']['stats']=self.b['evidence']['stats']
            if change=='steps':self.a['evidence']['stats']['stdout']=self.a['evidence']['stats']['stdout'].replace('20000','19999')
            if change=='boot':self.a['runtime_identity']['boot_id']='OTHER'
            self.assertEqual(self.check()['result'],'NO_GO');self.a=old

    def test_missing_topics_or_foreign_sysid_schema_blocks(self):
        self.obs['messages']['vehicle_status']['system_id']=2
        self.assertIn('PX4_SYSID_COMPONENT',self.check()['problems'])
        self.sw['schemas']['vehicle_status']['built_version_matches']=False
        self.assertIn('INSTALLED_BUILT_SCHEMA',self.check()['problems'])

    def test_offset_jump_reset_or_position_mismatch_blocks(self):
        for change in ('offset','reset','position'):
            old=copy.deepcopy(self.obs)
            if change=='offset':self.obs['messages']['timesync_status']['samples'][0]['estimated_offset_us']=0
            if change=='reset':self.obs['messages']['vehicle_local_position']['samples'][0]['resets']=[9]*6
            if change=='position':self.obs['messages']['vehicle_local_position']['samples'][-1]['position_ned']=[99]*3
            self.assertIn('DDS_NATIVE_TIME_POSITION_OR_RESET_CORRELATION',self.check()['problems'])
            self.obs=old

    def test_mixed_publishers_and_missing_groundtruth_block(self):
        self.obs['graph']['vehicle_status']['publisher_gids']=[[2]*16]
        self.assertIn('DDS_PARTICIPANT',self.check()['problems'])
        self.gt['samples']=[]
        self.assertIn('INDEPENDENT_GROUNDTRUTH_WINDOW',self.check()['problems'])


class ObserverMockTests(unittest.TestCase):
    def test_numpy_scalars_preserve_values_and_json_types(self):
        from pso_flight_demo.dds_observer import _python_scalars
        for value, expected_type in ((np.float32(.1), float), (np.float64(.1), float),
                                     (np.int32(7), int), (np.int64(2**60+1), int),
                                     (np.bool_(True), bool)):
            with self.subTest(dtype=type(value).__name__):
                payload = {'samples': [{'value': value}], 'pair': (value, None)}
                converted = _python_scalars(payload)
                scalar = converted['samples'][0]['value']
                self.assertIs(type(scalar), expected_type)
                self.assertEqual(scalar, value.item())
                self.assertEqual(json.loads(json.dumps(converted, allow_nan=False)),
                                 {'samples': [{'value': value.item()}], 'pair': [value.item(), None]})
                self.assertIs(payload['samples'][0]['value'], value)
                self.assertIsInstance(converted['pair'], tuple)

    def test_scalar_conversion_keeps_nonfinite_telemetry_and_rejects_unknown_objects(self):
        from pso_flight_demo.dds_observer import _python_scalars
        for dtype in (np.float32, np.float64):
            for number in (np.nan, np.inf, -np.inf):
                with self.subTest(dtype=dtype.__name__, value=number):
                    converted = _python_scalars({'value': dtype(number)})
                    self.assertIs(type(converted['value']), float)
                    np.testing.assert_equal(converted['value'], number)
                    # Conversion must not silently replace invalid telemetry with null.
                    with self.assertRaises(ValueError):
                        json.dumps(converted, allow_nan=False)
        with self.assertRaises(TypeError):
            json.dumps(_python_scalars({'value': object()}))

    def test_real_observer_uses_only_subscriptions_and_versioned_topics(self):
        from pso_flight_demo.dds_observer import observe
        clock=[0.]
        nodes=[]
        def message_type(name,version=0):
            return type(name,(),dict(MESSAGE_VERSION=version))
        names=('VehicleLocalPosition','VehicleStatus','VehicleCommandAck','OffboardControlMode',
               'TrajectorySetpoint','TimesyncStatus','VehicleAttitude','VehicleLandDetected',
               'EstimatorStatusFlags','VehicleCommand')
        types={n:message_type(n,1 if n in ('VehicleLocalPosition','VehicleCommandAck') else 4 if n=='VehicleStatus' else 0) for n in names}
        class Node:
            def __init__(self,*args,**kwargs):self.subs=[];nodes.append(self)
            def create_subscription(self,cls,topic,callback,qos):self.subs.append((cls,topic,callback));return object()
            def count_publishers(self,topic):return 1 if '/out/' in topic else 0
            def count_subscribers(self,topic):return 1
            def get_publishers_info_by_topic(self,topic):return [NS(endpoint_gid=[1]*16)]
            def destroy_node(self):pass
            def create_publisher(self,*args):raise AssertionError('NO PUBLISHERS')
        def spin(node,timeout_sec):
            clock[0]+=.2
            for cls,topic,callback in node.subs:
                m=NS(timestamp=int(1_000_000+clock[0]*1e6),timestamp_sample=1_000_000,
                     x=0.,y=0.,z=0.,xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
                     xy_reset_counter=0,z_reset_counter=0,vxy_reset_counter=0,vz_reset_counter=0,
                     heading_reset_counter=0,ref_timestamp=1,system_id=1,component_id=1,
                     gcs_connection_lost=False,pre_flight_checks_pass=True,failsafe=False,
                     arming_state=1,ARMING_STATE_ARMED=2,landed=True,cs_tilt_align=True,cs_yaw_align=True,
                     cs_mag_fault=False,cs_gnss_fault=False,get_fields_and_field_types=lambda:{},
                     source_protocol=2,SOURCE_PROTOCOL_DDS=2,estimated_offset=0,round_trip_time=100,
                     heading=np.float64(.25),heading_var=np.float32(.125),
                     q=np.asarray([1.,0.,0.,0.],dtype=np.float32))
                callback(m)
        modules={'rclpy':NS(init=lambda **kwargs:None,shutdown=lambda:None,spin_once=spin),
                 'rclpy.node':NS(Node=Node),
                 'rclpy.qos':NS(QoSProfile=lambda **kwargs:kwargs,ReliabilityPolicy=NS(BEST_EFFORT=0),DurabilityPolicy=NS(VOLATILE=0)),
                 'px4_msgs':NS(),'px4_msgs.msg':NS(**types)}
        with patch.dict(sys.modules,modules),patch('pso_flight_demo.dds_observer.time.monotonic',side_effect=lambda:clock[0]), \
             patch('subprocess.run',side_effect=AssertionError('NO HOST/PHYSICS COMMAND')):
            d=observe(load_profile(),1,'/tmp/test_ros_logs_unused')
            clock[0]=0.;calls=[]
            monitor=NS(pre_subscriptions_check=lambda controls:calls.append(('controls',len(controls))),
                subscriptions_created=lambda keys:calls.append(('created',keys)),
                begin=lambda deadline:calls.append(('begin',deadline)),
                update=lambda graph,now,errors:(calls.append(('spin',now)) or True),
                finish=lambda result:calls.append(('finish',result['timing']['spin_count'])))
            monitored=observe(load_profile(),1,'/tmp/test_ros_logs_unused',monitor=monitor)
        self.assertEqual(d['dds_evidence_level'],'TIMESTAMP_PROGRESSING')
        self.assertFalse(d['publishes_commands']);self.assertFalse(d['session_data_bound'])
        topics=[t for _,t,_ in nodes[0].subs]
        self.assertIn('/fmu/out/vehicle_local_position_v1',topics)
        self.assertIn('/fmu/out/vehicle_status_v4',topics)
        self.assertEqual([x[0] for x in calls[:3]],['controls','created','begin'])
        self.assertTrue(set(REQUIRED).issubset(calls[1][1]))
        self.assertEqual(calls[-1][0],'finish');self.assertGreater(calls[-1][1],0)
        self.assertGreater(monitored['timing']['spin_count'],0)
        for observed in (d, monitored):
            # Exact readiness boundary: plain json.dumps(observe(...)).
            decoded = json.loads(json.dumps(observed, allow_nan=False))
            self.assertEqual(decoded, observed)
            attitude = observed['messages']['vehicle_attitude']['samples'][0]
            self.assertEqual(attitude['quaternion'], [1., 0., 0., 0.])
            self.assertTrue(all(type(x) is float for x in attitude['quaternion']))
            local = observed['messages']['vehicle_local_position']['samples'][0]
            self.assertIs(type(local['heading']), float)
            self.assertIs(type(local['heading_var']), float)


class AnchorTests(unittest.TestCase):
    def setUp(self):
        self.r,self.a=fixture();self.p=rp.anchor_packet(self.r,self.a)

    def test_nonzero_origins_round_trip_and_heading_convention(self):
        self.assertNotEqual(self.a.gazebo_enu,self.a.px4_ned)
        v=np.array([-11.1,4.2,2.84])
        np.testing.assert_allclose(self.a.to_enu(self.a.to_ned(v)),v,atol=1e-12)
        self.assertTrue(all(self.p['frame_gates'].values()))
        self.assertTrue(rp.check_anchor_packet(self.p,self.r,self.a)['anchor_valid'])
        self.assertFalse(self.p['flight_ready'])

    def test_boot_pid_ticks_profile_sdf_session_changes_block(self):
        for key in rp.IDENTITY_FIELDS:
            r=copy.deepcopy(self.r);r['runtime_identity'][key]='CHANGED'
            with self.subTest(key=key):self.assertFalse(rp.check_anchor_packet(self.p,r,self.a)['anchor_valid'])
        for key in ('profile_sha256','session_id'):
            r=copy.deepcopy(self.r);r[key]='changed'
            self.assertFalse(rp.check_anchor_packet(self.p,r,self.a)['anchor_valid'])
        r=copy.deepcopy(self.r);r['world']['sdf_sha256']='changed'
        self.assertFalse(rp.check_anchor_packet(self.p,r,self.a)['anchor_valid'])

    def test_ekf_reset_and_translation_drift_block(self):
        a=replace(self.a,reset_counters=(6,*self.a.reset_counters[1:]))
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,a)['anchor_valid'])
        a=replace(self.a,px4_ned=(1.,*self.a.px4_ned[1:]))
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,a)['anchor_valid'])

    def test_stale_anchor_missing_current_and_native_gcs_failure_block(self):
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,self.a,now=time.time()+31)['anchor_valid'])
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,None)['anchor_valid'])
        self.r['problems']=['gcs_heartbeat_snapshot']
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,self.a)['anchor_valid'])

    def test_historical_heading_not_good_blocks(self):
        self.r['evidence']['vehicle_local_position']['stdout']=self.r['evidence']['vehicle_local_position']['stdout'].replace(
            'heading_good_for_control: True','heading_good_for_control: False')
        self.assertFalse(rp.check_anchor_packet(self.p,self.r,self.a)['anchor_valid'])

    def test_unsynchronized_native_pair_rejected(self):
        with self.assertRaises(ValueError):replace(self.a,px4_timestamp_us=1)


class CandidateAndHostTests(unittest.TestCase):
    def test_candidate_is_exact_saved_prefix_without_optimizer(self):
        with patch('pso_flight_demo.planner.run',side_effect=AssertionError('no PSO rerun')):
            d=rp.candidate_assessment()
        self.assertAlmostEqual(d['length_m'],.7754207580358945)
        self.assertTrue(d['nominal_segment_collision_free'])
        self.assertTrue(d['tracking_tube_025m_clear'])
        self.assertFalse(d['acceleration_bounded_by_current_follower'])
        self.assertEqual(d['result'],'NO_GO')

    def test_full_transitions_include_takeoff_holds_and_land_without_waiving_floor(self):
        _,a=fixture();d=rp.candidate_assessment(a,True)
        self.assertTrue(d['transitions']['obstacle_box_segments_clear'])
        self.assertFalse(d['transitions']['floor_envelope_all_segments_clear'])
        self.assertTrue(d['transitions']['relative_height_go'])
        self.assertLess(d['transitions']['round_trip_error_m'],1e-12)
        self.assertFalse(d['live_control_go'])

    def test_no_ned_conversion_before_anchor_check(self):
        _,a=fixture();d=rp.candidate_assessment(a,False)
        self.assertNotIn('ned',d['transitions'])

    def test_missing_owner_prevents_native_probes(self):
        with patch.object(rp,'read_session',side_effect=ValueError('Supervisor absent')), \
             patch.object(rp,'inspect_runtime',side_effect=AssertionError('no IPC')):
            with self.assertRaisesRegex(ValueError,'Supervisor absent'):rp.host_snapshot('missing',15000)

    def test_cli_fails_without_live_probes_or_old_artifact_overwrite(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'new'
            with patch.object(rp,'host_snapshot',side_effect=ValueError('host inaccessible')), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(rp.main(['--output',str(dest),'snapshot']),2)
            self.assertEqual(json.loads((dest/'error.json').read_text())['result'],'NO_GO')
            with self.assertRaises(FileExistsError):rp.main(['--output',str(dest),'candidate'])

    def test_clock_plan_never_executes_and_requires_auto_arm_proof(self):
        r,_=fixture()
        with patch('subprocess.run',side_effect=AssertionError('print only')):
            self.assertEqual(rp.clock_plan(r)['result'],'NO_GO')
            r['evidence']['params_COM_ARM_ON_BOOT']={'stdout':'x COM_ARM_ON_BOOT [1,2] : 0\n'}
            d=rp.clock_plan(r)
        self.assertEqual(d['result'],'CLOCK_WINDOW_PLAN_ONLY')
        self.assertTrue(d['requires_separate_approval']);self.assertFalse(d['executed'])


class GroundTruthTests(unittest.TestCase):
    def message(self, name='x500', entity=64):
        return NS(header=NS(stamp=NS(sec=15,nsec=0)),pose=[NS(name=name,id=entity,
            position=NS(x=1.,y=2.,z=3.),orientation=NS(w=1.,x=0.,y=0.,z=0.))])

    def test_name_entity_and_native_clock(self):
        p=pose_sample(self.message(),'x500',64)
        self.assertEqual(p['native_timestamp_us'],15_000_000)
        for msg in (self.message('other'),self.message(entity=65)):
            with self.assertRaises(ValueError):pose_sample(msg,'x500',64)

    def test_ambiguous_or_invalid_pose_fails(self):
        m=self.message();m.pose*=2
        with self.assertRaises(ValueError):pose_sample(m,'x500')
        m=self.message();m.pose[0].position.x=float('nan')
        with self.assertRaises(ValueError):pose_sample(m,'x500')


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        _,self.a=fixture();_,self.env=load_environment(load_profile())
        self.gt=(-11.5,4.3,2.5)
        self.s=WatchSample(self.gt,tuple(self.a.to_ned(self.gt)),self.gt,
            15_000_000,14_996_000,10.,10.,10.,self.a.reset_counters)
        self.w=GroundTruthWatchdog(self.a,self.env)

    def test_healthy_observes_without_commands(self):
        d=self.w.evaluate(self.s,10.1,True)
        self.assertEqual(d['action'],'CONTINUE_OBSERVING');self.assertFalse(d['executed'])

    def test_stream_loss_requests_land_only_if_descent_and_state_usable(self):
        s=replace(self.s,setpoint_wall=9.)
        self.assertEqual(self.w.evaluate(s,10.1,True)['action'],'REQUEST_LAND')
        self.assertEqual(self.w.evaluate(s,10.1,False)['action'],'REQUEST_EMERGENCY_PAUSE')

    def test_stale_telemetry_gt_dds_reset_estimator_never_claim_land(self):
        for kw in (dict(received_gt_wall=9.),dict(received_dds_wall=9.),dict(dds_connected=False),
                   dict(estimator_valid=False),dict(gcs_lost=True),dict(resets=(9,*self.a.reset_counters[1:]))):
            with self.subTest(kw=kw):
                self.assertEqual(GroundTruthWatchdog(self.a,self.env).evaluate(replace(self.s,**kw),10.1,True)['action'],'REQUEST_EMERGENCY_PAUSE')

    def test_divergence_and_time_pair_fail_closed(self):
        for kw in (dict(ekf_ned=(9.,9.,9.)),dict(native_ekf_us=1),dict(groundtruth_enu=(float('nan'),4.,2.))):
            self.assertEqual(GroundTruthWatchdog(self.a,self.env).evaluate(replace(self.s,**kw),10.1,True)['action'],'REQUEST_EMERGENCY_PAUSE')

    def test_tracking_fault_requests_land_not_success(self):
        d=self.w.evaluate(replace(self.s,target_enu=(-10.,4.3,2.5)),10.1,True)
        self.assertEqual(d['action'],'REQUEST_LAND');self.assertFalse(d['land_confirmed'])

    def test_swept_gt_segment_collision_is_checked(self):
        a=replace(self.s,groundtruth_enu=(-5.,2.,2.5),ekf_ned=tuple(self.a.to_ned((-5.,2.,2.5))),target_enu=(-5.,2.,2.5))
        self.w.evaluate(a,10.1,True)
        b=replace(a,groundtruth_enu=(-5.,-2.,2.5),ekf_ned=tuple(self.a.to_ned((-5.,-2.,2.5))),
                  target_enu=(-5.,-2.,2.5),native_gt_us=16_000_000,native_ekf_us=16_000_000)
        self.assertIn('GROUNDTRUTH_GEOMETRY_OR_NUMERIC',self.w.evaluate(b,10.1,True)['reasons'])

    def test_frozen_source_and_groundtruth_speed(self):
        self.w.evaluate(self.s,10.1,True)
        self.assertIn('GROUNDTRUTH_TIME_FROZEN_OR_BACKWARDS',self.w.evaluate(self.s,10.1,True)['reasons'])
        moved=replace(self.s,groundtruth_enu=(-11.3,4.3,2.5),ekf_ned=tuple(self.a.to_ned((-11.3,4.3,2.5))),
                      target_enu=(-11.3,4.3,2.5),native_gt_us=15_100_000,native_ekf_us=15_100_000)
        self.assertIn('GROUNDTRUTH_SPEED',self.w.evaluate(moved,10.1,True)['reasons'])

    def test_accepted_px4_landing_allows_bounded_vertical_descent_only(self):
        self.w.evaluate(self.s,10.1,True)
        z=.04
        landing=replace(self.s,groundtruth_enu=(self.gt[0],self.gt[1],self.gt[2]-z),
            ekf_ned=tuple(self.a.to_ned((self.gt[0],self.gt[1],self.gt[2]-z))),
            native_gt_us=15_050_000,native_ekf_us=15_046_000,
            received_gt_wall=10.15,received_dds_wall=10.15)
        result=self.w.evaluate(landing,10.15,True,landing_mode=True,
            landing_vertical_speed_limit=.9)
        self.assertNotIn('GROUNDTRUTH_SPEED',result['reasons'])

        too_fast=replace(self.s,groundtruth_enu=(self.gt[0],self.gt[1],self.gt[2]-.05),
            ekf_ned=tuple(self.a.to_ned((self.gt[0],self.gt[1],self.gt[2]-.05))),
            native_gt_us=15_050_000,native_ekf_us=15_046_000,
            received_gt_wall=10.15,received_dds_wall=10.15)
        fast_watch=GroundTruthWatchdog(self.a,self.env)
        fast_watch.evaluate(self.s,10.1,True)
        self.assertIn('GROUNDTRUTH_SPEED',fast_watch.evaluate(too_fast,10.15,True,
            landing_mode=True,landing_vertical_speed_limit=.9)['reasons'])

    def test_accepted_landing_still_enforces_horizontal_speed_limit(self):
        self.w.evaluate(self.s,10.1,True)
        xy=(self.gt[0]+.04,self.gt[1],self.gt[2])
        moved=replace(self.s,groundtruth_enu=xy,ekf_ned=tuple(self.a.to_ned(xy)),
            native_gt_us=15_050_000,native_ekf_us=15_046_000,
            received_gt_wall=10.15,received_dds_wall=10.15)
        self.assertIn('GROUNDTRUTH_SPEED',self.w.evaluate(moved,10.15,True,landing_mode=True)['reasons'])


class LandAckTests(unittest.TestCase):
    def setUp(self):self.land=LandConfirmation();self.land.request(10.)

    def test_request_is_not_ack_or_landed(self):
        self.assertEqual(self.land.evaluate(11.,None,18,True,False),'WAIT_LAND_CONFIRMATION')
        self.assertEqual(self.land.evaluate(11.,0,18,False,True),'WAIT_LAND_CONFIRMATION')
        self.assertEqual(self.land.evaluate(12.,None,4,True,False),'LAND_CONFIRMED')

    def test_denied_ack_and_mode_timeout(self):
        self.assertEqual(self.land.evaluate(10.1,2,18,False,True),'REQUEST_EMERGENCY_PAUSE')
        self.assertEqual(self.land.evaluate(13.1,None,18,False,True),'REQUEST_EMERGENCY_PAUSE')

    def test_landing_timeout_and_no_retry(self):
        self.land.evaluate(11.,0,18,False,True)
        self.assertEqual(self.land.evaluate(40.1,0,18,False,True),'REQUEST_EMERGENCY_PAUSE')
        with self.assertRaises(ValueError):self.land.request(12.)


if __name__ == '__main__':
    unittest.main()
