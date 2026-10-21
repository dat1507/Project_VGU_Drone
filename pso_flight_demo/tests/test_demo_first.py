"""Offline-only tests; fake Transport/native clients never advance physics."""

import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from pso_flight_demo import demo_first as demo
from pso_flight_demo.environments import load_environment, load_profile


class RoutePreparationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = patch.object(demo,'OUTPUT',Path(self.tmp.name)/'output')
        self.output.start(); self.addCleanup(self.output.stop)

    def test_exact_raw_copies_and_ned_displacements_not_absolute_setpoints(self):
        report = demo.prepare()
        out = Path(report['output'])
        for name, audit in report['routes'].items():
            self.assertEqual((out/(name+'_raw_enu.csv')).read_bytes(),Path(audit['source_csv']).read_bytes())
            rows = list(csv.reader((out/(name+'_relative_ned_NOT_SETPOINTS.csv')).read_text().splitlines()))
            self.assertEqual(rows[0],['north_from_A_m','east_from_A_m','down_from_A_m'])
            self.assertEqual([float(v) for v in rows[1]],[0.,0.,0.])
            delta = np.array(audit['goal_enu_m'])-audit['start_enu_m']
            np.testing.assert_allclose([float(v) for v in rows[-1]],[delta[1],delta[0],-delta[2]])
            self.assertFalse(audit['absolute_ned_exported'])
            self.assertFalse(audit['flight_go'])

    def test_real_full_routes_fail_current_120s_gate(self):
        report = demo.prepare()
        self.assertAlmostEqual(report['routes']['standard']['minimum_route_time_sim_s'],189.139904894)
        self.assertAlmostEqual(report['routes']['enhanced']['minimum_route_time_sim_s'],149.067212400)
        self.assertTrue(all(not r['current_timeout_go'] for r in report['routes'].values()))
        self.assertEqual(report['proposed_common_route_timeout_sim_s'],220)
        self.assertFalse(report['proposal_applied'])
        self.assertEqual(report['same_controller_config']['route_timeout_sim_s'],120)

    def test_raw_geometry_pass_does_not_claim_tracking_clearance(self):
        report = demo.prepare()
        for audit in report['routes'].values():
            self.assertEqual(audit['nominal_raw_geometry'],'PASS')
            self.assertFalse(audit['tracking_tube_go'])
            self.assertTrue(any(x['bounds_violation'] for x in audit['tracking_tube_failures']))
            self.assertEqual(audit['height_enu_range_m'][1],3.2)
            self.assertEqual(audit['floor_contact_and_takeoff_dynamics'],'NOT_PROVEN')

    def test_different_endpoints_rejected_before_output(self):
        routes = demo.load_routes()
        route = routes['enhanced']
        routes['enhanced'] = NS(points=route.points[:-1]+((0.,0.,2.5),))
        with patch.object(demo,'load_routes',return_value=routes):
            with self.assertRaisesRegex(ValueError,'identical Start/Goal'):
                demo.prepare()
        self.assertFalse(demo.OUTPUT.exists())

    def test_geometry_rejects_corrupted_path(self):
        world, env = load_environment(load_profile())
        route = NS(points=((0.,0.,2.5),(float('nan'),0.,2.5)))
        with self.assertRaisesRegex(ValueError,'Non-finite'):
            demo.route_audit(route,world,env)

    def test_new_run_never_overwrites_previous_output(self):
        first = demo.prepare(); second = demo.prepare()
        self.assertNotEqual(first['output'],second['output'])
        self.assertEqual(json.loads((Path(first['output'])/'demo_plan.json').read_text()),first)


class HostReadOnlyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.patches = []
        def mocked(target, **kwargs):
            p=patch(target,**kwargs); self.patches.append(p)
            return p.start()
        self.addCleanup(lambda:[p.stop() for p in reversed(self.patches)])
        mocked('pso_flight_demo.demo_first.OUTPUT',new=Path(self.tmp.name)/'outputs')
        mocked('pso_flight_demo.demo_first.host_visibility',return_value='host_terminal')
        from pso_flight_demo.session_launcher import digest
        from pso_flight_demo.runtime_preparation import IDENTITY_FIELDS
        profile=load_profile(); world,_=load_environment(profile)
        self.state=dict(session_id='fake',profile_sha256=digest(profile.snapshot()),
                        sdf_sha256=world.sdf_sha256,with_dds=True,
                        supervisor={'pid':1},children=[dict(owner={'pid':2})])
        mocked('pso_flight_demo.session_launcher.read_session',return_value=(Path('/fake'),self.state))
        mocked('pso_flight_demo.session_launcher.identity',return_value={})
        mocked('pso_flight_demo.session_launcher.same_process',return_value=True)
        mocked('pso_flight_demo.session_launcher.session_inventory',return_value=dict(
            alive=[1,2],absent=[],identity_mismatches=[],unrecorded_child_tasks=[]))
        self.runtime={k:i+1 for i,k in enumerate(IDENTITY_FIELDS)}
        self.runtime.update(px4_count=1,gazebo_server_count=1)
        self.ident=mocked('pso_flight_demo.runtime_readiness.process_identity',return_value=self.runtime)
        mocked('pso_flight_demo.runtime_preparation.owned_runtime',return_value=True)
        mocked('pso_flight_demo.runtime_readiness.probe',return_value=dict(exit=0,stdout='fake process table',stderr=''))
        self.gz=NS(baseline=(0,20,20000),services={},
                   scene=lambda:{'x500':NS(id=42),'pso_demo_visual_path':NS(id=105)},guard=lambda:None)
        self.factory=mocked('pso_flight_demo.visualization.raw_pso_gazebo.Gazebo',return_value=self.gz)
        graph={t:dict(control_publishers=0) for t in (
            '/fmu/in/offboard_control_mode','/fmu/in/trajectory_setpoint','/fmu/in/vehicle_command')}
        self.native=dict(gates=dict(disarmed=True,landed=True),
                         fields={'vehicle_local_position':{'heading_good_for_control':'False'}},
                         dds=dict(graph=graph,dds_evidence_level='GRAPH_ONLY'),runtime_identity=self.runtime)
        self.ready=mocked('pso_flight_demo.session_runtime.readiness',return_value=self.native)

    def test_readonly_candidate_keeps_live_false_and_does_not_adopt_id105(self):
        report=demo.host_check(Path('/fake'))
        self.assertEqual(report['result'],'READONLY_ENVIRONMENT_CANDIDATE')
        self.factory.assert_called_once_with(read_only=True)
        self.assertFalse(report['live_control_go'])
        self.assertFalse(report['heading_good_for_control'])
        self.assertFalse(report['scene']['ownership_verified'])
        self.assertEqual(report['scene']['demo_models'],{'pso_demo_visual_path':105})
        self.assertEqual(report['dds_evidence_level'],'GRAPH_ONLY')

    def test_sandbox_stops_before_transport_or_native(self):
        with patch.object(demo,'host_visibility',return_value='isolated_sandbox'):
            report=demo.host_check(Path('/fake'))
        self.assertEqual(report['result'],'BLOCKED')
        self.factory.assert_not_called(); self.ready.assert_not_called()

    def test_scene_timeout_stops_without_native_or_retry(self):
        self.factory.side_effect=ValueError('STATS_UNAVAILABLE')
        report=demo.host_check(Path('/fake'))
        self.assertEqual(report['result'],'BLOCKED')
        self.factory.assert_called_once(); self.ready.assert_not_called()

    def test_profile_mismatch_stops_before_transport(self):
        self.state['profile_sha256']='changed'
        self.assertEqual(demo.host_check(Path('/fake'))['result'],'BLOCKED')
        self.factory.assert_not_called()

    def test_duplicate_px4_stops_before_transport(self):
        self.runtime['px4_count']=2
        self.assertEqual(demo.host_check(Path('/fake'))['result'],'BLOCKED')
        self.factory.assert_not_called()

    def test_unexpected_control_publisher_blocks(self):
        self.native['dds']['graph']['/fmu/in/vehicle_command']['control_publishers']=1
        report=demo.host_check(Path('/fake'))
        self.assertIn('CONTROL_PUBLISHERS_ZERO_NOT_PROVEN',report['blockers'])

    def test_missing_control_graph_is_unknown_not_zero(self):
        self.native['dds']['graph'].pop('/fmu/in/vehicle_command')
        report=demo.host_check(Path('/fake'))
        self.assertIn('CONTROL_PUBLISHERS_ZERO_NOT_PROVEN',report['blockers'])

    def test_armed_or_not_landed_stays_blocked(self):
        self.native['gates']['disarmed']=False
        report=demo.host_check(Path('/fake'))
        self.assertIn('disarmed',report['blockers'])
        self.assertFalse(report['live_control_go'])

    def test_restart_during_check_rejected(self):
        changed=dict(self.runtime,px4_pid=999)
        self.ident.side_effect=[self.runtime,changed]
        report=demo.host_check(Path('/fake'))
        self.assertEqual(report['result'],'BLOCKED')
        self.assertTrue(any('SESSION_CHANGED' in x for x in report['blockers']))


class TransportDiagnosticTests(unittest.TestCase):
    # Reuse fake session/native fixtures; no real Node or process probes.
    def setUp(self):
        HostReadOnlyTests.setUp(self)
        replacement=patch('pso_flight_demo.session_launcher.identity',return_value=dict(pid=5))
        replacement.start(); self.addCleanup(replacement.stop)

    def test_diagnostic_can_read_scene_even_when_no_stats_but_never_grants_go(self):
        def fake_probe(argv,timeout):
            text = 'model { name: "x500" id: 42 }' if '-s' in argv else ''
            return dict(argv=argv,exit=0,stdout=text,stderr='')
        with patch('pso_flight_demo.runtime_readiness.probe',side_effect=fake_probe) as probe, \
                patch.object(Path,'read_bytes',return_value=b'GZ_IP=192.0.2.10\0SECRET_TOKEN=must_not_print\0'):
            report=demo.diagnose_transport(Path('/fake'))
        self.assertEqual(report['result'],'READONLY_DIAGNOSTICS_ONLY')
        self.assertEqual(report['stats_samples'],'')
        self.assertEqual(report['scene_response']['x500_ids'],[42])
        self.assertFalse(report['live_control_go'])
        self.assertFalse(report['visualization_go'])
        self.assertNotIn('SECRET_TOKEN',json.dumps(report))
        self.assertNotIn('must_not_print',json.dumps(report))
        self.factory.assert_not_called(); self.ready.assert_not_called()
        calls=[call.args[0] for call in probe.call_args_list]
        self.assertEqual(sum('-s' in cmd for cmd in calls),1)
        self.assertTrue(all(not any('/control' in arg or '/remove' in arg or '/create' in arg or '/set_pose' in arg for arg in cmd) for cmd in calls))
        samples=next(cmd for cmd in calls if '-e' in cmd)
        self.assertEqual(samples[-2:],['-d','3'])

    def test_diagnostic_timeout_no_retry_and_readiness_stays_false(self):
        def fake_probe(argv,timeout):
            return (dict(exit=0,stdout='fake process table',stderr='') if argv[0] == 'ps'
                    else dict(exit=None,stdout='',stderr='timeout'))
        with patch('pso_flight_demo.runtime_readiness.probe',side_effect=fake_probe) as probe:
            report=demo.diagnose_transport(Path('/fake'))
        self.assertEqual(report['result'],'READONLY_DIAGNOSTICS_ONLY')
        self.assertEqual(sum('-s' in c.args[0] for c in probe.call_args_list),1)
        self.assertEqual(sum('-e' in c.args[0] for c in probe.call_args_list),1)
        self.assertIsNone(report['probes']['stats_samples']['exit'])
        self.assertFalse(report['live_control_go'])

    def test_diagnostic_invalid_scene_is_not_success_even_with_exit_zero(self):
        with patch('pso_flight_demo.runtime_readiness.probe',return_value=dict(
                exit=0,stdout='Service call failed',stderr='')):
            report=demo.diagnose_transport(Path('/fake'))
        self.assertTrue(any('SCENE_INVALID_RESPONSE' in reason for reason in report['blockers']))
        self.assertFalse(report['live_control_go'])

    def test_diagnostic_server_restart_rejected(self):
        with patch('pso_flight_demo.session_launcher.same_process',return_value=False):
            report=demo.diagnose_transport(Path('/fake'))
        self.assertEqual(report['result'],'BLOCKED')
        self.assertIn('SESSION_CHANGED_DURING_DIAGNOSTIC',report['blockers'])


if __name__ == '__main__':
    unittest.main()
