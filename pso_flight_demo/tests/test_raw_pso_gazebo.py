"""Offline only: CSV, exact raw interpolation, visual geometry and target guards."""

import contextlib
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from pso_flight_demo.visualization import raw_pso_gazebo as demo
from pso_flight_demo.tests.visualization_fakes import model as scene_model, proof_for, make_run, services

TOKEN = 'a'*32


def model(name=demo.DRONE, entity_id=44, token=TOKEN):
    return scene_model(name,entity_id,token)


def fake_gazebo():
    gz = object.__new__(demo.Gazebo)
    gz.scene = Mock(return_value={demo.DRONE:model(), 'x500':model('x500',99)})
    gz.request = Mock()
    gz.stats = NS(paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0))
    gz.stats_wall = time.monotonic()
    gz.clock_changed = False
    gz.baseline = (20000,20,0)
    gz.ownership_specs = {name:proof_for(name,TOKEN) for name in demo.OWN_NAMES}
    gz.ownership_evidence = {}
    gz.pose_cache,gz.pose_requests = {},{}
    def pose(**kw):
        return NS(**kw,position=NS(x=0,y=0,z=0),orientation=NS(z=0,w=1))
    gz.types = {'Pose':pose}
    return gz


class CsvAndProvenanceTests(unittest.TestCase):
    def test_real_sources_and_continuous_segments(self):
        routes = demo.load_routes()
        self.assertEqual(set(routes),{'standard','enhanced'})
        for route, length in zip(routes.values(),(21.58798097888897,13.57344247996963)):
            self.assertEqual(len(route.points),17)
            self.assertAlmostEqual(route.length,length,places=9)
            self.assertTrue(route.provenance['geometry_rechecked'])
            self.assertEqual(route.provenance['raw_certificate']['segments_checked'],16)
            self.assertFalse(route.provenance['live_control_go'])
        self.assertEqual(routes['standard'].points[0],routes['enhanced'].points[0])
        self.assertEqual(routes['standard'].points[-1],(-5.,-3.,2.5))

    def test_valid_schema(self):
        p = demo.parse_csv('x_enu_m,y_enu_m,z_enu_m\n1,2,3\n4,5,6\n')
        self.assertEqual(p,((1.,2.,3.),(4.,5.,6.)))

    def test_bad_headers_not_ned_or_lut(self):
        for header in ['x,y,z','x_ned_m,y_ned_m,z_ned_m','x_enu_m,z_enu_m,y_enu_m']:
            with self.subTest(header=header), self.assertRaises(ValueError):
                demo.parse_csv(header+'\n1,2,3\n4,5,6\n')

    def test_nan_inf_missing_extra_and_empty(self):
        for row in ['nan,1,2','1,inf,2','1,2,-inf','1,2','1,2,3,4','a,b,c','']:
            with self.subTest(row=row), self.assertRaises(ValueError):
                demo.parse_csv('x_enu_m,y_enu_m,z_enu_m\n'+row+'\n1,2,3\n')

    def test_one_waypoint_rejected(self):
        with self.assertRaises(ValueError): demo.parse_csv('x_enu_m,y_enu_m,z_enu_m\n1,2,3\n')

    def test_corrupted_csv_hash_rejected(self):
        original = Path.read_bytes
        def read(path):
            data = original(path)
            return data+b'\n' if path == demo.SOURCE/'selected_baseline/raw_enu.csv' else data
        with patch.object(Path,'read_bytes',read),self.assertRaisesRegex(ValueError,'hash mismatch'):
            demo.load_routes()

    def test_changed_start_goal_rejected_even_with_new_index(self):
        original = Path.read_bytes
        config = json.loads((demo.SOURCE/'config.json').read_bytes())
        config['start_enu'][0] += 1
        new = json.dumps(config).encode()
        index = json.loads((demo.SOURCE/'artifact_sha256.json').read_text())
        index['config.json'] = demo.sha(new)
        read_text = Path.read_text
        def rb(path): return new if path == demo.SOURCE/'config.json' else original(path)
        def rt(path,*a,**kw):
            return json.dumps(index) if path == demo.SOURCE/'artifact_sha256.json' else read_text(path,*a,**kw)
        with patch.object(Path,'read_bytes',rb),patch.object(Path,'read_text',rt):
            with self.assertRaisesRegex(ValueError,'Start / Goal'): demo.load_routes()


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        self.route = demo.Route('standard',((0.,0.,1.),(1.,0.,1.),(1.,1.,1.)),{})

    def test_speed_and_linear_midpoint(self):
        frames = demo.timeline(self.route,speed=1,fps=2,hold=0)
        self.assertEqual(frames[-1]['t_wall_s'],2.)
        frame = next(f for f in frames if f['t_wall_s'] == .5)
        self.assertEqual(tuple(frame[k] for k in demo.HEADER),(.5,0.,1.))
        for f in frames:
            self.assertTrue(f['y_enu_m'] == 0 or f['x_enu_m'] == 1)

    def test_every_corner_included_even_between_frame_times(self):
        frames = demo.timeline(self.route,speed=.7,fps=3,hold=.1)
        for point in self.route.points:
            self.assertTrue(any(math.dist(point,tuple(f[k] for k in demo.HEADER)) < 1e-12 for f in frames))

    def test_no_smoothing_raw_height_preserved(self):
        route = demo.load_routes()['enhanced']
        frames = demo.timeline(route)
        self.assertEqual(tuple(frames[0][k] for k in demo.HEADER),route.points[0])
        self.assertEqual(tuple(frames[-1][k] for k in demo.HEADER),route.points[-1])
        self.assertLessEqual(max(f['z_enu_m'] for f in frames),3.2)
        for point in route.points:
            self.assertTrue(any(math.dist(point,tuple(f[k] for k in demo.HEADER)) < 1e-10 for f in frames))

    def test_hold_and_repeat_reset_are_explicit(self):
        frames = demo.timeline(self.route,hold=1,repeat=2)
        self.assertEqual(frames[-1]['t_wall_s'],8.)
        reset = [f for f in frames if f['event'] == 'loop_reset_teleport']
        self.assertEqual(len(reset),1)
        self.assertEqual(reset[0]['t_wall_s'],4.)
        self.assertEqual(tuple(reset[0][k] for k in demo.HEADER),self.route.points[0])

    def test_invalid_or_unbounded_config_rejected(self):
        for kw in [dict(speed=0),dict(speed=float('nan')),dict(fps=61),dict(hold=-1),
                   dict(repeat=21),dict(speed=.00001)]:
            with self.subTest(kw=kw),self.assertRaises(ValueError): demo.timeline(self.route,**kw)

    def test_duplicate_points_and_zero_route(self):
        r = demo.Route('standard',((0,0,0),(0,0,0),(1,0,0)),{})
        self.assertEqual(demo.timeline(r,hold=0)[0]['x_enu_m'],0.)
        with self.assertRaises(ValueError): demo.timeline(demo.Route('standard',((0,0,0),)*2,{}))


class SafetyTests(unittest.TestCase):
    def test_installed_transport_properties_and_isolated_cli_request_with_fake_node(self):
        # Import the installed message bindings; never construct a real Node.
        node = Mock()
        node.subscribe.side_effect = lambda kind,topic,cb: (cb(NS(
            paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0))) or True)
        specs = {'create':('EntityFactory','Boolean'),'set_pose':('Pose','Boolean'),
                 'remove':('Entity','Boolean'),'scene/info':('Empty','Scene')}
        def service_info(service):
            suffix = service.removeprefix('/world/warehouse_world/')
            req,rep = specs[suffix]
            return [NS(req_type_name='gz.msgs.'+req,rep_type_name='gz.msgs.'+rep,socket_id='fake-provider')]
        node.service_info.side_effect = service_info
        with patch('gz.transport13.Node',return_value=node):
            gz = demo.Gazebo()
        self.assertEqual(set(gz.services),set(specs))
        self.assertEqual(node.subscribe.call_args.args[1],'/world/warehouse_world/stats')
        req = gz.types['Empty']()
        reply = 'name: "warehouse_world"\nmodel { name: "x500" id: 99 }\n'
        with patch.object(demo.subprocess,'run',return_value=NS(returncode=0,stdout=reply,stderr='')) as run:
            self.assertEqual(gz.request('scene/info',req,'Scene').name,demo.WORLD)
        argv = run.call_args.args[0]
        self.assertEqual(argv[0],str(demo.TRANSPORT_CLI))
        self.assertEqual(argv[argv.index('--timeout')+1],'5000')
        self.assertEqual(argv[argv.index('--req')+1],'')
        self.assertEqual(argv[argv.index('--reqtype')+1],'gz.msgs.Empty')
        self.assertEqual(argv[argv.index('--reptype')+1],'gz.msgs.Scene')
        node.request.assert_not_called()
        with self.assertRaises(ValueError): gz.request('control',req,'Scene')

    def test_generated_sdf_has_only_visuals(self):
        route = demo.load_routes()['enhanced']
        for name in demo.OWN_NAMES:
            xml = demo.model_sdf(name,TOKEN,route)
            root = ET.fromstring(xml)
            self.assertEqual(root.find('model').get('name'),name)
            self.assertEqual(root.findtext('model/static'),'true')
            for forbidden in ['collision','sensor','plugin','joint','include','actor','physics']:
                self.assertEqual(root.findall('.//'+forbidden),[])
            if name == demo.PATH_MODEL:
                self.assertEqual(len(root.findall('.//visual')),17+16)

    def test_names_cannot_be_x500_or_user_supplied(self):
        for name in ['x500','x500_0','pso_demo_visual_drone::x500','foo','../x500']:
            with self.subTest(name=name),self.assertRaises(ValueError):
                demo.model_sdf(name,TOKEN,demo.Route('standard',((0,0,1),(1,0,1)),{}))

    def test_collision_plugin_or_model_name_injection_rejected(self):
        xml = demo.model_sdf(demo.DRONE,TOKEN,demo.Route('standard',((0,0,1),(1,0,1)),{}))
        for altered in [xml.replace('</link>','<collision/></link>'),
                        xml.replace('</link>','<plugin/></link>'),xml.replace(demo.DRONE,'x500')]:
            with self.assertRaises(ValueError): demo.validate_sdf(altered,demo.DRONE,TOKEN)

    def test_pose_for_forged_x500_id_refused(self):
        gz = fake_gazebo()
        frame = demo.timeline(demo.Route('standard',((0,0,1),(1,0,1)),{}))[0]
        with self.assertRaisesRegex(ValueError,'Ownership'):
            gz.pose(dict(name=demo.DRONE,id=99),frame,TOKEN)
        gz.request.assert_not_called()

    def test_foreign_token_or_replaced_entity_refused(self):
        gz = fake_gazebo()
        for entry, token in [(dict(name=demo.DRONE,id=45),TOKEN),
                             (dict(name=demo.DRONE,id=44),'b'*32)]:
            with self.assertRaises(ValueError): gz.owned(entry,token)

    def test_path_and_x500_cannot_be_moved(self):
        gz = fake_gazebo()
        for name in [demo.PATH_MODEL,'x500']:
            with self.assertRaises(ValueError): gz.pose(dict(name=name,id=99),{},TOKEN)
        gz.request.assert_not_called()

    def test_paused_stale_and_progressing_clock_guards(self):
        gz = fake_gazebo(); gz.guard()
        gz.stats.paused = False
        with self.assertRaises(ValueError): gz.guard()
        gz.stats.paused = True; gz.stats_wall -= 3
        with self.assertRaises(ValueError): gz.guard()
        gz.stats_wall = time.monotonic()
        gz.on_stats(NS(paused=True,iterations=20001,sim_time=NS(sec=20,nsec=1000000)))
        with self.assertRaises(ValueError): gz.guard()

    def test_approval_cannot_be_piped_or_defaulted(self):
        with patch.object(demo.sys.stdin,'isatty',return_value=False),patch('builtins.input') as prompt:
            with self.assertRaises(ValueError): demo.confirm()
            prompt.assert_not_called()
        with patch.object(demo.sys.stdin,'isatty',return_value=True),patch('builtins.input',return_value='yes'):
            with self.assertRaises(ValueError): demo.confirm()

    def test_existing_models_never_adopted(self):
        gz = fake_gazebo()
        with patch.object(demo,'confirm') as approval,self.assertRaisesRegex(ValueError,'already exist'):
            demo.replay(gz,None,[],Path('/unused'),TOKEN)
        approval.assert_not_called(); gz.request.assert_not_called()

    def test_cleanup_rejects_x500_before_transport(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)/'ownership.json'
            p.write_text(json.dumps(dict(world=demo.WORLD,models=[dict(name='x500',id=99)])))
            with patch.object(demo,'Gazebo') as transport,self.assertRaises(ValueError): demo.cleanup(p)
            transport.assert_not_called()

    def test_cleanup_stale_server_identity_refused(self):
        gz = fake_gazebo(); gz.services = services()
        gz.services['scene/info']['socket_id'] = 'new-server'
        with tempfile.TemporaryDirectory() as td,patch.object(demo,'OUTPUT',Path(td)):
            p = make_run(td,TOKEN,entries=[dict(name=demo.DRONE,id=44)])
            with patch.object(demo,'Gazebo',return_value=gz),patch.object(demo,'confirm') as approve:
                with self.assertRaisesRegex(ValueError,'identity'): demo.cleanup(p)
                approve.assert_not_called()

    def test_cleanup_only_removes_own_ids_and_checks_disappearance(self):
        from gz.msgs10.entity_pb2 import Entity
        gz = fake_gazebo(); gz.services = services()
        gz.types['Entity'] = Entity
        entries = [dict(name=demo.PATH_MODEL,id=43),dict(name=demo.DRONE,id=44)]
        scene = {e['name']:model(e['name'],e['id']) for e in entries}
        scene['x500'] = model('x500',99)
        gz.scene.side_effect = lambda: dict(scene)
        def remove(suffix,request,response):
            self.assertEqual(suffix,'remove')
            self.assertEqual(request.type,Entity.MODEL)
            self.assertNotEqual(request.id,99)
            self.assertEqual(scene[request.name].id,request.id)
            del scene[request.name]
        gz.request.side_effect = remove
        with tempfile.TemporaryDirectory() as td,patch.object(demo,'OUTPUT',Path(td)):
            p = make_run(td,TOKEN,entries=entries)
            with patch.object(demo,'Gazebo',return_value=gz),patch.object(demo,'confirm'), \
                    contextlib.redirect_stdout(io.StringIO()): demo.cleanup(p)
        self.assertEqual(set(scene),{'x500'})
        self.assertEqual(gz.request.call_count,2)

    def test_partial_create_journal_recovers_only_matching_owner(self):
        from gz.msgs10.entity_pb2 import Entity
        gz = fake_gazebo(); gz.services = services()
        gz.types['Entity'] = Entity
        scene = {demo.DRONE:model(),'x500':model('x500',99)}
        gz.scene.side_effect = lambda: dict(scene)
        gz.request.side_effect = lambda suffix,request,response: scene.pop(request.name)
        with tempfile.TemporaryDirectory() as td,patch.object(demo,'OUTPUT',Path(td)):
            p = make_run(td,TOKEN,events=[dict(creating=demo.DRONE)])
            with patch.object(demo,'Gazebo',return_value=gz),patch.object(demo,'confirm'), \
                    contextlib.redirect_stdout(io.StringIO()): demo.cleanup(p,expected_id=44)
        self.assertEqual(set(scene),{'x500'})

    def test_service_rejection_stops_without_mutation_retry(self):
        node = Mock()
        node.subscribe.side_effect = lambda kind,topic,cb: (cb(NS(
            paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0))) or True)
        specs = {'create':('EntityFactory','Boolean'),'set_pose':('Pose','Boolean'),
                 'remove':('Entity','Boolean'),'scene/info':('Empty','Scene')}
        node.service_info.side_effect = lambda service: [NS(
            req_type_name='gz.msgs.'+specs[service.removeprefix('/world/warehouse_world/')][0],
            rep_type_name='gz.msgs.'+specs[service.removeprefix('/world/warehouse_world/')][1],socket_id='fake')]
        with patch('gz.transport13.Node',return_value=node): gz = demo.Gazebo()
        with patch.object(demo.subprocess,'run',return_value=NS(returncode=0,stdout='data: false\n',stderr='')) as run:
            with self.assertRaisesRegex(ValueError,'BOOLEAN_REJECTED'):
                gz.request('set_pose',gz.types['Pose'](name=demo.DRONE,id=44),'Boolean')
        self.assertEqual(run.call_count,1)
        self.assertEqual(run.call_args.args[0][run.call_args.args[0].index('--timeout')+1],'1500')
        node.request.assert_not_called()

    def test_ack_not_proof_of_pose_update(self):
        gz = fake_gazebo()
        frame = dict(zip(demo.HEADER,(1.,2.,3.)))
        with patch.object(demo.time,'monotonic',side_effect=[0,0,3]),patch.object(demo.time,'sleep'):
            with self.assertRaisesRegex(ValueError,'ACK did not'): gz.verify_pose(dict(name=demo.DRONE,id=44),TOKEN,frame)

    def test_dry_run_never_initializes_transport_and_is_additive(self):
        with tempfile.TemporaryDirectory() as td,patch.object(demo,'OUTPUT',Path(td)), \
                patch.object(demo,'Gazebo',side_effect=AssertionError('No Transport in dry-run')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(demo.main(['--dry-run']),0)
            self.assertEqual(demo.main(['--dry-run']),0)
            runs = list(Path(td).iterdir())
            self.assertEqual(len(runs),2)
            for run in runs:
                for name in ['standard','enhanced']:
                    self.assertTrue((run/f'{name}_preview.html').is_file())
                    meta = json.loads((run/f'{name}_metadata.json').read_text())
                    self.assertFalse(meta['live_control_go'])
                    self.assertTrue(20 <= meta['duration_wall_s'] <= 40)


if __name__ == '__main__': unittest.main()
