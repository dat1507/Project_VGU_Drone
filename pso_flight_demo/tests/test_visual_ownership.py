"""Ownership/recovery proof tests. Fake scenes only; no live Gazebo requests."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V

from pso_flight_demo.visualization import raw_pso_gazebo as demo
from pso_flight_demo.visualization import ownership as own
from pso_flight_demo.tests.visualization_fakes import model,proof_for,make_run,services

TOKEN = 'dc7b7f2315c24dc5a2f2ad450dae14f2'
ENTRY = dict(name=demo.PATH_MODEL,id=105)


def fake_gazebo(scene=None):
    gz = object.__new__(demo.Gazebo)
    gz.scene = Mock(return_value=scene or {demo.PATH_MODEL:model(demo.PATH_MODEL,105,TOKEN),'x500':model('x500',99,TOKEN)})
    gz.ownership_specs = {n:proof_for(n,TOKEN) for n in demo.OWN_NAMES}
    gz.ownership_evidence = {}
    gz.baseline = (20000,20,0)
    gz.services = services()
    gz.guard = Mock()
    gz.request = Mock()
    gz.types = {'Entity':Entity}
    return gz


class OwnershipTests(unittest.TestCase):
    def test_static_and_parent_id_omitted_is_unknown_not_dynamic(self):
        gz = fake_gazebo()
        m = gz.owned(ENTRY,TOKEN)
        self.assertFalse(m.is_static)
        self.assertNotIn('is_static',{f.name for f,v in m.ListFields()})
        self.assertFalse(m.link[0].visual[0].parent_id)
        evidence = gz.ownership_evidence[demo.PATH_MODEL]
        self.assertEqual(evidence['visuals_checked'],33)
        self.assertEqual(evidence['static_source'],'VALIDATED_EXPORTED_SDF')
        self.assertEqual(evidence['static_scene'],'NOT_PUBLISHED')
        self.assertFalse(evidence['runtime_static_proven'])

    def test_reported_static_true_is_also_accepted(self):
        gz = fake_gazebo(); gz.scene.return_value[demo.PATH_MODEL].is_static = True
        gz.owned(ENTRY,TOKEN)
        self.assertTrue(gz.ownership_evidence[demo.PATH_MODEL]['runtime_static_proven'])

    def test_explicit_false_contradiction_is_not_ignored(self):
        gz = fake_gazebo(); real = gz.scene.return_value[demo.PATH_MODEL]
        class ExplicitFalse:
            def __getattr__(self,key): return getattr(real,key)
            def ListFields(self): return [*real.ListFields(),(NS(name='is_static'),False)]
        gz.scene.return_value[demo.PATH_MODEL] = ExplicitFalse()
        with self.assertRaisesRegex(ValueError,'static contradiction'): gz.owned(ENTRY,TOKEN)

    def test_x500_wrong_id_wrong_token_or_missing_proof_refused(self):
        gz = fake_gazebo()
        for entry,token in [(dict(name='x500',id=99),TOKEN),(dict(ENTRY,id=106),TOKEN),(ENTRY,'b'*32)]:
            with self.subTest(entry=entry),self.assertRaises(ValueError): gz.owned(entry,token)
        gz.ownership_specs.clear()
        with self.assertRaisesRegex(ValueError,'SDF proof missing'): gz.owned(ENTRY,TOKEN)
        gz.request.assert_not_called()

    def test_same_names_count_but_geometry_or_local_pose_changed_refused(self):
        for field in ['size','pose','parent','id']:
            gz = fake_gazebo(); m = gz.scene.return_value[demo.PATH_MODEL]
            visual = m.link[0].visual[0]
            if field == 'size': visual.geometry.sphere.radius += .1
            if field == 'pose': visual.pose.position.x += .1
            if field == 'parent': visual.parent_id = 900
            if field == 'id': visual.id = m.id
            with self.subTest(field=field),self.assertRaises(ValueError): gz.owned(ENTRY,TOKEN)

    def test_extra_missing_duplicate_visual_sensor_or_link_refused(self):
        for kind in ['extra','missing','duplicate','sensor','link','model','path_origin']:
            gz = fake_gazebo(); m = gz.scene.return_value[demo.PATH_MODEL]
            if kind == 'extra': m.link[0].visual.add(name='foreign_visual',id=9000)
            if kind == 'missing': del m.link[0].visual[-1]
            if kind == 'duplicate': m.link[0].visual[-1].name = m.link[0].visual[0].name
            if kind == 'sensor': m.link[0].sensor.add(name='not_visual')
            if kind == 'link': m.link.add(name='other_link',id=9000)
            if kind == 'model': m.model.add(name='nested',id=9000)
            if kind == 'path_origin': m.pose.position.x = 1
            with self.subTest(kind=kind),self.assertRaises(ValueError): gz.owned(ENTRY,TOKEN)

    def test_sdf_static_false_or_nonvisual_rejected(self):
        text = demo.model_sdf(demo.PATH_MODEL,TOKEN,demo.load_routes()['enhanced'])
        for changed in [text.replace('<static>true','<static>false'),text.replace('</link>','<collision/></link>')]:
            with self.assertRaises(ValueError): own.sdf_proof(changed,demo.PATH_MODEL,TOKEN)

    def test_actual_legacy_run_matches_generated_sdf_offline(self):
        path = demo.OUTPUT/'raw_pso_20261019T072352Z_dc7b7f23/ownership.jsonl'
        context = demo.cleanup_context(path)
        self.assertEqual(context['parsed']['unknown_ids'],[demo.PATH_MODEL])
        self.assertEqual(context['proofs'][demo.PATH_MODEL]['sdf_sha256'],
                         'ca774e033938dda64922c1cf0b9b7d508dd8027d54ede589cc4a4ca33d1a73ae')


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.patch = patch.object(demo,'OUTPUT',Path(self.temp.name)); self.patch.start(); self.addCleanup(self.patch.stop)
        self.path = make_run(self.temp.name,TOKEN,events=[dict(creating=demo.PATH_MODEL)])
        self.context = demo.cleanup_context(self.path)
        self.gz = fake_gazebo()

    def test_creating_only_cannot_adopt_id_from_name_alone(self):
        with self.assertRaisesRegex(ValueError,'expect-model-id'):
            demo.collect_cleanup(self.gz,self.context)
        self.gz.request.assert_not_called()

    def test_id_105_recovers_only_with_complete_proof(self):
        evidence = demo.collect_cleanup(self.gz,self.context,105)
        self.assertEqual([e['id'] for e in evidence],[105])
        self.assertEqual([e['name'] for e in evidence],[demo.PATH_MODEL])
        self.gz.request.assert_not_called()

    def test_wrong_expected_id_and_x500_id_are_refused(self):
        for entity_id in [99,106]:
            with self.subTest(entity_id=entity_id),self.assertRaises(ValueError):
                demo.collect_cleanup(self.gz,self.context,entity_id)
        self.gz.request.assert_not_called()

    def test_restart_same_clock_or_clock_progress_blocks_recovery(self):
        self.gz.services['remove']['socket_id'] = 'new-server'
        with self.assertRaisesRegex(ValueError,'identity'): demo.collect_cleanup(self.gz,self.context,105)
        self.gz.services = services(); self.gz.baseline = (20001,20,1_000_000)
        with self.assertRaisesRegex(ValueError,'identity'): demo.collect_cleanup(self.gz,self.context,105)

    def test_readonly_review_never_removes_or_prompts_and_preserves_manifest(self):
        before = self.path.read_bytes()
        with patch.object(demo,'Gazebo',return_value=self.gz) as constructor,patch.object(demo,'confirm') as confirm, \
                contextlib.redirect_stdout(io.StringIO()):
            report = demo.cleanup(self.path,105,review_only=True)
        constructor.assert_called_once_with(read_only=True); confirm.assert_not_called()
        self.gz.request.assert_not_called()
        self.assertFalse(report['mutations_executed'])
        self.assertEqual(self.path.read_bytes(),before)

    def test_cleanup_removes_only_105_and_leaves_x500(self):
        scene = dict(self.gz.scene.return_value)
        scene[demo.DRONE] = model(demo.DRONE,200,'a'*32)  # unrelated visual is not adopted or deleted
        self.gz.scene.side_effect = lambda: dict(scene)
        def request(suffix,message,response):
            self.assertEqual((suffix,message.name,message.id),('remove',demo.PATH_MODEL,105))
            self.assertEqual(message.type,Entity.MODEL)
            del scene[demo.PATH_MODEL]
        self.gz.request.side_effect = request
        with patch.object(demo,'Gazebo',return_value=self.gz),patch.object(demo,'confirm'), \
                contextlib.redirect_stdout(io.StringIO()):
            result = demo.cleanup(self.path,105)
        self.assertEqual(result['result'],'OWN_VISUALS_REMOVED')
        self.assertEqual(set(scene),{'x500',demo.DRONE})
        self.assertEqual(self.gz.request.call_count,1)

    def test_source_tamper_after_operator_confirmation_sends_no_remove(self):
        def changed(*a,**kw):
            file = self.path.parent/'enhanced_pso_demo_visual_path.sdf'
            file.write_text(file.read_text()+'\n')
        with patch.object(demo,'Gazebo',return_value=self.gz),patch.object(demo,'confirm',side_effect=changed), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError,'source changed'): demo.cleanup(self.path,105)
        self.gz.request.assert_not_called()

    def test_recreated_model_after_confirmation_sends_no_remove(self):
        def changed(*a,**kw): self.gz.scene.return_value[demo.PATH_MODEL].id = 106
        with patch.object(demo,'Gazebo',return_value=self.gz),patch.object(demo,'confirm',side_effect=changed), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError,'entity ID'): demo.cleanup(self.path,105)
        self.gz.request.assert_not_called()

    def test_committed_or_observed_id_does_not_need_manual_id(self):
        for event in ['observed','created']:
            header = own.read_manifest(self.path)['header']
            self.path.write_text('\n'.join(map(json.dumps,[header,dict(creating=demo.PATH_MODEL),{event:ENTRY}])))
            context = demo.cleanup_context(self.path)
            self.assertEqual(demo.collect_cleanup(self.gz,context)[0]['id'],105)

    def test_malformed_manifests_rejected_before_client_construction(self):
        original = self.path.read_text()
        cases = ['', '{bad json', original+'\n{"observed":',
                 original+'\n'+json.dumps({'observed':dict(name='x500',id=99)}),
                 original+'\n'+json.dumps({'unknown':'event'}),
                 original+'\n'+json.dumps({'creating':demo.PATH_MODEL}),
                 original+'\n'+json.dumps({'observed':dict(ENTRY,id=True)}),
                 original.replace('"models": []','"models": [], "models": []'),
                 original.replace('"token": "'+TOKEN+'"','"token": "bad"')]
        for text in cases:
            self.path.write_text(text)
            with self.subTest(text=text),patch.object(demo,'Gazebo') as client:
                with self.assertRaises(ValueError): demo.cleanup(self.path,105)
                client.assert_not_called()

    def test_static_sdf_tamper_rejected_before_client(self):
        file = self.path.parent/'enhanced_pso_demo_visual_path.sdf'
        file.write_text(file.read_text().replace('<static>true','<static>false'))
        with patch.object(demo,'Gazebo') as client:
            with self.assertRaises(ValueError): demo.cleanup(self.path,105)
            client.assert_not_called()

    def test_remove_timeout_does_not_retry_and_records_uncertainty(self):
        self.gz.request.side_effect = ValueError('REQUEST_TIMEOUT')
        with patch.object(demo,'Gazebo',return_value=self.gz),patch.object(demo,'confirm'), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError,'REQUEST_TIMEOUT'): demo.cleanup(self.path,105)
        self.assertEqual(self.gz.request.call_count,1)
        reports = list(Path(self.temp.name).glob('ownership_review_*/cleanup_failure.json'))
        report = json.loads(reports[0].read_text())
        self.assertTrue(report['mutations_executed'])
        self.assertEqual(report['remove_attempts'],[ENTRY])
        self.assertIn('x500',self.gz.scene.return_value)

    def test_remove_ack_without_scene_disappearance_is_not_success(self):
        with patch.object(demo,'Gazebo',return_value=self.gz),patch.object(demo,'confirm'), \
                patch.object(demo.time,'monotonic',side_effect=[0,1,2]),patch.object(demo.time,'sleep'), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError,'disappearance unconfirmed'): demo.cleanup(self.path,105)
        self.assertEqual(self.gz.request.call_count,1)

    def test_observed_id_conflict_is_malformed_not_recovery(self):
        text = self.path.read_text()+'\n'+json.dumps(dict(observed=ENTRY))+'\n'+json.dumps(dict(created=dict(ENTRY,id=106)))
        self.path.write_text(text)
        with self.assertRaisesRegex(ValueError,'Conflicting'): own.read_manifest(self.path)


class CreateAndPoseTests(unittest.TestCase):
    def test_candidate_id_is_recorded_before_failing_structure_verification(self):
        gz = fake_gazebo()
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        gz.types['EntityFactory'] = EntityFactory
        del gz.scene.return_value[demo.PATH_MODEL].link[0].visual[-1]
        observed = []
        with self.assertRaises(ValueError): gz.create(demo.PATH_MODEL,TOKEN,demo.load_routes()['enhanced'],observed=observed.append)
        self.assertEqual(observed,[ENTRY])
        self.assertEqual(gz.request.call_args.args[0],'create')

    def test_pose_verification_uses_fresh_stream_not_cached_scene(self):
        gz = fake_gazebo({demo.DRONE:model(demo.DRONE,105,TOKEN)})
        gz.pose_cache = {105:dict(name=demo.DRONE,position=(1.,2.,3.),received=10.)}
        gz.pose_requests = {105:9.}
        frame = dict(zip(demo.HEADER,(1.,2.,3.)))
        with patch.object(demo.time,'monotonic',return_value=10.):
            gz.verify_pose(dict(name=demo.DRONE,id=105),TOKEN,frame)
        self.assertEqual(gz.scene.return_value[demo.DRONE].pose.position.x,0.)

    def test_pose_stream_ignores_x500_and_rejects_foreign_clock(self):
        gz = fake_gazebo(); gz.pose_cache = {}; gz.pose_stream_error = None
        message = Pose_V(); message.header.stamp.sec = 20
        message.pose.add(name='x500',id=99)
        gz.on_poses(message); self.assertEqual(gz.pose_cache,{})
        message.pose.add(name=demo.DRONE,id=105)
        message.header.stamp.sec = 21
        gz.on_poses(message)
        self.assertIsNotNone(gz.pose_stream_error)


if __name__ == '__main__': unittest.main()
