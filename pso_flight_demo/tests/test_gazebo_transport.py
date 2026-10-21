"""Fake Transport/CLI only. No live Node construction, requests or physics."""

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene

from pso_flight_demo.visualization import raw_pso_gazebo as demo

SCENE = 'model { name: "white_floor" id: 2 }\nmodel { name: "x500" id: 99 }\n'


def adapter(read_only=False):
    gz = object.__new__(demo.Gazebo)  # never call real Node()
    gz.node = Mock()
    gz.node.options.partition = 'offline_test_partition'
    gz.types = dict(Boolean=Boolean,Empty=Empty,EntityFactory=EntityFactory,Entity=Entity,Pose=Pose,Scene=Scene)
    gz.read_only = read_only
    gz.environment = dict(os.environ)
    gz.request_log = []
    gz.last_scene_text = None
    gz.baseline = (20000,20,0)
    gz.stats = NS(paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0))
    gz.stats_wall = time.monotonic()
    gz.clock_changed = False
    gz.services = {s:dict(request='gz.msgs.'+q,response='gz.msgs.'+r,socket_id='provider-'+s)
                   for s,(q,r) in demo.SERVICE_TYPES.items()}
    gz.node.service_info.side_effect = lambda name: [NS(**dict(
        req_type_name=gz.services[name.removeprefix('/world/warehouse_world/')]['request'],
        rep_type_name=gz.services[name.removeprefix('/world/warehouse_world/')]['response'],
        socket_id=gz.services[name.removeprefix('/world/warehouse_world/')]['socket_id']))]
    return gz


def result(stdout=SCENE,stderr='',code=0):
    return NS(returncode=code,stdout=stdout,stderr=stderr)


class BindingTests(unittest.TestCase):
    def test_real_installed_wrapper_signature_serialization_and_return(self):
        # Exercise the actual installed Python wrapper with a fake C++ boundary.
        from gz.transport13 import Node
        scene = Scene(name='warehouse_world')
        raw = Mock(return_value=(True,scene.SerializeToString()))
        fake = NS(request_raw=raw)
        ok,reply = Node.request(fake,'/world/warehouse_world/scene/info',Empty(),Empty,Scene,1500)
        self.assertIs(ok,True)
        self.assertEqual(reply,scene)
        raw.assert_called_once_with('/world/warehouse_world/scene/info',b'','gz.msgs.Empty','gz.msgs.Scene',1500)


class DiscoveryTests(unittest.TestCase):
    def test_missing_ambiguous_schema_and_provider_change_separate_errors(self):
        gz = adapter()
        good = NS(
            req_type_name='gz.msgs.Empty',rep_type_name='gz.msgs.Scene',socket_id='provider-scene/info')
        for providers,code in [([], 'DISCOVERY_MISSING'),([good,good],'DISCOVERY_AMBIGUOUS'),
                ([NS(req_type_name='wrong',rep_type_name='gz.msgs.Scene',socket_id='provider-scene/info')],
                 'DISCOVERY_SCHEMA_MISMATCH'),
                ([NS(req_type_name='gz.msgs.Empty',rep_type_name='gz.msgs.Scene',socket_id='replacement')],
                 'PROVIDER_CHANGED')]:
            with self.subTest(code=code),patch.object(gz.node,'service_info',return_value=providers), \
                    patch.object(demo.subprocess,'run') as run:
                with self.assertRaises(demo.TransportError) as caught: gz.request('scene/info',Empty(),'Scene')
                self.assertEqual(caught.exception.code,code)
                run.assert_not_called()

    def test_provider_changed_during_request_rejects_valid_response(self):
        gz = adapter()
        good = NS(req_type_name='gz.msgs.Empty',rep_type_name='gz.msgs.Scene',socket_id='provider-scene/info')
        changed = NS(req_type_name='gz.msgs.Empty',rep_type_name='gz.msgs.Scene',socket_id='new-server')
        gz.node.service_info.side_effect = [[good],[changed]]
        with patch.object(demo.subprocess,'run',return_value=result()):
            with self.assertRaisesRegex(demo.TransportError,'PROVIDER_CHANGED'): gz.scene()

    def test_allowlist_and_readonly_mode_prevent_mutations_before_launching_client(self):
        gz = adapter(read_only=True)
        for suffix,message,reply in [('control',Empty(),'Boolean'),('create',EntityFactory(),'Boolean'),
                                      ('set_pose',Pose(name='x500',id=99),'Boolean'),('remove',Entity(),'Boolean')]:
            with self.subTest(suffix=suffix),patch.object(demo.subprocess,'run') as run:
                with self.assertRaises(demo.TransportError): gz.request(suffix,message,reply)
                run.assert_not_called()


class StatsDiagnosticTests(unittest.TestCase):
    def test_missing_stats_is_not_reported_as_unpaused_world(self):
        gz = adapter(); gz.stats = None
        with self.assertRaises(demo.TransportError) as caught:
            gz.guard()
        self.assertEqual(caught.exception.code,'STATS_NOT_RECEIVED')

    def test_fresh_running_world_is_not_reported_as_missing_stats(self):
        gz = adapter(); gz.stats.paused = False
        with self.assertRaises(demo.TransportError) as caught:
            gz.guard()
        self.assertEqual(caught.exception.code,'STATS_NOT_PAUSED')
        self.assertIn('paused=False',str(caught.exception))

    def test_stale_paused_snapshot_keeps_existing_two_second_gate(self):
        gz = adapter(); gz.stats_wall -= 3
        with self.assertRaises(demo.TransportError) as caught:
            gz.guard()
        self.assertEqual(caught.exception.code,'STATS_STALE')
        self.assertIn('limit=2.000s',str(caught.exception))

    def test_clock_change_is_still_latched_after_paused_stats_resume(self):
        gz = adapter()
        gz.on_stats(NS(paused=False,iterations=20001,sim_time=NS(sec=20,nsec=1000000)))
        gz.on_stats(NS(paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0)))
        with self.assertRaises(demo.TransportError) as caught:
            gz.guard()
        self.assertEqual(caught.exception.code,'CLOCK_CHANGED')


class RequestTests(unittest.TestCase):
    def test_success_uses_same_environment_and_separate_native_process(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run',return_value=result()) as run:
            self.assertEqual(gz.scene()['x500'].id,99)
        args,kw = run.call_args
        self.assertEqual(args[0][0],str(demo.TRANSPORT_CLI))
        self.assertEqual(kw['env'],gz.environment)
        self.assertEqual(kw['timeout'],6.)
        self.assertFalse(kw.get('shell',False))
        gz.node.request.assert_not_called()
        self.assertEqual(gz.request_log[0]['stdout_bytes'],len(SCENE.encode()))
        self.assertEqual(gz.request_log[0]['status'],'RESPONSE_PARSED')

    def test_cli_timeout_with_exit_zero_is_not_success(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run',return_value=result('', 'Service call timed out\n')) as run:
            with self.assertRaisesRegex(demo.TransportError,'REQUEST_TIMEOUT'): gz.scene()
        self.assertEqual(run.call_count,1)

    def test_outer_wall_timeout_is_bounded_no_retry(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run',side_effect=subprocess.TimeoutExpired('fake-cli',6)) as run:
            with self.assertRaisesRegex(demo.TransportError,'REQUEST_TIMEOUT'): gz.scene()
        self.assertEqual(run.call_count,1)

    def test_callback_failure_with_exit_zero_is_separate_from_timeout(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run',return_value=result('Service call failed\n')):
            with self.assertRaisesRegex(demo.TransportError,'REQUEST_FAILURE.*callback returned false'): gz.scene()

    def test_cli_os_error_exit_code_or_stderr_is_request_failure(self):
        for response in [result('',code=1),result(SCENE,'error in getifaddrs'),FileNotFoundError('fake CLI missing')]:
            kw = dict(side_effect=response) if isinstance(response,Exception) else dict(return_value=response)
            with self.subTest(response=response),patch.object(demo.subprocess,'run',**kw):
                with self.assertRaisesRegex(demo.TransportError,'REQUEST_FAILURE'): adapter().scene()

    def test_boolean_rejection_including_omitted_proto_default(self):
        for stdout in ['data: false\n','\n']:
            gz = adapter()
            with self.subTest(stdout=stdout),patch.object(demo.subprocess,'run',return_value=result(stdout)):
                with self.assertRaisesRegex(demo.TransportError,'BOOLEAN_REJECTED'):
                    gz.request('set_pose',Pose(name=demo.DRONE,id=44),'Boolean')

    def test_boolean_true_success(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run',return_value=result('data: true\n')):
            self.assertTrue(gz.request('set_pose',Pose(name=demo.DRONE,id=44),'Boolean').data)

    def test_invalid_or_empty_scene_response_is_not_success(self):
        for stdout in ['broken protobuf','data: true','', 'name: "warehouse_world"']:
            with self.subTest(stdout=stdout),patch.object(demo.subprocess,'run',return_value=result(stdout)):
                with self.assertRaisesRegex(demo.TransportError,'INVALID_RESPONSE'): adapter().scene()

    def test_wrong_request_response_schema_rejected_before_cli(self):
        gz = adapter()
        with patch.object(demo.subprocess,'run') as run:
            with self.assertRaisesRegex(demo.TransportError,'REQUEST_SCHEMA_MISMATCH'):
                gz.request('scene/info',Pose(),'Scene')
            with self.assertRaisesRegex(demo.TransportError,'REQUEST_SCHEMA_MISMATCH'):
                gz.request('scene/info',Empty(),'Boolean')
            run.assert_not_called()

    def test_transport_env_change_is_not_silently_inherited(self):
        gz = adapter()
        with patch.dict(demo.os.environ,{'GZ_PARTITION':'other_partition'}),patch.object(demo.subprocess,'run') as run:
            with self.assertRaisesRegex(demo.TransportError,'TRANSPORT_ENV_CHANGED'): gz.scene()
            run.assert_not_called()

    def test_stats_callbacks_continue_while_service_client_waits(self):
        gz = adapter()
        called = threading.Event()
        def callback():
            gz.on_stats(NS(paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0)))
            called.set()
        def cli(*a,**kw):
            worker = threading.Thread(target=callback)
            worker.start(); self.assertTrue(called.wait(1.)); worker.join(1.)
            return result()
        with patch.object(demo.subprocess,'run',side_effect=cli): gz.scene()
        self.assertTrue(called.is_set())

    def test_physics_change_during_service_call_stays_latched_and_blocks(self):
        gz = adapter()
        def cli(*a,**kw):
            gz.on_stats(NS(paused=False,iterations=20001,sim_time=NS(sec=20,nsec=1000000)))
            gz.on_stats(NS(paused=True,iterations=20000,sim_time=NS(sec=20,nsec=0)))
            return result()
        with patch.object(demo.subprocess,'run',side_effect=cli):
            with self.assertRaisesRegex(ValueError,'unchanged simulation clock'): gz.scene()
        self.assertTrue(gz.clock_changed)


class SceneTests(unittest.TestCase):
    def test_missing_or_matching_scene_name_uses_scoped_world_evidence(self):
        for prefix in ['', 'name: "warehouse_world"\n']:
            with self.subTest(prefix=prefix),patch.object(demo.subprocess,'run',return_value=result(prefix+SCENE)):
                gz = adapter(); self.assertEqual(gz.scene()['x500'].id,99)
                self.assertIn(gz.last_scene_name,('',demo.WORLD))

    def test_wrong_nonempty_scene_name_rejected(self):
        with patch.object(demo.subprocess,'run',return_value=result('name: "wrong_world"\n'+SCENE)):
            with self.assertRaisesRegex(demo.TransportError,'WRONG_SCENE'): adapter().scene()

    def test_unique_x500_and_nonzero_unique_ids_required(self):
        for text in ['model { name: "white_floor" id: 2 }',
                     SCENE+'model { name: "x500" id: 100 }',
                     SCENE+'model { name: "x500_0" id: 100 }',
                     SCENE+'model { name: "other" id: 99 }',
                     SCENE+'model { name: "other" }']:
            with self.subTest(text=text),patch.object(demo.subprocess,'run',return_value=result(text)):
                with self.assertRaisesRegex(demo.TransportError,'INVALID_SCENE_MODELS'): adapter().scene()

    def test_readonly_command_saves_evidence_without_any_mutating_service(self):
        gz = adapter(read_only=True)
        with tempfile.TemporaryDirectory() as td,patch.object(demo,'OUTPUT',Path(td)), \
                patch.object(demo,'Gazebo',return_value=gz) as constructor, \
                patch.object(demo.subprocess,'run',return_value=result()) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(demo.main(['--check-transport']),0)
            constructor.assert_called_once_with(read_only=True)
            files = list(Path(td).glob('*/transport.json'))
            self.assertEqual(len(files),1)
            report = json.loads(files[0].read_text())
            self.assertEqual(report['result'],'PASS_READONLY_TRANSPORT')
            self.assertFalse(report['mutations_executed'])
            self.assertFalse(report['visualization_go'])
            self.assertTrue((files[0].parent/'scene_response.pbtxt').is_file())
            self.assertEqual(run.call_count,1)
            self.assertEqual(run.call_args.args[0][run.call_args.args[0].index('-s')+1],'/world/warehouse_world/scene/info')


if __name__ == '__main__': unittest.main()
