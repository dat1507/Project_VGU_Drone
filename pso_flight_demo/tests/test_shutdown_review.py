"""Shutdown review tests: fake probes/processes only; never live shutdown/physics."""

import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from pso_flight_demo import session_launcher as sl, session_runtime as sr
from pso_flight_demo.environments import MODULE, load_profile, load_environment
from pso_flight_demo.launch_plan import launch_plan
from pso_flight_demo.tests.test_session_launcher import FakeHost, ident


def world(clock=10_000_000, paused=True, correct=True):
    sec,us = divmod(clock,1_000_000)
    return dict(correct=correct,paused=paused,iterations=clock//1000,
                evidence=dict(stats=dict(stdout=f'sim_time {{ sec: {sec} nsec: {us*1000} }}')))


def native(armed=False,landed=True,stamp=9_900_000,logger_exit=0):
    def probe(profile,module,*args):
        if module == 'logger': return dict(exit=logger_exit,stdout='logger running')
        if module == 'param': return dict(exit=0,stdout=' COM_ARM_ON_BOOT [1,2] : 0\n')
        text = 'armed: '+str(armed) if args[0] == 'actuator_armed' else 'landed: '+str(landed)
        return dict(exit=0,stdout=f'    timestamp: {stamp}\n    {text}\n')
    return probe


def state_fixture():
    p = load_profile()
    w,_ = load_environment(p)
    children = []
    for role,pid in [('GAZEBO',800),('AGENT',801),('PX4',802)]:
        owner = ident(pid)
        if role == 'PX4': owner['exe'] = str((Path(p.px4_root).expanduser()/'build/px4_sitl_default/bin/px4').resolve())
        children.append(dict(role=role,owner=owner,members=[owner]))
    return dict(state='SHUTDOWN_BLOCKED',supervisor=ident(700),children=children,profile=p.snapshot(),
                profile_sha256=sl.digest(p.snapshot()),sdf_sha256=w.sdf_sha256,with_dds=True)


class GateTests(unittest.TestCase):
    def gate(self,**kw):
        before = kw.pop('before',world())
        after = kw.pop('after',before)
        with patch.object(sr,'world_snapshot',side_effect=[before,after]),patch.object(sr,'native_probe',side_effect=native(**kw)):
            return sr.shutdown_gate(load_profile())

    def test_paused_disarmed_landed_safe_snapshot_but_not_shutdown(self):
        gate = self.gate()
        self.assertTrue(gate['safe'])
        self.assertFalse(gate['can_shutdown'])
        self.assertTrue(any('CLOCK_PAUSED' in b for b in gate['blockers']))
        self.assertFalse(any('Need correct' in b for b in gate['blockers']))

    def test_actual_running_clock_progress_is_required(self):
        gate = self.gate(before=world(paused=False),after=world(10_050_000,False))
        self.assertTrue(gate['can_shutdown'])
        self.assertTrue(gate['clock_progressing'])

    def test_not_paused_but_frozen_clock_fails_closed(self):
        gate = self.gate(before=world(paused=False))
        self.assertTrue(gate['safe'])
        self.assertFalse(gate['can_shutdown'])
        self.assertIn('CLOCK_NOT_PROGRESSING',gate['blockers'][0])

    def test_stale_future_zero_missing_or_non_numeric_status_rejected(self):
        for stamp in [8_999_999,10_000_001,0,'nan','missing']:
            with self.subTest(stamp=stamp): self.assertFalse(self.gate(stamp=stamp)['safe'])

    def test_armed_not_landed_or_logger_timeout_rejected(self):
        for change in [dict(armed=True),dict(landed=False),dict(logger_exit=None)]:
            with self.subTest(change=change): self.assertFalse(self.gate(**change)['safe'])

    def test_wrong_world_or_pause_transition_rejected(self):
        self.assertFalse(self.gate(before=world(correct=False))['safe'])
        self.assertFalse(self.gate(after=world(correct=False))['safe'])
        self.assertFalse(self.gate(before=world(),after=world(10_050_000,False))['can_shutdown'])

    def test_stats_timeout_reports_unknown_pause_state(self):
        replies=[dict(exit=None,stdout='',stderr='timeout'),
                 dict(exit=None,stdout='',stderr='timeout')]
        with patch.object(sr,'probe',side_effect=replies):
            snapshot=sr.world_snapshot(load_profile())
        self.assertIsNone(snapshot['paused'])
        self.assertFalse(snapshot['correct'])
        with patch.object(sr,'world_snapshot',return_value=snapshot), \
                patch.object(sr,'native_probe',side_effect=native()):
            gate=sr.shutdown_gate(load_profile())
        self.assertFalse(gate['can_shutdown'])
        self.assertFalse(gate['clock_progressing'])
        self.assertTrue(any('CLOCK_UNKNOWN_STATS_UNAVAILABLE' in b for b in gate['blockers']))
        self.assertFalse(any('CLOCK_NOT_PROGRESSING' in b for b in gate['blockers']))

    def test_empty_success_reply_cannot_establish_running_world(self):
        with patch.object(sr,'probe',return_value=dict(exit=0,stdout='',stderr='')):
            snapshot=sr.world_snapshot(load_profile())
        self.assertIsNone(snapshot['paused'])
        self.assertIsNone(snapshot['iterations'])
        self.assertFalse(snapshot['correct'])

    def test_readable_stats_can_establish_paused_or_proto_default_running(self):
        for paused_text,expected in [('paused: true',True),('',False)]:
            replies=[dict(exit=0,stdout='sim_time { sec: 10 }\niterations: 10000\n'+paused_text,stderr=''),
                     dict(exit=0,stdout='Requesting state for world [warehouse_world]\n - x500\n',stderr='')]
            with self.subTest(paused=expected),patch.object(sr,'probe',side_effect=replies):
                snapshot=sr.world_snapshot(load_profile())
            self.assertIs(snapshot['paused'],expected)
            self.assertTrue(snapshot['correct'])


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.state = state_fixture()
        self.inv = dict(alive=[700,800,801,802],identity_mismatches=[],unrecorded_child_tasks=[],absent=[])
        self.host = Mock()
        self.host.stop_gate.return_value = dict(safe=True,paused=True,can_shutdown=False,
            world=world(),world_before=world(),clock_before_us=10_000_000,clock_after_us=10_000_000,
            blockers=['CLOCK_PAUSED'])
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(sl,'Host',return_value=self.host))
        self.inventory = self.stack.enter_context(patch.object(sl,'session_inventory',return_value=self.inv))
        self.stack.enter_context(patch.object(sl,'native_probe',side_effect=native()))

    def test_paused_review_only_proposes_bounded_operator_window(self):
        result = sl.shutdown_review(self.path,self.state)
        self.assertEqual(result['result'],'READY_FOR_SEPARATE_CLOCK_APPROVAL')
        proposal = result['clock_proposal']
        self.assertEqual(proposal['maximum_end_iterations'],15000)
        self.assertIn('pause: true, multi_step: 5000',proposal['command'])
        self.assertFalse(result['shutdown_request_allowed'])
        self.host.shutdown_px4.assert_not_called()
        self.host.signal_owned.assert_not_called()
        self.assertEqual(list(self.path.iterdir()),[])

    def test_no_clock_proposal_for_stale_status_or_world(self):
        self.host.stop_gate.return_value.update(safe=False,blockers=['stale state / wrong world'])
        result = sl.shutdown_review(self.path,self.state)
        self.assertEqual(result['result'],'NO_GO')
        self.assertIsNone(result['clock_proposal'])

    def test_identity_mismatch_or_inventory_change_blocks(self):
        self.inv['identity_mismatches'] = [802]
        with self.assertRaisesRegex(ValueError,'identity'): sl.shutdown_review(self.path,self.state)
        self.inv['identity_mismatches'] = []
        changed = copy.deepcopy(self.inv); changed['alive'].remove(801)
        self.inventory.side_effect = [self.inv,changed]
        with self.assertRaisesRegex(ValueError,'inventory changed'): sl.shutdown_review(self.path,self.state)

    def test_profile_sdf_mismatch_blocks_before_probes(self):
        self.state['sdf_sha256'] = 'other'
        with self.assertRaisesRegex(ValueError,'profile/SDF'): sl.shutdown_review(self.path,self.state)
        self.host.stop_gate.assert_not_called()

    def test_unverified_auto_arm_blocks_clock_proposal(self):
        with patch.object(sl,'native_probe',return_value=dict(exit=0,stdout=' COM_ARM_ON_BOOT [1,2] : 1\n')):
            result = sl.shutdown_review(self.path,self.state)
        self.assertEqual(result['result'],'NO_GO')
        self.assertIsNone(result['clock_proposal'])

    def test_existing_native_attempt_or_pending_request_cannot_be_retried(self):
        for key,value in [('state','CLEANUP_REQUIRED'),('shutdown_result',dict(exit=None))]:
            changed = dict(self.state,**{key:value})
            self.assertEqual(sl.shutdown_review(self.path,changed)['result'],'NO_GO')
        (self.path/'shutdown.request').touch()
        self.assertEqual(sl.shutdown_review(self.path,self.state)['result'],'NO_GO')

    def test_running_review_can_queue_only_with_progress(self):
        self.host.stop_gate.return_value.update(paused=False,can_shutdown=True,clock_progressing=True,
                                              world_before=world(paused=False),world=world(10_050_000,False))
        self.assertTrue(sl.shutdown_review(self.path,self.state)['shutdown_request_allowed'])

    def test_cli_refuses_paused_request_before_queueing_to_old_owner(self):
        with patch.object(sl,'read_session',return_value=(self.path,self.state)),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sl.main(['shutdown',str(self.path),'--execute']),2)
        self.assertFalse((self.path/'shutdown.request').exists())


class InventoryTests(unittest.TestCase):
    def test_readonly_requester_excludes_only_verified_supervisor_not_foreign_controller(self):
        host = sl.Host()
        owner_line = '700 python3 python3 -B -m pso_flight_demo.session_launcher start --execute'
        foreign_line = '900 python3 python3 -m pso_flight_demo follow route --execute'
        for extra,blocked in [('',False),('\n'+foreign_line,True)]:
            outputs = [dict(exit=0,stderr='',stdout=owner_line+extra),dict(exit=0,stderr='',stdout='')]
            with patch.object(sl,'identity',return_value=ident(700)), \
                    patch('pso_flight_demo.runtime_readiness.probe',side_effect=outputs):
                if blocked:
                    with self.assertRaisesRegex(ValueError,'Stage conflict'):
                        host.stage_conflicts(load_profile(),[],True,supervisor=ident(700))
                else:
                    host.stage_conflicts(load_profile(),[],True,supervisor=ident(700))

    def test_supervisor_exclusion_cannot_hide_reused_pid(self):
        outputs = [dict(exit=0,stderr='',stdout=''),dict(exit=0,stderr='',stdout='')]
        with patch.object(sl,'identity',return_value=ident(700,start_ticks=999)), \
                patch('pso_flight_demo.runtime_readiness.probe',side_effect=outputs):
            with self.assertRaisesRegex(ValueError,'Supervisor identity changed'):
                sl.Host().stage_conflicts(load_profile(),[],True,supervisor=ident(700))

    def test_pid_reuse_and_unrecorded_descendants_are_detected(self):
        state = state_fixture()
        live = {r['pid']:r for r in [state['supervisor'],*(c['owner'] for c in state['children'])]}
        live[802] = dict(live[802],start_ticks=999)
        live[999] = ident(999,sid=800,ppid=800)
        with patch.object(Path,'iterdir',return_value=[Path('/proc')/str(pid) for pid in live]), \
                patch.object(sl,'identity',side_effect=lambda pid:live.get(pid)):
            result = sl.session_inventory(state)
        self.assertEqual(result['identity_mismatches'],[802])
        self.assertEqual(result['unrecorded_child_tasks'],[999])

    def test_cached_stopped_never_proves_exit(self):
        state = state_fixture(); state['state'] = 'STOPPED'
        for inventory,precheck in [
            (dict(alive=[802],identity_mismatches=[],unrecorded_child_tasks=[]),dict(blockers=[])),
            (dict(alive=[],identity_mismatches=[],unrecorded_child_tasks=[999]),dict(blockers=[])),
            (dict(alive=[],identity_mismatches=[],unrecorded_child_tasks=[]),dict(blockers=['port occupied']))]:
            with patch.object(sl,'session_inventory',return_value=inventory), \
                    patch.object(sl,'host_startup_check',return_value=precheck):
                self.assertFalse(sl.stopped_check(state)['restart_allowed'])

    def test_only_all_exited_and_host_clear_allows_start_candidate(self):
        state = state_fixture(); state['state'] = 'STOPPED'
        with patch.object(sl,'session_inventory',return_value=dict(alive=[],identity_mismatches=[],unrecorded_child_tasks=[])), \
                patch.object(sl,'host_startup_check',return_value=dict(blockers=[])):
            self.assertTrue(sl.stopped_check(state)['restart_allowed'])

    def test_verify_stopped_timeout_is_bounded_and_readonly(self):
        with patch.object(sl,'read_session',return_value=(Path('/unused'),{})), \
                patch.object(sl,'stopped_check',return_value=dict(restart_allowed=False,result='NO_GO')) as check, \
                patch.object(sl.time,'monotonic',side_effect=[0,2]), \
                patch.object(sl,'Host',side_effect=AssertionError('No lifecycle host mutations')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sl.main(['verify-stopped','/unused','--timeout','1']),1)
        self.assertEqual(check.call_count,1)


class SupervisorSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=MODULE)
        self.addCleanup(self.temp.cleanup)
        profile = load_profile()
        self.host = FakeHost()
        self.supervisor = sl.Supervisor(profile,launch_plan(profile),Path(self.temp.name),host=self.host,timeout=1)
        self.assertTrue(self.supervisor.start())

    def test_identity_change_during_native_checks_blocks_shutdown(self):
        def gate(profile):
            self.host.new_identity = dict(self.host.children[-1].owner,start_ticks=200)
            return self.host.gate
        self.host.stop_gate = gate
        self.assertFalse(self.supervisor.shutdown())
        self.assertNotIn('shutdown_PX4',self.host.events)
        self.assertFalse(self.host.signals)

    def test_missing_progress_evidence_is_not_true_by_default(self):
        self.host.gate = dict(safe=True,paused=False)
        self.assertFalse(self.supervisor.shutdown())
        self.assertFalse(self.host.signals)
        self.assertNotIn('shutdown_PX4',self.host.events)

    def test_px4_descendant_timeout_preserves_agent_and_gazebo(self):
        original = self.host.members
        self.host.members = lambda child: ([child.owner] if child.role == 'PX4' else original(child))
        with patch.object(sl.time,'monotonic',side_effect=[0,2]):
            self.assertFalse(self.supervisor.shutdown())
        self.assertFalse(self.host.signals)
        self.assertEqual(self.host.events.count('shutdown_PX4'),1)
        self.assertIn('descendants',self.supervisor.state['error'])


if __name__ == '__main__': unittest.main()
