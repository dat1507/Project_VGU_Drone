"""Read-only finite-window handshake. No Gazebo control client or publisher.

Status is mutable only inside this invocation's NEW output directory; events and
operator challenges are append/exclusive-create. Approval never triggers physics.
"""

from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import time
from uuid import uuid4

from .dds_evidence import REQUIRED
from .session_launcher import identity, same_process

APPROVAL = 'APPROVE_CLOCK_20000_25000'


def publish_once(path, value):
    """Publish complete challenge/response atomically, refusing prior artifacts."""
    path = Path(path)
    temporary = path.parent/('handshake_'+uuid4().hex+'.tmp')
    try:
        with temporary.open('x') as f:
            json.dump(value, f, indent=2, allow_nan=False)
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def graph_problems(graph):
    problems = []
    for key in REQUIRED:
        matches = [v for name,v in graph.items() if name.startswith('/fmu/out/'+key)
                   and (name == '/fmu/out/'+key or name.startswith('/fmu/out/'+key+'_v'))]
        if len(matches) != 1 or matches[0].get('publishers') != 1:
            problems.append('GRAPH_REQUIRED_'+key)
        elif (len(matches[0].get('publisher_gids', [])) != 1 or
              matches[0].get('actual_types') != [matches[0].get('expected_type')]):
            problems.append('GRAPH_SCHEMA_OR_GID_'+key)
    for key in ('offboard_control_mode', 'trajectory_setpoint', 'vehicle_command'):
        matches = [v for name,v in graph.items() if name == '/fmu/in/'+key]
        if len(matches) != 1 or matches[0].get('control_publishers') != 0 or matches[0].get('px4_readers') != 1:
            problems.append('CONTROL_GRAPH_'+key)
    return problems


class WindowProtocol:
    """Pure state machine; source data is supplied by read-only adapters."""
    def __init__(self, baseline):
        self.baseline = baseline
        self.state = 'START_DDS_AND_GAZEBO_OBSERVERS'
        self.events = []
        self.problems = []
        self.deadline = None
        self.ready_at = None
        self.approved_at = None
        self.complete_at = None
        for stage in ('CHECK_SESSION','CHECK_PX4_DISARMED_LANDED','CHECK_GAZEBO_PAUSED'):
            self.transition(stage, baseline.get('captured_monotonic', time.monotonic()))
        self.state = 'CHECK_NO_CONTROL_PUBLISHERS'

    def transition(self, state, now):
        self.state = state
        self.events.append(dict(state=state, monotonic=now))

    def fail(self, reason, now):
        if reason not in self.problems:
            self.problems.append(reason)
        if self.state != 'NO_GO':
            self.transition('NO_GO', now)

    def native(self, sample, now):
        if self.state == 'NO_GO':
            return
        if not isinstance(sample.get('iterations'), int) or not isinstance(sample.get('clock_us'), int):
            self.fail('NATIVE_CLOCK_MALFORMED', now)
            return
        for key in ('runtime_identity','session_id','profile_sha256','sdf_sha256'):
            if sample.get(key) != self.baseline.get(key):
                self.fail('SESSION_CHANGED', now)
        if (not sample.get('disarmed') or not sample.get('landed') or
                not sample.get('auto_arm_disabled') or sample.get('errors')):
            self.fail('NATIVE_SAFETY_OR_READ_FAILURE', now)
        if sample.get('reset_counters') != self.baseline.get('reset_counters'):
            self.fail('EKF_RESET_DURING_WINDOW', now)
        if (not math.isfinite(sample.get('heading', float('nan'))) or
                sample.get('heading_good_for_control') != self.baseline.get('heading_good_for_control') or
                abs((sample.get('heading', 0)-self.baseline.get('heading', 0)+math.pi) % (2*math.pi)-math.pi) > .15):
            self.fail('HEADING_CHANGED_DURING_WINDOW', now)
        if self.problems:
            return
        start = sample.get('iterations') == 20000 and sample.get('clock_us') == 20_000_000
        end = sample.get('iterations') == 25000 and sample.get('clock_us') == 25_000_000
        if self.ready_at is None:
            if not start or not sample.get('paused'):
                self.fail('PHYSICS_BEFORE_SUBSCRIPTIONS_READY', now)
        elif self.approved_at is None:
            if not start or not sample.get('paused'):
                self.fail('PHYSICS_WITHOUT_OPERATOR_APPROVAL_HANDSHAKE', now)
        elif end and sample.get('paused'):
            if self.complete_at is None and self.state != 'NO_GO':
                self.complete_at = now
                self.transition('VERIFY_EXACT_CLOCK_PROGRESS', now)
        elif start and sample.get('paused'):
            pass
        elif (not sample.get('paused') and 20000 < sample.get('iterations', -1) <= 25000 and
              20_000_000 < sample.get('clock_us', -1) <= 25_000_000):
            self.transition('CLOCK_WINDOW_EXECUTED_BY_OPERATOR', now)
        else:
            self.fail('WRONG_CLOCK_PROGRESS', now)

    def ready(self, graph, sample, gazebo_created, dds_created, now):
        self.native(sample, now)
        if any(v.get('control_publishers', 0) > 0 for v in graph.values()):
            self.fail('UNEXPECTED_CONTROL_PUBLISHER', now)
        if self.state == 'NO_GO' or not gazebo_created or not set(REQUIRED).issubset(dds_created):
            return False
        if graph_problems(graph):
            return False  # discovery may still be in progress; bounded deadline applies
        self.ready_at = now
        self.transition('SUBSCRIPTIONS_READY', now)
        self.transition('WAIT_FOR_OPERATOR_APPROVAL', now)
        return True

    def approve(self, phrase, sample, now):
        self.native(sample, now)
        if (phrase != APPROVAL or self.state != 'WAIT_FOR_OPERATOR_APPROVAL' or
                self.deadline is None or self.deadline-now < 12):
            raise ValueError('Observer not ready/alive with 12 seconds remaining, or approval phrase mismatch')
        self.approved_at = now
        self.events.append(dict(state='OPERATOR_APPROVAL_RECORDED_NO_PHYSICS_EXECUTED', monotonic=now))

    def finish(self, now):
        if self.state == 'NO_GO':
            return
        if self.complete_at is None:
            self.fail('OBSERVER_TIMEOUT_MID_WINDOW' if self.state == 'CLOCK_WINDOW_EXECUTED_BY_OPERATOR'
                      else 'OBSERVER_TIMEOUT_NO_CONFIRMED_CLOCK', now)
        else:
            self.transition('STOP_AND_SAVE_OBSERVATIONS', now)


class WindowMonitor:
    """Nonblocking ROS spin hook. Native checks run in one bounded background job."""
    def __init__(self, directory, baseline, native_probe, gazebo, plan):
        self.directory = Path(directory)
        self.protocol = WindowProtocol(baseline)
        self.native_probe = native_probe
        self.gazebo = gazebo
        self.plan = plan
        self.owner = identity(os.getpid())
        if self.owner is None:
            raise ValueError('Cannot establish observer process identity')
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.latest = None
        self.created = []
        self.last_poll = -1.
        self.last_status = -1.
        self.saved_events = 0
        self.request = None
        self.served = set()
        self.reported_failure = False
        self.ready_gids = None

    def pre_subscriptions_check(self, controls):
        if len(controls) != 3 or any(v.get('control_publishers') != 0 for v in controls.values()):
            raise ValueError('Unexpected control publishers before subscriptions')
        now = time.monotonic()
        self.protocol.transition('CHECK_NO_CONTROL_PUBLISHERS', now)
        self.protocol.transition('START_DDS_AND_GAZEBO_OBSERVERS', now)
        self.gazebo.start()
        if not self.gazebo.subscribed:
            raise ValueError('Gazebo subscriber was not created')

    def subscriptions_created(self, keys):
        self.created = list(keys)

    def begin(self, deadline):
        self.protocol.deadline = deadline

    def status(self, now):
        if self.protocol.state == 'NO_GO' and not self.reported_failure:
            print('NO_GO: '+', '.join(self.protocol.problems)+
                  '. Do not issue another clock request. If physics is running, OPERATOR must Pause; observer cannot control it.', flush=True)
            self.reported_failure = True
        status = dict(schema='finite_clock_handshake_v1', owner=self.owner,
            state=self.protocol.state, ready_at=self.protocol.ready_at,
            approved_at=self.protocol.approved_at, heartbeat_monotonic=now,
            deadline_monotonic=self.protocol.deadline, native=self.latest,
            problems=self.protocol.problems, operator_command=self.plan['operator_command'],
            ready_publisher_gids=self.ready_gids,
            rollback_command=self.plan['rollback_command'], live_control_go=False, controls_physics=False)
        temporary = self.directory/('status_'+uuid4().hex+'.tmp')
        with temporary.open('x') as f:
            json.dump(status, f, indent=2, allow_nan=False)
        os.replace(temporary, self.directory/'observer_status.json')
        with (self.directory/'coordination_events.jsonl').open('a') as f:
            for event in self.protocol.events[self.saved_events:]:
                f.write(json.dumps(event)+'\n')
        self.saved_events = len(self.protocol.events)

    def update(self, graph, now, errors):
        if errors or self.gazebo.errors:
            self.protocol.fail('OBSERVER_MESSAGE_OR_GROUNDTRUTH_ERROR', now)
        if self.protocol.ready_at is not None and graph_problems(graph):
            self.protocol.fail('GRAPH_OR_CONTROL_PUBLISHER_CHANGED', now)
        gids = {name:v.get('publisher_gids') for name,v in graph.items()
                if any(name == '/fmu/out/'+k or name.startswith('/fmu/out/'+k+'_v') for k in REQUIRED)}
        if self.ready_gids is not None and gids != self.ready_gids:
            self.protocol.fail('DDS_PARTICIPANT_OR_GID_CHANGED', now)
        for path in self.directory.glob('operator_check_*.json'):
            if path.name not in self.served and self.request is None:
                self.request = json.loads(path.read_text())
                self.served.add(path.name)
        if self.future is not None and self.future.done():
            try:
                self.latest = self.future.result()
                self.protocol.native(self.latest, now)
                if self.protocol.ready_at is None:
                    if self.protocol.ready(graph, self.latest, self.gazebo.subscribed, self.created, now):
                        self.ready_gids = gids
                        print('SUBSCRIPTIONS_READY: '+str(self.directory.resolve())+
                              ' — observer spinning; wait for separate operator approval', flush=True)
                if self.request is not None and self.latest['probe_started_monotonic'] >= self.request['created_monotonic']:
                    response = dict(nonce=self.request['nonce'], ok=False, controls_physics=False)
                    try:
                        if graph_problems(graph):
                            raise ValueError('Current ROS graph/control publishers failed')
                        self.protocol.approve(self.request.get('approval'), self.latest, now)
                        response.update(ok=True, checked_monotonic=now, native=self.latest, graph=graph)
                    except ValueError as exc:
                        response['error'] = str(exc)
                    publish_once(self.directory/('operator_response_'+self.request['nonce']+'.json'), response)
                    self.request = None
            except (ValueError, OSError, KeyError, RuntimeError) as exc:
                self.protocol.fail('NATIVE_CHECK_FAILED: '+str(exc), now)
            self.future = None
        if self.protocol.state == 'NO_GO':
            self.status(now)
            return False
        if self.future is None and now-self.last_poll >= .25:
            self.last_poll = now
            self.future = self.pool.submit(self.native_probe)
        if now-self.last_status >= .2:
            self.last_status = now
            self.status(now)
        # Allow DDS callbacks to drain after stats confirms exact endpoint.
        return self.protocol.complete_at is None or now-self.protocol.complete_at < .25

    def finish(self, observed):
        self.protocol.finish(time.monotonic())
        self.status(time.monotonic())
        self.pool.shutdown(wait=True, cancel_futures=True)  # each native probe has bounded timeouts
        observed['coordination'] = dict(state=self.protocol.state, problems=self.protocol.problems,
            events=self.protocol.events, physics_executed_by_observer=False)


def check_operator_window(directory, approval, read_native, monotonic=time.monotonic):
    """Terminal C: challenge the spinning observer; print control text, never run it."""
    directory = Path(directory)
    if approval != APPROVAL:
        raise ValueError('Operator must explicitly supply '+APPROVAL+'; no default approval')
    status = json.loads((directory/'observer_status.json').read_text())
    now = monotonic()
    if (status['state'] != 'WAIT_FOR_OPERATOR_APPROVAL' or
            not 0 <= now-status['heartbeat_monotonic'] <= 1 or
            status['deadline_monotonic']-now < 12 or
            not same_process(status['owner'], identity(status['owner']['pid']))):
        raise ValueError('Observer stopped, stale, wrong phase, or not enough time remaining; NO_GO, do not issue physics')
    # Native recheck on Terminal C too; this never grants observer control rights.
    native = read_native()
    if (native.get('errors') or not native.get('disarmed') or not native.get('landed') or
            not native.get('auto_arm_disabled') or
            not native.get('paused') or native.get('iterations') != 20000 or native.get('clock_us') != 20_000_000 or
            any(native.get(k) != status['native'].get(k) for k in ('runtime_identity','session_id','profile_sha256','sdf_sha256'))):
        raise ValueError('Host no longer matches the approved 20000/20s baseline')
    nonce = uuid4().hex
    request = dict(nonce=nonce, approval=approval, created_monotonic=monotonic())
    publish_once(directory/('operator_check_'+nonce+'.json'), request)
    until = monotonic()+6
    response_path = directory/('operator_response_'+nonce+'.json')
    while monotonic() < until:
        if response_path.exists():
            response = json.loads(response_path.read_text())
            current = json.loads((directory/'observer_status.json').read_text())
            if (not response.get('ok') or response.get('nonce') != nonce or
                    monotonic()-response['checked_monotonic'] > 1 or
                    current['deadline_monotonic']-monotonic() < 8 or
                    current['state'] != 'WAIT_FOR_OPERATOR_APPROVAL' or
                    not same_process(current['owner'], identity(current['owner']['pid']))):
                raise ValueError('Fresh challenge response failed; NO_GO')
            return dict(result='READY_FOR_SEPARATE_CLOCK_APPROVAL', observer_challenge=response,
                        instruction='Operator must directly run this command NOW in Terminal C after approval; observer will not execute it.',
                        operator_command=status['operator_command'], rollback_command=status['rollback_command'],
                        live_control_go=False, controls_physics=False)
        time.sleep(.05)
    raise ValueError('Observer challenge timed out; NO_GO, no automatic retry')
