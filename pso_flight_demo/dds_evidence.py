"""ROS-independent evidence accounting. Receive index is NOT a PX4 sequence."""

import math
import re

REQUIRED = ('vehicle_local_position', 'vehicle_status', 'vehicle_attitude',
            'vehicle_land_detected', 'estimator_status_flags', 'timesync_status')


def normalize_epochs(messages):
    """Normalize saved DDS samples without pairing across an offset reset.

    PX4 applies -estimated_offset on serialization. A receive-time bracket
    crossing PX4's 100 ms clock-jump threshold is ambiguous: do NOT choose the
    offset which happens to fit the requested native clock window.
    This proves neither UTC convergence nor session ownership.
    """
    sync = messages.get('timesync_status', {}).get('samples', [])
    normalized, problems, epochs = {}, [], []
    try:
        if len(sync) < 2:
            raise ValueError('TIMESYNC_SAMPLES_MISSING')
        for i, s in enumerate(sync):
            if s['source_protocol'] != 2 or not math.isfinite(s['received_monotonic']):
                raise ValueError('TIMESYNC_SOURCE')
            offset = int(s['estimated_offset_us'])
            if i and s['received_monotonic'] < sync[i-1]['received_monotonic']:
                raise ValueError('TIMESYNC_RECEIVE_ORDER')
            if i == 0 or abs(offset-int(sync[i-1]['estimated_offset_us'])) > 100_000:
                epochs.append(dict(received_monotonic=s['received_monotonic'], offset_us=offset))
        for key in REQUIRED:
            times = []
            for s in messages[key]['samples']:
                if key == 'timesync_status':
                    offset = s['estimated_offset_us']
                else:
                    wall = s['received_monotonic']
                    left = [t for t in sync if t['received_monotonic'] <= wall]
                    right = [t for t in sync if t['received_monotonic'] >= wall]
                    if left and right and abs(left[-1]['estimated_offset_us']-
                                              right[0]['estimated_offset_us']) > 100_000:
                        raise ValueError('AMBIGUOUS_RESET_BOUNDARY:'+key)
                    nearest = min(sync, key=lambda t: abs(t['received_monotonic']-wall))
                    if abs(nearest['received_monotonic']-wall) > .5:
                        raise ValueError('NO_NEARBY_TIMESYNC:'+key)
                    offset = nearest['estimated_offset_us']
                stamp = int(s['timestamp']) + int(offset)
                if stamp < 0 or (times and stamp < times[-1]):
                    raise ValueError('NATIVE_TIMESTAMP_REVERSED:'+key)
                times.append(stamp)
            if len(times) < 2 or times[-1] <= times[0]:
                raise ValueError('NATIVE_TIMESTAMP_FROZEN:'+key)
            normalized[key] = times
    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
        problems.append(str(exc))
    return dict(coherent=not problems, problems=problems, epochs=epochs,
                native_timestamps=normalized, utc_convergence_proven=False)


class LiveEpochClock:
    """No control. Strict live mapping; an offset reset invalidates the mission.

    Startup may reacquire a stable clock BEFORE publishing. In flight callers
    must abort on epoch changes, backwards native time or excessive uncertainty.
    """
    def __init__(self):
        self.latest = None
        self.epoch = 0
        self.stable_count = 0
        self.last_native = {}
        self.last_wire = {}

    def update(self, timestamp, offset, observed_offset, rtt, protocol, wall):
        if protocol != 2 or not all(math.isfinite(v) for v in
                (timestamp, offset, observed_offset, rtt, wall)) or timestamp <= 0 or rtt < 0:
            self.stable_count = 0
            raise ValueError('Invalid DDS timesync sample')
        if self.latest and abs(offset-self.latest['offset']) > 100_000:
            self.epoch += 1
            self.stable_count = 0
            self.last_native.clear()
            self.last_wire.clear()
        usable = abs(observed_offset-offset) <= 100_000 and rtt < 10_000
        native = int(timestamp)+int(offset)
        if native < 0:
            self.stable_count = 0
            raise ValueError('Negative native clock')
        previous = self.latest
        advanced = not previous or native > previous['native']
        if previous and native < previous['native']:
            self.stable_count = 0
            raise ValueError('Timesync native clock reversed')
        self.stable_count = self.stable_count+1 if usable and advanced else (0 if not usable else self.stable_count)
        self.latest = dict(offset=int(offset), native=native, wall=wall, usable=usable)

    def normalize(self, key, timestamp, now):
        s = self.latest
        if not s or not s['usable'] or self.stable_count < 3 or not 0 <= now-s['wall'] <= 1.5:
            raise ValueError('DDS timesync not freshly stable')
        if self.last_wire.get(key) == int(timestamp):
            return self.last_native[key]  # never reinterpret an old packet with a new offset
        native = int(timestamp)+s['offset']
        # A newer source sample can precede the next 1 Hz timesync status.
        if native <= 0 or not -1_500_000 <= native-s['native'] <= 1_500_000:
            raise ValueError('DDS source outside current clock epoch')
        if key in self.last_native and native < self.last_native[key]:
            raise ValueError('DDS native timestamp reversed')
        self.last_native[key] = native
        self.last_wire[key] = int(timestamp)
        return native


class MessageEvidence:
    def __init__(self):
        self.messages = {}

    def record(self, key, timestamp, wall, timestamp_sample=None, **fields):
        timestamp = int(timestamp)
        if timestamp <= 0:
            raise ValueError('Nonpositive source timestamp')
        item = self.messages.setdefault(key, dict(count=0, first_timestamp=timestamp,
            last_timestamp=timestamp, backwards=False, distinct=0, samples=[]))
        old = item['last_timestamp']
        item['count'] += 1
        item['backwards'] |= timestamp < old
        item['distinct'] += int(item['count'] == 1 or timestamp > old)
        item.update(last_timestamp=timestamp, **fields)
        if len(item['samples']) >= 10000:
            raise ValueError('Evidence sample capacity reached')
        item['samples'].append(dict(receive_index=item['count'], source_sequence=None,
            timestamp=timestamp, timestamp_sample=timestamp_sample, received_monotonic=wall))

    def summarize(self, graph, errors):
        received = any(v['count'] for v in self.messages.values())
        progressing = {k: bool((m := self.messages.get(k)) and m['distinct'] >= 2 and
                        m['last_timestamp'] > m['first_timestamp'] and not m['backwards'])
                       for k in REQUIRED}
        sync = self.messages.get('timesync_status', {}).get('samples', [])
        epochs = normalize_epochs(self.messages) if sync and all(
            'estimated_offset_us' in s and 'source_protocol' in s for s in sync) else None
        if epochs is not None:
            progressing = {k: bool(epochs['coherent'] and k in epochs['native_timestamps']) for k in REQUIRED}
        level = ('TIMESTAMP_PROGRESSING' if all(progressing.values()) and not errors else
                 'MESSAGE_RECEIVED' if received else
                 'GRAPH_ONLY' if any(v.get('publishers', 0) for v in graph.values()) else 'NO_DATA')
        return dict(dds_evidence_level=level, dds_entities_seen=any(v.get('publishers', 0) for v in graph.values()),
                    dds_messages_received=received, required_topic_progress=progressing,
                    source_clock_progressed=all(progressing.values()),
                    native_clock_progression_proven=False, session_data_bound=False, live_control_go=False,
                    timestamp_basis='NATIVE_EPOCHS' if epochs is not None else 'RAW_UNBOUND',
                    epoch_assessment=epochs,
                    progression_note='DDS source timestamps include XRCE offset; native sim clock must independently progress')


def assess_window(before, after, observed, groundtruth, software):
    """Fail closed on session/clock/schema mismatch; never claim flight readiness.

    Native source time = DDS timestamp + TimesyncStatus.estimated_offset:
    XRCE session.time_offset = -Timesync.offset(), see installed PX4 source.
    Reject receive-time brackets crossing a reset; never subtract an assumed epoch.
    """
    from .telemetry import simulation_us, gcs_progress
    from .runtime_readiness import topic_fields, number
    reasons = []
    keys = ('boot_id','px4_pid','px4_start_ticks','gazebo_server_pid','gazebo_start_ticks')
    if (not before.get('session_owner_verified') or not after.get('session_owner_verified') or
            any(before['runtime_identity'].get(k) is None or before['runtime_identity'][k] != after['runtime_identity'].get(k) for k in keys)):
        reasons.append('SESSION_IDENTITY')
    if (before.get('session_id') != after.get('session_id') or before.get('profile_sha256') != after.get('profile_sha256') or
            before['world']['sdf_sha256'] != after['world']['sdf_sha256']):
        reasons.append('SESSION_PROFILE_SDF')
    if before.get('problems') or after.get('problems'):
        reasons.append('NATIVE_GATES')
    for snapshot in (before, after):
        armed = topic_fields(snapshot['evidence']['actuator_armed']['stdout'])
        land = topic_fields(snapshot['evidence']['vehicle_land_detected']['stdout'])
        if armed.get('armed') != 'False' or land.get('landed') != 'True':
            reasons.append('PX4_NOT_DISARMED_LANDED')
    start = simulation_us(before['evidence']['stats']['stdout'])
    end = simulation_us(after['evidence']['stats']['stdout'])
    if start is None or end is None or end-start != 5_000_000:
        reasons.append('EXACT_5S_NATIVE_CLOCK')
    stats_pair = [r['evidence']['stats']['stdout'] for r in (before, after)]
    iterations = [re.search(r'^iterations: (\d+)$', s, re.M) for s in stats_pair]
    if (not all(iterations) or int(iterations[1][1])-int(iterations[0][1]) != 5000 or
            any('paused: true' not in s for s in stats_pair)):
        reasons.append('PAUSED_AND_EXACT_5000_STEPS')
    if not gcs_progress(before['gcs_evidence'], after['gcs_evidence']):
        reasons.append('GCS_RX_AND_TIMESTAMP')
    if observed.get('dds_evidence_level') != 'TIMESTAMP_PROGRESSING' or observed.get('errors'):
        reasons.append('REQUIRED_DDS_DATA')
    if not software.get('schemas') or software.get('errors') or not all(
            s['serialized_fields_match'] and s['built_version_matches'] for s in software['schemas'].values()):
        reasons.append('INSTALLED_BUILT_SCHEMA')
    publishers = [v for v in observed['graph'].values() if 'publishers' in v]
    if not publishers or any(v['publishers'] != 1 or len(v.get('publisher_gids', [])) != 1 for v in publishers):
        reasons.append('SINGLE_PUBLISHER_PER_TOPIC')
    if any(v.get('control_publishers', 0) != 0 for v in observed['graph'].values()):
        reasons.append('UNEXPECTED_CONTROL_PUBLISHER')
    prefixes = {tuple(v['publisher_gids'][0][:12]) for v in publishers if len(v.get('publisher_gids', [])) == 1}
    if len(prefixes) != 1 or any(not any(p) for p in prefixes):
        reasons.append('DDS_PARTICIPANT')
    messages = observed['messages']
    status = messages.get('vehicle_status', {})
    if status.get('system_id') != 1 or status.get('component_id') != 1:
        reasons.append('PX4_SYSID_COMPONENT')
    sync = messages.get('timesync_status', {}).get('samples', [])
    normalized = {}
    try:
        epoch_result = normalize_epochs(messages)
        if not epoch_result['coherent']:
            raise ValueError('Ambiguous/missing native timestamp epoch')
        for key in REQUIRED:
            stamps = epoch_result['native_timestamps'][key]
            if (len(stamps) < 2 or not all(math.isfinite(s) for s in stamps) or
                    any(b < a for a,b in zip(stamps,stamps[1:])) or
                    stamps[-1]-stamps[0] < 3_000_000 or
                    any(b-a > 1_500_000 for a,b in zip(stamps,stamps[1:])) or
                    start is None or end is None or min(stamps) < start-50_000 or max(stamps) > end+50_000):
                raise ValueError('Normalized source timestamps inconsistent with finite native window')
            normalized[key] = dict(first_native_us=stamps[0], last_native_us=stamps[-1])
        b = topic_fields(before['evidence']['vehicle_local_position']['stdout'])
        a = topic_fields(after['evidence']['vehicle_local_position']['stdout'])
        from .frames import RESET_FIELDS
        resets = [int(number(a,k)) for k in RESET_FIELDS]
        if resets != [int(number(b,k)) for k in RESET_FIELDS] or any(s['resets'] != resets for s in messages['vehicle_local_position']['samples']):
            raise ValueError('EKF reset in clock window')
        # The 0.15 rad preparation tolerance is unchanged; no yaw correction.
        if (abs((number(a,'heading')-number(b,'heading')+math.pi) % (2*math.pi)-math.pi) > .15 or
                a.get('heading_good_for_control') != b.get('heading_good_for_control')):
            raise ValueError('Heading changed in clock window')
        last = messages['vehicle_local_position']['samples'][-1]['position_ned']
        if sum((last[i]-number(a,k))**2 for i,k in enumerate(('x','y','z'))) > .02**2:
            raise ValueError('DDS/native EKF position mismatch')
    except (KeyError, TypeError, ValueError, IndexError):
        reasons.append('DDS_NATIVE_TIME_POSITION_OR_RESET_CORRELATION')
    poses = groundtruth.get('samples', [])
    if (groundtruth.get('errors') or len(poses) < 2 or
            poses[-1]['native_timestamp_us']-poses[0]['native_timestamp_us'] < 3_000_000 or
            len({p['entity_id'] for p in poses}) != 1 or
            any(b['native_timestamp_us'] < a['native_timestamp_us'] for a,b in zip(poses,poses[1:])) or
            start is None or end is None or any(not start-50000 <= p['native_timestamp_us'] <= end+50000 for p in poses)):
        reasons.append('INDEPENDENT_GROUNDTRUTH_WINDOW')
    else:
        from .runtime_readiness import pose_xyz
        try:
            endpoint_errors = [sum((x-y)**2 for x,y in zip(pose_xyz(r['evidence']['pose']['stdout']),p['position_enu']))
                               for r,p in ((before,poses[0]),(after,poses[-1]))]
            if any(e > .02**2 or not math.isfinite(e) for e in endpoint_errors):
                reasons.append('GROUNDTRUTH_NATIVE_POSE_MISMATCH')
        except (ValueError,KeyError,TypeError):
            reasons.append('GROUNDTRUTH_NATIVE_POSE_MISMATCH')
    return dict(result='PASS_FINITE_CLOCK_EVIDENCE_ONLY' if not reasons else 'NO_GO', problems=reasons,
                normalized_source_windows=normalized, session_data_bound=not reasons,
                binding_scope='single owned local PX4 + same DDS participant/sysid + native time/pose/reset correlation; not cryptographic authentication',
                live_control_go=False, flight_ready=False)
