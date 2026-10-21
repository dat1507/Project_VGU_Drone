"""Replay certified raw ENU polylines as visuals, never as flight commands.

Dry-run has no Gazebo imports. Execute and cleanup require interactive approval.
Only /create, /set_pose, /remove and /scene/info are accessible to this adapter.
Physics must remain paused and its clock / iterations must remain unchanged.
"""

import argparse
import bisect
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from .ownership import (DRONE, PATH_MODEL, OWN_NAMES, read_json, read_manifest,
                        sdf_proof, valid_entry, verify_scene_model)

PROJECT = Path(__file__).resolve().parents[2]
SOURCE = PROJECT / 'pso_flight_demo/outputs_c2/warehouse/offline_pso_20261019T021207Z/validated_run'
OUTPUT = PROJECT / 'pso_flight_demo/outputs_visualization'
WORLD = 'warehouse_world'
HEADER = ('x_enu_m', 'y_enu_m', 'z_enu_m')
APPROVAL = 'VISUALIZATION ONLY'
SERVICE_TYPES = {'create':('EntityFactory','Boolean'), 'set_pose':('Pose','Boolean'),
                 'remove':('Entity','Boolean'), 'scene/info':('Empty','Scene')}
TRANSPORT_CLI = Path('/usr/libexec/gz/transport13/gz-transport-service')
TRANSPORT_ENV = ('GZ_PARTITION','GZ_IP','GZ_DISCOVERY_MSG_PORT','GZ_DISCOVERY_SRV_PORT',
                 'GZ_DISCOVERY_MULTICAST_IP','GZ_CONFIG_PATH','GZ_VERBOSE')


class TransportError(ValueError):
    def __init__(self, code, detail):
        self.code = code
        super().__init__(f'{code}: {detail}; no automatic retry')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def parse_csv(text):
    rows = csv.reader(io.StringIO(text))
    if tuple(next(rows, ())) != HEADER:
        raise ValueError('CSV header must be exactly x_enu_m,y_enu_m,z_enu_m (ENU metres)')
    points = []
    for row in rows:
        if len(row) != 3:
            raise ValueError('Every waypoint must contain exactly three coordinates')
        point = tuple(float(v) for v in row)
        if not all(math.isfinite(v) for v in point):
            raise ValueError('Non-finite waypoint')
        points.append(point)
    if len(points) < 2:
        raise ValueError('Need at least two waypoints')
    return tuple(points)


def lengths(points):
    return tuple(math.dist(a, b) for a, b in zip(points, points[1:]))


@dataclass(frozen=True)
class Route:
    algorithm: str
    points: tuple
    provenance: dict

    @property
    def length(self):
        return sum(lengths(self.points))


def load_routes(source=SOURCE, *, validate_geometry=True):
    """Check the frozen export index, trial certificate and current SDF geometry.

    This only validates geometry; it does not import or run any optimizer.
    The SITL demo-only controller may opt out of geometry revalidation while
    retaining every frozen CSV/certificate/hash/schema/provenance check.
    """
    from pso_flight_demo.environments import load_profile, load_environment

    source = Path(source)
    index = json.loads((source / 'artifact_sha256.json').read_text())

    def checked(name):
        data = (source / name).read_bytes()
        if sha(data) != index.get(name):
            raise ValueError(f'Export hash mismatch: {name}')
        return data

    config = json.loads(checked('config.json'))
    if (config['frame'], config['units'], config['radius_m'], config['margin_m']) != (
            'GAZEBO_WORLD_ENU', 'm', .60, .25):
        raise ValueError('Unreviewed frame / envelope')
    profile = load_profile('warehouse')
    world, environment = load_environment(profile)
    profile_hash = sha((PROJECT / 'pso_flight_demo/profiles/warehouse.yaml').read_bytes())
    routes = {}
    for algorithm, selected in [('standard', 'selected_baseline'), ('enhanced', 'selected_enhanced')]:
        prefix = f'{selected}/'
        meta = json.loads(checked(prefix + 'metadata.json'))
        planner = 'baseline' if algorithm == 'standard' else 'enhanced'
        trial = json.loads(checked(f'{planner}_seed{meta["seed"]}/metadata.json'))
        raw = checked(prefix + 'raw_enu.csv')
        points = parse_csv(raw.decode('utf-8'))
        cert = trial['raw_certificate']
        if (meta['algorithm'] != planner or trial['algorithm'] != planner or
                trial['seed'] != meta['seed'] or meta['result'] != 'OFFLINE_GEOMETRY_GO' or
                meta['csvs']['raw_enu.csv'] != sha(raw) or trial['exports']['raw_enu.csv'] != sha(raw)):
            raise ValueError('Selected route provenance mismatch')
        if (len(points) != 17 or len(points) != meta['raw_waypoints'] or
                len(points) != trial['raw_waypoints'] or meta['raw_collision'] is not False or
                trial['raw_collision'] is not False or cert['collision_free'] is not True or
                cert['segments_checked'] != len(points)-1 or cert['collision_hits'] or
                cert['outside_bounds'] is not False or
                (cert['radius_m'], cert['margin_m'], cert['inflation_m']) != (.6, .25, .85)):
            raise ValueError('Raw collision certificate / waypoint count mismatch')
        if (points[0] != tuple(config['start_enu']) or points[-1] != tuple(config['goal_enu']) or
                points[0] != tuple(trial['start_enu']) or points[-1] != tuple(trial['goal_enu'])):
            raise ValueError('Start / Goal mismatch')
        if (trial['sdf_sha256'] != world.sdf_sha256 or trial['profile_sha256'] != profile_hash or
                (profile.radius_m, profile.margin_m) != (.6, .25)):
            raise ValueError('Current warehouse SDF / profile differs from certified geometry')
        if validate_geometry:
            environment.validate(points)  # continuous swept segments, not waypoint-only checks
        length = sum(lengths(points))
        if any(not math.isclose(length, m['raw_length_m'], abs_tol=1e-9, rel_tol=0)
               for m in (meta, trial)):
            raise ValueError('Raw length mismatch')
        routes[algorithm] = Route(algorithm, points, dict(
            csv=str(source / prefix / 'raw_enu.csv'), csv_sha256=sha(raw), seed=meta['seed'],
            sdf_sha256=world.sdf_sha256, profile_sha256=profile_hash,
            raw_certificate=cert, geometry_rechecked=True, frame='GAZEBO_WORLD_ENU', units='m',
            flight_tested=False, live_control_go=False))
    return routes


def timeline(route, speed=1., fps=20., hold=4., repeat=1):
    """Wall-clock animation; retain every original vertex and never smooth corners.

    Repetition explicitly teleports Goal -> Start; this is not a route segment.
    """
    if (not all(math.isfinite(v) for v in (speed, fps, hold)) or
            not 0 < speed <= 20 or not 1 <= fps <= 60 or not 0 <= hold <= 60 or
            type(repeat) is not int or not 1 <= repeat <= 20):
        raise ValueError('Require speed (0,20], fps [1,60], hold [0,60], repeat [1,20]')
    cumulative = [0.]
    for length in lengths(route.points):
        cumulative.append(cumulative[-1]+length)
    if cumulative[-1] <= 0:
        raise ValueError('Zero-length route')
    travel = cumulative[-1]/speed
    period = travel+2*hold
    if period*repeat*fps > 100_000:
        raise ValueError('Animation too large; increase speed or reduce fps / repeat')
    # Add vertex times to regular frame times so interpolation cannot cut corners.
    times = {0., period, hold, hold+travel}
    times.update(min(i/fps, period) for i in range(math.ceil(period*fps)+1))
    times.update(hold+d/speed for d in cumulative)
    frames = []
    for loop in range(repeat):
        for t in sorted(times):
            distance = min(max((t-hold)*speed, 0.), cumulative[-1])
            segment = min(bisect.bisect_right(cumulative, distance)-1, len(route.points)-2)
            span = cumulative[segment+1]-cumulative[segment]
            fraction = (distance-cumulative[segment])/span if span else 0.
            a, b = route.points[segment:segment+2]
            p = tuple(x+(y-x)*fraction for x,y in zip(a,b))
            if t == 0.: p = route.points[0]
            if t >= hold+travel: p = route.points[-1]
            frames.append(dict(t_wall_s=loop*period+t, loop=loop+1, segment=segment,
                               x_enu_m=p[0], y_enu_m=p[1], z_enu_m=p[2],
                               yaw_rad=math.atan2(b[1]-a[1], b[0]-a[0]),
                               event='loop_reset_teleport' if loop and t == 0 else 'pose'))
    return frames


def safe_target(name):
    if name not in OWN_NAMES:
        raise ValueError(f'Forbidden model target: {name!r}; x500 is protected')
    return name


def model_sdf(name, token, route):
    """Two independent static visual models: moving drone and stationary route."""
    safe_target(name)
    if not token.isalnum() or len(token) != 32:
        raise ValueError('Invalid ownership token')
    root = ET.Element('sdf', version='1.9')
    model = ET.SubElement(root, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'self_collide').text = 'false'
    link = ET.SubElement(model, 'link', name='owner_'+token)
    ET.SubElement(link, 'gravity').text = 'false'
    color = '1 0.65 0.04 1' if route.algorithm == 'standard' else '0.03 0.85 1 1'

    def visual(label, position, shape, dims, tint=color, rotation=(0,0,0)):
        v = ET.SubElement(link, 'visual', name=label)
        ET.SubElement(v, 'pose').text = ' '.join(str(x) for x in (*position, *rotation))
        ET.SubElement(v, 'cast_shadows').text = 'false'
        geometry = ET.SubElement(v, 'geometry')
        g = ET.SubElement(geometry, shape)
        for key,value in dims.items(): ET.SubElement(g, key).text = str(value)
        material = ET.SubElement(v, 'material')
        for key in ('ambient', 'diffuse', 'emissive'): ET.SubElement(material, key).text = tint

    if name == DRONE:
        visual('body', (0,0,0), 'box', {'size':'0.35 0.24 0.12'})
        for i,(x,y) in enumerate(((-.3,-.3),(-.3,.3),(.3,-.3),(.3,.3))):
            visual(f'arm{i}', (x/2,y/2,0), 'box', {'size':'.43 .045 .04'},
                   rotation=(0,0,math.atan2(y,x)))
            visual(f'rotor{i}', (x,y,.06), 'cylinder', {'radius':'.14','length':'.015'})
        visual('nose', (.24,0,0), 'sphere', {'radius':'.06'}, '1 1 1 1')
    else:
        for i,p in enumerate(route.points):
            tint = '0 1 0 1' if i == 0 else '1 0.1 0.2 1' if i == len(route.points)-1 else color
            visual(f'waypoint{i}', p, 'sphere', {'radius':'.14' if i in (0,len(route.points)-1) else '.065'}, tint)
        for i,(a,b) in enumerate(zip(route.points, route.points[1:])):
            length = math.dist(a,b)
            if length == 0: continue
            delta = tuple(y-x for x,y in zip(a,b))
            midpoint = tuple((x+y)/2 for x,y in zip(a,b))
            visual(f'raw_segment{i}', midpoint, 'cylinder', {'radius':'.025','length':length},
                   rotation=(0, math.acos(max(-1.,min(1.,delta[2]/length))), math.atan2(delta[1],delta[0])))
    result = ET.tostring(root, encoding='unicode')
    validate_sdf(result, name, token)
    return result


def validate_sdf(text, name, token):
    safe_target(name)
    root = ET.fromstring(text)
    models = root.findall('model')
    if (root.tag != 'sdf' or len(models) != 1 or models[0].get('name') != name or
            models[0].findtext('static') != 'true' or
            [l.get('name') for l in models[0].findall('link')] != ['owner_'+token] or
            any(e.tag not in {'sdf','model','static','self_collide','link','gravity','visual',
                             'pose','cast_shadows','geometry','box','size','sphere','radius',
                             'cylinder','length','material','ambient','diffuse','emissive'} for e in root.iter())):
        raise ValueError('Model must contain only approved static visual geometry')


def write_json(path, data):
    with Path(path).open('x') as f: json.dump(data, f, indent=2, allow_nan=False)


def export_preview(out, route, frames, token):
    name = route.algorithm
    with (out / f'{name}_timeline.csv').open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(frames[0]))
        writer.writeheader(); writer.writerows(frames)
    for model in OWN_NAMES:
        (out / f'{name}_{model}.sdf').write_text(model_sdf(model, token, route))
    write_json(out / f'{name}_metadata.json', dict(route.provenance, algorithm=name,
        waypoints=len(route.points), length_m=route.length, duration_wall_s=frames[-1]['t_wall_s'],
        timeline_frames=len(frames), simulation_clock_used=False, interpolation='piecewise_linear_raw',
        start_enu_m=route.points[0], goal_enu_m=route.points[-1], waypoints_enu_m=route.points,
        paused_pose_rendering_verified=False, kind='TRAJECTORY_VISUALIZATION_NOT_CONTROLLED_FLIGHT'))
    # Standalone fallback: no server, CDN, ROS, simulator or network dependency.
    payload = json.dumps(dict(algorithm=name, length=route.length, points=route.points, frames=frames), allow_nan=False)
    html = '''<!doctype html><html lang="vi"><meta charset="utf-8"><title>Raw PSO visualization</title>
<style>body{background:#101824;color:#eee;font:18px sans-serif;margin:24px}canvas{background:#192535;width:100%;max-width:1100px}button{font:inherit;margin:8px}small{display:block;color:#b7c9d9}</style>
<h2 id="title"></h2><small>TRAJECTORY VISUALIZATION • ENU • raw linear interpolation • NO PHYSICAL FLIGHT</small>
<canvas id="plot" width="1100" height="630"></canvas><br><button id="play">Play / Restart</button><button id="stop">Pause</button><span id="clock"></span>
<small>Top: XY plan (East / North). Bottom: altitude versus wall-clock animation time. Green = Start; red = Goal. Warehouse obstacles are not displayed in this fallback.</small>
<script>const d=__PAYLOAD__,c=document.getElementById('plot'),ctx=c.getContext('2d');
const color=d.algorithm==='standard'?'#ffa60a':'#08dbff',end=d.frames.at(-1).t_wall_s;
document.getElementById('title').textContent=d.algorithm.toUpperCase()+' PSO — '+d.length.toFixed(3)+' m — '+d.points.length+' raw waypoints';
const xs=d.points.map(p=>p[0]),ys=d.points.map(p=>p[1]),xmin=Math.min(...xs)-1,xmax=Math.max(...xs)+1,ymin=Math.min(...ys)-1,ymax=Math.max(...ys)+1;
const scale=Math.min(980/(xmax-xmin),410/(ymax-ymin)),xy=p=>[65+(p[0]-xmin)*scale,455-(p[1]-ymin)*scale];
function dot(p,r,col){let [x,y]=xy(p);ctx.beginPath();ctx.arc(x,y,r,0,7);ctx.fillStyle=col;ctx.fill()}
function draw(t){let f=d.frames[0];for(const a of d.frames){if(a.t_wall_s>t)break;f=a}
ctx.clearRect(0,0,c.width,c.height);ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();d.points.forEach((p,i)=>{const [x,y]=xy(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();
d.points.forEach(p=>dot(p,3,color));dot(d.points[0],8,'#0f6');dot(d.points.at(-1),8,'#f45');dot([f.x_enu_m,f.y_enu_m],11,'white');
ctx.fillStyle='white';ctx.font='17px sans-serif';ctx.fillText('East → / North ↑ (m)',25,25);ctx.fillText('START',...xy(d.points[0]).map((v,i)=>v+(i?-15:10)));ctx.fillText('GOAL',...xy(d.points.at(-1)).map((v,i)=>v+(i?-15:10)));
ctx.fillText('Z (ENU m): 1 → 3.2',25,495);ctx.strokeStyle=color;ctx.beginPath();d.frames.forEach((a,i)=>{const x=65+970*a.t_wall_s/end,y=605-85*(a.z_enu_m-1)/2.2;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();
ctx.fillStyle='white';ctx.fillRect(65+970*t/end,510,2,100);document.getElementById('clock').textContent=t.toFixed(2)+' / '+end.toFixed(2)+' wall s';}
let running=false,start=0;function tick(now){if(!running)return;let t=Math.min((now-start)/1000,end);draw(t);if(t<end)requestAnimationFrame(tick);else running=false}
document.getElementById('play').onclick=()=>{running=false;requestAnimationFrame(now=>{start=now;running=true;tick(now)})};document.getElementById('stop').onclick=()=>running=false;draw(0);
</script></html>'''
    (out / f'{name}_preview.html').write_text(html.replace('__PAYLOAD__',payload))


def confirm(cleanup=False):
    phrase = 'DELETE DEMO VISUALS' if cleanup else APPROVAL
    if not sys.stdin.isatty():
        raise ValueError('Interactive operator approval required; piped/default approval is forbidden')
    print('Only demo visual models will change. Physics stays paused; x500 is protected.')
    if input(f'Type {phrase!r} to approve this operation: ').strip() != phrase:
        raise ValueError('Operator did not approve')


class Gazebo:
    """Transport 13 discovery/stats + isolated native CLI requests.

    Python's blocking request_raw binding holds the GIL; stats callbacks on the
    shared reception thread need it. Use the installed CLI client in a separate
    process, with inherited environment, so stats continue during each request.
    This is a service client, never another simulator. No automatic fallback/retry.
    """

    def __init__(self, *, read_only=False):
        from gz.transport13 import Node
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        self.node = Node()
        self.types = dict(Boolean=Boolean, Empty=Empty, EntityFactory=EntityFactory,
                          Entity=Entity, Pose=Pose, Scene=Scene)
        self.stats = None
        self.baseline = None
        self.stats_wall = 0.
        self.clock_changed = False
        self.services = {}
        self.read_only = read_only
        self.request_log = []
        self.last_scene_text = None
        self.environment = dict(os.environ)
        self.ownership_specs = {}
        self.ownership_evidence = {}
        if not self.node.subscribe(WorldStatistics, f'/world/{WORLD}/stats', self.on_stats):
            raise ValueError('Cannot subscribe to world statistics')
        deadline = time.monotonic()+5
        while self.stats is None and time.monotonic() < deadline: time.sleep(.05)
        self.guard()
        self.baseline = self.clock(self.stats)
        # Only discovery polls; never retry a service request or a mutation.
        deadline = time.monotonic()+5.
        for suffix in SERVICE_TYPES:
            while True:
                try:
                    self.services[suffix] = self.service_info(suffix)
                    break
                except TransportError as exc:
                    if exc.code != 'DISCOVERY_MISSING' or time.monotonic() >= deadline: raise
                    time.sleep(.05)
                    self.guard()

    def service_info(self, suffix):
        if suffix not in SERVICE_TYPES:
            raise TransportError('SERVICE_NOT_ALLOWED',repr(suffix))
        service = f'/world/{WORLD}/{suffix}'
        try:
            info = self.node.service_info(service)
        except RuntimeError as exc:
            raise TransportError('DISCOVERY_FAILURE',f'{service}: {exc}') from exc
        if not info:
            raise TransportError('DISCOVERY_MISSING',service)
        if len(info) != 1:
            raise TransportError('DISCOVERY_AMBIGUOUS',f'{service}: {len(info)} providers')
        req,rep = SERVICE_TYPES[suffix]
        found = dict(request=info[0].req_type_name,response=info[0].rep_type_name,socket_id=info[0].socket_id)
        if (found['request'] != self.types[req].DESCRIPTOR.full_name or
                found['response'] != self.types[rep].DESCRIPTOR.full_name):
            raise TransportError('DISCOVERY_SCHEMA_MISMATCH',f'{service}: {found}')
        if not found['socket_id']:
            raise TransportError('DISCOVERY_INVALID_PROVIDER',service)
        if suffix in self.services and found != self.services[suffix]:
            raise TransportError('PROVIDER_CHANGED',f'{service}: identity/schema changed')
        return found

    @staticmethod
    def clock(stats):
        return (stats.iterations, stats.sim_time.sec, stats.sim_time.nsec)

    def on_stats(self, msg):
        if self.baseline is not None and (not msg.paused or self.clock(msg) != self.baseline):
            self.clock_changed = True
        self.stats = msg
        self.stats_wall = time.monotonic()

    def guard(self):
        if getattr(self,'pose_stream_error',None):
            raise ValueError('NO_GO: invalid own-model ground-truth stream: '+self.pose_stream_error)
        if self.stats is None:
            raise TransportError('STATS_NOT_RECEIVED',
                                 'No /world/warehouse_world/stats message received within bounded wait; physics not changed')
        state = f'latest_clock={self.clock(self.stats)}, baseline={self.baseline}, paused={self.stats.paused}'
        if self.clock_changed:
            raise TransportError('CLOCK_CHANGED',
                                 'Require unchanged simulation clock; change remains latched even after Pause; '+state)
        if not self.stats.paused:
            raise TransportError('STATS_NOT_PAUSED',
                                 'Received stats with paused=false; no automatic Pause/unpause; '+state)
        age = time.monotonic()-self.stats_wall
        if not 0 <= age <= 2.:
            raise TransportError('STATS_STALE',
                                 f'Last stats age={age:.3f}s, limit=2.000s; '+state)

    def request(self, suffix, message, response):
        from google.protobuf import text_format
        if suffix not in SERVICE_TYPES or suffix not in self.services:
            raise TransportError('SERVICE_NOT_ALLOWED',repr(suffix))
        if self.read_only and suffix != 'scene/info':
            raise TransportError('READONLY_VIOLATION',suffix)
        req,rep = SERVICE_TYPES[suffix]
        if (response != rep or type(message) is not self.types[req]):
            raise TransportError('REQUEST_SCHEMA_MISMATCH',suffix)
        if {k:os.environ.get(k) for k in TRANSPORT_ENV} != {k:self.environment.get(k) for k in TRANSPORT_ENV}:
            raise TransportError('TRANSPORT_ENV_CHANGED','Use the same environment as discovery/stats')
        self.guard()
        self.service_info(suffix)
        # Match the known working CLI's read timeout. Mutations retain 1500 ms.
        # A larger timeout alone cannot fix the Python binding GIL contention.
        timeout_ms = 5000 if suffix == 'scene/info' else 1500
        argv = [str(TRANSPORT_CLI),'-s',f'/world/{WORLD}/{suffix}',
                '--reqtype',message.DESCRIPTOR.full_name,'--reptype',self.types[response].DESCRIPTOR.full_name,
                '--timeout',str(timeout_ms),'--req',text_format.MessageToString(message)]
        record = dict(service=suffix,timeout_ms=timeout_ms,backend='native_transport13_cli',status='STARTED')
        self.request_log.append(record)
        started = time.monotonic()
        try:
            try:
                result = subprocess.run(argv,capture_output=True,text=True,env=self.environment,
                                        timeout=timeout_ms/1000+1.,check=False)
            except subprocess.TimeoutExpired as exc:
                raise TransportError('REQUEST_TIMEOUT',f'{suffix}: CLI wall deadline exceeded') from exc
            except OSError as exc:
                raise TransportError('REQUEST_FAILURE',f'{suffix}: unable to run Transport 13 client: {exc}') from exc
            record.update(exit_code=result.returncode,stdout_bytes=len(result.stdout.encode()),
                          stdout_sha256=sha(result.stdout.encode()),stderr=result.stderr[:2000])
            # gz CLI may exit 0 on callback failure or timeout: parse its output too.
            if 'Service call timed out' in result.stderr:
                raise TransportError('REQUEST_TIMEOUT',f'{suffix}: native service deadline {timeout_ms} ms')
            if result.stdout.strip() == 'Service call failed':
                raise TransportError('REQUEST_FAILURE',f'{suffix}: service callback returned false')
            if result.returncode != 0 or result.stderr.strip():
                raise TransportError('REQUEST_FAILURE',f'{suffix}: exit={result.returncode}, stderr={result.stderr[:500]!r}')
            reply = self.types[response]()
            try:
                text_format.Parse(result.stdout,reply)  # strict schema; no ignored unknown fields
            except text_format.ParseError as exc:
                raise TransportError('INVALID_RESPONSE',f'{suffix}: invalid {response} protobuf text: {exc}') from exc
            if not reply.IsInitialized():
                raise TransportError('INVALID_RESPONSE',f'{suffix}: missing required protobuf fields')
            self.guard()
            self.service_info(suffix)  # refuse even a valid reply from an in-flight restart
            if response == 'Boolean' and not reply.data:
                raise TransportError('BOOLEAN_REJECTED',f'{suffix}: data=false (application rejection)')
            if response == 'Scene':
                if not reply.model:
                    raise TransportError('INVALID_RESPONSE','scene/info: empty model list')
                self.last_scene_text = result.stdout
            record['status'] = 'RESPONSE_PARSED'
            return reply
        except (ValueError,RuntimeError) as exc:
            record['status'] = getattr(exc,'code','INVARIANT_FAILED')
            raise
        finally:
            record['elapsed_wall_s'] = time.monotonic()-started

    def scene(self):
        scene = self.request('scene/info', self.types['Empty'](), 'Scene')
        # SceneBroadcaster converts sdf::Scene (render settings), not the world:
        # name is normally empty. World identity comes from the pinned, fully
        # qualified service namespace AND the unchanged /world/.../stats stream.
        if scene.name not in ('', WORLD):
            raise TransportError('WRONG_SCENE',f'Expected {WORLD!r} or omitted Scene.name, got {scene.name!r}')
        names = [m.name for m in scene.model]
        ids = [m.id for m in scene.model]
        if (names.count('x500') != 1 or any(n.startswith('x500') and n != 'x500' for n in names)):
            raise TransportError('INVALID_SCENE_MODELS',f'Expected exactly one existing x500, got {names}')
        if (len(set(names)) != len(names) or any(not n for n in names) or
                len(set(ids)) != len(ids) or any(i <= 0 for i in ids)):
            raise TransportError('INVALID_SCENE_MODELS','Duplicate/missing model name or entity ID')
        self.last_scene_name = scene.name
        return {m.name:m for m in scene.model}

    def owned(self, entry, token):
        valid_entry(entry)
        name = safe_target(entry['name'])
        model = self.scene().get(name)
        if (model is None or model.id != entry['id'] or
                [link.name for link in model.link] != ['owner_'+token]):
            raise ValueError(f'Ownership / entity ID mismatch: {name}; refusing modification')
        proof = self.ownership_specs.get(name)
        if proof is None or proof['token'] != token:
            raise ValueError('Ownership SDF proof missing; refuse modification')
        self.ownership_evidence[name] = verify_scene_model(model,entry,proof)
        return model

    def begin_pose_observation(self):
        """Scene graph pose is cached; verify motion using current Pose_V instead."""
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.pose_cache,self.pose_requests,self.pose_stream_error = {},{},None
        if not self.node.subscribe(Pose_V,f'/world/{WORLD}/pose/info',self.on_poses):
            raise ValueError('Cannot subscribe to own-model ground-truth pose stream')

    def on_poses(self, message):
        received = time.monotonic()
        for pose in message.pose:
            if pose.name not in OWN_NAMES: continue  # x500 is never a visualization target
            position = (pose.position.x,pose.position.y,pose.position.z)
            stamp = message.header.stamp
            if (pose.id <= 0 or not all(math.isfinite(v) for v in position) or
                    (stamp.sec,stamp.nsec) != tuple(self.baseline[1:])):
                self.pose_stream_error = 'invalid entity/position or foreign simulation clock'
                continue
            self.pose_cache[pose.id] = dict(name=pose.name,position=position,received=received)

    def create(self, name, token, route, observed=None):
        safe_target(name)
        sdf = model_sdf(name, token, route)
        self.ownership_specs[name] = sdf_proof(sdf,name,token)
        req = self.types['EntityFactory'](name=name, sdf=sdf, allow_renaming=False)
        if name == DRONE:
            req.pose.position.x,req.pose.position.y,req.pose.position.z = route.points[0]
        req.pose.orientation.w = 1.
        self.request('create', req, 'Boolean')
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            model = self.scene().get(name)
            if model is not None:
                entry = dict(name=name, id=model.id)
                if observed is not None: observed(entry)  # persist ID even if verification fails
                self.owned(entry,token)
                return entry
            time.sleep(.05)
        raise ValueError('Create ACK received but scene not updated while Paused. Use offline preview; do not advance physics.')

    def pose(self, entry, frame, token):
        if safe_target(entry['name']) != DRONE:
            raise ValueError('Only the demo drone may move; route markers must stay fixed')
        self.owned(entry, token)  # Check BOTH name and ID; a forged ID must never target x500.
        if not hasattr(self,'pose_requests'):
            raise ValueError('Own-model pose observer must be created before animation')
        req = self.types['Pose'](name=DRONE, id=entry['id'])
        req.position.x,req.position.y,req.position.z = (frame[k] for k in HEADER)
        req.orientation.z = math.sin(frame['yaw_rad']/2)
        req.orientation.w = math.cos(frame['yaw_rad']/2)
        self.pose_requests[entry['id']] = time.monotonic()
        self.request('set_pose', req, 'Boolean')

    def verify_pose(self, entry, token, frame):
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            self.owned(entry,token)
            pose = self.pose_cache.get(entry['id'])
            if (pose and pose['name'] == entry['name'] and
                    pose['received'] >= self.pose_requests.get(entry['id'],float('inf')) and
                    time.monotonic()-pose['received'] <= 2. and
                    math.dist(pose['position'],tuple(frame[k] for k in HEADER)) < .005): return
            time.sleep(.025)
        raise ValueError('Pose ACK did not produce fresh matching Pose_V ground truth while Paused. Stop; never unpause.')


def replay(gz, route, frames, out, token):
    if OWN_NAMES.intersection(gz.scene()):
        raise ValueError('Demo models already exist. Use their own manifest for cleanup first; never adopt models.')
    gz.begin_pose_observation()
    confirm()
    gz.guard()  # approval may take time; recheck before any mutation
    entries = []
    log = dict(world=WORLD, token=token, initial_clock=gz.baseline,
               services=gz.services, models=entries, live_control_go=False,
               schema='visual_ownership_v2',algorithm=route.algorithm,
               sdf_sha256={name:sdf_proof(model_sdf(name,token,route),name,token)['sdf_sha256'] for name in OWN_NAMES})
    # Append-only recovery journal survives a partially completed create sequence.
    with (out/'ownership.jsonl').open('x') as journal:
        def event(value):
            journal.write(json.dumps(value)+'\n'); journal.flush(); os.fsync(journal.fileno())
        event(log)
        print('Cleanup command (separate explicit confirmation):\n'
              'PYTHONNOUSERSITE=1 /usr/bin/python3 -B -m '
              'pso_flight_demo.visualization.raw_pso_gazebo --cleanup '
              +shlex.quote(str(out/'ownership.jsonl')), flush=True)
        for name in (PATH_MODEL, DRONE):
            event(dict(creating=name))
            entry = gz.create(name,token,route,observed=lambda value:event(dict(observed=value)))
            entries.append(entry)
            event(dict(created=entry))
            write_json(out/f'owned_{name}.json', dict(log, models=list(entries)))
        drone = next(e for e in entries if e['name'] == DRONE)
        write_json(out/'ownership.json',log)
        # Prove actual scene movement before running the animation timeline.
        probe = next(f for f in frames if math.dist(route.points[0],tuple(f[k] for k in HEADER)) > .05)
        gz.pose(drone,probe,token); gz.verify_pose(drone,token,probe)
        gz.pose(drone,frames[0],token); gz.verify_pose(drone,token,frames[0])
        print('SCENE_POSE_CONFIRMED_WHILE_PAUSED. Verify GUI visually; video is visualization only.', flush=True)
        start = time.monotonic()
        for frame in frames:
            remaining = start+frame['t_wall_s']-time.monotonic()
            if remaining > 0: time.sleep(remaining)
            if time.monotonic()-(start+frame['t_wall_s']) > 1.:
                raise ValueError('Transport too slow for requested fps; animation stopped without retry')
            gz.pose(drone,frame,token)
        gz.verify_pose(drone,token,frames[-1])
        write_json(out/'visualization_complete.json', dict(status='VISUAL_REPLAY_COMPLETE_NOT_FLIGHT',
            final_clock=gz.clock(gz.stats), live_control_go=False, ownership=log))
    print(f'Visual models retained. Cleanup manifest: {out / "ownership.json"}')


def cleanup_context(manifest):
    """Reconstruct SDF proof from frozen CSV, not arbitrary files or a name alone."""
    path = Path(manifest).absolute()
    if (not path.resolve().is_relative_to(OUTPUT.resolve()) or
            any(p.is_symlink() for p in (path,*path.parents))):
        raise ValueError('Cleanup manifest must be a regular artifact inside outputs_visualization')
    parsed = read_manifest(path)
    header,run = parsed['header'],path.parent
    if not run.name.startswith('raw_pso_') or not run.name.endswith('_'+header['token'][:8]):
        raise ValueError('Manifest run/token mismatch')
    hashes = {str(path):parsed['manifest_sha256']}
    def read(name):
        artifact = run/name
        if artifact.is_symlink() or not artifact.is_file() or artifact.stat().st_size > 1_000_000:
            raise ValueError('Unsafe/missing ownership source artifact: '+name)
        raw = artifact.read_bytes(); hashes[str(artifact)] = sha(raw)
        return raw.decode('utf-8')
    config = read_json(read('config.json'))
    if not isinstance(config,dict): raise ValueError('Invalid execution configuration object')
    algorithm = config.get('selected_algorithm')
    if (not isinstance(algorithm,str) or algorithm not in {'standard','enhanced'} or config.get('world') != WORLD or
            config.get('mode') != 'execute_visualization' or config.get('source') != str(SOURCE) or
            not isinstance(config.get('model_names'),list) or
            not all(isinstance(name,str) for name in config['model_names']) or
            sorted(config.get('model_names',[])) != sorted(OWN_NAMES) or config.get('live_control_go') is not False):
        raise ValueError('Invalid execution configuration provenance')
    route = load_routes()[algorithm]
    metadata = read_json(read(algorithm+'_metadata.json'))
    if not isinstance(metadata,dict) or any(metadata.get(key) != route.provenance[key] for key in ('csv_sha256','sdf_sha256','profile_sha256')):
        raise ValueError('Raw CSV / warehouse provenance mismatch')
    proofs = {}
    for name in OWN_NAMES:
        text = read(f'{algorithm}_{name}.sdf')
        validate_sdf(text,name,header['token'])
        expected = model_sdf(name,header['token'],route)
        if text != expected:
            raise ValueError('Exported SDF differs from deterministic raw-route visual model')
        proofs[name] = sdf_proof(text,name,header['token'])
    if ('algorithm' in header and header['algorithm'] != algorithm or
            'sdf_sha256' in header and header['sdf_sha256'] != {n:p['sdf_sha256'] for n,p in proofs.items()}):
        raise ValueError('Manifest algorithm/SDF hash mismatch')
    return dict(path=path,parsed=parsed,proofs=proofs,source_hashes=hashes)


def cleanup_sources_unchanged(context):
    for filename,digest in context['source_hashes'].items():
        path = Path(filename)
        if path.is_symlink() or not path.is_file() or sha(path.read_bytes()) != digest:
            raise ValueError('Ownership source changed after review; refuse cleanup')


def collect_cleanup(gz, context, expected_id=None):
    parsed = context['parsed']; header = parsed['header']
    gz.guard()
    if list(gz.baseline) != header['initial_clock'] or gz.services != header['services']:
        raise ValueError('World / provider identity differs from manifest; refuse stale cleanup')
    entries = dict(parsed['entries'])
    names = set(entries) | set(parsed['creating'])
    if expected_id is not None:
        if type(expected_id) is not int or expected_id <= 0 or len(names) != 1:
            raise ValueError('Explicit expected ID is only valid for exactly one model')
        name = next(iter(names))
        if name in entries and entries[name]['id'] != expected_id: raise ValueError('Expected entity ID conflicts with journal')
        entries[name] = dict(name=name,id=expected_id)
    if names-set(entries):
        raise ValueError('Interrupted creating without recorded entity ID: require --expect-model-id from independent evidence')
    gz.ownership_specs.update(context['proofs'])
    evidence = []
    for name in sorted(names):
        gz.owned(entries[name],header['token'])
        evidence.append(dict(gz.ownership_evidence[name]))
    cleanup_sources_unchanged(context)
    return evidence


def cleanup(manifest, expected_id=None, *, review_only=False):
    context = cleanup_context(manifest)  # validate files before constructing any live client
    out = OUTPUT / ('ownership_review_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True,exist_ok=False)
    report = dict(result='NO_GO_CLEANUP',manifest=str(context['path']),
                  manifest_sha256=context['parsed']['manifest_sha256'],expected_id=expected_id,
                  source_hashes=context['source_hashes'],mutations_executed=False,live_control_go=False,
                  visualization_go=False,output=str(out),remove_attempts=[])
    try:
        gz = Gazebo(read_only=review_only)
        evidence = collect_cleanup(gz,context,expected_id)
        report.update(result='READY_FOR_SEPARATE_CLEANUP_APPROVAL',models=evidence,
                      initial_clock=gz.baseline,services=gz.services)
        write_json(out/'cleanup_review.json',report)
        print(json.dumps(report,indent=2),flush=True)
        if review_only: return report
        confirm(cleanup=True)
        # Recheck sources, identity and full hierarchy after the operator's wait.
        if collect_cleanup(gz,context,expected_id) != evidence:
            raise ValueError('Owned entity hierarchy changed after review')
        removed = []
        for proof in reversed(evidence):
            entry = dict(name=proof['name'],id=proof['id'])
            cleanup_sources_unchanged(context)
            gz.owned(entry,context['parsed']['header']['token'])
            if gz.ownership_evidence[entry['name']] != proof:
                raise ValueError('Owned entity hierarchy changed immediately before remove')
            request = gz.types['Entity'](name=entry['name'],id=entry['id'],type=gz.types['Entity'].MODEL)
            report['remove_attempts'].append(entry)
            report['mutations_executed'] = True
            write_json(out/f'remove_intent_{entry["id"]}.json',dict(entry,status='REQUEST_ABOUT_TO_BE_SENT'))
            gz.request('remove',request,'Boolean')
            deadline = time.monotonic()+2.
            while time.monotonic() < deadline:
                if entry['name'] not in gz.scene(): break
                time.sleep(.05)
            else:
                raise ValueError('Remove ACK received but disappearance unconfirmed while Paused; stop without retry')
            removed.append(entry)
        write_json(out/'cleanup_complete.json',dict(result='OWN_VISUALS_REMOVED',removed=removed,live_control_go=False))
        return dict(report,result='OWN_VISUALS_REMOVED',mutations_executed=True,removed=removed)
    except (ValueError,OSError,RuntimeError) as exc:
        write_json(out/'cleanup_failure.json',dict(report,error=str(exc),result='NO_GO_CLEANUP'))
        raise


def check_transport():
    """One bounded read-only Scene request with the exact adapter used for replay."""
    import gz.transport13 as transport
    token = uuid.uuid4().hex[:8]
    out = OUTPUT / ('transport_check_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+token)
    out.mkdir(parents=True,exist_ok=False)
    report = dict(result='NO_GO_TRANSPORT',world=WORLD,python=sys.executable,
        python_transport_module=transport.__file__,python_extension=transport._transport.__file__,
        native_client=str(TRANSPORT_CLI),environment={k:os.environ.get(k) for k in TRANSPORT_ENV},
        pythonpath=os.environ.get('PYTHONPATH'),ld_library_path=os.environ.get('LD_LIBRARY_PATH'),
        stats_topic=f'/world/{WORLD}/stats',request_type='gz.msgs.Empty',response_type='gz.msgs.Scene',
        read_timeout_ms=5000,mutation_timeout_ms=1500,live_control_go=False,visualization_go=False,
        mutations_executed=False,output=str(out))
    gz = None
    try:
        gz = Gazebo(read_only=True)
        models = gz.scene()
        gz.guard()
        report.update(result='PASS_READONLY_TRANSPORT',scene_name=gz.last_scene_name,
            scene_world_basis='Pinned /world/warehouse_world/scene/info provider + unchanged /world/warehouse_world/stats',
            model_count=len(models),x500_entity_id=models['x500'].id,
            demo_models_already_present=sorted(OWN_NAMES.intersection(models)),
            baseline_clock=gz.baseline,final_clock=gz.clock(gz.stats),
            partition=gz.node.options.partition,services=gz.services)
        (out/'scene_response.pbtxt').write_text(gz.last_scene_text)
    except (ValueError,OSError,RuntimeError) as exc:
        report['error'] = str(exc)
    finally:
        if gz is not None: report['requests'] = gz.request_log
        write_json(out/'transport.json',report)
    print(json.dumps(report,indent=2))
    return 0 if report['result'] == 'PASS_READONLY_TRANSPORT' else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--dry-run',action='store_true',help='Validate BOTH sources; export timelines, SDF and HTML; no Gazebo imports')
    mode.add_argument('--execute-visualization',action='store_true',help='Requires separate approval and interactive phrase')
    mode.add_argument('--check-transport',action='store_true',help='Read-only discovery/stats/Scene check; never create/set_pose/remove')
    mode.add_argument('--cleanup',type=Path,metavar='OWNERSHIP_JSON',help='Delete only verified models owned by this manifest')
    mode.add_argument('--review-cleanup',type=Path,metavar='OWNERSHIP_JSON',help='Read-only ownership/SDF/identity review; no remove')
    parser.add_argument('--expect-model-id',type=int,help='Required independent ID for legacy creating-only journal')
    parser.add_argument('--algorithm',choices=['standard','enhanced'],default='enhanced')
    parser.add_argument('--speed',type=float,default=1.,help='Animation m per wall second, not a PX4 setpoint')
    parser.add_argument('--fps',type=float,default=20.)
    parser.add_argument('--hold',type=float,default=4.,help='Wall seconds at each endpoint per loop')
    parser.add_argument('--repeat',type=int,default=1,help='Explicit Goal-to-Start visual teleport between loops')
    args = parser.parse_args(argv)
    if args.check_transport:
        return check_transport()
    if args.expect_model_id is not None and not (args.cleanup or args.review_cleanup):
        raise ValueError('--expect-model-id only applies to cleanup/review-cleanup')
    if args.review_cleanup:
        cleanup(args.review_cleanup,args.expect_model_id,review_only=True)
        return 0
    if args.cleanup:
        cleanup(args.cleanup,args.expect_model_id)
        return 0
    routes = load_routes()
    timelines = {k:timeline(v,args.speed,args.fps,args.hold,args.repeat) for k,v in routes.items()}
    token = uuid.uuid4().hex
    out = OUTPUT / ('raw_pso_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+token[:8])
    out.mkdir(parents=True,exist_ok=False)
    for key,route in routes.items(): export_preview(out,route,timelines[key],token)
    write_json(out/'config.json',dict(speed_animation_m_s=args.speed,fps=args.fps,hold_wall_s=args.hold,
        repeat=args.repeat,selected_algorithm=args.algorithm,mode='dry_run' if args.dry_run else 'execute_visualization',
        world=WORLD,model_names=sorted(OWN_NAMES),source=str(SOURCE),live_control_go=False))
    print(f'OUTPUT: {out}',flush=True)
    for k,v in routes.items():
        print(f'{k}: {len(v.points)} waypoints, {v.length:.9f} m; {timelines[k][-1]["t_wall_s"]:.3f} wall s; RAW_GEOMETRY_VALID')
    if args.execute_visualization:
        replay(Gazebo(),routes[args.algorithm],timelines[args.algorithm],out,token)
    else:
        print('DRY_RUN_ONLY: no Transport, no spawn, no pose, no physics. HTML previews ready.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, ImportError, KeyError, RuntimeError, EOFError, KeyboardInterrupt) as exc:
        print(f'NO_GO_VISUALIZATION: {exc}',file=sys.stderr)
        raise SystemExit(2)
