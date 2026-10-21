"""Pure ownership proofs for visual-only models; never imports Gazebo Transport.

Static declaration comes from the exact generated SDF. An omitted Scene boolean
is UNKNOWN, not evidence of a dynamic entity. Entity ownership additionally
requires the server identity, nonce, ID and full expected visual structure.
"""

import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

DRONE = 'pso_demo_visual_drone'
PATH_MODEL = 'pso_demo_visual_path'
OWN_NAMES = frozenset((DRONE,PATH_MODEL))


def json_object(pairs):
    result = {}
    for key,value in pairs:
        if key in result: raise ValueError('Duplicate JSON key: '+key)
        result[key] = value
    return result


def read_json(text):
    return json.loads(text,object_pairs_hook=json_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Non-finite JSON: '+value)))


def valid_token(token):
    if not isinstance(token,str) or not re.fullmatch('[0-9a-f]{32}',token):
        raise ValueError('Invalid ownership token')
    return token


def valid_entry(entry):
    if (not isinstance(entry,dict) or set(entry) != {'name','id'} or
            not isinstance(entry['name'],str) or entry['name'] not in OWN_NAMES or
            type(entry['id']) is not int or entry['id'] <= 0):
        raise ValueError('Invalid ownership model name / entity ID; x500 is protected')
    return dict(entry)


def read_manifest(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 65536:
        raise ValueError('Unsafe/missing/oversized ownership manifest')
    raw = path.read_bytes()
    text = raw.decode('utf-8')
    records = [read_json(line) for line in text.splitlines() if line.strip()] if path.suffix == '.jsonl' else [read_json(text)]
    if not records or not all(isinstance(r,dict) for r in records): raise ValueError('Empty/malformed ownership manifest')
    header = records[0]
    required = {'world','token','initial_clock','services','models','live_control_go'}
    allowed = required | {'schema','algorithm','sdf_sha256'}
    if (not required.issubset(header) or not set(header).issubset(allowed) or
            header['world'] != 'warehouse_world' or header['live_control_go'] is not False):
        raise ValueError('Invalid ownership header / world')
    valid_token(header['token'])
    if 'schema' in header and header['schema'] != 'visual_ownership_v2':
        raise ValueError('Unsupported ownership schema')
    clock = header['initial_clock']
    if (not isinstance(clock,list) or len(clock) != 3 or
            any(type(v) is not int or v < 0 for v in clock) or clock[2] >= 1_000_000_000):
        raise ValueError('Invalid manifest clock')
    services = {'create':('EntityFactory','Boolean'),'set_pose':('Pose','Boolean'),
                'remove':('Entity','Boolean'),'scene/info':('Empty','Scene')}
    if not isinstance(header['services'],dict) or set(header['services']) != set(services):
        raise ValueError('Invalid manifest service registry')
    for service,(req,rep) in services.items():
        fields = header['services'][service]
        if (not isinstance(fields,dict) or set(fields) != {'request','response','socket_id'} or
                fields['request'] != 'gz.msgs.'+req or fields['response'] != 'gz.msgs.'+rep or
                not isinstance(fields['socket_id'],str) or not fields['socket_id']):
            raise ValueError('Invalid manifest provider identity/schema')
    if not isinstance(header['models'],list) or len(header['models']) > 2:
        raise ValueError('Invalid manifest models')
    entries = {}
    for record in header['models']:
        record = valid_entry(record)
        if record['name'] in entries: raise ValueError('Duplicate committed model')
        entries[record['name']] = record
    if path.suffix == '.jsonl' and entries:
        raise ValueError('Journal must start with an empty model list')
    creating,observed,committed = [],{},set(entries)
    for record in records[1:]:
        if len(record) != 1: raise ValueError('Malformed journal event')
        event,value = next(iter(record.items()))
        if event == 'creating':
            if not isinstance(value,str) or value not in OWN_NAMES or value in creating or len(creating) >= 2:
                raise ValueError('Invalid/duplicate creating event')
            if creating and creating[-1] not in committed:
                raise ValueError('Previous create not committed')
            creating.append(value)
        elif event in {'observed','created'}:
            value = valid_entry(value)
            name = value['name']
            if name not in creating or name in committed: raise ValueError('Out-of-order create event')
            if name in entries and entries[name] != value: raise ValueError('Conflicting observed entity ID')
            if event == 'observed' and name in observed: raise ValueError('Duplicate observed event')
            entries[name] = value
            if event == 'observed': observed[name] = value
            else: committed.add(name)
        else: raise ValueError('Unknown ownership journal event')
    if not creating and not entries: raise ValueError('Manifest contains no create intent')
    if len({entry['id'] for entry in entries.values()}) != len(entries): raise ValueError('Duplicate entity IDs')
    return dict(header=header,entries=entries,creating=creating,
                unknown_ids=[name for name in creating if name not in entries],
                manifest_sha256=hashlib.sha256(raw).hexdigest())


def quaternion(r,p,y):
    cr,sr,cp,sp,cy,sy = math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


def sdf_pose(element):
    values = [float(v) for v in element.findtext('pose','0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values): raise ValueError('Invalid SDF pose')
    return values[:3],quaternion(*values[3:])


def sdf_proof(text,name,token):
    valid_token(token)
    if name not in OWN_NAMES: raise ValueError('Protected model name')
    root = ET.fromstring(text)
    model = root.find('model')
    links = model.findall('link') if model is not None else []
    if (root.tag != 'sdf' or len(root.findall('model')) != 1 or model.get('name') != name or
            len(model.findall('static')) != 1 or model.findtext('static') != 'true' or len(links) != 1 or
            links[0].get('name') != 'owner_'+token):
        raise ValueError('Invalid static visual SDF ownership declaration')
    if any(root.findall('.//'+tag) for tag in ('collision','sensor','plugin','joint','include','actor','light')):
        raise ValueError('Nonvisual entity in ownership SDF')
    visuals = {}
    for visual in links[0].findall('visual'):
        key = visual.get('name')
        geometry = visual.find('geometry')
        if not key or key in visuals or geometry is None or len(geometry) != 1: raise ValueError('Invalid SDF visuals')
        shape = geometry[0]
        if shape.tag == 'box': dims = tuple(float(v) for v in shape.findtext('size','').split())
        elif shape.tag == 'sphere': dims = (float(shape.findtext('radius','nan')),)
        elif shape.tag == 'cylinder': dims = (float(shape.findtext('radius','nan')),float(shape.findtext('length','nan')))
        else: raise ValueError('Unsupported visual shape')
        if len(dims) != {'box':3,'sphere':1,'cylinder':2}[shape.tag] or not all(math.isfinite(v) and v > 0 for v in dims):
            raise ValueError('Invalid visual dimensions')
        visuals[key] = dict(shape=shape.tag,dimensions=dims,pose=sdf_pose(visual))
    if not visuals: raise ValueError('No expected visuals')
    return dict(name=name,token=token,sdf_sha256=hashlib.sha256(text.encode()).hexdigest(),
                link='owner_'+token,link_pose=sdf_pose(links[0]),visuals=visuals,static_declared=True)


def _pose(message):
    p,q = message.position,message.orientation
    xyz,xyzw = (p.x,p.y,p.z),(q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in (*xyz,*xyzw)): raise ValueError('Non-finite scene pose')
    return xyz,xyzw


def _same_pose(actual,expected):
    ap,aq = actual; ep,eq = expected
    norm = math.sqrt(sum(v*v for v in aq))
    return (math.dist(ap,ep) < 1e-6 and abs(norm-1.) < 1e-6 and
            abs(sum(a*b for a,b in zip(aq,eq))) > 1.-1e-7)


def verify_scene_model(model,entry,proof):
    """Verify identity + full expected render structure, not a default bool."""
    valid_entry(entry)
    if proof.get('static_declared') is not True:
        raise ValueError('Missing validated static SDF declaration')
    if (model.name != entry['name'] or model.id != entry['id'] or proof['name'] != entry['name'] or
            len(model.link) != 1 or model.link[0].name != proof['link']):
        raise ValueError('Ownership / entity ID / token mismatch')
    if model.model or model.joint or model.visual or model.deleted or model.self_collide:
        raise ValueError('Unexpected nested entities in owned model')
    fields = {field.name for field,_ in model.ListFields()}
    if 'is_static' in fields and not model.is_static:
        raise ValueError('Explicit static contradiction in Scene')
    if entry['name'] == PATH_MODEL and not _same_pose(_pose(model.pose),((0,0,0),(0,0,0,1))):
        raise ValueError('Stationary path model origin changed')
    link = model.link[0]
    if any(getattr(link,field,[]) for field in ('sensor','light','particle_emitter','projector')):
        raise ValueError('Unexpected nonvisual link entities')
    if not _same_pose(_pose(link.pose),proof['link_pose']): raise ValueError('Owned link pose mismatch')
    visuals = {v.name:v for v in link.visual}
    if len(visuals) != len(link.visual) or set(visuals) != set(proof['visuals']):
        raise ValueError('Owned visual count/names mismatch')
    entity_ids = [model.id,link.id,*[v.id for v in link.visual]]
    if any(v <= 0 for v in entity_ids) or len(set(entity_ids)) != len(entity_ids):
        raise ValueError('Invalid owned hierarchy IDs')
    for name,expected in proof['visuals'].items():
        visual = visuals[name]; geometry = visual.geometry
        # SceneBroadcaster encodes ancestry by nesting; parent_id is also omitted
        # in Sim 8. A populated contradictory ID is rejected, never inferred.
        if visual.parent_id not in (0,link.id) or visual.plugin or visual.delete_me:
            raise ValueError('Unexpected visual parent/plugin')
        shape = geometry.Type.Name(geometry.type).lower()
        if any(field.name not in {'header','type',shape} for field,_ in geometry.ListFields()):
            raise ValueError('Unexpected additional geometry fields')
        if shape == 'box': dims = (geometry.box.size.x,geometry.box.size.y,geometry.box.size.z)
        elif shape == 'sphere': dims = (geometry.sphere.radius,)
        elif shape == 'cylinder': dims = (geometry.cylinder.radius,geometry.cylinder.length)
        else: raise ValueError('Unexpected Scene visual geometry')
        if (shape != expected['shape'] or len(dims) != len(expected['dimensions']) or
                any(not math.isclose(a,b,rel_tol=1e-7,abs_tol=1e-8) for a,b in zip(dims,expected['dimensions'])) or
                not _same_pose(_pose(visual.pose),expected['pose'])):
            raise ValueError('Owned visual geometry/local pose mismatch: '+name)
    return dict(name=entry['name'],id=entry['id'],token=proof['token'],sdf_sha256=proof['sdf_sha256'],
                visuals_checked=len(visuals),static_source='VALIDATED_EXPORTED_SDF',
                static_scene='TRUE' if 'is_static' in fields else 'NOT_PUBLISHED',
                runtime_static_proven='is_static' in fields,
                hierarchy_ids=sorted(entity_ids),link_id=link.id,
                visual_ids={name:visuals[name].id for name in sorted(visuals)})
