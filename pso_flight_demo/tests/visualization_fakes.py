"""Offline visual-model fixtures, including realistic protobuf field absence."""

import json
from pathlib import Path
from types import SimpleNamespace as NS

from gz.msgs10.model_pb2 import Model

from pso_flight_demo.visualization import raw_pso_gazebo as demo
from pso_flight_demo.visualization.ownership import sdf_proof


def proof_for(name,token,algorithm='enhanced'):
    return sdf_proof(demo.model_sdf(name,token,demo.load_routes()[algorithm]),name,token)


def model(name,entity_id,token,static=None):
    if name not in demo.OWN_NAMES:
        return NS(name=name,id=entity_id)
    proof = proof_for(name,token)
    result = Model(name=name,id=entity_id)
    if static is not None: result.is_static = static
    result.pose.orientation.w = 1.
    link = result.link.add(name=proof['link'],id=10000+entity_id*100)
    def pose(dst,values):
        xyz,xyzw = values
        dst.position.x,dst.position.y,dst.position.z = xyz
        dst.orientation.x,dst.orientation.y,dst.orientation.z,dst.orientation.w = xyzw
    pose(link.pose,proof['link_pose'])
    for index,(name,expected) in enumerate(proof['visuals'].items()):
        visual = link.visual.add(name=name,id=link.id+index+1)
        pose(visual.pose,expected['pose'])
        visual.geometry.type = visual.geometry.Type.Value(expected['shape'].upper())
        dims = expected['dimensions']
        if expected['shape'] == 'box':
            visual.geometry.box.size.x,visual.geometry.box.size.y,visual.geometry.box.size.z = dims
        elif expected['shape'] == 'sphere': visual.geometry.sphere.radius = dims[0]
        else: visual.geometry.cylinder.radius,visual.geometry.cylinder.length = dims
    return result


def services():
    return {name:dict(request='gz.msgs.'+req,response='gz.msgs.'+rep,socket_id='offline-provider')
            for name,(req,rep) in demo.SERVICE_TYPES.items()}


def make_run(directory,token,entries=None,events=None):
    run = Path(directory)/('raw_pso_OFFLINE_'+token[:8]); run.mkdir()
    route = demo.load_routes()['enhanced']
    config = dict(selected_algorithm='enhanced',world=demo.WORLD,mode='execute_visualization',
                  source=str(demo.SOURCE),model_names=sorted(demo.OWN_NAMES),live_control_go=False)
    (run/'config.json').write_text(json.dumps(config))
    (run/'enhanced_metadata.json').write_text(json.dumps(route.provenance))
    for name in demo.OWN_NAMES: (run/f'enhanced_{name}.sdf').write_text(demo.model_sdf(name,token,route))
    header = dict(world=demo.WORLD,token=token,models=entries or [],services=services(),
                  initial_clock=[20000,20,0],live_control_go=False)
    path = run/('ownership.jsonl' if events is not None else 'ownership.json')
    path.write_text('\n'.join(map(json.dumps,[header,*events])) if events is not None else json.dumps(header))
    return path
