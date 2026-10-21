"""Owned Gazebo Harmonic session and measured, collision-tolerant visual replay.

Replay is kinematic animation, never PX4 telemetry. Every commanded pose is
confirmed against Gazebo's pose stream. An isolated transport partition prevents
accidentally connecting to another simulator or physical flight controller.
"""
from pathlib import Path
import csv
import base64
import json
import math
import os
import re
import signal
import select
import sys
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np

PX4_ROOT = Path(os.environ.get('PSO_PX4_ROOT',str(Path.home()/'PX4-Autopilot')))


def drone_sdf(name, color='1 0.4 0.05 1'):
    model = ET.Element('sdf', version='1.9')
    m = ET.SubElement(model, 'model', name=name)
    ET.SubElement(m, 'static').text = 'true'
    link = ET.SubElement(m, 'link', name='body')
    def visual(name, shape, size, pose, rgba):
        v = ET.SubElement(link, 'visual', name=name)
        ET.SubElement(v, 'pose').text = pose
        g = ET.SubElement(v, 'geometry')
        s = ET.SubElement(g, shape)
        if shape == 'box':
            ET.SubElement(s, 'size').text = size
        else:
            ET.SubElement(s, 'radius').text = size
            ET.SubElement(s, 'length').text = '0.035'
        mat = ET.SubElement(v, 'material')
        ET.SubElement(mat, 'ambient').text = rgba
        ET.SubElement(mat, 'diffuse').text = rgba
    visual('fuselage', 'box', '0.38 0.26 0.16', '0 0 0 0 0 0', color)
    for i, yaw in enumerate((math.pi/4, -math.pi/4)):
        visual(f'arm{i}', 'box', '1.05 0.06 0.05', f'0 0 0 0 0 {yaw}', '0.08 0.08 0.08 1')
    for i, (x,y) in enumerate(((-.36,-.36),(-.36,.36),(.36,-.36),(.36,.36))):
        visual(f'rotor{i}', 'cylinder', '.19', f'{x} {y} .09 0 0 0', color)
    # No collision geometry: intentionally passes through walls in replay.
    return ET.tostring(model, encoding='unicode')


def marker_sdf(name,path,color):
    root=ET.Element('sdf',version='1.9');model=ET.SubElement(root,'model',name=name)
    ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='route')
    for i,(a,b) in enumerate(zip(path,path[1:])):
        a,b=np.array(a),np.array(b);delta=b-a;distance=float(np.linalg.norm(delta))
        if distance<1e-9:continue
        visual=ET.SubElement(link,'visual',name=f'segment_{i}')
        midpoint=(a+b)/2;yaw=math.atan2(delta[1],delta[0]);pitch=-math.atan2(delta[2],np.linalg.norm(delta[:2]))
        ET.SubElement(visual,'pose').text=' '.join(map(str,[*midpoint,0,pitch,yaw]))
        geometry=ET.SubElement(visual,'geometry');box=ET.SubElement(geometry,'box')
        ET.SubElement(box,'size').text=f'{distance} .035 .035'
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text=color;ET.SubElement(material,'diffuse').text=color
    return ET.tostring(root,encoding='unicode')


class GazeboSession:
    def __init__(self, world, output, gui=False, px4_root=PX4_ROOT, view_path=None, resource_paths=()):
        self.world = Path(world).resolve()
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.name = ET.parse(self.world).getroot().find('world').get('name')
        self.env = dict(os.environ)
        self.env.update(GZ_PARTITION='pso_' + uuid.uuid4().hex,
                        GZ_HOMEDIR=str(self.output / 'gz_home'),
                        XDG_CACHE_HOME=str(self.output / 'cache'),
                        GZ_LOG_PATH=str(self.output / 'gz_logs'), DCONF_BACKEND='memory')
        resources = [str(self.world.parent), str(px4_root / 'Tools/simulation/gz/models'),*map(str,resource_paths)]
        self.env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join([str(px4_root/'build/px4_sitl_default/src/modules/simulation/gz_plugins'),self.env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')])
        self.env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(resources + [self.env.get('GZ_SIM_RESOURCE_PATH','')])
        self.gui = gui
        self.view_path = view_path
        self.processes, self.handles = [], []
        self.positions = {}
        self.lock = threading.Lock()
        self.node = None
        self.telemetry = []
        self.cleanup_errors = []

    def spawn_process(self, args, log, **kwargs):
        handle = (self.output / log).open('w')
        self.handles.append(handle)
        p = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT,
                             env=self.env, start_new_session=True, **kwargs)
        self.processes.append(p)
        return p

    def __enter__(self):
        # Set before importing transport; its partition is fixed at node creation.
        self.previous_partition = os.environ.get('GZ_PARTITION')
        os.environ['GZ_PARTITION'] = self.env['GZ_PARTITION']
        try:
            tree=ET.parse(self.world);world=tree.getroot().find('world')
            plugins={p.get('name','').split('::')[-1] for p in world.findall('plugin')}
            missing={'Physics':'physics','UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster'}
            missing={k:v for k,v in missing.items() if k not in plugins}
            runtime_world=self.world
            if missing:
                for name,file in missing.items():ET.SubElement(world,'plugin',name='gz::sim::systems::'+name,filename=f'gz-sim-{file}-system')
                runtime_world=self.output/'runtime_world.sdf';tree.write(runtime_world,encoding='unicode')
            self.server = self.spawn_process(['gz','sim','-s','-r',str(runtime_world)], 'gazebo_server.log')
            from gz.transport13 import Node
            from gz.msgs10.pose_v_pb2 import Pose_V
            self.node = Node()
            def receive(msg):
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                with self.lock:
                    for p in msg.pose:
                        self.positions[p.name] = (time.monotonic(), stamp, (p.position.x,p.position.y,p.position.z))
            self.node.subscribe(Pose_V, f'/world/{self.name}/pose/info', receive)
            deadline = time.monotonic()+30
            while time.monotonic() < deadline:
                if self.server.poll() is not None:
                    raise RuntimeError('Gazebo server exited; inspect gazebo_server.log')
                if self.node.service_info(f'/world/{self.name}/create'):
                    break
                time.sleep(.1)
            else:
                raise TimeoutError('Gazebo create service not ready in 30 s')
            worker_log=(self.output/'transport.log').open('w');self.handles.append(worker_log)
            self.worker=subprocess.Popen([sys.executable,'-m','pso_flight_demo.transport_worker'],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=worker_log,text=True,bufsize=1,
                env=self.env,start_new_session=True)
            self.processes.append(self.worker)
            if self.gui:
                self.env['QT_QPA_PLATFORM'] = 'xcb'
                config = Path('/usr/share/gz/gz-sim8/gui/gui.config').read_text()
                config = config.replace('show_again="true"','show_again="false"')
                points=np.array(self.view_path if self.view_path is not None else [[0,0,2],[3,3,3]],float)
                center=(points.min(0)+points.max(0))/2
                distance=max(7.,float(np.ptp(points,axis=0)[:2].max())*1.15)
                camera=f'{center[0]} {center[1]-distance} {center[2]+distance*.85} 0 0.705 1.5708'
                config=re.sub(r'<camera_pose>.*?</camera_pose>',f'<camera_pose>{camera}</camera_pose>',config)
                config=re.sub(r'<plugin filename="ComponentInspector".*?</plugin>', '',config,flags=re.S)
                config=re.sub(r'<plugin filename="EntityTree".*?</plugin>', '',config,flags=re.S)
                gui_config=self.output/'gui.config';gui_config.write_text(config)
                self.gui_process = self.spawn_process(['gz','sim','-g','--gui-config',str(gui_config)], 'gazebo_gui.log')
                time.sleep(5)
                if self.gui_process.poll() is not None:raise RuntimeError('Gazebo GUI exited; inspect gazebo_gui.log')
            (self.output/'session.json').write_text(json.dumps(dict(world=str(self.world), world_name=self.name,
                partition=self.env['GZ_PARTITION'], server_pid=self.server.pid, gui_requested=self.gui), indent=2))
            return self
        except BaseException:
            self.__exit__(None,None,None)
            raise

    def request(self, suffix, req, reply='Boolean', timeout=4000):
        payload=dict(service=f'/world/{self.name}/{suffix}',module=req.__class__.__module__,
                     **{'class':req.__class__.__name__},request=base64.b64encode(req.SerializeToString()).decode(),
                     reply=reply,timeout=timeout)
        self.worker.stdin.write(json.dumps(payload)+'\n');self.worker.stdin.flush()
        if not select.select([self.worker.stdout],[],[],timeout/1000+3)[0]:
            raise TimeoutError(f'Gazebo {suffix} request worker stalled')
        result=json.loads(self.worker.stdout.readline())
        if not result['ok'] or (reply=='Boolean' and 'data: true' not in result.get('body','')):
            raise RuntimeError(f'Gazebo {suffix} failed: {result}')
        return result['body']

    def create_drone(self, name, color):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        self.request('create', EntityFactory(sdf=drone_sdf(name,color), name=name, allow_renaming=False))

    def set_pose(self, name, point):
        from gz.msgs10.pose_pb2 import Pose
        pose = Pose(name=name)
        pose.position.x,pose.position.y,pose.position.z = map(float,point)
        pose.orientation.w = 1.
        self.request('set_pose',pose)

    def measure(self, name, target, since, timeout=3):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.lock:
                value = self.positions.get(name)
            if value and value[0] >= since and np.linalg.norm(np.array(value[2])-target) < .015:
                return value
            time.sleep(.01)
        raise TimeoutError(f'No measured pose confirmation for {name} at {target}')

    def replay(self, path, algorithm='known_good', speed=2., fps=10., hold=1., screenshot=None, markers=False):
        from .visualization.raw_pso_gazebo import Route, timeline
        from gz.msgs10.entity_pb2 import Entity
        route = Route(algorithm, tuple(map(tuple,path)), {})
        frames = timeline(route, speed=speed, fps=fps, hold=hold)
        name = 'pso_replay_' + algorithm
        color='1 0.4 0.05 1' if algorithm != 'enhanced' else '0.05 0.7 1 1'
        self.create_drone(name,color)
        if markers:
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            self.request('create',EntityFactory(name=name+'_path',sdf=marker_sdf(name+'_path',path,color),allow_renaming=False))
        records = []
        began = time.monotonic()
        try:
            for i, f in enumerate(frames):
                point = [f[k] for k in ('x_enu_m','y_enu_m','z_enu_m')]
                wait = began+f['t_wall_s']-time.monotonic()
                if wait > 0: time.sleep(wait)
                sent = time.monotonic()
                self.set_pose(name,point)
                received,sim_time,actual = self.measure(name,point,sent)
                records.append(dict(wall_s=received-began,sim_time_s=sim_time,
                    x_enu_m=actual[0],y_enu_m=actual[1],z_enu_m=actual[2],
                    target_x=point[0],target_y=point[1],target_z=point[2],
                    error_m=float(np.linalg.norm(np.array(actual)-point))))
                if i % max(1,len(frames)//10) == 0:
                    print(f'{algorithm}: {i+1}/{len(frames)} measured Gazebo poses',flush=True)
                if screenshot and i in (len(frames)//4,3*len(frames)//4):
                    screenshot(self.output/f'{algorithm}_{i}.png')
            distance = sum(math.dist([a[k] for k in ('x_enu_m','y_enu_m','z_enu_m')],
                                    [b[k] for k in ('x_enu_m','y_enu_m','z_enu_m')]) for a,b in zip(records,records[1:]))
            return dict(mode='replay', success=True, px4_flight=False, measured_samples=len(records),
                        actual_traveled_m=distance, final_enu=actual, max_tracking_error_m=max(r['error_m'] for r in records))
        finally:
            if records:
                with (self.output/f'{algorithm}_replay_telemetry.csv').open('w') as file:
                    writer=csv.DictWriter(file,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
            try:self.request('remove',Entity(name=name,type=Entity.MODEL))
            except Exception as exc:self.cleanup_errors.append(str(exc))
            if markers:
                try:self.request('remove',Entity(name=name+'_path',type=Entity.MODEL))
                except Exception as exc:self.cleanup_errors.append(str(exc))

    def __exit__(self, *_):
        for p in reversed(self.processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    try:p.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        if self.node:
            self.node.unsubscribe(f'/world/{self.name}/pose/info')
        if getattr(self,'previous_partition',None) is None: os.environ.pop('GZ_PARTITION',None)
        else: os.environ['GZ_PARTITION']=self.previous_partition
        if hasattr(self,'worker'):
            self.worker.stdin.close();self.worker.stdout.close()
        for h in self.handles:h.close()
        (self.output/'cleanup.json').write_text(json.dumps(dict(owned_processes=[dict(pid=p.pid,exit_code=p.poll()) for p in self.processes],warnings=self.cleanup_errors),indent=2))
