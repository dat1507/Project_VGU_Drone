"""SITL-only adapter: owned Gazebo, isolated DDS, owned PX4, no serial devices."""
import json
import os
from pathlib import Path
import random
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from .gazebo_runner import GazeboSession, PX4_ROOT
from .e2e import write_json


def simulation_world(source,output,vehicle_names):
    tree=ET.parse(source);world=tree.getroot().find('world')
    for element in list(world):
        if element.tag in ('include','model') and (element.get('name') or element.findtext('name')) in vehicle_names:
            world.remove(element)
    required={'Physics':'physics','UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster',
              'Imu':'imu','AirPressure':'air-pressure','Magnetometer':'magnetometer','NavSat':'navsat'}
    present={p.get('name','').split('::')[-1] for p in world.findall('plugin')}
    for name,file in required.items():
        if name not in present:ET.SubElement(world,'plugin',name='gz::sim::systems::'+name,filename=f'gz-sim-{file}-system')
    if world.find('spherical_coordinates') is None:
        sc=ET.SubElement(world,'spherical_coordinates')
        for name,value in dict(surface_model='EARTH_WGS84',world_frame_orientation='ENU',latitude_deg='47.397971',longitude_deg='8.546164',elevation='0',heading_deg='0').items():
            ET.SubElement(sc,name).text=value
    target=output/'sitl_world.sdf';tree.write(target,encoding='unicode');return target


def execute(args,world,environment,route,output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    if args.allow_collisions:raise ValueError('Collision-tolerant option is restricted to replay')
    if world.floor_z is None:raise ValueError('PX4 spawning requires a supported horizontal floor')
    px4_root=args.px4_root.resolve()
    derived=simulation_world(args.world,output,world.vehicle_names)
    instance=random.SystemRandom().randint(40,99)
    domain=random.SystemRandom().randint(100,190)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    runtime=output/'rootfs';runtime.mkdir()
    config=dict(algorithm=route['algorithm'],route=route['selected_waypoints'],world=world.name,model='pso_x500',
        namespace='pso_sitl',instance=instance,domain=domain,port=port,timeout=args.px4_timeout,
        output=str(output),floor_z=world.floor_z,bounds=route['bounds'],radius=args.radius,margin=args.margin,
        obstacles=[dict(name=b.name,lower=b.lower,upper=b.upper) for b in world.boxes])
    write_json(output/'flight_config.json',config)
    with GazeboSession(derived,output/'gazebo',gui=args.gui,px4_root=px4_root,view_path=route['selected_waypoints'],resource_paths=[Path(args.world).resolve().parent,*args.resource_path]) as sim:
        sim.env['GZ_SIM_RESOURCE_PATH']+=os.pathsep+str(Path(args.world).resolve().parent)
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        spawn=[*route['start'][:2],world.floor_z+.24]
        from .geometry import FlightEnvironment
        placement=FlightEnvironment(world.boxes,[*environment.lower[:2],world.floor_z-.5],environment.upper,args.radius,args.margin)
        if placement.collision_mask([spawn,route['start']])[0]:raise ValueError('Spawn/takeoff column intersects inflated obstacles')
        include=f'<sdf version="1.9"><include><uri>model://x500</uri><name>pso_x500</name><pose>{spawn[0]} {spawn[1]} {spawn[2]} 0 0 0</pose></include></sdf>'
        sim.request('create',EntityFactory(sdf=include,name='pso_x500',allow_renaming=False))
        sim.env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',PX4_SYS_AUTOSTART='4001',
            PX4_SIM_MODEL='gz_x500',PX4_GZ_STANDALONE='1',PX4_GZ_WORLD=world.name,PX4_GZ_MODEL_NAME='pso_x500',
            PX4_UXRCE_DDS_NS='pso_sitl',PX4_UXRCE_DDS_PORT=str(port),HEADLESS='1',PX4_SIM_SPEED_FACTOR='1',
            PX4_GZ_MODELS=str(px4_root/'Tools/simulation/gz/models'),
            PX4_GZ_WORLDS=str(px4_root/'Tools/simulation/gz/worlds'),
            ROS_LOG_DIR=str(output/'ros_logs'))
        agent=sim.spawn_process(['MicroXRCEAgent','udp4','-p',str(port)],'xrce_agent.log')
        binary=px4_root/'build/px4_sitl_default/bin/px4'
        px4=sim.spawn_process([str(binary),'-i',str(instance),'-d','-w',str(runtime),str(px4_root/'build/px4_sitl_default/etc')], 'px4.log',cwd=runtime)
        config['px4_pid']=px4.pid;config['px4_binary']=str(binary)
        write_json(output/'flight_config.json',config)
        # Fixed installation paths, no interpolation of user strings into shell code.
        worker=sim.spawn_process(['bash','-c',
            'source "$1" && source "$2" && exec python3 -m pso_flight_demo.px4_worker "$3"',
            'pso_worker',os.environ.get('PSO_ROS_SETUP','/opt/ros/jazzy/setup.bash'),
            os.environ.get('PSO_PX4_ROS_SETUP',str(Path.home()/'ros2_px4_ws/install/setup.bash')),str(output/'flight_config.json')],'offboard.log',cwd=Path(__file__).resolve().parents[1])
        deadline=time.monotonic()+args.px4_timeout+70
        last_capture=time.monotonic()
        while worker.poll() is None and time.monotonic()<deadline:
            if px4.poll() is not None:raise RuntimeError('PX4 exited; inspect px4.log')
            if agent.poll() is not None:raise RuntimeError('DDS agent exited; inspect xrce_agent.log')
            if args.gui and args.screenshots and time.monotonic()-last_capture>10:
                from .screenshot import capture
                try:capture(output/f"px4_{int(time.monotonic())}.png")
                except Exception as exc:
                    with (output/"screenshot_errors.log").open("a") as file:file.write(str(exc)+"\n")
                last_capture=time.monotonic()
            time.sleep(.2)
        if worker.poll() is None:raise TimeoutError('Offboard worker exceeded total deadline')
        result=output/'execution.json'
        if not result.exists():raise RuntimeError('Offboard worker produced no result; inspect offboard.log')
        return json.loads(result.read_text())
