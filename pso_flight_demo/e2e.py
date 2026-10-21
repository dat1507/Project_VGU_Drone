"""Reproducible planning, simulation orchestration and CLI for arbitrary ENU A/B."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import time
import uuid
import numpy as np
from .geometry import FlightEnvironment, vector
from .planner import PlannerConfig, run, spline_polyline
from .sdf_world import load_sdf

ROOT=Path(__file__).resolve().parents[1]
WAREHOUSE=ROOT/'simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf'


def write_json(path, data):
    Path(path).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


def length(path):return float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())


def certificate(path,env):
    point_env=FlightEnvironment(env.boxes,env.lower,env.upper,0,0)
    return dict(collision=bool(point_env.collision_mask(path)[0]),
                safety_inflated_collision=bool(env.collision_mask(path)[0]),
                segments_checked=len(path)-1,length_m=length(path))


def select_path(raw,smooth,raw_safe,smooth_safe,choice,allow_collisions=False):
    if choice=='raw':chosen='raw'
    elif choice=='smoothed':chosen='smoothed'
    else:chosen='smoothed' if smooth_safe else 'raw'
    safe=smooth_safe if chosen=='smoothed' else raw_safe
    if not safe and not allow_collisions:raise ValueError(f'{chosen} route infeasible; inspect collision reports, or use --allow-collisions for replay')
    return chosen, smooth if chosen=='smoothed' else raw


def plan(args,out):
    world=load_sdf(args.world,args.resource_path)
    start,goal=vector(args.start),vector(args.goal)
    if np.linalg.norm(goal-start)<1e-9:raise ValueError('Start and goal must differ')
    env=world.environment(start,goal,args.radius,args.margin,args.bounds)
    endpoints={key:certificate([v,v],env) for key,v in [('start',start),('goal',goal)]}
    if any(v['safety_inflated_collision'] for v in endpoints.values()) and not args.allow_collisions:
        raise ValueError('Start/goal intersects inflated obstacle or leaves bounds')
    config=PlannerConfig(swarm=args.swarm,iterations=args.iterations,n_waypoints=args.waypoints,jitter=args.jitter)
    write_json(out/'world.json',dict(name=world.name,source=world.source,sha256=world.sha256,
        obstacles=[asdict(b) for b in world.boxes],floor_z=world.floor_z,diagnostics=world.diagnostics,
        resource_sha256=world.resources,vehicle_names=world.vehicle_names))
    # Freeze source without changing it. Includes retain original resource paths in metadata.
    shutil.copyfile(world.source,out/'source.sdf')
    routes=[]
    for algorithm in (['standard','enhanced'] if args.algorithms=='both' else [args.algorithms]):
        began=time.perf_counter()
        result=run(env,start,goal,env.lower,env.upper,'baseline' if algorithm=='standard' else 'enhanced',args.seed,config)
        elapsed=time.perf_counter()-began
        smooth=spline_polyline(result.path)
        raw_cert,smooth_cert=certificate(result.path,env),certificate(smooth,env)
        route=dict(algorithm=algorithm,parameters=asdict(config),seed=args.seed,
            coordinate_system='GAZEBO_WORLD_ENU',units='m',start=start.tolist(),goal=goal.tolist(),
            raw_waypoints=result.path.tolist(),smoothed_waypoints=smooth.tolist(),
            raw=raw_cert,smoothed=smooth_cert,endpoint_validation=endpoints,
            planning_time_s=elapsed,fitness=result.fitness,fitness_evaluations=result.fitness_evaluations,
            convergence_history=result.best_history.tolist(),bounds=[env.lower.tolist(),env.upper.tolist()],
            radius_m=args.radius,margin_m=args.margin,sdf_sha256=world.sha256,
            selected_path=None,feasible=False,execution=dict(mode=None,success=False,status='NOT_EXECUTED'))
        try:
            choice,path=select_path(result.path,smooth,not raw_cert['safety_inflated_collision'],
                                  not smooth_cert['safety_inflated_collision'],args.path,args.allow_collisions)
            route.update(selected_path=choice,selected_waypoints=path.tolist(),
                         feasible=not route[choice]['safety_inflated_collision'])
        except ValueError as exc:route['selection_error']=str(exc)
        for name,points in [('raw',result.path),('smoothed',smooth)]:
            np.savetxt(out/f'{algorithm}_{name}.csv',points,delimiter=',',header='x_enu_m,y_enu_m,z_enu_m',comments='')
        if route['selected_path'] is not None:
            np.savetxt(out/f'{algorithm}_selected.csv',route['selected_waypoints'],delimiter=',',header='x_enu_m,y_enu_m,z_enu_m',comments='')
        write_json(out/f'{algorithm}.json',route)
        print(f'{algorithm}: raw={raw_cert["length_m"]:.3f} m inflated_collision={raw_cert["safety_inflated_collision"]}; '
              f'smoothed={smooth_cert["length_m"]:.3f} m inflated_collision={smooth_cert["safety_inflated_collision"]}; selected={route["selected_path"]}',flush=True)
        routes.append(route)
    return world,env,routes


def doctor():
    def command(args):
        try:
            r=subprocess.run(args,capture_output=True,text=True,timeout=5)
            return dict(exit_code=r.returncode,output=(r.stdout+r.stderr).strip()[:1200])
        except (OSError,subprocess.TimeoutExpired) as exc:return dict(error=str(exc))
    gui={}
    for key,path in [('x11','/tmp/.X11-unix/X'+os.environ.get('DISPLAY',':0').split(':')[-1].split('.')[0]),
                     ('wayland',os.environ.get('XDG_RUNTIME_DIR','')+'/'+os.environ.get('WAYLAND_DISPLAY',''))]:
        try:
            with socket.socket(socket.AF_UNIX) as s:s.connect(path)
            gui[key]='PASS'
        except OSError as exc:gui[key]=str(exc)
    return dict(python=platform.python_version(),project=str(ROOT),gazebo=command(['gz','sim','--version']),
        px4=command(['git','-C',os.environ.get('PSO_PX4_ROOT',str(Path.home()/'PX4-Autopilot')),'describe','--tags','--always']),
        ros_distro=os.environ.get('ROS_DISTRO'),ros_jazzy=Path('/opt/ros/jazzy').is_dir(),gui=gui,
        binaries={c:shutil.which(c) for c in ('gz','MicroXRCEAgent','ros2','xwininfo')},
        python_modules={m:importlib.util.find_spec(m) is not None for m in ('sdformat14','numpy','rclpy','px4_msgs')},
        note='Socket access does not establish rendered GUI or physical flight; see per-run evidence')


def parser():
    p=argparse.ArgumentParser(description=__doc__,epilog="Legacy commands remain available: geometry, capture, waypoint, follow, and plan --anchor.")
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('doctor')
    compare=sub.add_parser('compare');compare.add_argument('run',type=Path)
    for name in ('plan','demo'):
        cmd=sub.add_parser(name)
        cmd.add_argument('--world',type=Path,required=True)
        cmd.add_argument('--start',nargs=3,type=float,required=True)
        cmd.add_argument('--goal',nargs=3,type=float,required=True)
        cmd.add_argument('--algorithms',choices=['standard','enhanced','both'],default='both')
        cmd.add_argument('--path',choices=['raw','smoothed','auto'],default='auto')
        cmd.add_argument('--seed',type=int,default=42)
        cmd.add_argument('--swarm',type=int,default=100)
        cmd.add_argument('--iterations',type=int,default=200)
        cmd.add_argument('--waypoints',type=int,default=15)
        cmd.add_argument('--jitter',type=float,default=2.5)
        cmd.add_argument('--radius',type=float,default=.6)
        cmd.add_argument('--margin',type=float,default=.25)
        cmd.add_argument('--bounds',nargs=6,type=float,metavar=('XMIN','YMIN','ZMIN','XMAX','YMAX','ZMAX'))
        cmd.add_argument('--resource-path',action='append',default=[],type=Path)
        cmd.add_argument('--allow-collisions',action='store_true',help='Permit unsafe output; executable only as replay')
        cmd.add_argument('--output',type=Path,help='New output directory; existing directories refused')
        if name=='demo':
            cmd.add_argument('--mode',choices=['replay','px4','auto'],default='replay')
            cmd.add_argument('--fallback',choices=['none','replay'],default='none')
            g=cmd.add_mutually_exclusive_group();g.add_argument('--gui',action='store_true');g.add_argument('--headless',action='store_true')
            cmd.add_argument('--speed',type=float,default=2.)
            cmd.add_argument('--fps',type=float,default=10.)
            cmd.add_argument('--hold',type=float,default=2.)
            cmd.add_argument('--markers',action='store_true',help='Show selected polyline in algorithm color during replay')
            cmd.add_argument('--screenshots',action='store_true',help='Capture Gazebo X11 window during replay')
            cmd.add_argument('--px4-root',type=Path,default=Path(os.environ.get('PSO_PX4_ROOT',str(Path.home()/'PX4-Autopilot'))))
            cmd.add_argument('--px4-timeout',type=float,default=120.)
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    if args.command=='doctor':print(json.dumps(doctor(),indent=2));return 0
    if args.command=='compare':
        for f in sorted(args.run.glob('*.json')):
            r=json.loads(f.read_text())
            if 'algorithm' in r:
                print(json.dumps({k:r[k] for k in ('algorithm','seed','planning_time_s','fitness','raw','smoothed','selected_path','feasible','execution')},indent=2))
        return 0
    out=(args.output or ROOT/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])).resolve()
    out.mkdir(parents=True,exist_ok=False)
    write_json(out/'arguments.json',{k:str(v) if isinstance(v,Path) else [str(x) for x in v] if k=='resource_path' else v for k,v in vars(args).items()})
    write_json(out/'doctor.json',doctor())
    print(f'OUTPUT={out}',flush=True)
    try:
        if args.command=='demo':args.resource_path.append(args.px4_root/'Tools/simulation/gz/models')
        world,env,routes=plan(args,out)
        if args.command=='demo':
            from .gazebo_runner import GazeboSession
            from .screenshot import capture
            for route in routes:
                algorithm=route['algorithm']
                if route['selected_path'] is None:continue
                if args.mode in ('px4','auto'):
                    try:
                        if not route['feasible']:raise ValueError('PX4 refuses geometrically infeasible route')
                        from .px4_adapter import execute
                        route['execution']=execute(args,world,env,route,out/algorithm/'px4')
                    except Exception as exc:
                        route['execution']=dict(mode='px4',success=False,error=f'{type(exc).__name__}: {exc}')
                    route['px4_attempt']=dict(route['execution'])
                    write_json(out/f'{algorithm}.json',route)
                if args.mode=='replay' or (args.mode=='auto' and not route['execution']['success'] and args.fallback=='replay'):
                    try:
                        with GazeboSession(args.world,out/algorithm/'replay',gui=args.gui,view_path=route['selected_waypoints'],resource_paths=args.resource_path) as sim:
                            route['execution']=sim.replay(route['selected_waypoints'],algorithm,args.speed,args.fps,args.hold,
                                                         capture if args.screenshots and args.gui else None,markers=args.markers)
                    except Exception as exc:
                        route['execution']=dict(mode='replay',success=False,error=f'{type(exc).__name__}: {exc}')
                write_json(out/f'{algorithm}.json',route)
        okay=all(r['selected_path'] is not None and (args.command=='plan' or r['execution']['success']) for r in routes)
        write_json(out/'summary.json',dict(success=okay,command=args.command,algorithms=[dict(algorithm=r['algorithm'],feasible=r['feasible'],execution=r['execution']) for r in routes]))
        return 0 if okay else 2
    except Exception as exc:
        write_json(out/'failure.json',dict(success=False,error=f'{type(exc).__name__}: {exc}'))
        print(f'FAILED: {exc}',flush=True);return 2
