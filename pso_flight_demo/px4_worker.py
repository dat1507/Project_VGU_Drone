"""ROS 2 Offboard executor, callable only with an owned SITL session config.

ROS message timestamps use agent-synchronized system time. Gazebo simulation
seconds drive arrival holds; monotonic time drives freshness and deadman limits.
"""
import csv
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import numpy as np
from .frames import enu_to_ned, ned_to_enu, RESET_FIELDS
from .geometry import AABB, FlightEnvironment
from .safety import topic_name


def execute(config):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import (VehicleLocalPosition,VehicleOdometry,VehicleStatus,VehicleLandDetected,
        VehicleCommandAck,VehicleCommand,TrajectorySetpoint,OffboardControlMode)
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    from pymavlink import mavutil
    out=Path(config['output'])
    # Enforce ownership, executable identity, isolated namespace and ROS domain.
    if (Path(f'/proc/{config["px4_pid"]}/exe').resolve()!=Path(config['px4_binary']).resolve() or
        os.environ.get('ROS_DOMAIN_ID')!=str(config['domain']) or config['namespace']!='pso_sitl' or
        not os.environ.get('GZ_PARTITION','').startswith('pso_')):
        raise RuntimeError('Owned SITL isolation proof failed')
    route=np.array(config['route'],float)
    records=[];acks=[];commands=[];events=[]
    truth={};lock=threading.Lock();gz=GzNode()
    def poses(msg):
        for p in msg.pose:
            if p.name==config['model']:
                with lock:truth.update(wall=time.monotonic(),sim=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                    point=np.array([p.position.x,p.position.y,p.position.z]))
    gz.subscribe(Pose_V,f'/world/{config["world"]}/pose/info',poses)
    rclpy.init();node=Node('pso_owned_sitl_executor')
    samples={};fresh={};last_timestamp={};advanced={}
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    pubq=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    prefix='/'+config['namespace']
    subs=[]
    def receive(key,msg):
        now=time.monotonic();samples[key]=msg;fresh[key]=now
        if msg.timestamp!=last_timestamp.get(key):advanced[key]=now;last_timestamp[key]=msg.timestamp
        if key=='ack':acks.append(dict(wall_s=now,command=int(msg.command),result=int(msg.result),timestamp=int(msg.timestamp)))
    for key,cls,base in [('position',VehicleLocalPosition,'vehicle_local_position'),('odometry',VehicleOdometry,'vehicle_odometry'),
        ('status',VehicleStatus,'vehicle_status'),('land',VehicleLandDetected,'vehicle_land_detected'),('ack',VehicleCommandAck,'vehicle_command_ack')]:
        subs.append(node.create_subscription(cls,prefix+topic_name('/fmu/out/'+base,cls),lambda m,k=key:receive(k,m),qos))
    pubs={}
    for key,cls,base in [('mode',OffboardControlMode,'offboard_control_mode'),('target',TrajectorySetpoint,'trajectory_setpoint'),('command',VehicleCommand,'vehicle_command')]:
        pubs[key]=node.create_publisher(cls,prefix+topic_name('/fmu/in/'+base,cls),pubq)
    gcs=mavutil.mavlink_connection(f'udpout:127.0.0.1:{18570+config["instance"]}',source_system=245,source_component=190)
    heartbeat_at=0.
    def spin():
        nonlocal heartbeat_at
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if now-heartbeat_at>1:
            gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            heartbeat_at=now
    def stamp():return node.get_clock().now().nanoseconds//1000
    def command(number,p1=0.,p2=0.):
        msg=VehicleCommand();msg.timestamp=stamp();msg.command=number
        msg.param1=float(p1);msg.param2=float(p2)
        msg.target_system=int(samples['status'].system_id);msg.target_component=1
        msg.source_system=245;msg.source_component=190;msg.from_external=True
        pubs['command'].publish(msg)
        commands.append(dict(command=number,param1=p1,param2=p2,wall=time.monotonic(),timestamp=msg.timestamp))
        print('COMMAND',commands[-1],flush=True)
    def publish(target):
        m=OffboardControlMode();m.timestamp=stamp();m.position=True;pubs['mode'].publish(m)
        sp=TrajectorySetpoint();sp.timestamp=stamp();sp.position=list(map(float,target));sp.yaw=yaw
        sp.velocity=[float('nan')]*3;sp.acceleration=[float('nan')]*3;sp.jerk=[float('nan')]*3;sp.yawspeed=float('nan')
        pubs['target'].publish(sp)
    def acked(number):
        sent=next((c['wall'] for c in reversed(commands) if c['command']==number),math.inf)
        answers=[a for a in acks if a['command']==number and a['wall_s']>=sent]
        if answers and answers[-1]['result'] not in (0,5):raise RuntimeError(f'Command {number} rejected: {answers[-1]}')
        return bool(answers and answers[-1]['result']==0)
    def check_fresh():
        now=time.monotonic()
        for key in ('position','odometry','status','land'):
            if now-fresh.get(key,0)>1.5 or now-advanced.get(key,0)>1.5:raise RuntimeError(f'Lost/stale {key}')
        p=samples['position'];o=samples['odometry']
        if not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid):raise RuntimeError('Invalid estimator position')
        if not np.isfinite([*o.position,*o.q]).all():raise RuntimeError('Invalid vehicle odometry')
        if tuple(getattr(p,k) for k in RESET_FIELDS)!=reset:raise RuntimeError('Estimator origin reset; anchor invalid')
        with lock:t=dict(truth)
        if now-t.get('wall',0)>1.:raise RuntimeError('Lost Gazebo ground truth')
        return t
    started=time.monotonic();state='readiness';success=False;route_completed=False;armed=False
    failure=None;offset=None;reset=None;yaw=0.;index=0;hold_since=None
    state_wall=started;flight_started=None;best=math.inf;progress_at=started;last_log=0
    boxes=[AABB(**b) for b in config['obstacles']]
    collision_env=FlightEnvironment(boxes,[config['bounds'][0][0],config['bounds'][0][1],config['floor_z']-.5],
                                    config['bounds'][1],config['radius'],0.)
    previous_point=None
    try:
        while time.monotonic()-started<config['timeout']+45:
            spin();now=time.monotonic()
            if state=='readiness':
                if now-started>45:raise TimeoutError('45 s readiness deadline: '+str({k:now-v for k,v in fresh.items()}))
                if not all(k in samples for k in ('position','odometry','status','land')) or not truth:continue
                p=samples['position'];s=samples['status']
                if s.system_id!=config['instance']+1:raise RuntimeError('Unexpected SITL system id')
                if s.arming_state==s.ARMING_STATE_ARMED:raise RuntimeError('SITL unexpectedly armed before readiness')
                if not(s.pre_flight_checks_pass and p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid and samples['land'].landed):continue
                if abs(stamp()-p.timestamp)>2_000_000:continue
                with lock:t=dict(truth)
                if now-t['wall']>.3 or now-fresh['position']>.3:continue
                reset=tuple(getattr(p,k) for k in RESET_FIELDS)
                offset=np.array([p.x,p.y,p.z])-enu_to_ned(t['point'])
                yaw=float(p.heading)
                (out/'anchor.json').write_text(json.dumps(dict(gazebo_enu=t['point'].tolist(),px4_ned=[p.x,p.y,p.z],
                    offset=offset.tolist(),reset_counters=reset,gazebo_sim_time=t['sim'],dds_timestamp_us=int(p.timestamp)),indent=2))
                state='warmup';state_wall=now;events.append(dict(state=state,wall_s=now-started));print('STATE warmup',flush=True)
            t=check_fresh();p=samples['position'];s=samples['status'];armed=s.arming_state==s.ARMING_STATE_ARMED
            if s.failsafe:raise RuntimeError('PX4 reports failsafe')
            target=offset+enu_to_ned(route[min(index,len(route)-1)])
            if state in ('warmup','mode','arm','follow','hover'):publish(target)
            actual_enu=ned_to_enu(np.array([p.x,p.y,p.z])-offset)
            distance=float(np.linalg.norm(actual_enu-route[min(index,len(route)-1)]))
            if previous_point is not None and collision_env.collision_mask([previous_point,t['point']])[0]:
                raise RuntimeError('Measured vehicle envelope intersects obstacle AABB/bounds')
            previous_point=t['point'].copy()
            records.append(dict(wall_s=now-started,sim_s=t['sim'],state=state,waypoint=index,armed=armed,
                nav_state=int(s.nav_state),x_enu_m=float(t['point'][0]),y_enu_m=float(t['point'][1]),z_enu_m=float(t['point'][2]),
                estimator_x=float(actual_enu[0]),estimator_y=float(actual_enu[1]),estimator_z=float(actual_enu[2]),
                target_x=float(route[min(index,len(route)-1),0]),target_y=float(route[min(index,len(route)-1),1]),
                target_z=float(route[min(index,len(route)-1),2]),target_error_m=distance))
            if state=='warmup' and now-state_wall>2.:
                command(176,1,6);state='mode';state_wall=now
            elif state=='mode':
                if acked(176) and s.nav_state==s.NAVIGATION_STATE_OFFBOARD:
                    command(400,1,0);state='arm';state_wall=now
                elif now-state_wall>8:raise TimeoutError('Offboard mode acceptance timeout')
            elif state=='arm':
                if acked(400) and armed:
                    state='follow';flight_started=now;state_wall=now;progress_at=now
                elif now-state_wall>10:raise TimeoutError('Arming acceptance timeout')
            elif state in ('follow','hover'):
                if not armed:raise RuntimeError('Disarmed before route completion')
                if s.nav_state!=s.NAVIGATION_STATE_OFFBOARD:raise RuntimeError('PX4 left Offboard')
                if now-flight_started>config['timeout']:raise TimeoutError('Flight deadline exceeded')
                if distance<best-.03:best=distance;progress_at=now
                if now-progress_at>20:raise TimeoutError('Vehicle stalled approaching waypoint')
                if state=='follow':
                    if distance<.18 and np.linalg.norm(t['point']-route[index])<.30:
                        if index==len(route)-1:
                            state='hover';hold_since=t['sim'];route_completed=True
                        else:
                            index+=1;best=math.inf;progress_at=now
                    if samples['land'].landed and now-flight_started>12:raise RuntimeError('Vehicle remains landed after arming')
                elif t['sim']-hold_since>2:
                    command(21);state='landing';state_wall=now
            elif state=='landing':
                landing_accepted=acked(21)
                if landing_accepted and samples['land'].landed and not armed:
                    success=route_completed;break
                if now-state_wall>40:raise TimeoutError('Landing/disarm not confirmed')
            if now-last_log>2:
                print(f'STATE {state} waypoint {index}/{len(route)-1} target_error={distance:.3f} sim={t["sim"]:.3f}',flush=True);last_log=now
            # Limit publish rate near 20 Hz without using ROS simulated timers.
            time.sleep(.025)
        if not success:raise TimeoutError('Total execution deadline exceeded')
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}';print('FAILED',failure,flush=True)
        if armed:
            command(21)
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                spin()
                if 'land' in samples and samples['land'].landed:break
    finally:
        result=dict(mode='px4',success=success,px4_flight=success,route_completed=route_completed,
                    status='LANDED_AFTER_ROUTE' if success else 'FAILED',error=failure,measured_samples=len(records),
                    commands=commands,acknowledgements=acks,events=events)
        if records:
            with (out/'telemetry.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
            points=np.array([[r[k] for k in ('x_enu_m','y_enu_m','z_enu_m')] for r in records])
            result['actual_traveled_m']=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
            result['max_target_error_m']=max(r['target_error_m'] for r in records)
            route_records=[r for r in records if r['state'] in ('follow','hover') and r['waypoint']>0]
            if route_records:
                flown=np.array([[r[k] for k in ('x_enu_m','y_enu_m','z_enu_m')] for r in route_records])
                result['actual_route_traveled_m']=float(np.linalg.norm(np.diff(flown,axis=0),axis=1).sum())
                a=route[:-1];d=np.diff(route,axis=0);den=np.maximum(np.sum(d*d,axis=1),1e-12)
                fraction=np.clip(np.sum((flown[:,None]-a)*d,axis=2)/den,0,1)
                errors=np.linalg.norm(flown[:,None]-(a+fraction[:,:,None]*d),axis=2).min(1)
                result['route_cross_track_rms_m']=float(np.sqrt(np.mean(errors**2)))
                result['route_cross_track_max_m']=float(errors.max())
            hovering=[r for r in records if r['state']=='hover']
            if hovering:
                result['goal_hover_enu']=[hovering[-1][k] for k in ('x_enu_m','y_enu_m','z_enu_m')]
                result['goal_error_m']=float(np.linalg.norm(np.array(result['goal_hover_enu'])-route[-1]))
        (out/'execution.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        gz.unsubscribe(f'/world/{config["world"]}/pose/info');gcs.close();node.destroy_node();rclpy.shutdown()
    return result


def main():
    config=json.loads(Path(sys.argv[1]).read_text())
    try:result=execute(config)
    except Exception as exc:
        result=dict(mode='px4',success=False,error=f'{type(exc).__name__}: {exc}')
        (Path(config['output'])/'execution.json').write_text(json.dumps(result,indent=2))
        print(result,flush=True)
    return 0 if result['success'] else 2


if __name__=='__main__':raise SystemExit(main())
