"""Explicit SITL-only collision policy. No process, ROS or Gazebo control."""
from contextlib import contextmanager
import os
from pathlib import Path
import re

import numpy as np

from .geometry import FlightEnvironment

RISK_EVENT = 'DEMO_ONLY_COLLISION_RISK_ACCEPTED'
CEILING_EVENT = 'DEMO_ONLY_PROJECT_CEILING_BYPASSED'
DEMO_CONFIRMATION = 'PX4_SITL_DEMO_ONLY_FLIGHT_APPROVED'


class _DemoPositionEnvironment(FlightEnvironment):
    """No obstacle/upper-Z feasibility gate; finite, XY and lower bounds remain.

    Original bounds stay available for audit. This does not set a substitute
    ceiling or alter any PX4 limit, waypoint or Gazebo geometry.
    """
    def collision_mask(self, paths):
        p = np.asarray(paths, dtype=float)
        if p.ndim == 2:
            p = p[None]
        if p.ndim != 3 or p.shape[-1] != 3 or p.shape[1] < 2:
            raise ValueError('Expected (batch, >=2, 3) paths')
        if not np.isfinite(p).all():
            raise ValueError('Non-finite path')
        return ((p < self.lower).any(axis=(1, 2)) |
                (p[:, :, :2] > self.upper[:2]).any(axis=(1, 2)))


def position_environment(env):
    """Explicit demo-only collision and project ceiling exemption."""
    return _DemoPositionEnvironment([],env.lower,env.upper,env.radius,env.margin)


def verify_sitl(profile,state,native,process_env):
    """Fail closed on local executable/env/sensor/Agent evidence mismatch.

    Called after launcher/native identity checks and again before activation.
    DDS control reader GIDs must additionally match this session's output GIDs.
    """
    expected=str((Path(profile.px4_root).expanduser()/'build/px4_sitl_default/bin/px4').resolve())
    roots=[c['owner'] for c in state['children'] if c['role']=='PX4']
    agents=[c for c in state['children'] if c['role']=='AGENT']
    runtime=native['runtime_identity']
    if (profile.id!='warehouse' or profile.world_name!='warehouse_world' or profile.vehicle_name!='x500'
            or profile.instance!=0 or len(roots)!=1 or len(agents)!=1 or roots[0]['exe']!=expected
            or roots[0]['pid']!=runtime['px4_pid'] or roots[0]['start_ticks']!=runtime['px4_start_ticks']
            or roots[0]['boot_id']!=runtime['boot_id'] or
            any(runtime.get(k)!=1 for k in ('px4_count','gazebo_server_count','agent_count'))):
        raise ValueError('DEMO_ONLY_REQUIRES_OWNED_LOCAL_PX4_SITL_WAREHOUSE_X500')
    required=dict(PX4_SIMULATOR='gz',PX4_GZ_WORLD='warehouse_world',PX4_GZ_MODEL_NAME='x500',
                  PX4_GZ_STANDALONE='1',PX4_SYS_AUTOSTART='4001')
    if any(process_env.get(k)!=v for k,v in required.items()):
        raise ValueError('PX4_SIMULATOR_ENVIRONMENT_NOT_PROVEN')
    text=native['evidence']['uxrce_dds_client']['stdout']
    if ('Running, connected' not in text or not re.search(r'Agent IP:\s+127\.0\.0\.1\b',text)
            or not re.search(r'Agent port:\s+8888\b',text)):
        raise ValueError('SITL_AGENT_MUST_BE_LOCAL_LOOPBACK_8888')
    if any('SIMULATION' not in native['evidence'][key]['stdout'] for key in ('sensor_accel','sensor_gyro')):
        raise ValueError('SIMULATED_SENSOR_IDENTITY_NOT_PROVEN')
    return dict(sitl_verified=True,px4_executable=expected,px4_pid=roots[0]['pid'],
                world=profile.world_name,model=profile.vehicle_name,remote_hardware_allowed=False)


@contextmanager
def localhost_ros(enabled):
    """Confine the demo controller's DDS discovery to this host, restore on exit.

    No existing process/network configuration is modified. External discovery
    overrides are rejected rather than silently rewritten.
    """
    keys=('ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE')
    saved={k:os.environ.get(k) for k in keys}
    if enabled:
        if any(os.environ.get(k) for k in ('ROS_STATIC_PEERS','ROS_DISCOVERY_SERVER',
                'FASTDDS_DEFAULT_PROFILES_FILE','FASTRTPS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI')):
            raise ValueError('DEMO_ONLY_REJECTS_EXTERNAL_DDS_DISCOVERY_OVERRIDES')
        os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    try:
        yield
    finally:
        for k,v in saved.items():
            if v is None: os.environ.pop(k,None)
            else: os.environ[k]=v


class ObservedCollision:
    """Conservative BODY proxy on observed ground truth, not future feasibility.

    Latches suspected obstacle contact using radius .60, no safety/tracking
    inflation. Ground plane contact during takeoff/Land is expected. Every pose
    from the existing observer buffer is examined, not only the 20 Hz journal.
    A suspicion prevents success; no claim of an instrumented physics contact.
    """
    def __init__(self,world):
        self.body=FlightEnvironment(world.boxes,[-1e6]*3,[1e6]*3,.60,0.)
        self.previous=None
        self.suspected=False

    def update(self,poses):
        points=[p['position_enu'] for p in poses]
        if not points: return self.suspected
        if self.previous is not None: points.insert(0,self.previous)
        if len(points)==1: points.append(points[0])
        self.suspected |= bool(self.body.collision_mask(np.asarray(points))[0])
        self.previous=points[-1]
        return self.suspected
