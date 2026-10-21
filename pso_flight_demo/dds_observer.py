"""Optional subscription-only DDS probe; no Agent start or control publications."""

import os
import time
import math

import numpy as np

from .safety import topic_name
from .dds_evidence import MessageEvidence


def _python_scalars(value):
    """Normalize NumPy scalars for JSON without changing telemetry values."""
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _python_scalars(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_python_scalars(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_python_scalars(item) for item in value)
    return value


def observe(profile, seconds, log_directory, ready=None, monitor=None):
    result = dict(publishes_commands=False, agent_started=False, messages={}, graph={},
                  errors=[], live_control_go=False)
    if not 0 < seconds <= 60:
        raise ValueError("DDS observation window must be >0 and <=60 wall seconds")
    collector = MessageEvidence()
    result['messages'] = collector.messages
    old_log = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = str(log_directory)
    node = None
    initialized = False
    created = time.monotonic()
    result['timing'] = dict(created_monotonic=created, configured_wall_seconds=seconds,
                           spin_count=0, max_spin_gap_s=0.)
    result['subscription_qos'] = dict(reliability='BEST_EFFORT', durability='VOLATILE', depth=100)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from px4_msgs.msg import (VehicleLocalPosition, VehicleStatus, VehicleCommandAck,
                                 OffboardControlMode, TrajectorySetpoint, TimesyncStatus,
                                 VehicleAttitude, VehicleLandDetected, EstimatorStatusFlags, VehicleCommand)
        if int(os.environ.get("ROS_DOMAIN_ID", "0")) != profile.dds_domain:
            raise ValueError("ROS_DOMAIN_ID differs from profile; configuration was not changed")
        rclpy.init(args=[])
        initialized = True
        node = Node("pso_c2_read_only_probe", enable_rosout=False, start_parameter_services=False)
        prefix = "/"+profile.namespace.strip("/") if profile.namespace.strip("/") else ""
        types = {"vehicle_local_position": VehicleLocalPosition, "vehicle_status": VehicleStatus,
                 "vehicle_command_ack": VehicleCommandAck, "timesync_status": TimesyncStatus,
                 'vehicle_attitude': VehicleAttitude, 'vehicle_land_detected': VehicleLandDetected,
                 'estimator_status_flags': EstimatorStatusFlags}
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        if monitor is not None:
            monitor.pre_subscriptions_check({
                prefix+topic_name('/fmu/in/'+key, cls): dict(
                    control_publishers=node.count_publishers(prefix+topic_name('/fmu/in/'+key, cls)))
                for key,cls in {'offboard_control_mode': OffboardControlMode,
                                'trajectory_setpoint': TrajectorySetpoint, 'vehicle_command': VehicleCommand}.items()})
        subscriptions = []
        def receive(key, msg):
            try:
                collector.record(key, msg.timestamp, time.monotonic(),
                                 getattr(msg, 'timestamp_sample', None))
            except ValueError as exc:
                result['errors'].append(str(exc))
                return
            item = result['messages'][key]
            if key == "vehicle_local_position":
                item.update(position_ned=[msg.x, msg.y, msg.z],
                            validity=[msg.xy_valid, msg.z_valid, msg.v_xy_valid, msg.v_z_valid])
                item['resets'] = [int(getattr(msg, k)) for k in
                    ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter',
                     'vz_reset_counter', 'heading_reset_counter', 'ref_timestamp')]
                item['samples'][-1].update(position_ned=item['position_ned'], resets=item['resets'],
                    validity=item['validity'], heading=getattr(msg,'heading',None),
                    heading_var=getattr(msg,'heading_var',None),
                    heading_good_for_control=getattr(msg,'heading_good_for_control',None))
                if not all(item['validity']) or not all(math.isfinite(x) for x in item['position_ned']):
                    result['errors'].append('Invalid local position')
            if key == 'vehicle_status':
                item.update(system_id=int(msg.system_id), component_id=int(msg.component_id),
                            gcs_lost=bool(msg.gcs_connection_lost), preflight=bool(msg.pre_flight_checks_pass))
                item['samples'][-1].update(system_id=item['system_id'], component_id=item['component_id'],
                    gcs_lost=item['gcs_lost'], preflight=item['preflight'],
                    armed=msg.arming_state == msg.ARMING_STATE_ARMED, failsafe=bool(msg.failsafe),
                    nav_state=getattr(msg,'nav_state',None))
                if (msg.system_id != 1 or msg.component_id != 1 or msg.failsafe or
                        msg.gcs_connection_lost or not msg.pre_flight_checks_pass or
                        msg.arming_state == msg.ARMING_STATE_ARMED):
                    result['errors'].append('Status identity/health/disarmed gate failed')
            if key == 'vehicle_land_detected' and not msg.landed:
                result['errors'].append('Unexpected airborne state')
            if key == 'vehicle_attitude':
                item['samples'][-1]['quaternion'] = list(msg.q)
                if not all(math.isfinite(x) for x in msg.q) or abs(sum(x*x for x in msg.q)-1) > .04:
                    result['errors'].append('Invalid attitude quaternion')
            if key == 'estimator_status_flags':
                item['samples'][-1]['flags'] = {k: bool(getattr(msg,k)) for k in
                    msg.get_fields_and_field_types() if k.startswith(('cs_','fs_'))}
                if (not msg.cs_tilt_align or not msg.cs_yaw_align or msg.cs_mag_fault or msg.cs_gnss_fault or
                        any(getattr(msg,k) for k in msg.get_fields_and_field_types() if k.startswith('fs_'))):
                    result['errors'].append('Estimator alignment/fault gate failed')
            if key == "timesync_status":
                item.update(source_protocol=msg.source_protocol, estimated_offset_us=msg.estimated_offset,
                            round_trip_time_us=msg.round_trip_time)
                item['samples'][-1].update(estimated_offset_us=int(msg.estimated_offset),
                    source_protocol=int(msg.source_protocol), round_trip_time_us=int(msg.round_trip_time),
                    observed_offset_us=getattr(msg,'observed_offset',None),
                    remote_timestamp=getattr(msg,'remote_timestamp',None))
                if msg.source_protocol != msg.SOURCE_PROTOCOL_DDS:
                    result['errors'].append('Timesync source is not DDS')
        for key, cls in types.items():
            name = prefix+topic_name("/fmu/out/"+key, cls)
            subscription = node.create_subscription(cls, name, lambda m, k=key: receive(k, m), qos)
            if subscription is None:
                raise RuntimeError('Subscription was not created: '+name)
            subscriptions.append(subscription)
        result['timing']['subscriptions_created_monotonic'] = time.monotonic()
        if monitor is not None:
            monitor.subscriptions_created(list(types))
        if ready is not None:
            ready()
        def graph_snapshot():
            graph = {}
            for key, cls in types.items():
                name = prefix+topic_name('/fmu/out/'+key, cls)
                endpoints = node.get_publishers_info_by_topic(name)
                graph[name] = dict(publishers=node.count_publishers(name),
                    expected_type=f'px4_msgs/msg/{cls.__name__}',
                    publisher_gids=[list(p.endpoint_gid) for p in endpoints],
                    actual_types=[getattr(p, 'topic_type', None) for p in endpoints],
                    publisher_qos=[dict(reliability=str(getattr(getattr(p,'qos_profile',None),'reliability',None)),
                                        durability=str(getattr(getattr(p,'qos_profile',None),'durability',None))) for p in endpoints])
            for key, cls in {'offboard_control_mode': OffboardControlMode,
                             'trajectory_setpoint': TrajectorySetpoint, 'vehicle_command': VehicleCommand}.items():
                name = prefix+topic_name('/fmu/in/'+key, cls)
                graph[name] = dict(px4_readers=node.count_subscribers(name),
                                  control_publishers=node.count_publishers(name))
            return graph
        until = time.monotonic()+seconds
        if monitor is not None:
            monitor.begin(until)
        previous_spin = time.monotonic()
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., until-time.monotonic())))
            now = time.monotonic()
            result['timing']['spin_count'] += 1
            result['timing']['max_spin_gap_s'] = max(result['timing']['max_spin_gap_s'], now-previous_spin)
            previous_spin = now
            if monitor is not None and not monitor.update(graph_snapshot(), now, result['errors']):
                break
        result['graph'] = graph_snapshot()
        for name, endpoint in result['graph'].items():
            if endpoint.get('publishers', 0) > 1:
                result['errors'].append('Multiple publishers: '+name)
            if endpoint.get('control_publishers', 0):
                result['errors'].append('Unexpected control publisher: '+name)
        result['errors'] = sorted(set(result['errors']))
        result.update(collector.summarize(result['graph'], result['errors']))
        result["note"] = "Graph discovery is not data flow; paused PX4 may publish no new samples."
    except (ImportError, ValueError, RuntimeError, OSError) as exc:
        result["errors"].append(f"{type(exc).__name__}: {exc}")
        result.update(collector.summarize(result['graph'], result['errors']))
    finally:
        result['timing']['finished_monotonic'] = time.monotonic()
        if monitor is not None:
            monitor.finish(result)
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        if old_log is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = old_log
    return _python_scalars(result)
