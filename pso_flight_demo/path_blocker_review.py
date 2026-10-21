"""Offline-only tracking/anchor evidence review. No ROS or Gazebo calls.

Use the EXISTING inflated-AABB collision model; never change flight constraints
to make an adjusted path pass. Rejected trial coordinates are not flight CSVs.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from .demo_first import new_output
from .environments import load_environment, load_profile
from .geometry import FlightEnvironment
from .integrated_flight import CONFIG
from .visualization.raw_pso_gazebo import load_routes


def tracking_environment(world, env, tracking=CONFIG.tracking):
    if tracking != .25 or env.radius != .6 or env.margin != .25:
        raise ValueError('This review is pinned to radius .60 + margin .25 + tracking .25')
    return FlightEnvironment(world.boxes,env.lower+tracking,env.upper-tracking,
                             env.radius,env.margin+tracking)


def separator_certificate(world, env, start, goal):
    """Prove the y=0 plane is blocked for ALL admissible x/z, not a grid search.

    Every continuous A/B path must meet this plane. A finite union of inflated
    boxes spanning its full x/z rectangle is a sufficient infeasibility proof
    under the current collision model. False means UNPROVEN, not feasible.
    """
    tube=tracking_environment(world,env)
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    intervals=[]
    for box,low,high in zip(tube.boxes,tube.low,tube.high):
        if (low[1] <= 0 <= high[1] and low[2] <= tube.lower[2] and high[2] >= tube.upper[2]):
            left,right=max(float(low[0]),float(tube.lower[0])),min(float(high[0]),float(tube.upper[0]))
            if left <= right:
                intervals.append(dict(obstacle=box.name,x_interval=[left,right],
                    inflated_z_interval=[float(low[2]),float(high[2])]))
    merged=[]
    for item in sorted(intervals,key=lambda x:x['x_interval'][0]):
        a,b=item['x_interval']
        if merged and a <= merged[-1][1]: merged[-1][1]=max(merged[-1][1],b)
        else: merged.append([a,b])
    covered=(len(merged)==1 and merged[0][0] <= tube.lower[0] and merged[0][1] >= tube.upper[0])
    endpoints_inside=bool(np.all(start >= tube.lower) and np.all(start <= tube.upper) and
                          np.all(goal >= tube.lower) and np.all(goal <= tube.upper))
    opposite=bool(start[1]*goal[1] < 0)
    return dict(proven_infeasible=covered and opposite and endpoints_inside,
        model='EXISTING_CLOSED_INFLATED_AABB_NOT_EXACT_EUCLIDEAN_SWEPT_SPHERE',
        plane_y_enu_m=0.,center_x_interval=tube.lower[[0]].tolist()+tube.upper[[0]].tolist(),
        center_z_interval=[float(tube.lower[2]),float(tube.upper[2])],
        total_obstacle_inflation_m=env.radius+env.margin+CONFIG.tracking,
        endpoints_inside_robust_bounds=endpoints_inside,opposite_sides=opposite,
        contributors=intervals,merged_x_intervals=merged,plane_fully_covered=bool(covered),
        caveat='Model-specific proof; do not claim physical infeasibility under a different collision model')


def segment_audit(points, world, env):
    points=np.asarray(points,float)
    tube=tracking_environment(world,env)
    obstacles_only=FlightEnvironment(world.boxes,[-1e6]*3,[1e6]*3,env.radius,env.margin+CONFIG.tracking)
    vertices=[]
    for i,p in enumerate(points):
        violations=[]
        for axis,name in enumerate(('x','y','z')):
            if p[axis] < tube.lower[axis]:
                violations.append(dict(axis=name,side='lower',deficit_m=float(tube.lower[axis]-p[axis])))
            if p[axis] > tube.upper[axis]:
                violations.append(dict(axis=name,side='upper',deficit_m=float(p[axis]-tube.upper[axis])))
        if violations:vertices.append(dict(index=i,position_enu=p.tolist(),violations=violations))
    segments=[]
    for i in range(len(points)-1):
        leg=points[i:i+2]
        names=[box.name for box in world.boxes if FlightEnvironment(
            [box],[-1e6]*3,[1e6]*3,env.radius,env.margin+CONFIG.tracking).collision_mask(leg)[0]]
        segments.append(dict(indices=[i,i+1],collision=bool(obstacles_only.collision_mask(leg)[0]),
            obstacle_names=names,bounds_violation=bool(np.any(leg<tube.lower) or np.any(leg>tube.upper)),
            required_tracking_m=CONFIG.tracking,
            go=not bool(tube.collision_mask(leg)[0])))
    return dict(waypoint_violations=vertices,segments=segments,all_segments_go=all(s['go'] for s in segments))


def review_route(route,world,env):
    raw=np.asarray(route.points,float)
    env.validate(raw)
    tube=tracking_environment(world,env)
    trial=raw.copy()
    trial[:,2]=np.clip(trial[:,2],tube.lower[2],tube.upper[2])
    # Endpoint changes are forbidden; do not 'repair' incompatible A/B silently.
    endpoints_unchanged=bool(np.array_equal(raw[[0,-1]],trial[[0,-1]]))
    audit=segment_audit(raw,world,env)
    clipped=segment_audit(trial,world,env)
    certificate=separator_certificate(world,env,raw[0],raw[-1])
    return dict(algorithm=route.algorithm,source_csv=route.provenance['csv'],
        source_sha256=route.provenance['csv_sha256'],waypoint_count=len(raw),
        raw_length_m=route.length,start_enu=raw[0].tolist(),goal_enu=raw[-1].tolist(),
        cruise_bounds=dict(lower=env.lower.tolist(),upper=env.upper.tolist()),
        robust_center_bounds=dict(lower=tube.lower.tolist(),upper=tube.upper.tolist()),
        safety=dict(radius_m=.6,margin_m=.25,tracking_m=.25,total_inflation_m=1.1),
        raw_nominal_geometry='PASS',raw_tracking_audit=audit,
        rejected_altitude_clip_trial=dict(status='REJECTED_NOT_FOR_FLIGHT',
            length_m=float(np.linalg.norm(np.diff(trial,axis=0),axis=1).sum()),
            endpoints_unchanged=endpoints_unchanged,xy_unchanged=bool(np.array_equal(raw[:,:2],trial[:,:2])),
            modified_waypoints=[dict(index=i,raw_z=float(raw[i,2]),trial_z=float(trial[i,2]))
                                for i in range(len(raw)) if raw[i,2] != trial[i,2]],
            validation=clipped),separator_certificate=certificate,
        adjusted_flight_length_m=None,flight_trajectory_csv=None,static_path='NO_GO',
        reason='PROVEN_AABB_SEPARATOR' if certificate['proven_infeasible'] else 'NO_VALIDATED_ADJUSTED_PATH',
        live_control_go=False)


def anchor_observation_plan():
    """A conditional proposal, not a request to execute physics now."""
    return dict(live_anchor='NO_GO',fresh_anchor_created=False,executed=False,
        reason='No new live telemetry or ground-truth captured in this offline task',
        observation_needed_eventually=True,execute_now_recommended=False,
        defer_reason='Resolve the proven static-path blocker before spending another physics window',
        session_reference='7ae2da20fe854e098d511d580d1ee335',reference_iterations=15000,
        reference_sim_us=15_000_000,reference_is_not_current_runtime_verification=True,
        proposed_single_window=dict(wall_seconds=60,additional_steps_max=15000,
            conditional_end_iterations=30000,conditional_end_sim_us=30_000_000,
            separate_operator_approval_required=True,no_auto_unpause=True,no_retry=True,
            rationale='Previous pause-offset reset took about 11 simulation seconds; allow a stable tail within one bounded window',
            whole_cold_window_pass_not_implied_by_tail=True),
        reuse=['dds_observer.observe (updated per-message metadata)',
               'dds_evidence.LiveEpochClock (epoch/observed-offset/RTT validation)',
               'groundtruth.GazeboPoseObserver (same x500 entity)',
               'integrated_ros.measured_anchor (initial-yaw/frame confirmation)'],
        groundtruth_buffer='Drain/journal the existing bounded pose buffer as integrated_ros does; do not accumulate >10000 poses in a 15 s window',
        existing_cli_caveat='observe-window is pinned to historical 20000->25000; generic observe assessment expects 5 s. Do not reuse either as a 15000-step PASS checker without a reviewed adapter.',
        minimum_evidence=dict(identity=['boot_id','PX4/Gazebo/Agent PIDs, exe and start ticks',
            'launcher owner/session ID','profile/SDF hashes','world name and x500 entity ID'],
            control_publishers=0,px4_disarmed=True,px4_landed=True,auto_arm_disabled=True,
            required_topics=['vehicle_local_position_v1','vehicle_status_v4','vehicle_attitude',
                'vehicle_land_detected','estimator_status_flags','timesync_status'],
            namespace='/fmu/out/',resolve_versions_from_installed_schema=True,
            groundtruth='/world/warehouse_world/pose/info',native_stats_before_during_after=True,
            native_px4_local_position_estimator_and_gcs_before_after=True,
            gcs_rx_and_native_timestamp_progress=True,
            six_topics_source_progress_in_same_stable_epoch=True,stable_tail_min_sim_s=3.,
            timesync_stable_advanced_samples_min=3,timesync_rtt_us_lt=10000,
            observed_minus_estimated_offset_abs_us_max=100000,pair_skew_us_max=20000,
            native_dds_position_delta_m_max=.02,ekf_gt_translation_drift_m_max=.15,
            ekf_gt_yaw_error_rad_max=.15,reset_counters_unchanged_in_accepted_epoch=True,
            single_matching_dds_participant_and_px4_sysid_component=[1,1],
            source_wall_receive_and_timestamp_sample_recorded=True,
            paused_at_exact_end=True,initial_yaw_alignment=True,
            final_heading_remains_required_before_horizontal_flight=True),
        anchor_formula='p_NED = p_EKF_anchor + R_ENU_NED * (p_ENU - p_Gazebo_anchor)',
        validity='Candidate bound to measured epoch/session/resets only; TTL and live recapture apply before flight, never stale logs or spawn coordinates',
        live_control_go=False)


def main():
    world,env=load_environment(load_profile())
    routes=load_routes()
    reports={name:review_route(route,world,env) for name,route in routes.items()}
    out=new_output('last_blockers_review')
    before={Path(r.provenance['csv']):r.provenance['csv_sha256'] for r in routes.values()}
    before[Path(world.sdf_path)]=world.sdf_sha256
    for name,r in reports.items():
        with (out/(name+'_tracking_review.json')).open('x') as f:json.dump(r,f,indent=2)
    with (out/'anchor_observation_plan.json').open('x') as f:json.dump(anchor_observation_plan(),f,indent=2)
    unchanged={str(p):hashlib.sha256(p.read_bytes()).hexdigest()==h for p,h in before.items()}
    with (out/'protected_inputs.json').open('x') as f:json.dump(unchanged,f,indent=2)
    print(json.dumps(dict(output=str(out),static_path='NO_GO',live_anchor='NO_GO',
        takeoff_flight='NO_GO',flight_trajectory_csv=None,protected_inputs_unchanged=all(unchanged.values()),
        physics_requests=0,control_commands_sent=0),indent=2))


if __name__=='__main__':main()
