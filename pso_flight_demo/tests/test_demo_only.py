"""DEMO-ONLY policy tests. No ROS init, IPC, simulator or flight execution."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import copy
import json
import os
import unittest

import numpy as np

from pso_flight_demo.demo_policy import (DEMO_CONFIRMATION,RISK_EVENT,ObservedCollision,
    localhost_ros,position_environment,verify_sitl)
from pso_flight_demo.environments import load_profile,load_environment
from pso_flight_demo.frames import FrameAnchor
from pso_flight_demo.geometry import AABB,FlightEnvironment
from pso_flight_demo.integrated_flight import Mission,Sample,prepare_plan,transition_checks
from pso_flight_demo.integrated_ros import validate_graph,run
from pso_flight_demo.runtime_safety import GroundTruthWatchdog,WatchSample
from pso_flight_demo.tests import test_integrated_flight as fixtures
from pso_flight_demo.visualization.raw_pso_gazebo import load_routes


class PolicyTests(unittest.TestCase):
    def test_default_still_static_no_go_and_demo_has_no_static_collision_blocker(self):
        for algorithm,length in [('standard',21.587980979),('enhanced',13.573442480)]:
            default=prepare_plan(algorithm);demo=prepare_plan(algorithm,demo_only=True)
            self.assertTrue(default['static_blockers'])
            self.assertEqual(demo['static_blockers'],[])
            self.assertEqual(demo['collision_risk_policy'],RISK_EVENT)
            self.assertAlmostEqual(demo['length_m'],length)
            self.assertEqual(demo['waypoint_count'],17)
            self.assertEqual(default['csv_sha256'],demo['csv_sha256'])
            self.assertEqual(default['controller'],demo['controller'])
            self.assertFalse(demo['live_control_go'] or demo['controls_physics'])

    def test_only_demo_ceiling_and_obstacles_are_exempt(self):
        env=FlightEnvironment([AABB('obstacle',(-1,-1,1),(1,1,2))],[-5,-5,1],[5,5,3.2])
        demo=position_environment(env)
        demo.validate([[0,0,1.5],[0,0,1.5]])
        with self.assertRaises(ValueError):env.validate([[0,0,1.5],[0,0,1.5]])
        demo.validate([[0,0,3.21],[0,0,4.]])
        with self.assertRaises(ValueError):env.validate([[0,0,3.21],[0,0,4.]])
        for p in [[0,0,.99],[5.1,0,2],[-5.1,0,2],[0,5.1,2],[0,-5.1,2],
                  [0,0,float('inf')],[0,0,float('nan')]]:
            with self.assertRaises(ValueError):demo.validate([p,p])
        self.assertEqual(demo.collision_mask([[[0,0,4.],[0,0,5.]],[[0,0,4.],[6,0,4.]]]).tolist(),[False,True])

    def test_demo_waives_relative_cap_but_keeps_measured_start_and_positive_takeoff(self):
        world,env=load_environment(load_profile())
        p=np.array([[-11.741300836980269,4.31172650287941,2.5],[-5,-3,3.2]])
        a=FrameAnchor(tuple([*p[0,:2],.2]),(0,0,0),15_000_000,15_000_000,(1,1,1,1,1,1000),
                      'px4_groundtruth_and_local_position')
        self.assertFalse(transition_checks(p,a,world,env)['go'])
        self.assertTrue(transition_checks(p,a,world,env,demo_only=True)['go'])
        a=replace(a,gazebo_enu=tuple([*p[0,:2],-.013]))
        result=transition_checks(p,a,world,env,demo_only=True)
        self.assertIs(result['go'],True)
        json.dumps(result)  # Live preflight/journal must serialize this result.
        self.assertFalse(transition_checks(p,a,world,env)['go'])
        a=replace(a,gazebo_enu=tuple([*p[0,:2],4.]))
        self.assertFalse(transition_checks(p,a,world,env,demo_only=True)['go'])
        a=replace(a,gazebo_enu=(0,0,0))
        self.assertFalse(transition_checks(p,a,world,env,demo_only=True)['go'])
        p[1,2]=float('nan')
        with self.assertRaises(ValueError):transition_checks(p,a,world,env,demo_only=True)

    def test_watchdog_demo_ceiling_waiver_keeps_default_and_tracking(self):
        fixture=fixtures.MissionTests();fixture.setUp()
        pos=(0,0,3.5)
        s=WatchSample(pos,tuple(fixture.anchor.to_ned(pos)),pos,16_000_000,16_000_000,
                      10.,10.,10.,fixture.resets)
        env=position_environment(fixture.env)
        default=GroundTruthWatchdog(fixture.anchor,env).evaluate(s,10.)
        self.assertIn('RELATIVE_HEIGHT',default['reasons'])
        watch=GroundTruthWatchdog(fixture.anchor,env,project_ceiling_required=False)
        self.assertEqual(watch.evaluate(s,10.)['reasons'],[])
        bad=replace(s,target_enu=(0,0,3.),native_gt_us=16_050_000,native_ekf_us=16_050_000)
        self.assertIn('GROUNDTRUTH_TRACKING',watch.evaluate(bad,10.)['reasons'])

    def test_real_raw_peak_at_negative_ground_passes_mock_and_logs_exemption(self):
        for algorithm in ('standard','enhanced'):
            plan=prepare_plan(algorithm,demo_only=True)
            self.assertIn('PROJECT_CEILING_3_2M_ABSOLUTE_AND_RELATIVE',plan['bypassed_project_gates'])
            self.assertEqual(plan['project_ceiling_policy'],'DEMO_ONLY_PROJECT_CEILING_BYPASSED')
            self.assertIn('TAKEOFF_HEIGHT_CAP_3_2M',plan['runtime_position_limits'])
        m,s=self.mission()
        m.target=np.array([0,0,3.5])
        d=m.step(replace(s,actual=(0,0,3.5)))
        self.assertEqual(d['stage'],'FOLLOW')
        self.assertIsNone(m.failure)

    def test_distinct_live_approval_cannot_use_default_token(self):
        args=NS(execute_flight=True,confirm_flight='PX4_PSO_FLIGHT_APPROVED',demo_only=True)
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(ValueError,'APPROVAL'):
                run(args,dict(static_blockers=[],demo_only=True),Path('/unused'))

    def test_demo_flag_and_plan_cannot_disagree(self):
        args=NS(execute_flight=True,confirm_flight=DEMO_CONFIRMATION,demo_only=True)
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaises(ValueError):run(args,dict(static_blockers=[],demo_only=False),Path('/unused'))

    def test_controller_localhost_scope_restores_environment(self):
        with patch.dict(os.environ,{'ROS_LOCALHOST_ONLY':'0'},clear=True):
            with localhost_ros(True):
                self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'],'1')
                self.assertEqual(os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'],'LOCALHOST')
            self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'],'0')
            self.assertNotIn('ROS_AUTOMATIC_DISCOVERY_RANGE',os.environ)

    def test_external_discovery_overrides_rejected(self):
        with patch.dict(os.environ,{'ROS_STATIC_PEERS':'192.0.2.1'},clear=True):
            with self.assertRaises(ValueError):
                with localhost_ros(True):pass

    def test_default_does_not_change_ros_scope(self):
        with patch.dict(os.environ,{'ROS_LOCALHOST_ONLY':'0'},clear=True):
            with localhost_ros(False):self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'],'0')

    def evidence(self):
        p=load_profile();exe=str((Path(p.px4_root).expanduser()/'build/px4_sitl_default/bin/px4').resolve())
        state=dict(children=[dict(role='PX4',owner=dict(exe=exe,pid=10,start_ticks=20,boot_id='boot')),
                             dict(role='AGENT',owner={})])
        native=dict(runtime_identity=dict(px4_pid=10,px4_start_ticks=20,boot_id='boot',
            px4_count=1,gazebo_server_count=1,agent_count=1),evidence={
                'uxrce_dds_client':dict(stdout='Running, connected\nAgent IP: 127.0.0.1\nAgent port: 8888\n'),
                'sensor_accel':dict(stdout='SIMULATION'), 'sensor_gyro':dict(stdout='SIMULATION')})
        env=dict(PX4_SIMULATOR='gz',PX4_GZ_WORLD='warehouse_world',PX4_GZ_MODEL_NAME='x500',
                 PX4_GZ_STANDALONE='1',PX4_SYS_AUTOSTART='4001')
        return p,state,native,env

    def test_local_sitl_evidence_accepted(self):
        self.assertTrue(verify_sitl(*self.evidence())['sitl_verified'])

    def test_physical_remote_or_wrong_executable_sensor_and_identity_rejected(self):
        for kind in ['exe','pid','sensor','remote','env','duplicate']:
            p,state,n,e=self.evidence()
            if kind=='exe':state['children'][0]['owner']['exe']='/usr/bin/real_vehicle'
            if kind=='pid':n['runtime_identity']['px4_pid']=11
            if kind=='sensor':n['evidence']['sensor_accel']['stdout']='hardware sensor'
            if kind=='remote':n['evidence']['uxrce_dds_client']['stdout']='Running, connected\nAgent IP: 192.0.2.1\nAgent port: 8888'
            if kind=='env':e['PX4_SIMULATOR']='hardware'
            if kind=='duplicate':n['runtime_identity']['agent_count']=2
            with self.assertRaises(ValueError):verify_sitl(p,state,n,e)

    def test_input_reader_must_be_same_participant_as_sitl_telemetry(self):
        g=fixtures.AnchorGraphTests().graph()
        for name,v in g.items():
            if name.startswith('/fmu/in/'):
                v.update(reader_gids=[[1]*12+[50]],expected_type='px4_msgs/msg/Input',reader_types=['px4_msgs/msg/Input'])
        validate_graph(g,require_sitl_readers=True)
        g['/fmu/in/vehicle_command']['reader_gids'][0][0]=9
        with self.assertRaises(ValueError):validate_graph(g,require_sitl_readers=True)

    def test_missing_reader_identity_is_not_allowed_in_demo(self):
        with self.assertRaises(ValueError):validate_graph(fixtures.AnchorGraphTests().graph(),require_sitl_readers=True)

    def test_actual_collision_proxy_latches_without_path_feasibility_gate(self):
        world=NS(boxes=[AABB('wall',(0,0,0),(.1,1,2))])
        m=ObservedCollision(world)
        self.assertFalse(m.update([dict(position_enu=[-1,.5,1])]))
        self.assertTrue(m.update([dict(position_enu=[1,.5,1])]))
        self.assertTrue(m.update([]))

    def test_no_static_safety_or_tracking_inflation_on_observed_body_proxy(self):
        m=ObservedCollision(NS(boxes=[AABB('wall',(0,0,0),(.1,1,2))]))
        self.assertFalse(m.update([dict(position_enu=[-1,.5,2.8]),dict(position_enu=[1,.5,2.8])]))

    def mission(self):
        fixture=fixtures.MissionTests();fixture.setUp()
        m=Mission(fixture.path,fixture.anchor,fixture.env,fixture.contact,demo_only=True)
        s=fixture.s(armed=True,landed=False,nav=14,actual=tuple(fixture.path[0]),final_heading=True)
        m.target=fixture.path[0].copy();m.started_wall=s.wall;m.started_native=s.native_us
        m.enter('FOLLOW',s);m.route_started=s.native_us;m.begin_leg(fixture.path[0],fixture.path[1],s,follow=True)
        return m,s

    def test_collision_never_reports_success_even_after_goal_and_land(self):
        m,s=self.mission()
        d=m.step(replace(s,collision_suspected=True));self.assertEqual(d['command']['command'],21)
        self.assertTrue(m.collision_detected and m.aborted)
        m.goal_confirmed=True;m.ack(21,0,s.wall,s.native_us)
        m.step(replace(s,wall=s.wall+.05,native_us=s.native_us+50_000,nav=18,
                       armed=False,landed=True,collision_suspected=True))
        self.assertFalse(m.success)

    def test_bad_frame_time_reset_offboard_and_position_still_abort_in_demo(self):
        for kind in ['frame','time','reset','offboard','xy_bounds','tracking']:
            m,s=self.mission()
            if kind=='frame':s=replace(s,frame_good=False)
            if kind=='time':s=replace(s,coherent=False)
            if kind=='reset':s=replace(s,resets=(2,1,1,1,1,1000))
            if kind=='offboard':s=replace(s,nav=4)
            if kind=='xy_bounds':s=replace(s,actual=(100,0,2.5))
            if kind=='tracking':s=replace(s,actual=(.3,0,2.5))
            d=m.step(s)
            self.assertIn(d['stage'],['LANDING','ABORT_OPERATOR_REQUIRED'])
            self.assertFalse(m.success)

    def test_both_original_raw_routes_complete_in_ideal_demo_mock(self):
        world,env=load_environment(load_profile())
        for route in load_routes(validate_geometry=False).values():
            ground=(*route.points[0][:2],-.013)
            resets=(1,1,1,1,1,1000)
            anchor=FrameAnchor(ground,(1,2,3),15_000_000,15_000_000,resets,
                               'px4_groundtruth_and_local_position')
            contact=FlightEnvironment(world.boxes,[-14.05,-11.05,-1],[14.05,11.05,3.2])
            m=Mission(route.points,anchor,env,contact,demo_only=True)
            collision=ObservedCollision(world)
            nav=4;armed=False;landing=False;old=np.array(ground);commands=[];visited=set();heading_aligned=False
            for tick in range(8000):
                wall=1+tick*.05
                actual=m.target.copy() if not landing else old.copy()
                if landing:
                    actual[2]=max(ground[2],actual[2]-.01)
                    if actual[2]<=ground[2]+1e-6:armed=False
                velocity=(actual-old)/.05;old=actual.copy()
                suspected=collision.update([dict(position_enu=actual.tolist())])
                heading_aligned |= actual[2]>2.
                s=Sample(wall,15_000_000+tick*50_000,tuple(actual),tuple(velocity),resets,
                    armed=armed,landed=actual[2]<=ground[2]+1e-6,nav=nav,final_heading=bool(heading_aligned),
                    collision_suspected=suspected)
                d=m.step(s)
                if m.stage=='FOLLOW':visited.add(m.index)
                if d['command']:
                    c=d['command']['command'];commands.append(c);m.ack(c,0,wall,s.native_us)
                    if c==176:nav=14
                    elif c==400:armed=True
                    elif c==21:nav=18;landing=True
                if m.stage in ['DONE','ABORT_OPERATOR_REQUIRED']:break
            self.assertTrue(m.success,(route.algorithm,m.failure))
            self.assertEqual(commands,[176,400,21])
            self.assertEqual(visited,set(range(1,17)))


if __name__=='__main__':unittest.main()
