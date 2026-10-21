"""Offline geometry certificates and observation-plan tests. No runtime APIs."""
import copy
import unittest
from types import SimpleNamespace as NS

import numpy as np

from pso_flight_demo.environments import load_environment,load_profile
from pso_flight_demo.geometry import FlightEnvironment
from pso_flight_demo.path_blocker_review import (anchor_observation_plan,review_route,
    segment_audit,separator_certificate,tracking_environment)
from pso_flight_demo.visualization.raw_pso_gazebo import load_routes


class TrackingReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world,cls.env=load_environment(load_profile())
        cls.routes=load_routes()
        cls.standard=review_route(cls.routes['standard'],cls.world,cls.env)
        cls.enhanced=review_route(cls.routes['enhanced'],cls.world,cls.env)

    def test_robust_bounds_and_envelope_unchanged(self):
        tube=tracking_environment(self.world,self.env)
        np.testing.assert_allclose(tube.lower,[-13.8,-10.8,1.25])
        np.testing.assert_allclose(tube.upper,[13.8,10.8,2.95])
        self.assertEqual(tube.radius,.6);self.assertEqual(tube.margin,.5)
        with self.assertRaises(ValueError):tracking_environment(self.world,self.env,.1)

    def test_standard_bad_waypoints_exactly_identified(self):
        v=self.standard['raw_tracking_audit']['waypoint_violations']
        self.assertEqual([x['index'] for x in v],[3,5,6,7,8,9,10,11,12,13,14,15])
        self.assertTrue(all(x['axis']=='z' for item in v for x in item['violations']))

    def test_enhanced_bad_waypoints_exactly_identified(self):
        v=self.enhanced['raw_tracking_audit']['waypoint_violations']
        self.assertEqual([x['index'] for x in v],[3,4,5,8,9,10,11,12,13,14,15])

    def test_all_16_segments_checked_not_only_vertices(self):
        for report in [self.standard,self.enhanced]:
            segments=report['raw_tracking_audit']['segments']
            self.assertEqual([s['indices'] for s in segments],[[i,i+1] for i in range(16)])
            self.assertTrue(all(not s['collision'] for s in segments))
            self.assertFalse(report['raw_tracking_audit']['all_segments_go'])

    def test_lowering_standard_does_not_clear_the_wall(self):
        trial=self.standard['rejected_altitude_clip_trial']
        self.assertEqual([s['indices'] for s in trial['validation']['segments'] if s['collision']],
                         [[10,11],[11,12]])
        self.assertTrue(trial['endpoints_unchanged'] and trial['xy_unchanged'])
        self.assertAlmostEqual(trial['length_m'],20.841797545656462)

    def test_lowering_enhanced_does_not_clear_the_wall(self):
        trial=self.enhanced['rejected_altitude_clip_trial']
        self.assertEqual([s['indices'] for s in trial['validation']['segments'] if s['collision']],
                         [[7,8],[8,9],[9,10],[10,11],[11,12]])
        self.assertAlmostEqual(trial['length_m'],13.096807806055383)

    def test_plane_proof_covers_all_x_and_all_allowed_z(self):
        c=self.standard['separator_certificate']
        self.assertTrue(c['proven_infeasible'])
        self.assertEqual(c['merged_x_intervals'],[[-13.8,13.8]])
        self.assertEqual(c['center_z_interval'],[1.25,2.95])
        names={x['obstacle'] for x in c['contributors']}
        self.assertTrue({'mid_wall_left','mid_wall_right','mid_vertical_upper','mid_vertical_lower'}<=names)

    def test_same_endpoints_imply_same_separator_for_both_algorithms(self):
        self.assertEqual(self.standard['start_enu'],self.enhanced['start_enu'])
        self.assertEqual(self.standard['goal_enu'],self.enhanced['goal_enu'])
        self.assertEqual(self.standard['separator_certificate'],self.enhanced['separator_certificate'])

    def test_proof_does_not_apply_when_a_and_b_on_same_side(self):
        a=np.array(self.routes['standard'].points[0]);b=a+[1,0,0]
        self.assertFalse(separator_certificate(self.world,self.env,a,b)['proven_infeasible'])

    def test_proof_does_not_ignore_a_real_gap(self):
        world=NS(boxes=[b for b in self.world.boxes if b.name not in ['mid_vertical_upper','mid_vertical_lower']])
        c=separator_certificate(world,self.env,self.standard['start_enu'],self.standard['goal_enu'])
        self.assertFalse(c['plane_fully_covered'] or c['proven_infeasible'])

    def test_proof_is_height_and_model_specific(self):
        # Hypothetical test bounds only: no profile/SDF/runtime mutation.
        env=FlightEnvironment(self.world.boxes,self.env.lower,[*self.env.upper[:2],4.],.6,.25)
        c=separator_certificate(self.world,env,self.standard['start_enu'],self.standard['goal_enu'])
        self.assertFalse(c['proven_infeasible'])

    def test_no_rejected_trajectory_can_be_mistaken_for_flight_csv(self):
        for r in [self.standard,self.enhanced]:
            self.assertIsNone(r['flight_trajectory_csv'])
            self.assertIsNone(r['adjusted_flight_length_m'])
            self.assertEqual(r['static_path'],'NO_GO')
            self.assertEqual(r['rejected_altitude_clip_trial']['status'],'REJECTED_NOT_FOR_FLIGHT')
            self.assertFalse(r['live_control_go'])

    def test_raw_lengths_remain_raw_pso_comparison(self):
        self.assertAlmostEqual(self.standard['raw_length_m'],21.58798097888897)
        self.assertAlmostEqual(self.enhanced['raw_length_m'],13.57344247996963)

    def test_anchor_plan_is_not_evidence_or_physics_authorization(self):
        p=anchor_observation_plan()
        self.assertEqual(p['live_anchor'],'NO_GO')
        self.assertFalse(p['executed'] or p['fresh_anchor_created'] or p['live_control_go'])
        self.assertFalse(p['execute_now_recommended'])
        self.assertTrue(p['proposed_single_window']['separate_operator_approval_required'])
        self.assertEqual(p['minimum_evidence']['pair_skew_us_max'],20000)
        self.assertEqual(p['proposed_single_window']['wall_seconds'],60)


if __name__=='__main__':unittest.main()
