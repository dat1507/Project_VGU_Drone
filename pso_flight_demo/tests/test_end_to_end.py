"""Scientific parity, frame composition, collision selection and real replay."""
from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
from pso_flight_demo.geometry import AABB,FlightEnvironment,load_world
from pso_flight_demo.sdf_world import load_sdf
from pso_flight_demo.planner import PlannerConfig,run,spline_polyline
from pso_flight_demo.e2e import select_path,certificate,WAREHOUSE

ROOT=Path(__file__).resolve().parents[2]


class WorldTests(unittest.TestCase):
    def world(self,content,resources=None):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        path=Path(self.tmp.name)/'test.sdf'
        path.write_text('<sdf version="1.9"><world name="custom">'+content+'</world></sdf>')
        if resources:
            for name,text in resources.items():(path.parent/name).write_text(text)
        return load_sdf(path)

    def test_warehouse_matches_legacy_geometry(self):
        old=load_world(WAREHOUSE);new=load_sdf(WAREHOUSE)
        self.assertEqual(len(old.boxes),len(new.boxes))
        for a,b in zip(old.boxes,new.boxes):
            np.testing.assert_allclose(a.lower,b.lower);np.testing.assert_allclose(a.upper,b.upper)
        self.assertEqual(new.vehicle_names,['x500'])

    def test_nested_model_frame_rotation(self):
        w=self.world('''<model name="outer"><static>true</static><pose>10 0 0 0 0 1.5707963267948966</pose>
        <frame name="pivot"><pose>2 0 0 0 0 0</pose></frame>
        <model name="inner"><pose relative_to="pivot">1 0 0 0 0 0</pose>
        <link name="body"><pose>1 0 0 0 0 0</pose><collision name="box"><pose>1 0 0 0 0 0</pose>
        <geometry><box><size>2 4 6</size></box></geometry></collision></link></model></model>''')
        np.testing.assert_allclose(w.boxes[0].lower,[8,4,-3],atol=1e-9)
        np.testing.assert_allclose(w.boxes[0].upper,[12,6,3],atol=1e-9)

    def test_sphere_cylinder_and_obj(self):
        w=self.world('''<model name="shapes"><static>true</static><link name="l">
        <collision name="s"><geometry><sphere><radius>2</radius></sphere></geometry></collision>
        <collision name="c"><pose>5 0 0 0 0 0</pose><geometry><cylinder><radius>1</radius><length>4</length></cylinder></geometry></collision>
        <collision name="m"><pose>0 5 0 0 0 0</pose><geometry><mesh><uri>shape.obj</uri><scale>2 1 1</scale></mesh></geometry></collision>
        </link></model>''',{'shape.obj':'v -1 -1 -1\nv 1 1 1\n'})
        np.testing.assert_allclose(w.boxes[0].upper,[2,2,2]);np.testing.assert_allclose(w.boxes[1].lower,[4,-1,-2])
        np.testing.assert_allclose(w.boxes[2].lower,[-2,4,-1]);self.assertTrue(w.resources)

    def test_static_resource_include(self):
        w=self.world('<include><uri>file://block.sdf</uri><pose>4 0 0 0 0 0</pose></include>',
            {'block.sdf':'<sdf version="1.9"><model name="block"><static>true</static><link name="l"><collision name="c"><geometry><box><size>2 2 2</size></box></geometry></collision></link></model></sdf>'})
        np.testing.assert_allclose(w.boxes[0].lower,[3,-1,-1])

    def test_unsupported_shape_and_dynamic_obstacle_rejected(self):
        with self.assertRaises(ValueError):self.world('<model name="moving"><link name="l"/></model>')
        with self.assertRaises(ValueError):self.world('<model name="static"><static>true</static><link name="l"><collision name="c"><geometry><ellipsoid><radii>1 2 3</radii></ellipsoid></geometry></collision></link></model>')

    def test_empty_world_and_two_endpoint_configs(self):
        w=load_sdf(ROOT/'worlds/empty.sdf')
        for a,b in [([0,0,2],[3,2,3]),([-2,1,2],[1,-2,2])]:
            e=w.environment(a,b);self.assertFalse(e.collision_mask([a,b])[0])
        with self.assertRaises(ValueError):w.environment([0,0,float('nan')],[1,2,3])
        with self.assertRaises(ValueError):w.environment([0,0,.5],[1,2,3])


class PlannerTests(unittest.TestCase):
    def test_saved_warehouse_scientific_fixture(self):
        fixtures=ROOT/'tests/fixtures'
        data=json.loads((fixtures/'warehouse_seed42_summary.json').read_text())
        world=load_sdf(WAREHOUSE)
        for alg,record in data.items():
            self.assertEqual(record['sdf_sha256'],world.sha256)
            raw=np.loadtxt(fixtures/f'warehouse_{alg}_seed42_raw.csv',delimiter=',',skiprows=1)
            env=world.environment(record['start'],record['goal'])
            cert=certificate(raw,env);smooth=certificate(spline_polyline(raw),env)
            self.assertEqual(cert['safety_inflated_collision'],record['raw']['safety_inflated_collision'])
            self.assertEqual(smooth['safety_inflated_collision'],record['smoothed']['safety_inflated_collision'])
            self.assertAlmostEqual(cert['length_m'],record['raw']['length_m'],places=9)
            np.testing.assert_allclose(raw[[0,-1]],[record['start'],record['goal']])

    def test_original_equations_and_rng_are_preserved(self):
        from environment import Environment,Cube
        from pso_core import run_pso
        from scenarios import START,GOAL
        cube=Cube((50,50,50),15)
        original=Environment([cube])
        adapted=FlightEnvironment([AABB('cube',tuple(cube.lower),tuple(cube.upper))],[0,0,0],[100,100,100],0,0)
        cfg=PlannerConfig(swarm=12,iterations=15,n_waypoints=15)
        for alg in ('baseline','enhanced'):
            a=run_pso(original,alg,42,swarm=12,iterations=15)
            b=run(adapted,START,GOAL,[0,0,0],[100,100,100],alg,42,cfg)
            np.testing.assert_array_equal(a.positions,b.positions)
            np.testing.assert_array_equal(a.best_history,b.best_history)
            self.assertEqual(a.fitness,b.fitness)

    def test_spline_crosses_obstacle_while_raw_is_clear(self):
        path=np.array([[-2,0,2],[-1,2,2],[1,2,2],[2,0,2]],float)
        smooth=spline_polyline(path)
        # Put a small obstacle on the curved bow above the flat raw segment.
        point=smooth[np.argmax(smooth[:,1])]
        box=AABB('spline_only',tuple(point-[.03,.03,.03]),tuple(point+[.03,.03,.03]))
        env=FlightEnvironment([box],[-5,-5,0],[5,5,5],0,0)
        self.assertFalse(env.collision_mask(path)[0]);self.assertTrue(env.collision_mask(smooth)[0])
        chosen,p=select_path(path,smooth,True,False,'auto')
        self.assertEqual(chosen,'raw');np.testing.assert_array_equal(p,path)

    def test_full_segment_collision_and_tolerant_selection(self):
        env=FlightEnvironment([AABB('wall',(-.1,-1,0),(.1,1,3))],[-5,-5,0],[5,5,5],.5,.1)
        path=np.array([[-2,0,2],[2,0,2]])
        self.assertTrue(certificate(path,env)['collision'])
        with self.assertRaises(ValueError):select_path(path,path,False,False,'auto')
        name,p=select_path(path,path,False,False,'raw',True)
        self.assertEqual(name,'raw')

    def test_distinct_algorithms_equal_budgets_and_reproducibility(self):
        env=FlightEnvironment([],[-5,-5,1],[5,5,5],0,0);cfg=PlannerConfig(swarm=10,iterations=12,n_waypoints=4)
        a=run(env,[-2,0,2],[2,0,2],env.lower,env.upper,'baseline',42,cfg)
        b=run(env,[-2,0,2],[2,0,2],env.lower,env.upper,'enhanced',42,cfg)
        again=run(env,[-2,0,2],[2,0,2],env.lower,env.upper,'baseline',42,cfg)
        np.testing.assert_array_equal(a.path,again.path);self.assertEqual(a.fitness_evaluations,b.fitness_evaluations)
        self.assertFalse(np.array_equal(a.path,b.path))


@unittest.skipUnless(os.environ.get('PSO_GAZEBO_TEST')=='1','Set PSO_GAZEBO_TEST=1 for real Gazebo integration')
class RealGazeboTests(unittest.TestCase):
    def test_two_pso_routes_and_colliding_replay_measured(self):
        from pso_flight_demo.gazebo_runner import GazeboSession
        from pso_flight_demo.e2e import main
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'empty'
            self.assertEqual(main(['demo','--world',str(ROOT/'worlds/empty.sdf'),'--start','0','0','2',
                '--goal','3','2','3','--swarm','12','--iterations','20','--waypoints','4','--headless',
                '--speed','10','--hold','0','--output',str(out)]),0)
            for alg in ('standard','enhanced'):
                r=json.loads((out/f'{alg}.json').read_text());self.assertTrue(r['execution']['success'])
                self.assertGreater(r['execution']['actual_traveled_m'],3)
                np.testing.assert_allclose(r['execution']['final_enu'],[3,2,3])
            out=Path(temp)/'collision'
            self.assertEqual(main(['demo','--world',str(WAREHOUSE),'--start','-3','-3','1.5',
                '--goal','3','-3','1.5','--jitter','0','--swarm','4','--iterations','2','--algorithms','standard',
                '--path','raw','--allow-collisions','--headless','--speed','10','--hold','0','--output',str(out)]),0)
            r=json.loads((out/'standard.json').read_text());self.assertTrue(r['raw']['collision'])
            self.assertTrue(r['execution']['success']);self.assertFalse(r['execution']['px4_flight'])


if __name__=='__main__':unittest.main()
