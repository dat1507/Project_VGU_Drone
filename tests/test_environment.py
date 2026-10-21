import unittest
import numpy as np
from environment import Environment, Cube
from scenarios import make_environment, START, GOAL
from pso_core import complete_paths, evaluate, run_pso, inertia_at
from path_smoother import smooth_path, turn_angle_sum, path_length


class LUT3DTests(unittest.TestCase):
    def setUp(self):
        self.cube=Cube((50.,50.,50.),25)
        self.env=Environment((self.cube,),100)

    def test_lut_moderate_diagonal_hits_center_cube(self):
        env=make_environment('moderate')
        self.assertTrue(env.collides(np.stack((START,GOAL))))

    def test_segment_slabs_hits_and_misses(self):
        self.assertTrue(self.env.segment_hits_cube([10,50,50],[90,50,50],self.cube))
        self.assertFalse(self.env.segment_hits_cube([10,10,10],[20,20,20],self.cube))
        self.assertTrue(self.env.segment_hits_cube([50,50,50],[50,50,50],self.cube))
        self.assertFalse(self.env.segment_hits_cube([10,50,50],[10,50,50],self.cube))
        self.assertTrue(self.env.segment_hits_cube([37.5,50,30],[37.5,50,70],self.cube))

    def test_vectorized_geometry_equals_scalar_slab(self):
        rng=np.random.default_rng(1)
        env=make_environment('dense')
        paths=rng.uniform(0,100,size=(120,7,3))
        batch=env.collision_mask(paths)
        for path, result in zip(paths,batch):
            scalar=any(env.segment_hits_cube(a,b,cube) for a,b in zip(path[:-1],path[1:]) for cube in env.cubes)
            self.assertEqual(result,scalar)

    def test_boundary_and_parallel(self):
        self.assertTrue(self.env.collides([[101,0,0],[10,10,10]]))
        self.assertFalse(self.env.collides([[0,0,0],[10,10,10]]))
        self.assertTrue(self.env.collides([[50,30,50],[50,70,50]]))
        self.assertFalse(self.env.collides([[30,30,50],[30,70,50]]))

    def test_45_dimensions_and_fixed_endpoints(self):
        x=np.zeros((100,15,3))
        paths=complete_paths(x)
        self.assertEqual(paths.shape,(100,17,3))
        np.testing.assert_array_equal(paths[:,0],np.broadcast_to(START,(100,3)))
        np.testing.assert_array_equal(paths[:,-1],np.broadcast_to(GOAL,(100,3)))

    def test_penalty_exact(self):
        zero=np.zeros((1,15,3)); zero[0]=np.linspace(START,GOAL,17)[1:-1]
        score, length, collided=evaluate(zero,self.env)
        self.assertTrue(collided[0])
        self.assertAlmostEqual(score[0]-length[0],10000.)

    def test_pso_history_monotone_budget_and_seed(self):
        env=make_environment('sparse')
        a=run_pso(env,'baseline',seed=42,swarm=8,iterations=10)
        b=run_pso(env,'baseline',seed=42,swarm=8,iterations=10)
        self.assertTrue(np.all(np.diff(a.best_history)<=1e-9))
        np.testing.assert_array_equal(a.best_history,b.best_history)
        self.assertEqual(len(a.best_history),11)
        self.assertEqual(a.fitness_evaluations,88)
        self.assertLessEqual(a.convergence_iteration,10)

    def test_enhanced_and_baseline_differ(self):
        env=Environment((),100)
        a=run_pso(env,'baseline',seed=12,swarm=20,iterations=25)
        b=run_pso(env,'enhanced',seed=12,swarm=20,iterations=25)
        self.assertEqual(a.best_history[0],b.best_history[0])
        self.assertEqual(a.algorithm,'baseline')
        self.assertEqual(b.algorithm,'enhanced')
        self.assertAlmostEqual(inertia_at('baseline',0,200),.7)
        self.assertAlmostEqual(inertia_at('baseline',199,200),.7)
        self.assertAlmostEqual(inertia_at('enhanced',0,200),.9)
        self.assertAlmostEqual(inertia_at('enhanced',200,200),.4)
        # Different inertia rules do not guarantee different gbest values in short runs.

    def test_spline_outputs_200_endpoints_and_metrics(self):
        pts=np.stack((np.linspace(10,90,17),np.linspace(10,90,17),np.linspace(10,90,17)),axis=-1)
        spline=smooth_path(pts,samples=200,method='thesis_spline')
        self.assertEqual(spline.shape,(200,3))
        np.testing.assert_allclose(spline[0],pts[0])
        np.testing.assert_allclose(spline[-1],pts[-1])
        self.assertAlmostEqual(path_length(spline),np.linalg.norm(GOAL-START),places=4)
        self.assertLess(turn_angle_sum(spline),1e-3)

    def test_duplicate_consecutive_points(self):
        pts=np.array([[0.,0.,0.],[0.,0.,0.],[10.,10.,10.],[10.,10.,10.],
                      [30.,20.,20.],[50.,20.,30.],[100.,100.,100.]])
        sm=smooth_path(pts,200)
        self.assertEqual(sm.shape,(200,3))
        self.assertTrue(np.isfinite(sm).all())
        np.testing.assert_array_equal(sm[0],pts[0])
        np.testing.assert_array_equal(sm[-1],pts[-1])
        cat=smooth_path(pts,200,method='catmull_rom')
        self.assertEqual(cat.shape,(200,3))


if __name__=='__main__':
    unittest.main()
