"""Batch experiments reproducing LUT §4.1, producing figures + raw and summary CSV."""

# Điều khiển toàn bộ thí nghiệm

import argparse
import json
import platform
import sys
from pathlib import Path
import numpy as np
import scipy
import matplotlib
from analysis import plot_3d, plot_convergence, make_reference_comparison
from baseline_pso import optimize as baseline
from pso import optimize as enhanced
from data_processor import save_csv, summarize
from path_smoother import smooth_path, path_length, turn_angle_sum
from pso_core import POPULATION, ITERATIONS, N_WAYPOINTS, C1, C2, JITTER, PENALTY
from scenarios import SCENARIOS, START, GOAL, make_environment


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials',type=int,default=10)
    parser.add_argument('--particles',type=int,default=POPULATION)
    parser.add_argument('--iterations',type=int,default=ITERATIONS)
    parser.add_argument('--scenario',choices=['all',*SCENARIOS],default='all')
    parser.add_argument('--smoothing',choices=['thesis_spline','catmull_rom'],default='thesis_spline')
    parser.add_argument('--jitter',type=float,default=JITTER,help='Sensitivity analysis; thesis default is 2.5')
    parser.add_argument('--output',type=Path,default=Path('results/recreation'))
    args=parser.parse_args()
    if min(args.trials,args.particles,args.iterations)<1 or args.jitter<0:
        parser.error('trials/particles/iterations must be >=1, jitter must be >=0')
    args.output.mkdir(parents=True,exist_ok=True)
    config=dict(N_waypoints=N_WAYPOINTS,particles=args.particles,iterations=args.iterations,
        trials=args.trials,jitter=args.jitter,penalty=PENALTY,c1=C1,c2=C2,
        start=START.tolist(),goal=GOAL.tolist(),scenario=args.scenario,
        smoothing=args.smoothing,python=sys.version,numpy=np.__version__,scipy=scipy.__version__,
        matplotlib=matplotlib.__version__,os=platform.platform(),
        provenance='Independently authored methodological reproduction, NOT original LUT source',
        limits='Velocity initialization/implementation random ordering unknown; duplicate chordal knots fixed')
    (args.output/'config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    rows=[]
    names=SCENARIOS if args.scenario=='all' else [args.scenario]
    for name in names:
        env=make_environment(name)
        samples={}
        curves={}
        for algorithm,optim in [('baseline',baseline),('enhanced',enhanced)]:
            for seed in range(args.trials):
                r=optim(env, seed=seed,swarm=args.particles,iterations=args.iterations,jitter=args.jitter)
                spline=smooth_path(r.path,method=args.smoothing)
                smoothing_hit=env.collides(spline)
                rows.append(dict(scenario=name,algorithm=algorithm,seed=seed,
                    fitness=r.fitness,raw_length=r.raw_length,
                    smoothed_length=path_length(spline),smoothness_rad=turn_angle_sum(spline),
                    raw_collision=int(r.raw_collision),smoothed_collision=int(smoothing_hit),
                    convergence_iteration=r.convergence_iteration,
                    runtime_sec=r.runtime_seconds,fitness_evaluations=r.fitness_evaluations))
                if seed==0:
                    curves[algorithm]=r.best_history.tolist()
                    samples[algorithm]=(r.path,spline,algorithm,r.raw_collision,smoothing_hit)
                    save_csv([{'iteration':i,'gbest_fitness':f}
                              for i,f in enumerate(r.best_history)],
                             args.output/f'convergence_{name}_{algorithm}_seed0.csv')
            print(f'Finished {name}/{algorithm}: {args.trials} trial(s)',flush=True)
        plot_3d(name,env,samples['baseline'],args.output/f'path_{name}_baseline_seed0.png')
        plot_3d(name,env,samples['enhanced'],args.output/f'path_{name}_enhanced_seed0.png')
        plot_convergence(name,curves,args.output/f'convergence_{name}_seed0.png')
        save_csv(rows,args.output/'runs.csv')
    summary=summarize(rows)
    save_csv(summary,args.output/'summary.csv')
    make_reference_comparison(summary,args.output/'thesis_comparison.csv')
    print('Written:',args.output.resolve(),flush=True)


if __name__=='__main__':
    main()
