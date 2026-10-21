"""Generate 3-D scene examples, convergence charts, and LUT Table 1 comparisons."""

# Vẽ 3D, convergence và so sánh kết quả LUT
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

REFERENCE = {
    ('sparse', 'baseline'): (179.08, 16.92, 0., 191.5),
    ('sparse', 'enhanced'): (162.96, 11.15, 0., 198.8),
    ('moderate', 'baseline'): (141.10, 5.18, 100., 189.6),
    ('moderate', 'enhanced'): (139.53, 2.93, 100., 199.6),
    ('dense', 'baseline'): (187.95, 13.57, 50., 181.0),
    ('dense', 'enhanced'): (186.50, 13.77, 10., 199.5),
}


def _cube_faces(low, high):
    x0,y0,z0=low; x1,y1,z1=high
    v=np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
                [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])
    return [v[idx] for idx in ([0,1,2,3],[4,5,6,7],[0,1,5,4],
                              [2,3,7,6],[1,2,6,5],[0,3,7,4])]


def plot_3d(name, env, paths, output):
    fig=plt.figure(figsize=(9,7))
    ax=fig.add_subplot(111, projection='3d')
    for c in env.cubes:
        ax.add_collection3d(Poly3DCollection(_cube_faces(c.lower, c.upper),
              alpha=.16, facecolor='red', edgecolor='darkred', linewidth=.5))
    raw, curve, algorithm, collided, smooth_hit=paths
    ax.plot(*raw.T, '--o', markersize=2, lw=.85, color='royalblue', label='Raw PSO waypoints')
    ax.plot(*curve.T, color='green', lw=2, label='Spline (200 samples)')
    ax.scatter(*raw[0], c='green', s=48, label='Start')
    ax.scatter(*raw[-1], c='red', s=48, label='Goal')
    ax.set(xlim=(0,100),ylim=(0,100),zlim=(0,100),
           xlabel='X', ylabel='Y', zlabel='Z',
           title=f'{name} | {algorithm} | raw collision={collided}, spline collision={smooth_hit}')
    ax.legend(loc='upper left',fontsize=8)
    fig.tight_layout();fig.savefig(output,dpi=145);plt.close(fig)


def plot_convergence(name, histories, output):
    fig, ax=plt.subplots(figsize=(7,4))
    for alg, hist in histories.items():
        ax.plot(hist,label=alg)
    ax.set(xlabel='PSO update iteration',ylabel='Best-so-far fitness',
           title=f'{name} | seed 0 | fitness = raw length + collision penalty')
    ax.legend();ax.grid(alpha=.25);fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)


def make_reference_comparison(summary, outfile):
    rows=[]
    for row in summary:
        key=(row['scenario'],row['algorithm'])
        ref=REFERENCE[key]
        rows.append(dict(scenario=key[0],algorithm=key[1],
            thesis_length_mean=ref[0],ours_length_mean=row['length_mean'],
            thesis_smoothness_mean=ref[1],ours_smoothness_mean=row['smoothness_mean_rad'],
            thesis_collision_rate_pct=ref[2],ours_raw_collision_rate_pct=row['collision_rate_pct'],
            ours_spline_collision_rate_pct=row['spline_collision_rate_pct'],
            thesis_convergence_mean=ref[3],ours_convergence_mean=row['convergence_mean']))
    with open(outfile,'w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()));w.writeheader();w.writerows(rows)
