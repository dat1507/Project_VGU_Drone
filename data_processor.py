"""LUT §3.1: aggregate 3 scenarios x 2 PSO algorithms (mean ± sample std)."""
# Tổng hợp trung bình, độ lệch chuẩn, tỷ lệ va chạm

import csv
from collections import defaultdict
import numpy as np


def save_csv(rows, filepath):
    if not rows:
        return
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def describe(items):
    if len(items) == 0:
        return float('nan'), float('nan')
    x = np.asarray(items, dtype=float)
    return float(x.mean()), float(np.std(x, ddof=1)) if len(x) > 1 else 0.0


def summarize(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['scenario'], row['algorithm'])].append(row)
    out = []
    for (scenario, algorithm), records in sorted(grouped.items()):
        mean_l, std_l = describe([r['smoothed_length'] for r in records])
        mean_s, std_s = describe([r['smoothness_rad'] for r in records])
        mean_c, std_c = describe([r['convergence_iteration'] for r in records])
        mean_t, std_t = describe([r['runtime_sec'] for r in records])
        good = [r for r in records if not r['raw_collision']]
        mean_feas, sd_feas = describe([r['smoothed_length'] for r in good])
        out.append(dict(scenario=scenario, algorithm=algorithm, n=len(records),
                        length_mean=mean_l, length_std=std_l,
                        smoothness_mean_rad=mean_s, smoothness_std_rad=std_s,
                        collision_rate_pct=100*sum(r['raw_collision'] for r in records)/len(records),
                        spline_collision_rate_pct=100*sum(r['smoothed_collision'] for r in records)/len(records),
                        convergence_mean=mean_c, convergence_std=std_c,
                        runtime_mean_sec=mean_t, runtime_std_sec=std_t,
                        feasible_smoothed_length_mean=mean_feas,
                        feasible_smoothed_length_std=sd_feas))
    return out
