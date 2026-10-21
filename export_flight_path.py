import csv
import numpy as np

from baseline_pso import optimize
from environment import Cube, Environment
from scenarios import START, GOAL, make_environment

SCENARIO = "sparse"
SAFETY_INFLATION = 10.0
MAX_SEEDS = 20

original_env = make_environment(SCENARIO)

# Giữ nguyên tâm chướng ngại vật, chỉ mở rộng kích thước
# dùng cho kiểm tra khoảng cách an toàn của demo.
safe_env = Environment(
    [
        Cube(cube.center, cube.size + SAFETY_INFLATION)
        for cube in original_env.cubes
    ],
    original_env.bound,
)

result = None

for seed in range(MAX_SEEDS):
    candidate = optimize(
        safe_env,
        seed=seed,
        swarm=100,
        iterations=200,
        jitter=2.5,
    )

    if not candidate.raw_collision:
        result = candidate
        print(f"Found collision-free route: seed={seed}")
        break

if result is None:
    raise SystemExit(
        "No collision-free route found in the tested seeds. "
        "No flight route was exported."
    )

path = result.path

print(f"Algorithm: {result.algorithm}")
print(f"Seed: {result.seed}")
print(f"Raw path length (PSO units): {result.raw_length:.3f}")
print(f"Collision in safety-inflated environment: {result.raw_collision}")
print(f"Collision in original environment: {original_env.collides(path)}")
print(f"Start: {path[0].tolist()}")
print(f"Goal:  {path[-1].tolist()}")

# CSV thứ nhất: tọa độ gốc của mô hình PSO 100 x 100 x 100.
with open("flight_path_pso_units.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z"])
    writer.writerows(path)

# CSV thứ hai: thu nhỏ 10 lần để phù hợp bản đồ Gazebo khoảng 10 m.
# PX4 dùng NED: z âm là hướng lên.
gazebo_path = path * 0.1

with open("flight_path_gazebo_ned.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z"])
    for x, y, z_up in gazebo_path:
        writer.writerow([x, y, -z_up])

print("Exported flight_path_pso_units.csv")
print("Exported flight_path_gazebo_ned.csv")
print("Gazebo scale: 0.1 m per PSO unit")
print("PX4 convention: NED, therefore z is negative above the origin")
