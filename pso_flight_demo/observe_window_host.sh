#!/usr/bin/env bash
# Source existing installations and run READ-ONLY preparation, never physics.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
source /home/phuocdat/ros2_px4_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=0
export PYTHONNOUSERSITE=1
if [[ "${1:-}" == "check-window" ]]; then
  exec python3 -B -m pso_flight_demo.runtime_preparation "$@"
fi
if [[ $# -ne 0 ]]; then
  echo 'Usage: bash observe_window_host.sh [check-window DIRECTORY --operator-approval APPROVE_CLOCK_20000_25000]' >&2
  exit 2
fi
exec python3 -B -m pso_flight_demo.runtime_preparation --expect-iterations 20000 observe-window --wall-seconds 60
