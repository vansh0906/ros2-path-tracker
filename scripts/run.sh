#!/bin/bash
# Container entry point.
#   (no args) or launch args  -> ros2 launch path_tracker sim.launch.py <args>
#   bench                     -> C++ benchmark (writes /results/bench_traj.csv)
#   bash                      -> a shell with the workspace sourced
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
mkdir -p /results
case "$1" in
  bench) cd /results && exec /ws/install/path_tracker/lib/path_tracker/bench ;;
  bash)  exec bash ;;
  *)     exec ros2 launch path_tracker sim.launch.py "$@" ;;
esac
