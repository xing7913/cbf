#!/usr/bin/env bash
set -euo pipefail
# Only target processes started with this project's world / launch file.
pkill -INT -f 'gz sim.*ellipsoid_obstacle_world.sdf' 2>/dev/null || true
pkill -INT -f 'ros2 launch safe_obstacle_avoidance safe_obstacle_avoidance.launch.py' 2>/dev/null || true
sleep 1
pkill -TERM -f 'gz sim.*ellipsoid_obstacle_world.sdf' 2>/dev/null || true
pkill -TERM -f 'ros_gz_bridge.*parameter_bridge' 2>/dev/null || true
echo "Stop signals sent to this project's simulation processes."
