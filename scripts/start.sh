#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
source "$ROOT/install/setup.bash"
exec ros2 launch safe_obstacle_avoidance safe_obstacle_avoidance.launch.py "$@"
