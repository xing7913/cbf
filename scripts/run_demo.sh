#!/usr/bin/env bash
# ROS setup scripts may reference unset variables; source before enabling nounset.
source /opt/ros/lyrical/setup.bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo CLI 'gz' was not found. Install Gazebo separately; this script will not install it." >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. Run ./scripts/install_ros_deps.sh first." >&2
  exit 2
fi
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  "$ROOT/scripts/build.sh"
fi
source "$ROOT/install/setup.bash"
cd "$ROOT"
exec ros2 launch safe_obstacle_avoidance safe_obstacle_avoidance.launch.py "$@"
