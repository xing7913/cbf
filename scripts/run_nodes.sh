#!/usr/bin/env bash
set -euo pipefail
: "${ROS_DISTRO:?Source ROS 2 first: source /opt/ros/lyrical/setup.bash}"
cd "$(dirname "$0")/.."
source install/setup.bash
ros2 launch safe_obstacle_avoidance safe_obstacle_avoidance.launch.py
