#!/usr/bin/env bash
set -euo pipefail
: "${ROS_DISTRO:?Source ROS 2 Lyrical first: source /opt/ros/lyrical/setup.bash}"
if [[ "${ROS_DISTRO}" != "lyrical" ]]; then
  echo "This workspace is targeted at ROS 2 Lyrical; current ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi
cd "$(dirname "$0")/.."
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y
colcon build --symlink-install
