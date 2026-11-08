#!/usr/bin/env bash
set -euo pipefail
: "${ROS_DISTRO:?Source ROS 2 first: source /opt/ros/lyrical/setup.bash}"
source "$(dirname "$0")/../install/setup.bash"
required=(/camera/depth/image_raw /odom /safe_set_config /ref_vel /cmd_vel)
for topic in "${required[@]}"; do
  if ros2 topic list | grep -Fxq "$topic"; then
    printf '\033[32m[OK]\033[0m %s : ' "$topic"
    ros2 topic type "$topic" || true
  else
    printf '\033[31m[MISSING]\033[0m %s\n' "$topic"
  fi
done
