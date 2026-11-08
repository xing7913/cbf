#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "lyrical" ]]; then
  echo "ERROR: source ROS 2 Lyrical first: source /opt/ros/lyrical/setup.bash" >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-opencv \
  ros-lyrical-cv-bridge \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-teleop-twist-keyboard \
  python3-matplotlib

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# Installs only the ROS bridge package, not ros_gz_sim or the Gazebo simulator.
echo "ROS dependencies installed. Gazebo itself was intentionally left untouched."
