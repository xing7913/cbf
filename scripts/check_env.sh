#!/usr/bin/env bash
set -uo pipefail

fail=0
ok()   { printf '\033[32m[OK]\033[0m %s\n' "$1"; }
warn() { printf '\033[33m[WARN]\033[0m %s\n' "$1"; }
err()  { printf '\033[31m[FAIL]\033[0m %s\n' "$1"; fail=1; }

printf '\n=== Ubuntu / ROS 2 / Gazebo environment check ===\n'
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  echo "OS: ${PRETTY_NAME:-unknown}"
  [[ "${VERSION_ID:-}" == "26.04" ]] && ok "Ubuntu 26.04" || warn "Expected Ubuntu 26.04; detected ${VERSION_ID:-unknown}"
else
  err "/etc/os-release not found"
fi

if [[ -n "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO=${ROS_DISTRO}"
  [[ "${ROS_DISTRO}" == "lyrical" ]] && ok "ROS 2 Lyrical environment sourced" || warn "This repo was prepared for ROS 2 Lyrical; current distro is ${ROS_DISTRO}"
else
  err "ROS_DISTRO is empty. Run: source /opt/ros/lyrical/setup.bash"
fi

if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 command: $(command -v ros2)"
else
  err "ros2 command not found"
fi

# Gazebo is an assumption, not an installation target. This script never installs it.
if command -v gz >/dev/null 2>&1; then
  ok "Gazebo CLI found: $(command -v gz)"
  gz sim --versions 2>/dev/null || gz --versions 2>/dev/null || warn "Could not query Gazebo version; `gz` exists"
else
  err "Gazebo CLI 'gz' not found. Install Gazebo separately; this project will not install it."
fi

for pkg in rclpy cv_bridge sensor_msgs nav_msgs geometry_msgs ros_gz_bridge rviz2 visualization_msgs launch launch_ros safe_obstacle_avoidance_msgs; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    ok "ROS package available: $pkg"
  else
    warn "ROS package not found in current environment: $pkg (some are built from this workspace)"
  fi
done

python3 - <<'PY' || fail=1
mods = ["numpy", "cv2"]
for name in mods:
    try:
        mod = __import__(name)
        print(f"[OK] Python {name}: {getattr(mod, '__version__', 'available')}")
    except Exception as exc:
        print(f"[FAIL] Python {name}: {exc}")
        raise SystemExit(1)
PY

printf '\n'
if [[ "$fail" -eq 0 ]]; then
  ok "Environment check completed (warnings may still require attention)."
else
  err "One or more required checks failed."
fi
exit "$fail"
