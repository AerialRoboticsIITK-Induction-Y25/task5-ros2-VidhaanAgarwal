#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

if [[ -d /workspace/fleet_ws/src/drone_fleet ]]; then
  cd /workspace/fleet_ws
  if command -v colcon >/dev/null 2>&1; then
    colcon build --symlink-install
  fi
  if [[ -f /workspace/fleet_ws/install/setup.bash ]]; then
    source /workspace/fleet_ws/install/setup.bash
  fi
else
  if [[ -f /workspace/fleet_ws/install/setup.bash ]]; then
    source /workspace/fleet_ws/install/setup.bash
  fi
fi

exec "$@"