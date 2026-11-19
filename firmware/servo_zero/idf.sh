#!/usr/bin/env bash
# Standalone ESP-IDF project: does not rebuild or modify the micro-ROS cache.
set -euo pipefail
servo_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
servo_repo_root="$(cd -- "$servo_project_dir/../.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO PYTHONPATH
source "${CHASSIS_IDF_PATH:-$HOME/esp/esp-idf}/export.sh" >/dev/null
export CCACHE_DIR="$servo_repo_root/.ref/ccache"
export IDF_COMPONENT_CACHE_PATH="$servo_repo_root/.ref/idf-component-cache"
cd "$servo_project_dir"
exec idf.py "$@"
