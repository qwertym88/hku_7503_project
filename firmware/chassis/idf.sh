#!/usr/bin/env bash
# Keep ESP-IDF and micro-ROS Python dependencies separate from the ROS workspace.
set -e
chassis_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$chassis_dir/../.." && pwd)"
microros_dir="$project_root/.ref/micro_ros_espidf_component"
if ! git -C "$microros_dir" apply --reverse --check "$chassis_dir/microros-idf51.patch" 2>/dev/null; then
    git -C "$microros_dir" apply --check "$chassis_dir/microros-idf51.patch"
    git -C "$microros_dir" apply "$chassis_dir/microros-idf51.patch"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO
unset PYTHONPATH
source "${CHASSIS_IDF_PATH:-$HOME/esp/esp-idf}/export.sh" >/dev/null
export PYTHONPATH="$project_root/.ref/microros-python${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$project_root/.ref/microros-python/bin:$PATH"
export COLCON_HOME="$project_root/.ref/colcon"
export CCACHE_DIR="$project_root/.ref/ccache"
export IDF_COMPONENT_CACHE_PATH="$project_root/.ref/idf-component-cache"
cd "$chassis_dir"
# The vendor makefile does not track app-colcon.meta. Both chassis and motor_test
# share libmicroros.a, so check the built pool capacities before using that cache.
microros_app_dir="$chassis_dir"
previous_arg=""
for arg in "$@"; do
    if [ "$previous_arg" = "-C" ]; then microros_app_dir="$arg"; fi
    previous_arg="$arg"
done
if ! python - "$microros_app_dir/app-colcon.meta" "$microros_dir/include/rmw_microxrcedds_c/config.h" <<'PY'
import json, re, sys
from pathlib import Path
meta = json.loads(Path(sys.argv[1]).read_text())
config_path = Path(sys.argv[2])
config = config_path.read_text() if config_path.exists() else ''
for arg in meta['names']['rmw_microxrcedds']['cmake-args']:
    match = re.fullmatch(r'-D(RMW_UXRCE_MAX_\w+)=(\d+)', arg)
    if match:
        actual = re.search(r'^#define ' + match[1] + r'\s+(\d+)\s*$', config, re.M)
        if not actual or int(actual[1]) < int(match[2]):
            sys.exit(1)
PY
then
    touch "$microros_dir/esp32_toolchain.cmake.in"
    idf.py -C "$microros_app_dir" reconfigure
fi
# Vendor `cp -R install/include include` can leave old top-level headers when
# include already exists. Keep the headers consistent with the rebuilt archive.
if [ -d "$microros_dir/micro_ros_src/install/include" ]; then
    mkdir -p "$microros_dir/include"
    cp -a "$microros_dir/micro_ros_src/install/include/." "$microros_dir/include/"
fi
exec idf.py "$@"
