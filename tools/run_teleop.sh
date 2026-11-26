#!/usr/bin/env bash
set -eo pipefail

tools_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    echo '用法: ./tools/run_teleop.sh [小车 IP] [--gamepad [设备编号]]'
    echo '默认使用窗口摇杆。--gamepad 使用连接到本机的手柄，设备编号默认为 0。'
    exit 0
fi

peer=${ROS_STATIC_PEERS:-10.199.166.59}
if [[ $# -gt 0 && $1 != --* ]]; then
    peer=$1
    shift
fi

source /opt/ros/jazzy/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="$peer"
exec /usr/bin/python3 "$tools_dir/robot_teleop.py" "$@"
