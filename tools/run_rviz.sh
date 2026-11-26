#!/usr/bin/env bash
set -eo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    echo "用法: $0 [小车 IP，默认 10.199.166.59]"
    exit 0
fi
source /opt/ros/jazzy/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=${1:-${ROS_STATIC_PEERS:-10.199.166.59}}
exec rviz2 -d "$root/ros2_ws/src/robot_navigation/rviz/fixed_route.rviz"
