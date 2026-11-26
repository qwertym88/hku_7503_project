#!/usr/bin/env bash
set -eo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task_unit=dase-fixed-route.service
agent_unit=dase-chassis-agent.service
operation=${1:-start}
robot_mode=${DASE_ROBOT_MODE:-amcl}

case "$robot_mode" in
    amcl|encoder) ;;
    *) echo "未知定位模式: $robot_mode" >&2; exit 2 ;;
esac

case "$operation" in
    logs) exec journalctl --user -u "$task_unit" -n 25 -f ;;
    shutdown) exec systemctl --user stop "$task_unit" ;;
    start|reset|stop) if (( $# )); then shift; fi ;;
    -h|--help)
        script=run_robot.sh
        if [[ "$robot_mode" == encoder ]]; then script=run_robot_encoder.sh; fi
        echo "用法: $script [start|reset|stop|logs|shutdown] [ROS 启动参数]"
        exit 0 ;;
    *) echo "未知操作: $operation" >&2; exit 2 ;;
esac

setup=${DASE_ROS_SETUP:-$root/tools/run_robot.env.bash}
if [[ ! -f "$setup" && -z ${DASE_ROS_SETUP:-} ]]; then
    setup=$root/ros2_ws/install/setup.bash
fi
if [[ ! -f "$setup" ]]; then
    echo "找不到 ROS 工作区环境: $setup。请先构建 ros2_ws。" >&2
    exit 1
fi
setup=$(realpath -- "$setup")
source /opt/ros/jazzy/setup.bash
source "$setup"
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

if ! python3 -c 'from mission_control.hardware_command import main' >/dev/null 2>&1; then
    printf 'ROS 环境无法加载 mission_control.hardware_command: %s\n' "$setup" >&2
    exit 1
fi

service_env=(--expand-environment=no --property=UnsetEnvironment=ROS_LOCALHOST_ONLY)
# Reuse the checked overlay environment in the systemd processes.
for name in PATH PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH \
    ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE \
    ROS_STATIC_PEERS RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE; do
    if [[ -v $name ]]; then service_env+=("--setenv=$name=${!name}"); fi
done

if [[ "$operation" != stop ]]; then
    if ! ros2 pkg prefix robot_navigation >/dev/null 2>&1; then
        printf 'ROS 环境找不到 robot_navigation: %s\n' "$setup" >&2
        exit 1
    fi
    ros2_executable=$(command -v ros2)
    serial=${DASE_CHASSIS_PORT:-/dev/ttyUSB0}
    if systemctl --user is-active --quiet "$agent_unit"; then
        echo '使用已运行的底盘 Agent 服务'
    else
        serial_users=
        if command -v fuser >/dev/null && [[ -c "$serial" ]]; then
            serial_users=$(fuser "$serial" 2>/dev/null) || true
        fi
        if command -v docker >/dev/null && [[ $(docker inspect -f '{{.State.Running}}' hku-chassis-agent 2>/dev/null) == true ]]; then
            echo '使用已运行的底盘 Agent 容器'
        elif [[ -n "$serial_users" ]]; then
            if [[ $serial_users =~ ^[[:space:]]*([0-9]+)[[:space:]]*$ ]] &&
                    [[ $(ps -p "${BASH_REMATCH[1]}" -o comm=) == micro_ros_agent ]]; then
                echo '使用已占用该串口的 micro-ROS Agent'
            else
                echo "底盘串口已被其他进程占用: $serial" >&2
                exit 1
            fi
        elif [[ ! -c "$serial" ]]; then
            echo "找不到底盘串口: $serial" >&2
            exit 1
        elif ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
            systemd-run --user --collect --unit="$agent_unit" "${service_env[@]}" \
                "$ros2_executable" run micro_ros_agent micro_ros_agent serial --dev "$serial" -b 921600
        elif command -v docker >/dev/null; then
            systemd-run --user --collect --unit="$agent_unit" "${service_env[@]}" \
                docker run --rm --pull=never --name hku-chassis-agent --network host --device "$serial" \
                -e ROS_DOMAIN_ID=42 microros/micro-ros-agent:jazzy serial --dev "$serial" -b 921600
        else
            echo '找不到 micro_ros_agent 或 Docker' >&2
            exit 1
        fi
    fi
    if ! systemctl --user is-active --quiet "$task_unit"; then
        systemd-run --user --collect --unit="$task_unit" --working-directory="$root/ros2_ws" "${service_env[@]}" \
            --setenv="DASE_ROBOT_MODE=$robot_mode" \
            "$ros2_executable" launch robot_navigation fixed_route_hardware.launch.py "$@" \
            autostart:=false ros_domain_id:=42 localization_mode:="$robot_mode"
    else
        task_environment=$(systemctl --user show -p Environment --value "$task_unit")
        active_mode=amcl
        if [[ " $task_environment " == *" DASE_ROBOT_MODE=encoder "* ]]; then active_mode=encoder; fi
        if [[ "$active_mode" != "$robot_mode" ]]; then
            echo "任务服务当前使用 $active_mode 模式。切换模式前请先执行 shutdown。" >&2
            exit 2
        fi
        if (( $# )); then
            echo '任务服务已运行。使用同一脚本 shutdown 后可应用新的启动参数。' >&2
            exit 2
        fi
    fi
elif ! systemctl --user is-active --quiet "$task_unit"; then
    echo '任务服务未运行'
    exit 0
fi

if [[ "$operation" == stop ]]; then operation=cancel; fi
if ! python3 -m mission_control.hardware_command "$operation"; then
    journalctl --user -u "$task_unit" -u "$agent_unit" -n 15 --no-pager >&2
    exit 1
fi
