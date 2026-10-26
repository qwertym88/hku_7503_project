#!/usr/bin/env bash
# =============================================================================
# DASE7503 —— 在全新 Ubuntu 24.04 机器上一键装好开发环境
#
#   目标：装完即可 ① 起 Gazebo Harmonic 仿真场地 ② colcon build 工作区
#
#   用法：
#     bash tools/bootstrap_jazzy.sh              # 正常安装
#     bash tools/bootstrap_jazzy.sh --no-build   # 只装依赖，不构建
#     bash tools/bootstrap_jazzy.sh --no-linorobot2
#     bash tools/bootstrap_jazzy.sh --check      # 只体检，不装任何东西
#
#   幂等：可以反复跑。
#   注意：脚本**不会**替你安装 ROS 2 Jazzy 本体（那步有官方流程，见下方提示）。
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_TARGET="jazzy"
LINOROBOT2_BRANCH="jazzy"
WS="${REPO_ROOT}/ros2_ws"

DO_BUILD=1
DO_LINOROBOT2=1
CHECK_ONLY=0

for a in "$@"; do
  case "$a" in
    --no-build)      DO_BUILD=0 ;;
    --no-linorobot2) DO_LINOROBOT2=0 ;;
    --check)         CHECK_ONLY=1 ;;
    -h|--help)       sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数: $a"; exit 2 ;;
  esac
done

ok()   { printf "  \033[32mOK\033[0m   %s\n" "$*"; }
warn() { printf "  \033[33m注意\033[0m %s\n" "$*"; }
bad()  { printf "  \033[31m缺失\033[0m %s\n" "$*"; }
step() { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

FAIL=0

# -----------------------------------------------------------------------------
step "0/7 前置检查"
# -----------------------------------------------------------------------------
if [ -r /etc/os-release ]; then
  . /etc/os-release
  echo "  系统: ${PRETTY_NAME:-未知}"
  if [ "${VERSION_ID:-}" != "24.04" ]; then
    warn "本仓库只支持 Ubuntu 24.04（需求指定 ROS 2 Jazzy）。当前为 ${VERSION_ID:-未知}。"
    warn "在 22.04 上继续会拿到 Humble/Fortress，与本仓库配置不符 —— 建议换机器或装 24.04。"
    FAIL=1
  fi
fi

echo "  仓库根目录: ${REPO_ROOT}"
[ -f "${REPO_ROOT}/sim/scripts/generate_field_world.py" ] || { bad "找不到 sim/ 脚本，当前目录对不上仓库根"; FAIL=1; }

if [ -f "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash" ]; then
  ok "ROS 2 ${ROS_DISTRO_TARGET} 已安装"
  ROS_OK=1
else
  ROS_OK=0
  bad "未发现 /opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
fi

if command -v gz >/dev/null 2>&1; then
  if gz sim --version >/dev/null 2>&1; then
    ok "Gazebo Harmonic 可用（gz sim）"
  else
    warn "存在 gz 命令但没有 gz sim —— 很可能是 Gazebo Classic，不是我们要的 Harmonic"
  fi
else
  warn "尚无 gz 命令（将由 ros-jazzy-ros-gz 带入）"
fi

if [ "$ROS_OK" = 0 ]; then
  cat <<'EOF'

  ---------------------------------------------------------------
  请先安装 ROS 2 Jazzy（Ubuntu 24.04），官方步骤：
    https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
  最小集合（桌面版，含 RViz）：
    sudo apt install software-properties-common curl
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-jazzy-desktop ros-dev-tools
  装完重跑本脚本。
  ---------------------------------------------------------------
EOF
  [ "$CHECK_ONLY" = 1 ] && exit 1
  warn "先装 ROS 2 Jazzy 再回来。本次到此为止。"
  exit 1
fi

if [ "$CHECK_ONLY" = 1 ]; then
  [ "$FAIL" = 0 ] && echo -e "\n体检通过（--check 模式，未安装任何东西）" || echo -e "\n体检发现问题（见上）"
  exit "$FAIL"
fi

# -----------------------------------------------------------------------------
step "1/7 安装系统依赖与 ROS 2 软件包"
# -----------------------------------------------------------------------------
SUDO=""
[ "$(id -u)" != 0 ] && SUDO="sudo"

$SUDO apt update -y
# ROS 2 侧的依赖：仿真桥接 / 导航 / 建图 / 定位 / 图像
PKGS=(
  ros-jazzy-ros-gz              # Gazebo Harmonic ↔ ROS 2 桥接（含 ros_gz_sim / ros_gz_bridge）
  ros-jazzy-nav2-bringup        # Nav2
  ros-jazzy-nav2-common
  ros-jazzy-slam-toolbox        # 建图
  ros-jazzy-robot-localization  # EKF（linorobot2 的 odom 融合依赖）
  ros-jazzy-xacro
  ros-jazzy-robot-state-publisher
  python3-opencv                # M3 的 QR 解码
  python3-colcon-common-extensions
  git curl
)
MISSING=()
for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p"); done
if [ ${#MISSING[@]} -eq 0 ]; then
  ok "依赖齐全，无需安装"
else
  echo "  将安装: ${MISSING[*]}"
  $SUDO apt install -y "${MISSING[@]}" || { bad "apt 安装失败，请检查网络/源"; exit 1; }
  ok "依赖安装完成"
fi

# micro-ROS agent（可选，做 M2 联调时才需要）
if dpkg -s "ros-jazzy-micro-ros-agent" >/dev/null 2>&1; then
  ok "micro-ROS agent 已装"
else
  warn "未装 micro-ROS agent（只有联调 ESP32 时才需要）"
  warn "  可尝试: sudo apt install ros-jazzy-micro-ros-agent"
  warn "  若源里没有，按 https://micro.ros.org 的发行版对应方式安装，并注意"
  warn "  **agent 版本必须与 Pi/固件所属的 ROS 发行版成对**。"
fi

# -----------------------------------------------------------------------------
step "2/7 克隆 linorobot2（${LINOROBOT2_BRANCH} 分支）"
# -----------------------------------------------------------------------------
if [ "$DO_LINOROBOT2" = 1 ]; then
  DEST="${WS}/src/linorobot2"
  if [ -d "$DEST/.git" ]; then
    CUR="$(git -C "$DEST" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')"
    ok "已存在（分支 ${CUR}）"
    [ "$CUR" != "$LINOROBOT2_BRANCH" ] && warn "当前分支不是 ${LINOROBOT2_BRANCH}，需要时手动切：git -C $DEST checkout ${LINOROBOT2_BRANCH}"
  else
    mkdir -p "${WS}/src"
    git clone -b "$LINOROBOT2_BRANCH" https://github.com/linorobot/linorobot2.git "$DEST" \
      && ok "已克隆到 ${DEST}" || bad "克隆失败（网络？）"
  fi
else
  warn "按参数跳过 linorobot2"
fi

# -----------------------------------------------------------------------------
step "3/7 生成仿真世界（Harmonic 版）"
# -----------------------------------------------------------------------------
python3 "${REPO_ROOT}/sim/scripts/generate_field_world.py" --target harmonic >/dev/null \
  && ok "已生成 worlds/dase7503_field.sdf" || bad "生成失败"

# -----------------------------------------------------------------------------
step "4/7 环境变量"
# -----------------------------------------------------------------------------
cat <<'EOF'
  建议加进 ~/.bashrc（本脚本不自动改你的 shell 配置）：

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=197          # 仿真用 197；实机用 42
    export LINOROBOT2_BASE=mecanum    # linorobot2 选麦轮模型
    # 工作区编译后：
    # source ~/7139/ros2_ws/install/setup.bash
EOF

# -----------------------------------------------------------------------------
step "5/7 构建工作区"
# -----------------------------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
  if [ -d "$WS" ]; then
    ( set -e
      cd "$WS"
      # shellcheck disable=SC1091
      source "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
      colcon build --symlink-install
    ) && ok "colcon build 完成" || bad "colcon build 失败"
    echo "  提示：本仓库的 6 个 ROS 包目前都还是 README 空壳，"
    echo "        colcon 报 0 个包属正常（还没开始写代码）。"
  else
    bad "找不到 ${WS}"
  fi
else
  warn "按参数跳过构建"
fi

# -----------------------------------------------------------------------------
step "6/7 验证"
# -----------------------------------------------------------------------------
( set +u; # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
  echo "  --- 关键软件 ---"
  ros2 pkg prefix ros_gz_sim    >/dev/null 2>&1 && ok "ros_gz_sim（Harmonic 桥接）" || bad "ros_gz_sim"
  ros2 pkg prefix nav2_bringup  >/dev/null 2>&1 && ok "nav2_bringup（Nav2）"       || bad "nav2_bringup"
  ros2 pkg prefix slam_toolbox  >/dev/null 2>&1 && ok "slam_toolbox（建图）"       || bad "slam_toolbox"
  ros2 pkg prefix robot_localization >/dev/null 2>&1 && ok "robot_localization（EKF）" || bad "robot_localization"
  command -v gz >/dev/null 2>&1 && echo "  --- Gazebo ---" && (gz sim --version 2>&1 | head -2)
)

echo "  --- 场地自检 ---"
python3 "${REPO_ROOT}/sim/scripts/generate_field_world.py" --check >/dev/null 2>&1 \
  && ok "场地几何自检通过（无出界、无重叠）" || bad "场地自检未通过"

# -----------------------------------------------------------------------------
step "7/7 下一步"
# -----------------------------------------------------------------------------
cat <<EOF
  1) 起仿真场地（看布局）：
       source /opt/ros/jazzy/setup.bash
       ros2 launch sim/launch/field_world.launch.py

  2) 场地 + 麦轮机器人（需 linorobot2 已 build）：
       ros2 launch sim/launch/field_with_robot.launch.py

  3) 通读交接说明：${REPO_ROOT}/HANDOFF.md  （含不变量 / 通道尺寸 / 待办）
EOF

echo
if [ "$FAIL" = 0 ]; then echo "完成。"; else echo "完成，但有需要你处理的事项（见上面的「注意/缺失」）。"; fi
exit 0
