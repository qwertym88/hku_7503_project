#!/usr/bin/env bash
# M3 感知模块 —— 树莓派一键部署 + 冒烟测试（在开发机上运行）
#
# 用法：
#   bash tools/setup_pi_qr.sh                    # 默认 ubuntu-pi@10.42.0.1
#   bash tools/setup_pi_qr.sh ubuntu-pi@10.42.0.1 ~/ros2_ws
#
# 做五件事：
#   1. SSH 连通性检查
#   2. 装 Pi 侧依赖（python3-picamera2 / python3-opencv / libzbar0 + 可选 pyzbar）
#   3. rsync 同步 qr_perception 包 + tools + 离线数据集到 Pi
#   4. Pi 上 colcon build + 离线解码回归（验证 Pi 的 OpenCV 上 6 类码 100%）
#   5. picamera2 实拍冒烟：60 帧测帧率（M3-1 的 ≥15fps 验收）
#
# 注意：第 2 步需要 Pi 上的免密 sudo；不行就按提示手动装，脚本其余步骤照常。

set -euo pipefail

PI_HOST="${1:-ubuntu-pi@10.42.0.1}"
PI_DIR="${2:-~/hku_7503_project}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SSH_OPTS="-o ConnectTimeout=8"

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

# ---------- 1) 连通性 ----------
say "1/5 检查 SSH 连通性: $PI_HOST"
if ! ssh $SSH_OPTS "$PI_HOST" 'echo ok' >/dev/null 2>&1; then
  echo "❌ 连不上 $PI_HOST。检查：树莓派是否上电启动完、网线是否插好（本机 eno1 应为 10.42.0.2/24）。"
  exit 1
fi
echo "✅ SSH OK"

# ---------- 2) Pi 侧依赖 ----------
say "2/5 安装 Pi 侧依赖"
# Ubuntu 24.04 没有 python3-picamera2 包（那是 Pi OS 的）。取流两条路：
#   A) pip 装 picamera2==0.3.10（匹配 noble 的 libcamera 0.2.0）+ apt python3-libcamera —— 需要 sudo
#   B) rpicam-vid 子进程帧源 —— rpicam-apps 已预装，零依赖（setup 脚本自动探测）
if ssh $SSH_OPTS "$PI_HOST" 'sudo -n true' 2>/dev/null; then
  ssh $SSH_OPTS "$PI_HOST" '
    sudo apt-get update -qq || true
    sudo apt-get install -y -qq python3-libcamera libzbar0 python3-opencv >/dev/null
    echo "apt 依赖 OK"
  '
else
  echo "⚠️  Pi 上 sudo 需要密码。请手动在 Pi 上执行（装完重跑本脚本即可继续）："
  echo "    sudo apt install python3-libcamera libzbar0"
fi
ssh $SSH_OPTS "$PI_HOST" '
  if python3 -c "import cv2" 2>/dev/null; then echo "cv2 OK"; else echo "⚠️  缺 python3-opencv（apt 装）"; fi
  if python3 -c "import libcamera" 2>/dev/null; then echo "libcamera OK"; else echo "⚠️  缺 python3-libcamera（apt 装）"; fi
  if python3 -c "import picamera2" 2>/dev/null; then
    echo "picamera2 OK"
  elif python3 -c "import libcamera" 2>/dev/null && \
       pip install --user --break-system-packages --quiet "picamera2==0.3.10" >/dev/null 2>&1 && \
       python3 -c "import picamera2" 2>/dev/null; then
    echo "picamera2 已装（pip 0.3.10）"
  else
    echo "ℹ️  picamera2 不可用 —— 没关系，将使用 rpicam-vid 帧源（零依赖）"
  fi
  if python3 -c "import pyzbar" 2>/dev/null; then
    echo "pyzbar 已装"
  elif pip install --user --break-system-packages --quiet pyzbar >/dev/null 2>&1; then
    echo "pyzbar 已装（pip）"
  else
    echo "ℹ️  pyzbar 装不上（可选兜底，跳过不影响主流程）"
  fi
'

# ---------- 3) 同步代码 ----------
say "3/5 同步代码到 $PI_HOST:$PI_DIR"
ssh $SSH_OPTS "$PI_HOST" "mkdir -p $PI_DIR/ros2_ws/src $PI_DIR/tests"
rsync -az --delete "$REPO_ROOT/ros2_ws/src/qr_perception" "$PI_HOST:$PI_DIR/ros2_ws/src/"
rsync -az --delete "$REPO_ROOT/tools"                     "$PI_HOST:$PI_DIR/"
rsync -az --delete "$REPO_ROOT/tests/qr_dataset"          "$PI_HOST:$PI_DIR/tests/"
echo "✅ 同步完成"

# ---------- 4) Pi 上构建 + 离线回归 ----------
say "4/5 Pi 上 colcon build + 离线解码回归"
ssh $SSH_OPTS "$PI_HOST" "
  set -e
  cd $PI_DIR/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select qr_perception >/dev/null
  echo '✅ colcon build OK'
  python3 $PI_DIR/tools/qr_eval.py $PI_DIR/tests/qr_dataset/ || echo '❌ 离线回归未达标，先别上车'
"

# ---------- 5) 实拍冒烟 ----------
say "5/5 实拍冒烟（60 帧，自动选可用的帧源；可 Ctrl+C 跳过）"
ssh $SSH_OPTS "$PI_HOST" "
  cd $PI_DIR
  if python3 -c 'import picamera2' >/dev/null 2>&1; then
    SRC_FLAG=''
  else
    SRC_FLAG='--rpicam'
  fi
  if ! timeout 90 python3 tools/qr_preview.py \$SRC_FLAG --no-window --max-frames 60; then
    echo '⚠️  实拍冒烟失败：看上面的报错（常见：相机被占用 / CSI 线没插好）'
  fi
"

say "部署完成。下一步（M3-4 截图归档，在 Pi 上跑）："
echo "  python3 ~/hku_7503_project/tools/qr_preview.py --no-window --save ~/hku_7503_project/tests/qr_shots/"
echo "  然后把 6 类码（可用 tests/qr_print_sheet.png 打印或手机展示）依次举到相机前。"
