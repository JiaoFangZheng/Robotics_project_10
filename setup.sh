#!/bin/bash
# =============================================================
#  UR5e HRI 一键启动 + 滤波对比实验 — WSL 安装脚本
#  用法: bash setup.sh
# =============================================================
set -e

echo "============================================"
echo "  UR5e HRI 仿真环境安装"
echo "============================================"

# 检查 ROS2
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[错误] 未找到 ROS2 Humble，请先安装:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
source /opt/ros/humble/setup.bash

# 检查 scipy
python3 -c "import scipy" 2>/dev/null || {
    echo "[安装] scipy..."
    pip3 install scipy
}

WORKSPACE=~/ws_my_ros
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# 1. 创建工作空间
echo ""
echo "[1/4] 创建 ROS2 工作空间..."
mkdir -p $WORKSPACE/src
cd $WORKSPACE

# 2. 安装 linear_hri_sim 包
echo "[2/4] 安装 linear_hri_sim 包..."
cp -r "$SCRIPT_DIR/linear_hri_sim" $WORKSPACE/src/
# 确保 resource 文件存在
mkdir -p $WORKSPACE/src/linear_hri_sim/resource
touch $WORKSPACE/src/linear_hri_sim/resource/linear_hri_sim

# 3. 编译
echo "[3/4] 编译 ROS2 包 (首次约需 1 分钟)..."
colcon build --packages-select linear_hri_sim
source $WORKSPACE/install/setup.bash

# 4. 安装启动脚本
echo "[4/4] 安装启动脚本到 ~/ ..."
cp "$SCRIPT_DIR/scripts/launch_ur5e.py" ~/
cp "$SCRIPT_DIR/scripts/run_experiments.py" ~/
mkdir -p ~/experiments
cp -r "$SCRIPT_DIR/experiments/"* ~/experiments/
chmod +x ~/launch_ur5e.py ~/run_experiments.py

echo ""
echo "============================================"
echo "  安装完成！"
echo "============================================"
echo ""
echo "  前提条件 (自行安装):"
echo "    1. ROS2 Humble"
echo "    2. UR ROS2 Driver (ur_simulation_gz, ur_moveit_config)"
echo "       安装方法:"
echo "       sudo apt install ros-humble-ur-robot-driver"
echo "       mkdir -p ~/ws_ur_sim/src"
echo "       cd ~/ws_ur_sim/src"
echo "       git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git"
echo "       cd ~/ws_ur_sim"
echo "       source /opt/ros/humble/setup.bash"
echo "       colcon build"
echo ""
echo "  用法:"
echo "    python3 ~/launch_ur5e.py          # 摄像头手势控制"
echo "    python3 ~/run_experiments.py v2   # V2 滤波实验"
echo "    python3 ~/run_experiments.py list # 查看所有实验"
echo ""
