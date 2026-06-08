# UR5e HRI 仿真 — 一键启动 + 滤波对比实验

本仓库提供完整的 UR5e 人机交互仿真环境：Windows 摄像头手势控制机械臂，以及五组滤波器性能对比实验。

## 快速开始

**组员:** 跳过下面，直接看 [安装](#安装) → [用法](#用法)。  
**自己:** `python3 ~/run_experiments.py v3`

---

## 功能

| 功能 | 入口 | 说明 |
|------|------|------|
| 摄像头手势控制 | `scripts/launch_ur5e.py` | Windows 摄像头 → MediaPipe → UDP → ROS2 → 机械臂跟随 |
| 滤波对比实验 | `scripts/run_experiments.py` | 五组实验，一键启动 Gazebo + MoveIt + 数据记录 |

## 五组滤波实验

| 实验 | 滤波器 | 命令信号 STD 改善 | 核心结论 |
|------|--------|:---------------:|------|
| V1 | EKF + FIR | 81.4% | 基础对比，但起点不一致 |
| V2 | EKF + FIR + 起点对齐 | 75.3% | 公平对比基准 |
| **V3** | **KF + 创新门** | **88.4%** | 🏆 **综合最优** |
| V4 | 纯 KF（无创新门） | 6.2% | 证明创新门对野值拒绝的必要性 |
| V5 | KF 2.0（距离跳变+FIR+MA）| 84.9% | 最强平滑，但引入滞后 |

每个实验文件夹 (`experiments/vN/`) 包含：启动脚本 + CSV 数据 + MATLAB 对比脚本 + 结果图。

## 安装

**前提:** WSL Ubuntu 22.04, ROS2 Humble, UR ROS2 Driver.

```bash
# 1. 克隆仓库到 WSL
cd ~
git clone https://github.com/JiaoFangZheng/Robotics_project_10.git

# 2. 安装 ROS2 依赖
sudo apt install ros-humble-ur-robot-driver
pip3 install scipy

# 3. 编译 UR 仿真工作空间 (如果还没有)
mkdir -p ~/ws_ur_sim/src
cd ~/ws_ur_sim/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ws_ur_sim
source /opt/ros/humble/setup.bash
colcon build

# 4. 一键安装本仓库
cd ~/Robotics_project_10
bash setup.sh
```

**Windows 端** (摄像头手势控制需要):
```powershell
pip install opencv-python mediapipe
```

## 用法

### 摄像头手势控制

```bash
# WSL 终端
python3 ~/launch_ur5e.py

# Windows PowerShell (另一个终端)
cd Robotics_project_10/windows
python windows_vision_sensor.py
```

对着摄像头比 **V 字手势** (食指+中指伸直), 机械臂跟随手部移动。

数据流:
```
Windows 摄像头 → MediaPipe → UDP :5005
  → WSL udp_bridge → intersection_calc
  → gesture_trigger → auto_commander (MoveIt)
  → Gazebo 中 UR5e 运动
```

### 滤波对比实验

```bash
# 查看所有实验
python3 ~/run_experiments.py list

# 运行实验 (推荐 V3)
python3 ~/run_experiments.py v3
```

每个实验自动: 启动 Gazebo → 启动 MoveIt → 依次运行无滤波和滤波版本 → 生成两个 CSV 到 `~/experiments/vN/`。

### MATLAB 绘图

CSV 和 MATLAB 脚本已在 `experiments/vN/` 中，直接打开 `.m` 文件运行即可。

## 文件结构

```
Robotics_project_10/
├── README.md
├── setup.sh                           ← WSL 一键安装
├── .gitignore
├── windows/
│   └── windows_vision_sensor.py       ← Windows 摄像头脚本
├── scripts/
│   ├── launch_ur5e.py                 ← 手势控制启动
│   └── run_experiments.py             ← 实验统一入口
├── linear_hri_sim/                    ← ROS2 包源码
│   ├── setup.py / package.xml
│   └── linear_hri_sim/
│       ├── udp_bridge.py              ← UDP → ROS2 桥接
│       ├── auto_commander.py          ← MoveIt 自动执行
│       ├── intersection_calc.py       ← 射线-桌面交点
│       ├── gesture_trigger.py         ← 手势触发 → TF
│       ├── hand_monitor.py            ← 实时数据监控
│       ├── virtual_finger.py          ← RViz 手动虚拟手指
│       ├── exp_non_filtered.py        ← V1 无滤波
│       ├── exp_ekf_fir_filtered.py    ← V1 EKF+FIR
│       ├── exp_non_filtered_v2.py     ← V2 无滤波+对齐
│       ├── exp_ekf_fir_v2.py          ← V2 EKF+FIR+对齐
│       ├── exp_kf_v3.py               ← V3 KF+创新门
│       ├── exp_kf_v4.py               ← V4 纯KF
│       └── exp_kf_v5.py               ← V5 KF 2.0
└── experiments/
    ├── TEMPLATE.py                    ← 新实验模板
    ├── v1/  (脚本+CSV+MATLAB+图)
    ├── v2/  (脚本+CSV+MATLAB+图)
    ├── v3/  (脚本+CSV+MATLAB+图)
    ├── v4/  (脚本+CSV+MATLAB+图)
    └── v5/  (脚本+CSV+MATLAB+图)
```

## 添加新实验

```bash
mkdir -p ~/experiments/v6
cp ~/experiments/TEMPLATE.py ~/experiments/v6/run_my_experiment.py
# 编辑 EXPERIMENTS 列表, 改节点名即可
```

## 所有 ROS2 节点

| 节点 | 用途 |
|------|------|
| `udp_bridge` | UDP 手势数据 → ROS2 |
| `auto_commander` | MoveIt 自动执行 |
| `intersection_calc` | 射线-桌面交点计算 |
| `gesture_trigger` | V字手势 → TF 目标 |
| `hand_monitor` | 终端实时打印手部位置 |
| `virtual_finger` | RViz 交互式虚拟手指 |
| `exp_non_filtered` | V1 无滤波实验 |
| `exp_ekf_fir` | V1 EKF+FIR |
| `exp_non_filtered_v2` | V2 无滤波+对齐 |
| `exp_ekf_fir_v2` | V2 EKF+FIR+对齐 |
| `exp_kf_v3` | V3 KF+创新门 |
| `exp_kf_v4` | V4 纯KF |
| `exp_kf_v5` | V5 KF 2.0 |
