# UR5e HRI 仿真 — 一键启动 + 滤波对比实验

本仓库提供完整的 UR5e 人机交互仿真环境，包含手势追踪控制和滤波器性能对比实验。

## 功能

| 功能 | 脚本 | 说明 |
|------|------|------|
| 摄像头手势控制 | `scripts/launch_ur5e.py` | Windows 摄像头 → MediaPipe 手势 → UDP → ROS2 → 机械臂跟随 |
| 滤波对比实验 V1 | `experiments/v1/` | 无滤波 vs EKF+FIR 基础对比 |
| 滤波对比实验 V2 | `experiments/v2/` | 公平起点对齐版 (推荐用于论文) |
| 新实验模板 | `experiments/TEMPLATE.py` | 复制后改一行即可添加新对比 |

## 前提条件

**WSL (Ubuntu 22.04):**

```bash
# 1. ROS2 Humble
#    参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 2. UR ROS2 Driver
sudo apt install ros-humble-ur-robot-driver

# 或者从源码编译:
mkdir -p ~/ws_ur_sim/src
cd ~/ws_ur_sim/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ws_ur_sim
source /opt/ros/humble/setup.bash
colcon build

# 3. scipy (滤波实验需要)
pip3 install scipy
```

**Windows:**

```powershell
pip install opencv-python mediapipe
```

## 安装

```bash
# 将 Robotics_project_10/ 放到 WSL 中，然后:
cd Robotics_project_10/
bash setup.sh
```

安装完成后 `~/` 下会出现 `launch_ur5e.py` 和 `run_experiments.py`。

## 用法

### 摄像头手势控制 (论文 HRI 部分)

```bash
# 终端 1: WSL
python3 ~/launch_ur5e.py

# 终端 2: Windows PowerShell
python windows/windows_vision_sensor.py

# 对着摄像头比 V 字手势 (食指+中指伸直)，机械臂跟随手移动
```

**数据流:**
```
Windows 摄像头 → MediaPipe 手指追踪
    → UDP JSON (172.27.64.221:5005)
    → WSL udp_bridge → intersection_calc
    → gesture_trigger → auto_commander (MoveIt)
    → Gazebo 中 UR5e 机械臂运动
```

### 滤波对比实验 (论文滤波部分)

```bash
# 查看可用实验
python3 ~/run_experiments.py list

# 运行 V2 实验 (推荐 — 公平起点对齐)
python3 ~/run_experiments.py v2

# 运行 V1 实验
python3 ~/run_experiments.py v1
```

每个实验自动启动 Gazebo + MoveIt，依次运行无滤波和 EKF+FIR 两个版本，生成两个 CSV 文件保存在 `~/experiments/vN/`。

### MATLAB 对比绘图

将 CSV 拷贝到 Windows，运行配套 MATLAB 脚本:

```matlab
run('CN_tracking_mappingerror_filtered_vs_non1.m')   % V1
run('CN_tracking_filtered_vs_non_v2.m')               % V2
```

输出: 统计报告 + Boxplot + 误差时序 + 3D 轨迹对比图。

## 添加新实验

```bash
cp ~/experiments/TEMPLATE.py ~/experiments/v3/run_new_experiment.py
# 编辑 EXPERIMENTS 列表 (只需改这一行)
python3 ~/experiments/v3/run_new_experiment.py
```

可用的 ROS2 节点 (在 `linear_hri_sim` 包中):

| 节点名 | 说明 |
|--------|------|
| `exp_non_filtered` | V1 无滤波 |
| `exp_ekf_fir` | V1 EKF+FIR |
| `exp_non_filtered_v2` | V2 无滤波 + 起点对齐 |
| `exp_ekf_fir_v2` | V2 EKF+FIR + 起点对齐 |
| `udp_bridge` | UDP 手势数据桥接 |
| `hand_monitor` | 实时监控手部位置 |
| `auto_commander` | MoveIt 自动执行 |

## 文件结构

```
Robotics_project_10/
├── README.md
├── setup.sh                        ← WSL 一键安装
├── windows/
│   └── windows_vision_sensor.py    ← Windows 摄像头手势捕捉
├── scripts/
│   ├── launch_ur5e.py              ← 摄像头手势控制 (WSL)
│   └── run_experiments.py          ← 实验统一入口
├── linear_hri_sim/                 ← ROS2 包源码
│   ├── setup.py
│   ├── package.xml
│   └── linear_hri_sim/
│       ├── udp_bridge.py           ← UDP 桥接
│       ├── hand_monitor.py         ← 实时监控
│       ├── auto_commander.py       ← 自动执行
│       ├── intersection_calc.py    ← 交点计算
│       ├── gesture_trigger.py      ← 手势触发
│       ├── virtual_finger.py       ← 虚拟手指
│       ├── exp_non_filtered.py     ← V1 无滤波实验
│       ├── exp_ekf_fir_filtered.py ← V1 滤波实验
│       ├── exp_non_filtered_v2.py  ← V2 无滤波实验
│       └── exp_ekf_fir_v2.py       ← V2 滤波实验
└── experiments/
    ├── TEMPLATE.py                 ← 新实验模板
    ├── v1/run_filter_experiment.py
    └── v2/run_filter_experiment_v2.py
```

## 依赖

- Ubuntu 22.04 (WSL2)
- ROS2 Humble
- UR ROS2 Driver
- Python 3.10+, scipy, numpy
- MoveIt 2, Gazebo
- Windows: opencv-python, mediapipe
