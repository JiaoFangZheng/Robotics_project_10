#!/usr/bin/env python3
"""
UR5e 仿真一键启动（VS Code / WSL 集成终端版）
用法: python3 ~/launch_ur5e.py
所有进程在后台运行，日志写入 ~/logs/，Ctrl+C 一键终止全部。
"""

import subprocess
import time
import os
import sys
import signal
from pathlib import Path

HOME = Path.home()
LOGDIR = HOME / "logs"

# 工作空间 setup 路径
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_UR = HOME / "ws_ur_sim/install/setup.bash"          # UR 仿真 + MoveIt 共用
WS_MY_ROS = HOME / "ws_my_ros/install/setup.bash"     # 你的算法包 (HRI 节点)
HRI_PKG = HOME / "ws_my_ros/src/gesture-finger/linear_hri_sim"

# 颜色
GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
RED = "\033[1;31m"
CYAN = "\033[1;36m"
RESET = "\033[0m"

# 所有子进程，用于 Ctrl+C 时统一终止
procs = []


def run_bg(label: str, script: str, logname: str) -> subprocess.Popen:
    """后台启动一个 bash 进程，输出写入日志文件"""
    LOGDIR.mkdir(exist_ok=True)
    logfile = open(LOGDIR / logname, "w")
    proc = subprocess.Popen(
        ["bash", "-c", script],
        stdout=logfile,
        stderr=subprocess.STDOUT,
    )
    procs.append(proc)
    print(f"  {GREEN}✓{RESET} {label}  PID={proc.pid}  log → ~/logs/{logname}")
    return proc


def cleanup(sig=None, frame=None):
    """终止所有子进程"""
    if not procs:
        return
    print(f"\n{YELLOW}[终止] 正在关闭全部 {len(procs)} 个进程...{RESET}")
    for proc in procs:
        try:
            proc.terminate()
        except Exception:
            pass
    time.sleep(1)
    for proc in procs:
        try:
            proc.kill()
        except Exception:
            pass
    print(f"{GREEN}[完成] 全部进程已终止{RESET}")
    sys.exit(0)


def ensure_hri_built():
    """检查 linear_hri_sim 是否已编译到 install/ 中"""
    hri_install = HOME / "ws_my_ros/install/linear_hri_sim"
    if hri_install.exists():
        return

    print(f"{YELLOW}[预检] linear_hri_sim 首次编译中...{RESET}")
    result = subprocess.run(
        f"source {ROS_SETUP} && colcon build --packages-select linear_hri_sim",
        cwd=HOME / "ws_my_ros",
        shell=True,
        executable="/bin/bash",
    )
    if result.returncode != 0:
        print(f"{RED}[错误] linear_hri_sim 编译失败！请手动检查{RESET}")
        sys.exit(1)
    print(f"{GREEN}[完成] linear_hri_sim 编译成功！\n{RESET}")


def ensure_moveit_built():
    """检查 ur_moveit_config 是否已编译到 ws_ur_sim/install/"""
    moveit_install = HOME / "ws_ur_sim/install/ur_moveit_config"
    if moveit_install.exists():
        return

    print(f"{YELLOW}[预检] ur_moveit_config 未编译，正在执行一次性构建...{RESET}")

    moveit_src = HOME / "ur_backup/Universal_Robots_ROS2_Driver/ur_moveit_config"
    moveit_link = HOME / "ws_ur_sim/src/ur_moveit_config"

    # 1. 软链接源码
    if not moveit_link.exists():
        print(f"  [1/2] 软链接 {moveit_link} -> {moveit_src}")
        os.symlink(moveit_src, moveit_link)

    # 2. 编译（需要先 source ROS 环境）
    print(f"  [2/2] colcon build --packages-select ur_moveit_config ...")
    result = subprocess.run(
        f"source {ROS_SETUP} && colcon build --packages-select ur_moveit_config",
        cwd=HOME / "ws_ur_sim",
        shell=True,
        executable="/bin/bash",
    )
    if result.returncode != 0:
        print(f"{RED}[错误] 编译失败，请手动检查{RESET}")
        sys.exit(1)

    print(f"{GREEN}[完成] ur_moveit_config 编译成功！\n{RESET}")


def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f"""
{CYAN}╔══════════════════════════════════════╗
║     UR5e 仿真一键启动 (VS Code)      ║
╚══════════════════════════════════════╝{RESET}
""")

    # 预检
    ensure_moveit_built()
    ensure_hri_built()

    # 1. Gazebo 仿真
    print(f"{CYAN}[1/4] 启动 Gazebo 物理仿真...{RESET}")
    run_bg(
        "Gazebo 仿真",
        f"source {ROS_SETUP} && source {WS_UR} && "
        f"ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e",
        "gazebo.log",
    )
    print(f"  {YELLOW}⏳ 等待 Gazebo 初始化 (约 8 秒)...{RESET}")
    time.sleep(8)

    # 2. MoveIt2 + RViz
    print(f"\n{CYAN}[2/4] 启动 MoveIt2 + RViz...{RESET}")
    run_bg(
        "MoveIt2 + RViz",
        f"source {ROS_SETUP} && source {WS_UR} && "
        f"ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true",
        "moveit.log",
    )
    time.sleep(5)

    # 3. HRI 算法节点 (UDP 桥接 + 交点计算 + 手势触发 + 自动执行)
    print(f"\n{CYAN}[3/4] 启动 HRI 算法节点...{RESET}")
    os.system(f"fuser -k 5005/udp 2>/dev/null")
    run_bg(
        "UDP 桥接 (接收 Windows 手势数据)",
        f"source {ROS_SETUP} && source {WS_MY_ROS} && "
        f"ros2 run linear_hri_sim udp_bridge",
        "udp_bridge.log",
    )
    time.sleep(0.3)
    run_bg(
        "交点计算 intersection_calc",
        f"source {ROS_SETUP} && source {WS_MY_ROS} && "
        f"ros2 run linear_hri_sim intersection_calc",
        "intersection.log",
    )
    time.sleep(0.3)
    run_bg(
        "手势触发 + TF 发布 gesture_trigger",
        f"source {ROS_SETUP} && source {WS_MY_ROS} && "
        f"ros2 run linear_hri_sim gesture_trigger",
        "trigger.log",
    )
    time.sleep(0.3)
    run_bg(
        "MoveIt 自动执行 auto_commander",
        f"source {ROS_SETUP} && source {WS_MY_ROS} && "
        f"ros2 run linear_hri_sim auto_commander",
        "commander.log",
    )
    time.sleep(0.3)
    # hand_monitor 运行在前台，用户可以直接看到实时数据
    print(f"\n{CYAN}[4/4] 实时数据监控 (前台运行){RESET}")
    print(f"  {GREEN}↓ 下面会实时打印手的位置，证明互通 ↓{RESET}\n")

    monitor_proc = subprocess.Popen(
        ["bash", "-c",
         f"source {ROS_SETUP} && source {WS_MY_ROS} && "
         f"ros2 run linear_hri_sim hand_monitor"],
    )
    procs.append(monitor_proc)

    print(f"{YELLOW}在 Windows 端运行: python windows_vision_sensor.py{RESET}")
    print(f"{YELLOW}比出 V 字手势 → 机械臂自动移动！{RESET}")
    print(f"{YELLOW}Ctrl+C 退出{RESET}")
    print(f"{'-'*50}\n")

    # 等待任意子进程结束或用户 Ctrl+C
    try:
        while any(p.poll() is None for p in procs):
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    main()
