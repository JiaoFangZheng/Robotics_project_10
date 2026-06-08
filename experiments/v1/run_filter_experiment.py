#!/usr/bin/env python3
"""
滤波对比实验：依次运行无滤波版 / EKF+FIR 滤波版，生成两个 CSV 文件。
用法: python3 ~/run_filter_experiment.py
输出: ~/tracking_error_non_filtered_fair.csv
      ~/tracking_error_ekf_fir_stronger1.csv
"""
import subprocess
import time
import os
import sys
import signal
from pathlib import Path

HOME = Path.home()
LOGDIR = HOME / "logs"
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_UR = HOME / "ws_ur_sim/install/setup.bash"
WS_MY_ROS = HOME / "ws_my_ros/install/setup.bash"

GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
RED = "\033[1;31m"
CYAN = "\033[1;36m"
RESET = "\033[0m"

procs = []


def run_bg(label: str, script: str, logname: str):
    LOGDIR.mkdir(exist_ok=True)
    logfile = open(LOGDIR / logname, "w")
    proc = subprocess.Popen(
        ["bash", "-c", script],
        stdout=logfile, stderr=subprocess.STDOUT,
    )
    procs.append(proc)
    print(f"  {GREEN}✓{RESET} {label}  PID={proc.pid}")
    return proc


def run_fg(script: str):
    """前台运行，实时显示输出"""
    return subprocess.run(["bash", "-c", script]).returncode


def cleanup(sig=None, frame=None):
    if not procs:
        return
    print(f"\n{YELLOW}[终止] 正在关闭全部进程...{RESET}")
    for p in procs:
        try: p.terminate()
        except: pass
    time.sleep(1)
    for p in procs:
        try: p.kill()
        except: pass
    print(f"{GREEN}[完成]{RESET}")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    mode = sys.argv[1] if len(sys.argv) > 1 else None
    if mode not in (None, "non", "ekf", "both"):
        print(f"用法: python3 ~/run_filter_experiment.py [non|ekf|both]")
        print(f"      不带参数 = 两个都跑")
        sys.exit(1)

    print(f"""
{CYAN}╔══════════════════════════════════════╗
║   滤波对比实验 (EKF+FIR vs 无滤波)   ║
╚══════════════════════════════════════╝{RESET}
""")

    # ========== 1. 启动 Gazebo ==========
    print(f"{CYAN}[1/3] 启动 Gazebo 仿真...{RESET}")
    run_bg(
        "Gazebo",
        f"source {ROS_SETUP} && source {WS_UR} && "
        f"ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e",
        "exp_gazebo.log",
    )
    print(f"  {YELLOW}等待 Gazebo 就绪 (约 10 秒)...{RESET}")
    time.sleep(10)

    # ========== 2. 启动 MoveIt ==========
    print(f"\n{CYAN}[2/3] 启动 MoveIt2 (提供 IK 服务)...{RESET}")
    run_bg(
        "MoveIt2",
        f"source {ROS_SETUP} && source {WS_UR} && "
        f"ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true",
        "exp_moveit.log",
    )
    print(f"  {YELLOW}等待 MoveIt 就绪 (约 8 秒)...{RESET}")
    time.sleep(8)

    # ========== 3. 运行实验 ==========
    print(f"\n{CYAN}[3/3] 运行实验...{RESET}")

    experiments = []
    if mode in (None, "both", "non"):
        experiments.append(("无滤波 (直通)", "exp_non_filtered",
                           "tracking_error_non_filtered_fair.csv"))
    if mode in (None, "both", "ekf"):
        experiments.append(("EKF+FIR 滤波", "exp_ekf_fir",
                           "tracking_error_ekf_fir_stronger1.csv"))

    for i, (label, node, csv_name) in enumerate(experiments):
        csv_path = HOME / csv_name
        # 删除旧 CSV
        if csv_path.exists():
            csv_path.unlink()
            print(f"  已删除旧文件: {csv_name}")

        print(f"\n  {YELLOW}>>> 实验 {i+1}/{len(experiments)}: {label} (15 秒){RESET}")
        print(f"  运行中", end="", flush=True)

        # 前台运行实验节点
        ret = run_fg(
            f"source {ROS_SETUP} && source {WS_MY_ROS} && "
            f"cd {HOME} && "
            f"timeout 25 ros2 run linear_hri_sim {node}"
        )
        print(f"\n  实验完成 (exit={ret})")

        if csv_path.exists():
            lines = csv_path.read_text().count('\n')
            size_kb = csv_path.stat().st_size / 1024
            print(f"  {GREEN}✓{RESET} CSV 已生成: {csv_name} ({lines} 行, {size_kb:.1f} KB)")
        else:
            print(f"  {RED}✗ CSV 未生成！{RESET}")

    # ========== 完成 ==========
    print(f"""
{GREEN}╔══════════════════════════════════════════════╗
║  实验完成！                                  ║
║                                              ║
║  输出文件:                                   ║
║    ~/tracking_error_non_filtered_fair.csv     ║
║    ~/tracking_error_ekf_fir_stronger1.csv     ║
║                                              ║
║  将这两个 CSV 拷贝到 MATLAB 脚本所在目录:    ║
║    C:\\Users\\jiaof\\OneDrive...\\FINAL\\达哥\\  ║
║                                              ║
║  然后在 MATLAB 中运行:                       ║
║    CN_tracking_mappingerror_filtered_vs_non1  ║
║                                              ║
║  Ctrl+C 终止所有后台进程                     ║
╚══════════════════════════════════════════════╝{RESET}
""")

    # 保持后台进程运行，等待 Ctrl+C
    try:
        while any(p.poll() is None for p in procs):
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    main()
