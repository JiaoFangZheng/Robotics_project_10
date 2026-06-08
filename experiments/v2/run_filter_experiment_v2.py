#!/usr/bin/env python3
"""
滤波对比实验 V2（公平起点对齐版）
用法: python3 ~/run_filter_experiment_v2.py
输出: ~/tracking_error_non_filtered_v2.csv
      ~/tracking_error_ekf_fir_v2.csv
"""
import subprocess, time, os, sys, signal
from pathlib import Path

HOME = Path.home()
LOGDIR = HOME / "logs"
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_UR = HOME / "ws_ur_sim/install/setup.bash"
WS_MY_ROS = HOME / "ws_my_ros/install/setup.bash"
GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
CYAN = "\033[1;36m"
RESET = "\033[0m"
procs = []

def run_bg(label, script, logname):
    LOGDIR.mkdir(exist_ok=True)
    f = open(LOGDIR / logname, "w")
    p = subprocess.Popen(["bash", "-c", script], stdout=f, stderr=subprocess.STDOUT)
    procs.append(p)
    print(f"  {GREEN}✓{RESET} {label}  PID={p.pid}")
    return p

def cleanup(sig=None, frame=None):
    for p in procs:
        try: p.terminate()
        except: pass
    time.sleep(1)
    for p in procs:
        try: p.kill()
        except: pass
    print(f"\n{GREEN}[完成] 全部进程已终止{RESET}")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    mode = sys.argv[1] if len(sys.argv) > 1 else "both"

    print(f"""
{CYAN}╔══════════════════════════════════════╗
║  滤波对比实验 V2 (公平起点对齐版)    ║
╚══════════════════════════════════════╝{RESET}
""")

    # 1. Gazebo
    print(f"{CYAN}[1/3] 启动 Gazebo 仿真...{RESET}")
    run_bg("Gazebo",
           f"source {ROS_SETUP} && source {WS_UR} && "
           f"ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e",
           "exp_v2_gazebo.log")
    print(f"  {YELLOW}等待 Gazebo 就绪 (10 秒)...{RESET}")
    time.sleep(10)

    # 2. MoveIt
    print(f"\n{CYAN}[2/3] 启动 MoveIt2...{RESET}")
    run_bg("MoveIt2",
           f"source {ROS_SETUP} && source {WS_UR} && "
           f"ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true",
           "exp_v2_moveit.log")
    print(f"  {YELLOW}等待 MoveIt 就绪 (8 秒)...{RESET}")
    time.sleep(8)

    # 3. Experiments
    print(f"\n{CYAN}[3/3] 运行 V2 实验...{RESET}")

    experiments = [
        ("无滤波 V2", "exp_non_filtered_v2", "tracking_error_non_filtered_v2.csv"),
        ("EKF+FIR V2", "exp_ekf_fir_v2", "tracking_error_ekf_fir_v2.csv"),
    ]

    for i, (label, node, csv_name) in enumerate(experiments):
        csv_path = HOME / csv_name
        if csv_path.exists():
            csv_path.unlink()

        print(f"\n  {YELLOW}>>> {label}{RESET}")
        # V2 has ALIGN_START phase + 15s trajectory, give it 35s total
        subprocess.run(
            f"source {ROS_SETUP} && source {WS_MY_ROS} && cd {HOME} && "
            f"timeout 40 ros2 run linear_hri_sim {node}",
            shell=True, executable="/bin/bash"
        )

        if csv_path.exists():
            lines = csv_path.read_text().count('\n')
            size = csv_path.stat().st_size / 1024
            print(f"  {GREEN}✓{RESET} {csv_name} ({lines} 行, {size:.1f} KB)")
        else:
            print(f"  {RED}✗ CSV 未生成！{RESET}")

    print(f"""
{GREEN}╔══════════════════════════════════════╗
║  V2 实验完成！                        ║
║  ~/tracking_error_non_filtered_v2.csv ║
║  ~/tracking_error_ekf_fir_v2.csv      ║
║  Ctrl+C 终止后台进程                  ║
╚══════════════════════════════════════╝{RESET}
""")

    while any(p.poll() is None for p in procs):
        time.sleep(1)

if __name__ == "__main__":
    main()
