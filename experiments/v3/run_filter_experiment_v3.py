#!/usr/bin/env python3
"""V3: 无滤波 vs Kalman Filter (KF + FIR Kaiser + 阈值 MA) — 公平起点对齐"""
import subprocess, time, os, sys, signal
from pathlib import Path

HOME = Path.home()
LOGDIR = HOME / "logs"
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_UR = HOME / "ws_ur_sim/install/setup.bash"
WS_MY_ROS = HOME / "ws_my_ros/install/setup.bash"
GREEN, YELLOW, CYAN, RESET = "\033[1;32m", "\033[1;33m", "\033[1;36m", "\033[0m"
procs = []

def run_bg(label, script, logname):
    LOGDIR.mkdir(exist_ok=True)
    f = open(LOGDIR / logname, "w")
    p = subprocess.Popen(["bash", "-c", script], stdout=f, stderr=subprocess.STDOUT)
    procs.append(p)
    print(f"  {GREEN}✓{RESET} {label}  PID={p.pid}")

def cleanup(sig=None, frame=None):
    for p in procs:
        try: p.terminate()
        except: pass
    time.sleep(1)
    for p in procs:
        try: p.kill()
        except: pass
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f"{CYAN}╔══════════════════════════════════════╗")
    print(f"║  V3: 无滤波 vs KF (三阶段滤波器)    ║")
    print(f"╚══════════════════════════════════════╝{RESET}")

    # 1. Gazebo
    print(f"{CYAN}[1/3] Gazebo...{RESET}")
    run_bg("Gazebo", f"source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e", "exp_v3_gazebo.log")
    time.sleep(10)

    # 2. MoveIt
    print(f"{CYAN}[2/3] MoveIt2...{RESET}")
    run_bg("MoveIt2", f"source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true", "exp_v3_moveit.log")
    time.sleep(8)

    # 3. Experiments: non-filtered (V2) vs KF (V3)
    print(f"{CYAN}[3/3] V3 实验...{RESET}")
    experiments = [
        ("无滤波", "exp_non_filtered_v2", "tracking_error_non_filtered_v3.csv"),
        ("Kalman Filter", "exp_kf_v3", "tracking_error_kf_v3.csv"),
    ]

    for label, node, csv_name in experiments:
        csv_path = HOME / csv_name
        if csv_path.exists(): csv_path.unlink()
        print(f"  {YELLOW}>>> {label}{RESET}")
        subprocess.run(
            f"source {ROS_SETUP} && source {WS_MY_ROS} && cd {HOME} && timeout 40 ros2 run linear_hri_sim {node}",
            shell=True, executable="/bin/bash")
        if csv_path.exists():
            print(f"  {GREEN}✓{RESET} {csv_name} ({csv_path.read_text().count(chr(10))} 行)")

    print(f"{GREEN}V3 完成！Ctrl+C 终止{RESET}")
    while any(p.poll() is None for p in procs):
        time.sleep(1)

if __name__ == "__main__":
    main()
