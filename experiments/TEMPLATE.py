#!/usr/bin/env python3
"""
新实验模板 — 复制并修改 EXPERIMENTS 列表即可
用法: 复制此文件到 experiments/vN/ 并修改
"""
import subprocess, time, os, sys, signal
from pathlib import Path

HOME = Path.home()
LOGDIR = HOME / 'logs'
ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_UR = HOME / 'ws_ur_sim/install/setup.bash'
WS_MY_ROS = HOME / 'ws_my_ros/install/setup.bash'
GREEN, YELLOW, CYAN, RESET = '\033[1;32m', '\033[1;33m', '\033[1;36m', '\033[0m'
procs = []

def run_bg(label, script, logname):
    LOGDIR.mkdir(exist_ok=True)
    f = open(LOGDIR / logname, 'w')
    p = subprocess.Popen(['bash', '-c', script], stdout=f, stderr=subprocess.STDOUT)
    procs.append(p)
    print(f'  {GREEN}✓{RESET} {label}  PID={p.pid}')

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

    print(f'{CYAN}╔══════════════════════════════╗')
    print(f'{CYAN}║     新实验 (TEMPLATE)       ║')
    print(f'{CYAN}╚══════════════════════════════╝{RESET}')

    # 1. Gazebo
    print(f'{CYAN}[1/3] Gazebo...{RESET}')
    run_bg('Gazebo', f'source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e', 'exp_gazebo.log')
    time.sleep(10)

    # 2. MoveIt
    print(f'{CYAN}[2/3] MoveIt2...{RESET}')
    run_bg('MoveIt2', f'source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true', 'exp_moveit.log')
    time.sleep(8)

    # 3. 修改这里的实验列表
    EXPERIMENTS = [
        # (标签, ROS节点名, CSV输出文件名)
        ('实验A', 'exp_non_filtered', 'tracking_error_A.csv'),
        ('实验B', 'exp_ekf_fir', 'tracking_error_B.csv'),
    ]

    print(f'{CYAN}[3/3] 运行实验...{RESET}')
    for label, node, csv_name in EXPERIMENTS:
        csv_path = HOME / csv_name
        if csv_path.exists(): csv_path.unlink()
        print(f'  {YELLOW}>>> {label}{RESET}')
        subprocess.run(
            f'source {ROS_SETUP} && source {WS_MY_ROS} && cd {HOME} && timeout 40 ros2 run linear_hri_sim {node}',
            shell=True, executable='/bin/bash')
        if csv_path.exists():
            print(f'  {GREEN}✓{RESET} {csv_name} ({csv_path.read_text().count(chr(10))} 行)')

    print(f'{GREEN}完成！Ctrl+C 终止{RESET}')
    while any(p.poll() is None for p in procs):
        time.sleep(1)

if __name__ == '__main__':
    main()
