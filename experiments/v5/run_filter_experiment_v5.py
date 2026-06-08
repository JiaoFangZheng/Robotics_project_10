#!/usr/bin/env python3
"""V5: 无滤波 vs KF 2.0 (距离跳变+FIR+MA) — 公平起点对齐"""
import subprocess, time, sys, signal
from pathlib import Path
HOME = Path.home(); LOGDIR = HOME/"logs"
ROS="/opt/ros/humble/setup.bash"; WU=HOME/"ws_ur_sim/install/setup.bash"; WM=HOME/"ws_my_ros/install/setup.bash"
G,Y,C,R="\033[1;32m","\033[1;33m","\033[1;36m","\033[0m"; procs=[]
def bg(l,s,n):
    LOGDIR.mkdir(exist_ok=True); f=open(LOGDIR/n,"w")
    p=subprocess.Popen(["bash","-c",s],stdout=f,stderr=subprocess.STDOUT); procs.append(p); print(f"  {G}✓{R} {l}")
def cl(*a):
    for p in procs:
        try:p.terminate()
        except:pass
    time.sleep(1)
    for p in procs:
        try:p.kill()
        except:pass
    sys.exit(0)
def main():
    signal.signal(signal.SIGINT,cl); signal.signal(signal.SIGTERM,cl)
    print(f"{C}╔══════════════════════════════════════╗")
    print(f"║  V5: 无滤波 vs KF 2.0               ║")
    print(f"╚══════════════════════════════════════╝{R}")
    print(f"{C}[1/3] Gazebo...{R}"); bg("G",f"source {ROS} && source {WU} && ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e","exp_v5_gazebo.log"); time.sleep(10)
    print(f"{C}[2/3] MoveIt2...{R}"); bg("M",f"source {ROS} && source {WU} && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true","exp_v5_moveit.log"); time.sleep(8)
    print(f"{C}[3/3] V5...{R}")
    for l,n,csv in [("无滤波","exp_non_filtered_v2","tracking_error_non_filtered_v5.csv"),("KF 2.0","exp_kf_v5","tracking_error_kf_v5.csv")]:
        p=HOME/csv
        if p.exists():p.unlink()
        print(f"  {Y}>>> {l}{R}")
        subprocess.run(f"source {ROS} && source {WM} && cd {HOME} && timeout 40 ros2 run linear_hri_sim {n}",shell=True,executable="/bin/bash")
        if p.exists():print(f"  {G}✓{R} {csv}")
    print(f"{G}V5 完成！Ctrl+C 终止{R}")
    while any(p.poll() is None for p in procs):time.sleep(1)
if __name__=="__main__":main()
