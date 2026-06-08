#!/usr/bin/env python3
"""V4: 无滤波 vs 纯 Kalman Filter (无创新门) — 公平起点对齐"""
import subprocess, time, sys, signal
from pathlib import Path
HOME = Path.home()
LOGDIR = HOME / "logs"
ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_UR = HOME / "ws_ur_sim/install/setup.bash"
WS_MY_ROS = HOME / "ws_my_ros/install/setup.bash"
G,Y,C,R = "\033[1;32m","\033[1;33m","\033[1;36m","\033[0m"
procs = []
def run_bg(l,s,ln):
    LOGDIR.mkdir(exist_ok=True); f=open(LOGDIR/ln,"w")
    p=subprocess.Popen(["bash","-c",s],stdout=f,stderr=subprocess.STDOUT)
    procs.append(p); print(f"  {G}✓{R} {l} PID={p.pid}")
def cleanup(sig=None,frame=None):
    for p in procs:
        try: p.terminate()
        except: pass
    time.sleep(1)
    for p in procs:
        try: p.kill()
        except: pass
    sys.exit(0)
def main():
    signal.signal(signal.SIGINT,cleanup); signal.signal(signal.SIGTERM,cleanup)
    print(f"{C}╔══════════════════════════════════════╗")
    print(f"║  V4: 无滤波 vs 纯 KF (无创新门)      ║")
    print(f"╚══════════════════════════════════════╝{R}")
    print(f"{C}[1/3] Gazebo...{R}"); run_bg("Gazebo",f"source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e","exp_v4_gazebo.log"); time.sleep(10)
    print(f"{C}[2/3] MoveIt2...{R}"); run_bg("MoveIt2",f"source {ROS_SETUP} && source {WS_UR} && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true","exp_v4_moveit.log"); time.sleep(8)
    print(f"{C}[3/3] V4 实验...{R}")
    for l,n,csv in [("无滤波","exp_non_filtered_v2","tracking_error_non_filtered_v4.csv"),("纯 KF","exp_kf_v4","tracking_error_kf_pure_v4.csv")]:
        p=HOME/csv
        if p.exists(): p.unlink()
        print(f"  {Y}>>> {l}{R}")
        subprocess.run(f"source {ROS_SETUP} && source {WS_MY_ROS} && cd {HOME} && timeout 40 ros2 run linear_hri_sim {n}",shell=True,executable="/bin/bash")
        if p.exists(): print(f"  {G}✓{R} {csv} ({p.read_text().count(chr(10))} 行)")
    print(f"{G}V4 完成！Ctrl+C 终止{R}")
    while any(p.poll() is None for p in procs): time.sleep(1)
if __name__=="__main__": main()
