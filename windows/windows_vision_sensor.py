import cv2
import mediapipe as mp
import socket
import json
import numpy as np
import math
from collections import deque
import mediapipe
print("当前使用的 mediapipe 路径是:", mediapipe.__file__)
# ==========================================
# 🌟 【参数调优区】
# ==========================================
UDP_IP = "172.27.64.221" 
UDP_PORT = 5005

# 1. 你定义的坐标轴映射范围 (单位: 米)
# X轴 (手左右平移) -> 映射范围: 左 -0.4 到 右 0.4
BALL_X_MIN, BALL_X_MAX = -0.40, 0.40 
# Y轴 (手上下移动) -> 映射范围: 下 0.1 到 上 0.5
BALL_Y_MIN, BALL_Y_MAX = 0.10, 0.50  
# Z轴 (手前后移动) -> 映射范围: 缩回 0.3 到 伸出 0.7
BALL_Z_MIN, BALL_Z_MAX = 0.10, 0.90  

# 2. 深度模拟参数 (观察终端里的"手掌大小"来微调这两个值)
PALM_SIZE_FAR = 0.12   # 手离屏幕【最远】时的像素大小
PALM_SIZE_NEAR = 0.4  # 手离屏幕【最近】时的像素大小

# 滤波器强度
SMOOTH_BUFFER_SIZE = 5 
# ==========================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
coords_buffer = deque(maxlen=SMOOTH_BUFFER_SIZE)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.8)
cap = cv2.VideoCapture(0) 

def remap(val, in_min, in_max, out_min, out_max):
    """数值区间线性映射"""
    mapped = (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return np.clip(mapped, min(out_min, out_max), max(out_min, out_max))

print("🚀 深度多点追踪 & V字手势识别 已启动...")

while True:
    success, frame = cap.read()
    if not success: continue

    frame = cv2.flip(frame, 1)
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            lm = hand_landmarks.landmark
            
            # ==========================================
            # 🌟 1. 提取手掌中间的三个点计算质心和深度
            # 使用点: 0(手腕), 9(中指根部), 13(无名指根部)
            # ==========================================
            mid_pts = [[lm[i].x, lm[i].y] for i in [0, 9, 13]]
            centroid_x, centroid_y = np.mean(mid_pts, axis=0)
            
            # 计算深度：手腕(0)到中指根(9) 和 手腕(0)到无名指根(13) 的平均距离
            d1 = math.hypot(lm[0].x - lm[9].x, lm[0].y - lm[9].y)
            d2 = math.hypot(lm[0].x - lm[13].x, lm[0].y - lm[13].y)
            palm_size = (d1 + d2) / 2.0
            
            print(f"手掌像素大小: {palm_size:.3f}", end='\r')

            # ==========================================
            # 🌟 2. 坐标映射 (严格按照你的 X, Y, Z 定义)
            # ==========================================
            # 手左右 (X轴): 屏幕 0.2(左) -> 0.8(右) 映射到 X_MIN -> X_MAX
            rob_x = remap(centroid_x, 0.2, 0.8, BALL_X_MIN, BALL_X_MAX)
            
            # 手上下 (Y轴): 屏幕 0.8(下) -> 0.2(上) 映射到 Y_MIN -> Y_MAX
            rob_y = remap(centroid_y, 0.8, 0.2, BALL_Y_MIN, BALL_Y_MAX)
            
            # 手前后 (Z轴): 近(palm大) -> Z_MAX, 远(palm小) -> Z_MIN
            rob_z = remap(palm_size, PALM_SIZE_NEAR, PALM_SIZE_FAR, BALL_Z_MIN, BALL_Z_MAX)
            
            coords_buffer.append([rob_x, rob_y, rob_z])
            smooth_x, smooth_y, smooth_z = np.mean(coords_buffer, axis=0)

            # ==========================================
            # 🌟 3. 触发抓取动作：仅食指和中指伸出 (V字手势)
            # ==========================================
            # 判断逻辑：指尖的 Y 坐标必须比对应手指下方的关节 Y 坐标小 (前提是手掌竖直朝上)
            index_up = lm[8].y < lm[6].y   # 食指伸直
            middle_up = lm[12].y < lm[10].y # 中指伸直
            ring_down = lm[16].y > lm[14].y # 无名指弯曲
            pinky_down = lm[20].y > lm[18].y # 小指弯曲
            
            # 只有食指和中指伸直，且无名指和小指弯曲时，才触发
            is_trigger_active = bool(index_up and middle_up and ring_down and pinky_down)

            # 屏幕UI反馈
            status_color = (0, 0, 255) if is_trigger_active else (0, 255, 0)
            status_text = "GRASP! (Trigger: ON)" if is_trigger_active else "Tracking... (Trigger: OFF)"
            cv2.putText(frame, status_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)

            # 发送数据包
            packet = {
                "base": {"x": round(smooth_x, 4), "y": round(smooth_y, 4), "z": round(smooth_z, 4)},
                "tip":  {"x": round(smooth_x, 4), "y": round(smooth_y, 4), "z": round(smooth_z, 4)},
                "trigger": is_trigger_active
            }
            sock.sendto(json.dumps(packet).encode('utf-8'), (UDP_IP, UDP_PORT))

    cv2.imshow("Linear HRI - Custom Axis & Gesture", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
