"""
UDP 桥接节点：接收 Windows 端 MediaPipe 手势数据，转为 ROS2 消息。
Windows 脚本 → UDP :5005 → 本节点 → /virtual_finger_pose + /gesture_trigger
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import socket
import json
import threading
import math


class UDPBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')

        # 发布手指位姿（替代 RViz InteractiveMarker 手动拖拽）
        self.pose_pub = self.create_publisher(PoseStamped, '/virtual_finger_pose', 10)
        # 发布手势触发信号（替代键盘回车）
        self.trigger_pub = self.create_publisher(Bool, '/gesture_trigger', 10)

        # UDP 监听
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.settimeout(0.5)

        self.get_logger().info('UDP 桥接已启动，监听 0.0.0.0:5005')

        self.running = True
        self.recv_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.recv_thread.start()

    def _udp_loop(self):
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)
                self._process(json.loads(data.decode('utf-8')))
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                self.get_logger().warn(f'无效 JSON: {data[:100]}')
            except Exception as e:
                self.get_logger().warn(f'UDP 异常: {e}')

    def _process(self, packet: dict):
        base = packet.get('base', {})
        tip = packet.get('tip', {})
        trigger = packet.get('trigger', False)

        bx, by, bz = base.get('x', 0.0), base.get('y', 0.0), base.get('z', 0.0)
        tx, ty, tz = tip.get('x', 0.0), tip.get('y', 0.0), tip.get('z', 0.0)

        # 计算手指指向方向 (base → tip)，默认朝下 (-Z)
        dx, dy, dz = tx - bx, ty - by, tz - bz
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 1e-6:
            dx, dy, dz = 0.0, 0.0, -1.0
        else:
            dx, dy, dz = dx/mag, dy/mag, dz/mag

        # 方向向量 → 四元数（使 X 轴指向手指方向）
        qx, qy, qz, qw = self._dir_to_quat(dx, dy, dz)

        # 发布 PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = bx
        msg.pose.position.y = by
        msg.pose.position.z = bz
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

        # 发布触发信号
        self.trigger_pub.publish(Bool(data=bool(trigger)))

    @staticmethod
    def _dir_to_quat(dx: float, dy: float, dz: float):
        """将方向向量转为四元数 (X 轴对齐到该方向)"""
        dot = dx  # dot((1,0,0), dir)
        if dot > 0.9999:
            return (0.0, 0.0, 0.0, 1.0)
        if dot < -0.9999:
            return (0.0, 1.0, 0.0, 0.0)

        angle = math.acos(dot)
        # cross((1,0,0), dir) = (0, dz, -dy)
        ax, ay, az = 0.0, dz, -dy
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        ay, az = ay/mag, az/mag

        half = angle / 2.0
        s = math.sin(half)
        return (ax * s, ay * s, az * s, math.cos(half))

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
