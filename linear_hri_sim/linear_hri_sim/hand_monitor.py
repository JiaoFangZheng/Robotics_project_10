"""
实时监控节点：打印从 Windows 端收到的 UDP 手势数据，证明互通。
用法: ros2 run linear_hri_sim hand_monitor
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool


class HandMonitor(Node):
    def __init__(self):
        super().__init__('hand_monitor')
        self.create_subscription(PoseStamped, '/virtual_finger_pose', self.pose_cb, 10)
        self.create_subscription(PointStamped, '/target_intersection', self.intersection_cb, 10)
        self.create_subscription(Bool, '/gesture_trigger', self.trigger_cb, 10)

        self.last_trigger = False
        self.get_logger().info('=' * 50)
        self.get_logger().info('  手势数据实时监控已启动')
        self.get_logger().info('  等待 Windows 端数据...')
        self.get_logger().info('=' * 50)

    def pose_cb(self, msg):
        p = msg.pose.position
        self.get_logger().info(
            f'👆 手部位置:  X={p.x:+.3f}  Y={p.y:+.3f}  Z={p.z:+.3f}'
        )

    def intersection_cb(self, msg):
        p = msg.point
        self.get_logger().info(
            f'🎯 桌面交点:  X={p.x:+.3f}  Y={p.y:+.3f}  Z={p.z:+.3f}'
        )

    def trigger_cb(self, msg):
        if msg.data and not self.last_trigger:
            self.get_logger().info('🔥 V 字手势触发！机械臂应开始移动...')
        elif not msg.data and self.last_trigger:
            self.get_logger().info('✋ 手势释放')
        self.last_trigger = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = HandMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
