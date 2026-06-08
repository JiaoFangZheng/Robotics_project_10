import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class GestureTrigger(Node):
    def __init__(self):
        super().__init__('gesture_trigger')
        # 订阅算法节点计算出的交点 (XY 目标)
        self.sub_intersection = self.create_subscription(
            PointStamped, '/target_intersection', self.target_callback, 10)
        # 订阅手指原始位姿 (用于获取真实 Z 高度)
        self.sub_finger = self.create_subscription(
            PoseStamped, '/virtual_finger_pose', self.finger_callback, 10)
        # 订阅手势触发信号 (来自 UDP 桥接节点的 V 字手势检测)
        self.sub_trigger = self.create_subscription(
            Bool, '/gesture_trigger', self.trigger_callback, 10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.latest_target = None
        self.latest_hand_z = None  # 手在空间中的实际 Z 坐标

        self.get_logger().info('手势触发节点已就绪，等待 Windows 端 V 字手势...')

    def target_callback(self, msg):
        self.latest_target = msg

    def finger_callback(self, msg):
        self.latest_hand_z = msg.pose.position.z

    def trigger_callback(self, msg):
        """收到 UDP 桥接的 V 字手势触发信号"""
        if not msg.data:
            return
        if self.latest_target is None:
            self.get_logger().warn('还未收到手指数据！')
            return
        self.trigger_execution()

    def trigger_execution(self):
        x = self.latest_target.point.x
        y = self.latest_target.point.y
        # 使用手的实际 Z 坐标（非桌面投影），略加偏移防撞桌面
        z = (self.latest_hand_z if self.latest_hand_z is not None else 0.2) + 0.05

        self.get_logger().info(f'V 字手势触发！目标: X={x:.3f} Y={y:.3f} Z={z:.3f}')

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.latest_target.header.frame_id
        t.child_frame_id = 'gesture_target'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # 末端垂直向下 (绕 Y 轴 90°)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.707
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.707

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('已发布目标位姿 gesture_target，机械臂开始移动')

def main(args=None):
    rclpy.init(args=args)
    node = GestureTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
