"""
自动执行节点：收到 V 字手势触发后，通过 MoveIt Action 指挥机械臂运动。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive


class AutoCommander(Node):
    def __init__(self):
        super().__init__('auto_commander')
        # 订阅交点 (XY 目标)
        self.create_subscription(PointStamped, '/target_intersection', self.point_cb, 10)
        # 订阅手指位姿 (获取 Z 高度)
        self.create_subscription(PoseStamped, '/virtual_finger_pose', self.finger_cb, 10)
        # 订阅手势触发 (来自 UDP 桥接)
        self.create_subscription(Bool, '/gesture_trigger', self.trigger_cb, 10)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        self.latest_target = None
        self.latest_hand_z = None
        self.last_trigger = False

        self.get_logger().info('自动执行节点已就绪，等待 V 字手势...')

    def point_cb(self, msg):
        self.latest_target = msg.point

    def finger_cb(self, msg):
        self.latest_hand_z = msg.pose.position.z

    def trigger_cb(self, msg):
        if msg.data and not self.last_trigger:
            if self.latest_target:
                self.send_goal()
            else:
                self.get_logger().warn('收到触发但无手指数据！')
        self.last_trigger = msg.data

    def send_goal(self):
        x = self.latest_target.x
        y = self.latest_target.y
        z = (self.latest_hand_z if self.latest_hand_z is not None else 0.3) + 0.05

        self.get_logger().info(f'机械臂目标: X={x:.3f} Y={y:.3f} Z={z:.3f}')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 位置约束
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.link_name = "tool0"

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        # 末端朝下
        pose_msg.pose.orientation.y = 0.707
        pose_msg.pose.orientation.w = 0.707

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(pose_msg.pose)
        pos_con.weight = 1.0

        # 姿态约束
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.link_name = "tool0"
        ori_con.orientation.y = 0.707
        ori_con.orientation.w = 0.707
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        self._action_client.wait_for_server(timeout_sec=5.0)
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('已发送 MoveIt 运动请求')


def main(args=None):
    rclpy.init(args=args)
    node = AutoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
