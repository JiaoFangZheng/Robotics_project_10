import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
import csv
import time

class HRINonFilteredNode(Node):
    def __init__(self):
        super().__init__('hri_non_filtered_fair_node')

        self.CENTER_X = 0.40
        self.CENTER_Y = 0.0
        self.CENTER_Z = 0.30
        self.RADIUS = 0.12
        self.OMEGA = 1.5
        self.Z_AMPLITUDE = 0.15
        self.TIME_PER_CYCLE = 15.0
        self.TOTAL_CYCLES = 1
        self.TOTAL_DURATION = self.TIME_PER_CYCLE * self.TOTAL_CYCLES

        # Keep these identical to wsl_EKF_FIR_stronger.py for a fair comparison.
        self.NOISE_STD_DEV = 0.005
        self.OUTLIER_PROB = 0.05
        self.OUTLIER_MAG = 0.08
        self.rng = np.random.default_rng(42)
        self.sample_idx = 0

        self.start_time = time.time()
        self.is_moving = False
        self.experiment_finished = False
        self.last_command = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log_filename = 'tracking_error_non_filtered_fair.csv'
        self.log_file = open(self.log_filename, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'Timestamp', 'SimTime', 'Ideal_X', 'Ideal_Y', 'Ideal_Z',
            'Noisy_X', 'Noisy_Y', 'Noisy_Z',
            'Filt_X', 'Filt_Y', 'Filt_Z',
            'Real_X', 'Real_Y', 'Real_Z',
            'Error_Noisy_Ideal', 'Error_Filt_Ideal', 'Error_Real_Ideal',
            'Error_Filt_Real', 'Command_Step', 'Injected_Outlier'
        ])

        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.main_timer = self.create_timer(0.05, self.experiment_loop)
        self.get_logger().info(f"🧪 [公平对比直通版]启动。数据存至 {self.log_filename}")

    def experiment_loop(self):
        if self.experiment_finished:
            return

        elapsed_time = self.sample_idx * 0.05
        self.sample_idx += 1

        if elapsed_time >= self.TOTAL_DURATION:
            self.experiment_finished = True
            self.finish_experiment()
            return

        ideal_x = self.CENTER_X + self.RADIUS * math.cos(self.OMEGA * elapsed_time)
        ideal_y = self.CENTER_Y + self.RADIUS * math.sin(self.OMEGA * elapsed_time)
        vertical_phase = (2.0 * math.pi * elapsed_time) / self.TIME_PER_CYCLE
        ideal_z = self.CENTER_Z + self.Z_AMPLITUDE * math.cos(vertical_phase)
        ideal = np.array([ideal_x, ideal_y, ideal_z])

        noisy = ideal + self.rng.normal(0.0, self.NOISE_STD_DEV, 3)
        injected_outlier = False
        if self.rng.random() < self.OUTLIER_PROB:
            noisy += self.rng.choice([-1, 1], 3) * self.OUTLIER_MAG
            injected_outlier = True

        # Direct pass-through version.
        filt_x, filt_y, filt_z = noisy[0], noisy[1], noisy[2]
        command = noisy.copy()

        command_step = 0.0
        if self.last_command is not None:
            command_step = float(np.linalg.norm(command - self.last_command))
        self.last_command = command.copy()

        actual = self.get_current_pose()
        if actual:
            real = np.array([actual.translation.x, actual.translation.y, actual.translation.z])
            err_noisy_ideal = float(np.linalg.norm(noisy - ideal))
            err_filt_ideal = err_noisy_ideal
            err_real_ideal = float(np.linalg.norm(real - ideal))
            err_filt_real = float(np.linalg.norm(command - real))

            self.log_writer.writerow([
                time.time(), elapsed_time, ideal_x, ideal_y, ideal_z,
                noisy[0], noisy[1], noisy[2], filt_x, filt_y, filt_z,
                real[0], real[1], real[2],
                err_noisy_ideal, err_filt_ideal, err_real_ideal,
                err_filt_real, command_step, int(injected_outlier)
            ])

            if not self.is_moving:
                self.is_moving = True
                self.send_ik_request(filt_x, filt_y, filt_z, actual.rotation)

        self.publish_marker(filt_x, filt_y, filt_z)

    def finish_experiment(self):
        if hasattr(self, 'main_timer'):
            self.main_timer.cancel()
        self.log_file.flush()
        self.log_file.close()
        self.get_logger().info("📊 [公平对比直通版]实验数据记录完毕。")

    def get_current_pose(self):
        try:
            return self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time()).transform
        except Exception:
            return None

    def send_ik_request(self, x, y, z, current_rotation):
        if not self.ik_client.wait_for_service(timeout_sec=0.1):
            self.is_moving = False
            return
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.robot_state.is_diff = True
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation = current_rotation
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            res = future.result()
            if res.error_code.val == 1:
                joints = res.solution.joint_state
                msg = JointTrajectory()
                msg.joint_names = joints.name[:6]
                point = JointTrajectoryPoint()
                point.positions = list(joints.position[:6])
                point.time_from_start = Duration(sec=0, nanosec=400000000)
                msg.points.append(point)
                self.traj_pub.publish(msg)
                self.unlock_timer = self.create_timer(0.15, self.unlock)
            else:
                self.is_moving = False
        except Exception:
            self.is_moving = False

    def unlock(self):
        self.is_moving = False
        if hasattr(self, 'unlock_timer'):
            self.unlock_timer.cancel()

    def publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id, m.header.stamp = "base_link", self.get_clock().now().to_msg()
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r, m.color.a = 1.0, 1.0
        self.marker_pub.publish(m)

    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = HRINonFilteredNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
