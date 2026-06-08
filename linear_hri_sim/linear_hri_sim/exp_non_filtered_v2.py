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
        super().__init__('hri_non_filtered_v2_fairstart_node')

        # Trajectory parameters
        self.CENTER_X = 0.40
        self.CENTER_Y = 0.0
        self.CENTER_Z = 0.30
        self.RADIUS = 0.12
        self.OMEGA = 1.5
        self.Z_AMPLITUDE = 0.15
        self.TIME_PER_CYCLE = 15.0
        self.TOTAL_CYCLES = 1
        self.TOTAL_DURATION = self.TIME_PER_CYCLE * self.TOTAL_CYCLES
        self.DT = 0.05

        # Must be identical to wsl_EKF_FIR_v2_fairstart.py.
        self.NOISE_STD_DEV = 0.005
        self.OUTLIER_PROB = 0.05
        self.OUTLIER_MAG = 0.08
        self.RNG_SEED = 42
        self.rng = np.random.default_rng(self.RNG_SEED)

        # Start alignment avoids comparing one run from the correct start and the other from a stale/unsettled pose.
        self.phase = 'ALIGN_START'
        self.START_SETTLE_TIME = 1.0
        self.START_TOLERANCE = 0.025
        self.START_COMMAND_PERIOD = 0.75
        self.ready_since = None
        self.last_start_command_time = 0.0

        self.sample_idx = 0
        self.experiment_finished = False
        self.last_command = None
        self.last_command_send_time = 0.0
        self.COMMAND_PERIOD = 0.10
        self.TRAJECTORY_TIME_SEC = 0
        self.TRAJECTORY_TIME_NSEC = 180_000_000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log_filename = 'tracking_error_non_filtered_v2.csv'
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
        self.main_timer = self.create_timer(self.DT, self.timer_callback)
        self.get_logger().info(f"🧪 [V2 Non-filtered] Waiting for TF, then aligning to start pose. CSV: {self.log_filename}")

    def target_at(self, elapsed_time):
        x = self.CENTER_X + self.RADIUS * math.cos(self.OMEGA * elapsed_time)
        y = self.CENTER_Y + self.RADIUS * math.sin(self.OMEGA * elapsed_time)
        vertical_phase = (2.0 * math.pi * elapsed_time) / self.TIME_PER_CYCLE
        z = self.CENTER_Z + self.Z_AMPLITUDE * math.cos(vertical_phase)
        return np.array([x, y, z], dtype=float)

    def timer_callback(self):
        if self.experiment_finished:
            return

        actual = self.get_current_pose()
        if actual is None:
            return

        if self.phase == 'ALIGN_START':
            self.align_to_start(actual)
            return

        self.experiment_loop(actual)

    def align_to_start(self, actual):
        start_pos = self.target_at(0.0)
        self.publish_marker(start_pos[0], start_pos[1], start_pos[2])
        real = np.array([actual.translation.x, actual.translation.y, actual.translation.z], dtype=float)
        dist = float(np.linalg.norm(real - start_pos))
        now = time.time()

        if (now - self.last_start_command_time) >= self.START_COMMAND_PERIOD and dist > self.START_TOLERANCE:
            self.last_start_command_time = now
            self.send_ik_request(start_pos[0], start_pos[1], start_pos[2], actual.rotation)
            self.get_logger().info(f"Aligning to start pose, current start error = {dist * 1000:.1f} mm")

        if dist <= self.START_TOLERANCE:
            if self.ready_since is None:
                self.ready_since = now
            if (now - self.ready_since) >= self.START_SETTLE_TIME:
                self.start_experiment()
        else:
            self.ready_since = None

    def start_experiment(self):
        self.phase = 'RUNNING'
        self.sample_idx = 0
        self.last_command = None
        self.last_command_send_time = 0.0
        self.rng = np.random.default_rng(self.RNG_SEED)
        self.get_logger().info("✅ Start pose aligned. Resetting timer/noise seed and starting non-filtered logging.")

    def experiment_loop(self, actual):
        elapsed_time = self.sample_idx * self.DT
        self.sample_idx += 1

        if elapsed_time >= self.TOTAL_DURATION:
            self.experiment_finished = True
            self.finish_experiment()
            return

        ideal = self.target_at(elapsed_time)
        noisy = ideal + self.rng.normal(0.0, self.NOISE_STD_DEV, 3)
        injected_outlier = False
        if self.rng.random() < self.OUTLIER_PROB:
            noisy += self.rng.choice([-1, 1], 3) * self.OUTLIER_MAG
            injected_outlier = True

        # Direct pass-through version: command = noisy measurement.
        command = noisy.copy()
        filt_x, filt_y, filt_z = command

        command_step = 0.0
        if self.last_command is not None:
            command_step = float(np.linalg.norm(command - self.last_command))
        self.last_command = command.copy()

        real = np.array([actual.translation.x, actual.translation.y, actual.translation.z], dtype=float)
        err_noisy_ideal = float(np.linalg.norm(noisy - ideal))
        err_filt_ideal = err_noisy_ideal
        err_real_ideal = float(np.linalg.norm(real - ideal))
        err_filt_real = float(np.linalg.norm(command - real))

        self.log_writer.writerow([
            time.time(), elapsed_time, ideal[0], ideal[1], ideal[2],
            noisy[0], noisy[1], noisy[2], filt_x, filt_y, filt_z,
            real[0], real[1], real[2],
            err_noisy_ideal, err_filt_ideal, err_real_ideal,
            err_filt_real, command_step, int(injected_outlier)
        ])

        now = time.time()
        if (now - self.last_command_send_time) >= self.COMMAND_PERIOD:
            self.last_command_send_time = now
            self.send_ik_request(filt_x, filt_y, filt_z, actual.rotation)

        self.publish_marker(filt_x, filt_y, filt_z)

    def finish_experiment(self):
        if hasattr(self, 'main_timer'):
            self.main_timer.cancel()
        self.log_file.flush()
        self.log_file.close()
        self.get_logger().info("📊 [V2 Non-filtered] Experiment finished. CSV saved.")

    def get_current_pose(self):
        try:
            return self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time()).transform
        except Exception:
            return None

    def send_ik_request(self, x, y, z, current_rotation):
        if not self.ik_client.wait_for_service(timeout_sec=0.1):
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
                point.time_from_start = Duration(sec=self.TRAJECTORY_TIME_SEC, nanosec=self.TRAJECTORY_TIME_NSEC)
                msg.points.append(point)
                self.traj_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"IK callback failed: {e}")

    def publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
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
