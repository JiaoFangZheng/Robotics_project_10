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
from scipy.signal import firwin


class RobustEKFFIRSignalProcessor:
    """
    3-stage filter for Cartesian target positions:
    1) EKF/KF prediction
    2) innovation gate for spike/outlier rejection
    3) short FIR on the EKF correction residual, not on the full trajectory

    The measurement is already Cartesian [x, y, z], so the measurement matrix is direct.
    This avoids the unnecessary Cartesian -> polar -> EKF conversion used in the older file.
    """
    def __init__(self, dt=0.05, measurement_std=0.005):
        self.dt = float(dt)
        self.measurement_std = float(measurement_std)
        self.I = np.eye(6)
        self.F = np.eye(6)
        self.H = np.zeros((3, 6))
        for axis in range(3):
            self.F[2 * axis, 2 * axis + 1] = self.dt
            self.H[axis, 2 * axis] = 1.0

        # Constant-velocity process model. Increase accel_std if the target is more agile.
        accel_std = 2.0
        q = accel_std ** 2
        q_block = np.array([
            [self.dt ** 4 / 4.0, self.dt ** 3 / 2.0],
            [self.dt ** 3 / 2.0, self.dt ** 2]
        ]) * q
        self.Q = np.zeros((6, 6))
        for axis in range(3):
            i = 2 * axis
            self.Q[i:i + 2, i:i + 2] = q_block

        self.R = np.eye(3) * (self.measurement_std ** 2)

        # Very short FIR. 3 taps gives small smoothing but almost no visible lag.
        num_taps = 3
        nyq_rate = (1.0 / self.dt) / 2.0
        cutoff_hz = 5.0
        self.fir_coeff = firwin(num_taps, cutoff_hz / nyq_rate)

        self.reset()

    def reset(self, initial_position=None):
        self.x_hat = np.zeros((6, 1))
        self.P = np.eye(6) * 0.005
        self.residual_hist = np.zeros((3, len(self.fir_coeff)))
        self.outlier_count = 0
        self.rejected_last = False
        self.is_initialized = False
        if initial_position is not None:
            p = np.asarray(initial_position, dtype=float).reshape(3)
            self.x_hat[[0, 2, 4], 0] = p
            self.is_initialized = True

    def process(self, measurement):
        z = np.asarray(measurement, dtype=float).reshape((3, 1))
        self.rejected_last = False

        if not self.is_initialized:
            self.x_hat[[0, 2, 4], 0] = z[:, 0]
            self.is_initialized = True
            return z[:, 0].copy()

        # 1) Predict
        x_minus = self.F @ self.x_hat
        P_minus = self.F @ self.P @ self.F.T + self.Q
        pred_pos = self.H @ x_minus

        # 2) Innovation gate: reject single-frame 8 cm spikes, accept persistent changes.
        innovation = z - pred_pos
        innovation_norm = float(np.linalg.norm(innovation))
        gate = max(0.035, 7.0 * self.measurement_std)

        if innovation_norm > gate:
            self.outlier_count += 1
            if self.outlier_count <= 2:
                z_used = pred_pos.copy()
                R_eff = self.R * 200.0
                self.rejected_last = True
            else:
                z_used = z
                R_eff = self.R * 8.0
                self.outlier_count = 0
        else:
            z_used = z
            R_eff = self.R
            self.outlier_count = 0

        # KF update
        y = z_used - self.H @ x_minus
        S = self.H @ P_minus @ self.H.T + R_eff
        K = P_minus @ self.H.T @ np.linalg.inv(S)
        self.x_hat = x_minus + K @ y
        self.P = (self.I - K @ self.H) @ P_minus @ (self.I - K @ self.H).T + K @ R_eff @ K.T

        # Physical velocity cap. This avoids a bad spike corrupting the velocity estimate.
        for axis in range(3):
            self.x_hat[2 * axis + 1, 0] = np.clip(self.x_hat[2 * axis + 1, 0], -0.60, 0.60)

        # 3) FIR only the correction residual, then add it back to prediction.
        ekf_pos = self.H @ self.x_hat
        residual = (ekf_pos - pred_pos)[:, 0]
        residual_fir = np.zeros(3)
        for axis in range(3):
            self.residual_hist[axis] = np.roll(self.residual_hist[axis], 1)
            self.residual_hist[axis, 0] = residual[axis]
            residual_fir[axis] = float(np.dot(self.fir_coeff, self.residual_hist[axis]))

        filtered_pos = pred_pos[:, 0] + residual_fir
        return filtered_pos


class HRIFilteredNode(Node):
    def __init__(self):
        super().__init__('hri_ekf_fir_v2_fairstart_node')

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

        # Same noise challenge must be used by the non-filtered comparison file.
        self.NOISE_STD_DEV = 0.005
        self.OUTLIER_PROB = 0.05
        self.OUTLIER_MAG = 0.08
        self.RNG_SEED = 42
        self.rng = np.random.default_rng(self.RNG_SEED)

        # Start alignment avoids the false negative caused by logging while the robot is far from the start pose.
        self.phase = 'ALIGN_START'
        self.START_SETTLE_TIME = 1.0
        self.START_TOLERANCE = 0.025
        self.START_COMMAND_PERIOD = 0.75
        self.ready_since = None
        self.last_start_command_time = 0.0

        self.sample_idx = 0
        self.is_moving = False
        self.experiment_finished = False
        self.last_command = None
        self.last_command_send_time = 0.0
        self.COMMAND_PERIOD = 0.10
        self.TRAJECTORY_TIME_SEC = 0
        self.TRAJECTORY_TIME_NSEC = 180_000_000

        self.processor = RobustEKFFIRSignalProcessor(dt=self.DT, measurement_std=self.NOISE_STD_DEV)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log_filename = 'tracking_error_ekf_fir_v2.csv'
        self.log_file = open(self.log_filename, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'Timestamp', 'SimTime', 'Ideal_X', 'Ideal_Y', 'Ideal_Z',
            'Noisy_X', 'Noisy_Y', 'Noisy_Z',
            'Filt_X', 'Filt_Y', 'Filt_Z',
            'Real_X', 'Real_Y', 'Real_Z',
            'Error_Noisy_Ideal', 'Error_Filt_Ideal', 'Error_Real_Ideal',
            'Error_Filt_Real', 'Command_Step', 'Injected_Outlier', 'Rejected_By_Filter'
        ])

        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.main_timer = self.create_timer(self.DT, self.timer_callback)
        self.get_logger().info(f"🧪 [V2 EKF+FIR] Waiting for TF, then aligning to start pose. CSV: {self.log_filename}")

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
                self.start_experiment(start_pos)
        else:
            self.ready_since = None

    def start_experiment(self, start_pos):
        self.phase = 'RUNNING'
        self.sample_idx = 0
        self.last_command = None
        self.last_command_send_time = 0.0
        self.rng = np.random.default_rng(self.RNG_SEED)
        self.processor.reset(start_pos)
        self.get_logger().info("✅ Start pose aligned. Resetting timer/noise seed and starting EKF+FIR logging.")

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

        filtered_pos = self.processor.process(noisy)
        filt_x, filt_y, filt_z = filtered_pos

        command_step = 0.0
        if self.last_command is not None:
            command_step = float(np.linalg.norm(filtered_pos - self.last_command))
        self.last_command = filtered_pos.copy()

        real = np.array([actual.translation.x, actual.translation.y, actual.translation.z], dtype=float)
        err_noisy_ideal = float(np.linalg.norm(noisy - ideal))
        err_filt_ideal = float(np.linalg.norm(filtered_pos - ideal))
        err_real_ideal = float(np.linalg.norm(real - ideal))
        err_filt_real = float(np.linalg.norm(filtered_pos - real))

        self.log_writer.writerow([
            time.time(), elapsed_time, ideal[0], ideal[1], ideal[2],
            noisy[0], noisy[1], noisy[2], filt_x, filt_y, filt_z,
            real[0], real[1], real[2],
            err_noisy_ideal, err_filt_ideal, err_real_ideal,
            err_filt_real, command_step, int(injected_outlier), int(self.processor.rejected_last)
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
        self.get_logger().info("📊 [V2 EKF+FIR] Experiment finished. CSV saved.")

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
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        self.marker_pub.publish(m)

    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = HRIFilteredNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
