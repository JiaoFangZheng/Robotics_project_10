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
    Robust EKF + FIR residual filter.

    Difference from the old version:
    1) The measurement is already Cartesian [x,y,z], so H is direct instead of converting to polar.
    2) EKF/KF prediction is used to reject single-frame 6-8 cm spikes.
    3) FIR is applied only to the EKF correction residual, not to the whole position.
       This keeps the trajectory responsive and avoids the big lag caused by FIR-ing x/y/z directly.
    """
    def __init__(self, dt=0.05, measurement_std=0.005):
        self.dt = dt
        self.measurement_std = measurement_std

        # State: [x, vx, y, vy, z, vz]^T
        self.x_hat = np.zeros((6, 1))
        self.P = np.eye(6) * 0.01

        self.F = np.eye(6)
        self.H = np.zeros((3, 6))
        for axis in range(3):
            self.F[2 * axis, 2 * axis + 1] = self.dt
            self.H[axis, 2 * axis] = 1.0

        # Constant-velocity process model with acceleration noise.
        # Larger accel_std = more responsive; smaller = smoother.
        accel_std = 1.2
        q = accel_std ** 2
        q_block = np.array([
            [self.dt ** 4 / 4.0, self.dt ** 3 / 2.0],
            [self.dt ** 3 / 2.0, self.dt ** 2]
        ]) * q
        self.Q = np.zeros((6, 6))
        for axis in range(3):
            i = 2 * axis
            self.Q[i:i+2, i:i+2] = q_block

        # Slightly inflate R so millimeter noise is smoothed, but not enough to create huge delay.
        self.R = np.eye(3) * (measurement_std * 1.2) ** 2
        self.I = np.eye(6)

        # FIR on residual only. Three taps gives small smoothing with much less delay than FIR-ing position.
        num_taps = 3
        cutoff_hz = 5.0
        nyq_rate = (1.0 / self.dt) / 2.0
        self.fir_coeff = firwin(num_taps, cutoff_hz / nyq_rate)
        self.residual_hist = np.zeros((3, num_taps))

        self.is_initialized = False
        self.outlier_count = 0
        self.rejected_last = False

    def process(self, measurement):
        z = np.asarray(measurement, dtype=float).reshape((3, 1))
        self.rejected_last = False

        if not self.is_initialized:
            self.x_hat[[0, 2, 4], 0] = z[:, 0]
            self.is_initialized = True
            return z[:, 0].copy()

        # ---------- Predict ----------
        x_minus = self.F @ self.x_hat
        P_minus = self.F @ self.P @ self.F.T + self.Q
        pred_pos = self.H @ x_minus

        # ---------- Innovation gate / spike rejection ----------
        innovation = z - pred_pos
        innovation_norm = float(np.linalg.norm(innovation))

        # Normal target step is about 1 cm per 0.05 s. 2.5 cm is a good default gate.
        # Increase this if your real target can move faster.
        gate = max(0.025, 5.0 * self.measurement_std)

        if innovation_norm > gate:
            self.outlier_count += 1
            if self.outlier_count <= 2:
                # Single/few-frame spike: do not chase it. Use prediction as pseudo-measurement.
                z_used = pred_pos.copy()
                R_eff = self.R * 100.0
                self.rejected_last = True
            else:
                # If the change persists, accept it slowly as a true new trajectory.
                z_used = z
                R_eff = self.R * 10.0
                self.outlier_count = 0
        else:
            self.outlier_count = 0
            z_used = z
            R_eff = self.R

        # ---------- EKF/KF update ----------
        y = z_used - self.H @ x_minus
        S = self.H @ P_minus @ self.H.T + R_eff
        K = P_minus @ self.H.T @ np.linalg.inv(S)
        self.x_hat = x_minus + K @ y

        # Joseph form improves numerical stability.
        self.P = (self.I - K @ self.H) @ P_minus @ (self.I - K @ self.H).T + K @ R_eff @ K.T

        # Physical velocity limit. Tune according to robot/target capability.
        for axis in range(3):
            self.x_hat[2 * axis + 1, 0] = np.clip(self.x_hat[2 * axis + 1, 0], -0.45, 0.45)

        # ---------- FIR residual smoothing ----------
        # Important: filter only the correction residual, not the full position.
        # This preserves prediction dynamics and avoids visible trajectory lag.
        ekf_pos = self.H @ self.x_hat
        residual = (ekf_pos - pred_pos)[:, 0]

        residual_fir = np.zeros(3)
        for axis in range(3):
            self.residual_hist[axis] = np.roll(self.residual_hist[axis], 1)
            self.residual_hist[axis, 0] = residual[axis]
            residual_fir[axis] = np.dot(self.fir_coeff, self.residual_hist[axis])

        filtered_pos = pred_pos[:, 0] + residual_fir
        return filtered_pos

class HRIFilteredNode(Node):
    def __init__(self):
        super().__init__('hri_ekf_fir_stronger_node')

        self.CENTER_X = 0.40
        self.CENTER_Y = 0.0
        self.CENTER_Z = 0.30
        self.RADIUS = 0.12
        self.OMEGA = 1.5
        self.Z_AMPLITUDE = 0.15
        self.TIME_PER_CYCLE = 15.0
        self.TOTAL_CYCLES = 1
        self.TOTAL_DURATION = self.TIME_PER_CYCLE * self.TOTAL_CYCLES

        # Same challenge parameters should also be used in the non-filtered file.
        # For a stronger filter-vs-no-filter comparison, use this stress test.
        # If you want the original mild test, set these back to 0.0015, 0.02, 0.06.
        self.NOISE_STD_DEV = 0.005
        self.OUTLIER_PROB = 0.05
        self.OUTLIER_MAG = 0.08
        self.rng = np.random.default_rng(42)
        self.sample_idx = 0

        self.processor = RobustEKFFIRSignalProcessor(dt=0.05, measurement_std=self.NOISE_STD_DEV)

        self.start_time = time.time()
        self.is_moving = False
        self.experiment_finished = False
        self.last_command = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log_filename = 'tracking_error_ekf_fir_stronger1.csv'
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

        self.main_timer = self.create_timer(0.05, self.experiment_loop)
        self.get_logger().info(f"🧪 [增强EKF+FIR版]启动。数据存至 {self.log_filename}")

    def experiment_loop(self):
        if self.experiment_finished:
            return

        # Use deterministic simulation time so filtered and non-filtered tests are comparable.
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

        filtered_pos = self.processor.process(noisy)
        filt_x, filt_y, filt_z = filtered_pos[0], filtered_pos[1], filtered_pos[2]

        command_step = 0.0
        if self.last_command is not None:
            command_step = float(np.linalg.norm(filtered_pos - self.last_command))
        self.last_command = filtered_pos.copy()

        actual = self.get_current_pose()
        if actual:
            real = np.array([actual.translation.x, actual.translation.y, actual.translation.z])
            err_noisy_ideal = float(np.linalg.norm(noisy - ideal))
            err_filt_ideal = float(np.linalg.norm(filtered_pos - ideal))
            err_real_ideal = float(np.linalg.norm(real - ideal))
            err_filt_real = float(np.linalg.norm(filtered_pos - real))

            self.log_writer.writerow([
                time.time(), elapsed_time, ideal_x, ideal_y, ideal_z,
                noisy[0], noisy[1], noisy[2], filt_x, filt_y, filt_z,
                real[0], real[1], real[2],
                err_noisy_ideal, err_filt_ideal, err_real_ideal,
                err_filt_real, command_step, int(injected_outlier), int(self.processor.rejected_last)
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
        self.get_logger().info("📊 [增强EKF+FIR版]实验数据记录完毕。")

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
