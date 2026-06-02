import rclpy
from rclpy import qos
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState # <--- IMPORTANTE: Importamos el nuevo tipo de mensaje
from std_msgs.msg import Float32
import math
import numpy as np


KNOWN_MARKERS = {
    70: (1.84, -0.30),
    705: (0.90, -1.20),
    706: (2.39, -1.26),
    708: (1.19, -1.21),
    703: (1.23, -2.07),
    702: (0.28, -1.82),
    75: (2.74, -2.40),
    701: (2.84, 0.00),
}

CAMERA_TO_BASE_TRANSLATION = (0.1241, 0.0, 0.067)


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')
        
        # Gazebo publica joint_states; el Puzzlebot fisico publica velocidades de encoder.
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.joint_sensor_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, qos.qos_profile_sensor_data
        )
        self.wr_sub = self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, 10)
        self.wr_sensor_sub = self.create_subscription(
            Float32, 'VelocityEncR', self.wr_callback, qos.qos_profile_sensor_data
        )
        self.wl_sub = self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, 10)
        self.wl_sensor_sub = self.create_subscription(
            Float32, 'VelocityEncL', self.wl_callback, qos.qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('odom_offset_x', 0.0)
        self.declare_parameter('odom_offset_y', 0.0)
        self.declare_parameter('odom_offset_theta', 0.0)
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('kr', 0.01)
        self.declare_parameter('kl', 0.01)
        self.declare_parameter('encoder_timeout', 1.0)
        self.declare_parameter('initial_covariance_x', 0.05)
        self.declare_parameter('initial_covariance_y', 0.05)
        self.declare_parameter('initial_covariance_theta', 0.10)
        self.declare_parameter('use_aruco_correction', True)
        self.declare_parameter('aruco_detection_topic', '/marker_publisher/markers')
        self.declare_parameter('camera_offset_x', CAMERA_TO_BASE_TRANSLATION[0])
        self.declare_parameter('camera_offset_y', CAMERA_TO_BASE_TRANSLATION[1])
        self.declare_parameter('camera_offset_z', CAMERA_TO_BASE_TRANSLATION[2])
        self.declare_parameter('aruco_measurement_std_x', 0.08)
        self.declare_parameter('aruco_measurement_std_y', 0.08)
        self.declare_parameter('aruco_max_correction_distance', 3.0)

        self.r = 0.05
        self.l = 0.19

        self.wr = 0.0
        self.wl = 0.0
        self.last_wr_time = None
        self.last_wl_time = None
        self.last_joint_time = None
        self.last_diagnostic_time = self.get_clock().now()

        self.x = self.get_parameter('initial_x').value + self.get_parameter('odom_offset_x').value
        self.y = self.get_parameter('initial_y').value + self.get_parameter('odom_offset_y').value
        self.theta = self.normalize_angle(
            self.get_parameter('initial_theta').value +
            self.get_parameter('odom_offset_theta').value
        )
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value
        self.encoder_timeout = self.get_parameter('encoder_timeout').value
        self.use_aruco_correction = self.get_parameter('use_aruco_correction').value
        self.aruco_detection_topic = self.get_parameter('aruco_detection_topic').value
        self.camera_to_base_translation = (
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value,
        )
        self.aruco_measurement_std_x = self.get_parameter('aruco_measurement_std_x').value
        self.aruco_measurement_std_y = self.get_parameter('aruco_measurement_std_y').value
        self.aruco_max_correction_distance = self.get_parameter('aruco_max_correction_distance').value

        self.v = 0.0
        self.w = 0.0
        self.sigma = np.diag([
            self.get_parameter('initial_covariance_x').value,
            self.get_parameter('initial_covariance_y').value,
            self.get_parameter('initial_covariance_theta').value,
        ])
        self.last_aruco_time = None
        self.aruco_correction_count = 0

        if self.use_aruco_correction:
            try:
                from aruco_msgs.msg import MarkerArray
                self.aruco_sub = self.create_subscription(
                    MarkerArray,
                    self.aruco_detection_topic,
                    self.aruco_callback,
                    qos.qos_profile_sensor_data,
                )
                self.get_logger().info(
                    f'Correccion EKF con ArUco activa en {self.aruco_detection_topic}.'
                )
            except ImportError:
                self.use_aruco_correction = False
                self.get_logger().warn(
                    'No pude importar aruco_msgs; localisation funcionara solo con encoders.'
                )

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_localisation)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # NUEVO CALLBACK: Extrae la velocidad (rad/s) de las llantas desde el JointState
    def joint_callback(self, msg):
        try:
            idx_r = msg.name.index('wheel_r_joint')
            idx_l = msg.name.index('wheel_l_joint')
            self.wr = msg.velocity[idx_r]
            self.wl = msg.velocity[idx_l]
            self.last_joint_time = self.get_clock().now()
        except ValueError:
            pass # Si el mensaje no trae las ruedas, lo ignoramos

    def wr_callback(self, msg):
        self.wr = msg.data
        self.last_wr_time = self.get_clock().now()

    def wl_callback(self, msg):
        self.wl = msg.data
        self.last_wl_time = self.get_clock().now()

    def propagate_covariance(self):
        delta_d = self.v * self.dt
        delta_theta = self.w * self.dt

        theta_prev = self.theta

        H = np.array([
            [1.0, 0.0, -delta_d * math.sin(theta_prev)],
            [0.0, 1.0,  delta_d * math.cos(theta_prev)],
            [0.0, 0.0,  1.0]
        ])

        q_r = self.kr * abs(self.wr * self.dt)
        q_l = self.kl * abs(self.wl * self.dt)

        Q = np.array([
            [q_r, 0.0, 0.0],
            [0.0, q_l, 0.0],
            [0.0, 0.0, abs(delta_theta) * (self.kr + self.kl)]
        ])

        self.sigma = H @ self.sigma @ H.T + Q
    
    def compute_robot_velocities(self):
        self.v = self.r * (self.wr + self.wl) / 2.0
        self.w = self.r * (self.wr - self.wl) / self.l

    def integrate_odometry(self):
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        theta_dot = self.w

        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.theta += theta_dot * self.dt
        self.theta = self.normalize_angle(self.theta)

    def aruco_callback(self, msg):
        best_marker = None
        best_distance = float('inf')

        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in KNOWN_MARKERS:
                continue

            marker_in_robot = self.marker_camera_pose_to_robot_xy(marker.pose.pose)
            distance = math.sqrt(marker_in_robot[0] ** 2 + marker_in_robot[1] ** 2)
            if distance > self.aruco_max_correction_distance:
                continue

            if distance < best_distance:
                best_distance = distance
                best_marker = (marker_id, marker_in_robot)

        if best_marker is None:
            return

        marker_id, marker_in_robot = best_marker
        self.correct_with_aruco(marker_id, marker_in_robot)
        self.last_aruco_time = self.get_clock().now()

    def marker_camera_pose_to_robot_xy(self, pose):
        tx, ty, _ = self.camera_to_base_translation
        return (
            pose.position.z - tx,
            -pose.position.x - ty,
        )

    def correct_with_aruco(self, marker_id, marker_in_robot):
        marker_map_x, marker_map_y = KNOWN_MARKERS[marker_id]
        marker_robot_x, marker_robot_y = marker_in_robot
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        predicted_marker_x = (
            self.x +
            cos_theta * marker_robot_x -
            sin_theta * marker_robot_y
        )
        predicted_marker_y = (
            self.y +
            sin_theta * marker_robot_x +
            cos_theta * marker_robot_y
        )

        z = np.array([marker_map_x, marker_map_y])
        h = np.array([predicted_marker_x, predicted_marker_y])
        innovation = z - h

        H = np.array([
            [
                1.0,
                0.0,
                -sin_theta * marker_robot_x - cos_theta * marker_robot_y,
            ],
            [
                0.0,
                1.0,
                cos_theta * marker_robot_x - sin_theta * marker_robot_y,
            ],
        ])
        R = np.diag([
            self.aruco_measurement_std_x ** 2,
            self.aruco_measurement_std_y ** 2,
        ])
        S = H @ self.sigma @ H.T + R

        try:
            K = self.sigma @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn('EKF ArUco omitido: matriz de innovacion singular.')
            return

        correction = K @ innovation
        self.x += correction[0]
        self.y += correction[1]
        self.theta = self.normalize_angle(self.theta + correction[2])

        identity = np.eye(3)
        joseph = identity - K @ H
        self.sigma = joseph @ self.sigma @ joseph.T + K @ R @ K.T
        self.aruco_correction_count += 1

    def publish_odometry(self):
        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        if self.tf_prefix:
            odom_msg.header.frame_id = f'{self.tf_prefix}/odom'
            odom_msg.child_frame_id = f'{self.tf_prefix}/base_footprint'
        else:
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.w

        pose_covariance = [0.0] * 36

        pose_covariance[0] = self.sigma[0, 0]    # x-x
        pose_covariance[1] = self.sigma[0, 1]    # x-y
        pose_covariance[5] = self.sigma[0, 2]    # x-yaw

        pose_covariance[6] = self.sigma[1, 0]    # y-x
        pose_covariance[7] = self.sigma[1, 1]    # y-y
        pose_covariance[11] = self.sigma[1, 2]   # y-yaw

        pose_covariance[30] = self.sigma[2, 0]   # yaw-x
        pose_covariance[31] = self.sigma[2, 1]   # yaw-y
        pose_covariance[35] = self.sigma[2, 2]   # yaw-yaw

        odom_msg.pose.covariance = pose_covariance
        odom_msg.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom_msg)

    def update_localisation(self):
        if not self.encoder_data_fresh():
            self.publish_diagnostics()
            return

        self.compute_robot_velocities()
        self.propagate_covariance()
        self.integrate_odometry()
        self.publish_odometry()
        self.publish_diagnostics()

    def encoder_data_fresh(self):
        now = self.get_clock().now()
        wr_age = self.message_age(now, self.last_wr_time)
        wl_age = self.message_age(now, self.last_wl_time)
        joint_age = self.message_age(now, self.last_joint_time)

        has_fresh_encoders = (
            wr_age is not None and wr_age <= self.encoder_timeout and
            wl_age is not None and wl_age <= self.encoder_timeout
        )
        has_fresh_joint_state = joint_age is not None and joint_age <= self.encoder_timeout
        return has_fresh_encoders or has_fresh_joint_state

    def publish_diagnostics(self):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return

        self.last_diagnostic_time = now
        wr_age = self.message_age(now, self.last_wr_time)
        wl_age = self.message_age(now, self.last_wl_time)
        joint_age = self.message_age(now, self.last_joint_time)

        if (wr_age is None or wr_age > 1.0) and (joint_age is None or joint_age > 1.0):
            self.get_logger().warn('No estoy recibiendo encoder derecho reciente.')
        if (wl_age is None or wl_age > 1.0) and (joint_age is None or joint_age > 1.0):
            self.get_logger().warn('No estoy recibiendo encoder izquierdo reciente.')

        self.get_logger().info(
            f'posicion: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}'
        )

    def message_age(self, now, last_time):
        if last_time is None:
            return None
        return (now - last_time).nanoseconds * 1.0e-9

    def format_age(self, age):
        if age is None:
            return 'nunca'
        return f'{age:.1f}s'

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
