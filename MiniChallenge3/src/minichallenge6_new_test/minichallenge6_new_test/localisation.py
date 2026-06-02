import rclpy
from rclpy import qos
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState # <--- IMPORTANTE: Importamos el nuevo tipo de mensaje
from std_msgs.msg import Float32
import math
import numpy as np

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
        self.declare_parameter('publish_without_fresh_encoders', True)
        self.declare_parameter('initial_covariance_x', 0.05)
        self.declare_parameter('initial_covariance_y', 0.05)
        self.declare_parameter('initial_covariance_theta', 0.10)

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
        self.publish_without_fresh_encoders = self.get_parameter('publish_without_fresh_encoders').value

        self.v = 0.0
        self.w = 0.0
        self.sigma = np.diag([
            self.get_parameter('initial_covariance_x').value,
            self.get_parameter('initial_covariance_y').value,
            self.get_parameter('initial_covariance_theta').value,
        ])

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
            if self.publish_without_fresh_encoders:
                self.v = 0.0
                self.w = 0.0
                self.publish_odometry()
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
            f'posicion: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}, '
            f'v={self.v:.3f}, w={self.w:.3f}'
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
