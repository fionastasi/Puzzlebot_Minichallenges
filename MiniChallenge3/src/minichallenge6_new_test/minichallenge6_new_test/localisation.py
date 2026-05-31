import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Float32
from visualization_msgs.msg import Marker
import math
import numpy as np

class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')
        
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('kr', 0.01)
        self.declare_parameter('kl', 0.01)
        self.declare_parameter('encoder_right_topic', '/VelocityEncR')
        self.declare_parameter('encoder_left_topic', '/VelocityEncL')

        self.encoder_right_topic = self.get_parameter('encoder_right_topic').value
        self.encoder_left_topic = self.get_parameter('encoder_left_topic').value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.marker_pub = self.create_publisher(Marker, 'localisation_covariance', 10)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(Float32, self.encoder_right_topic, self._cb_wr, qos_sensor)
        self.create_subscription(Float32, self.encoder_left_topic, self._cb_wl, qos_sensor)

        self.r = 0.05
        self.l = 0.19

        self.wr = 0.0
        self.wl = 0.0

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.theta = self.get_parameter('initial_theta').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value

        self.v = 0.0
        self.w = 0.0
        self.sigma = np.zeros((3, 3))

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_localisation)

    # NUEVO CALLBACK: Extrae la velocidad (rad/s) de las llantas desde el JointState
    def _cb_wr(self, msg):
        self.wr = msg.data

    def _cb_wl(self, msg):
        self.wl = msg.data

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

    def publish_odometry(self):
        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self._odom_frame()
        odom_msg.child_frame_id = self._base_frame()

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
        self.publish_covariance_marker()

    def _odom_frame(self):
        return f'{self.tf_prefix}/odom' if self.tf_prefix else 'odom'

    def _base_frame(self):
        return f'{self.tf_prefix}/base_footprint' if self.tf_prefix else 'base_footprint'

    def publish_covariance_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._odom_frame()
        marker.ns = 'localisation_covariance'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        marker.scale.x = max(0.05, float(self.sigma[0, 0]))
        marker.scale.y = max(0.05, float(self.sigma[1, 1]))
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4

        self.marker_pub.publish(marker)

    def update_localisation(self):
        self.compute_robot_velocities()
        self.propagate_covariance()
        self.integrate_odometry()
        self.publish_odometry()

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()