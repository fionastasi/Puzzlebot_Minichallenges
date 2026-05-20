#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def euler_to_quaternion(roll, pitch, yaw):
    roll /= 2.0; pitch /= 2.0; yaw /= 2.0
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)

    qx = cp * sr * cy - sp * cr * sy
    qy = cp * cr * sy + sp * sr * cy
    qz = cp * cr * cy - sp * sr * sy
    qw = cp * cr * cy + sp * sr * sy
    return qx, qy, qz, qw


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)

        self.wheel_radius = 0.05
        self.wheel_base = 0.19
        self.pose_x = float(self.get_parameter('x0').value)
        self.pose_y = float(self.get_parameter('y0').value)
        self.heading = 0.0
        self.speed_left = 0.0
        self.speed_right = 0.0

        self.covariance = np.zeros((3, 3))
        self.sigma_x = 0.0005
        self.sigma_y = 0.0001
        self.sigma_theta = 0.010
        self.cross_term = 0.0

        self.create_subscription(JointState, '/joint_states', self._joint_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_est', 10)
        self.create_timer(0.01, self._step)

    def _joint_callback(self, msg):
        try:
            left_index = msg.name.index('wheel_l_joint')
            right_index = msg.name.index('wheel_r_joint')
            self.speed_left = msg.velocity[left_index]
            self.speed_right = msg.velocity[right_index]
        except ValueError:
            pass

    def _predict_covariance(self, linear, dt):
        jac = np.array([
            [1.0, 0.0, -linear * dt * math.sin(self.heading)],
            [0.0, 1.0, linear * dt * math.cos(self.heading)],
            [0.0, 0.0, 1.0],
        ])
        q = np.array([
            [self.sigma_x, self.cross_term, self.cross_term],
            [self.cross_term, self.sigma_y, self.cross_term],
            [self.cross_term, self.cross_term, self.sigma_theta],
        ]) * abs(linear) * dt
        self.covariance = jac @ self.covariance @ jac.T + q

    def _step(self):
        linear_vel = self.wheel_radius * (self.speed_right + self.speed_left) / 2.0
        angular_vel = self.wheel_radius * (self.speed_right - self.speed_left) / self.wheel_base

        self._predict_covariance(linear_vel, 0.01)
        self.pose_x += linear_vel * math.cos(self.heading) * 0.01
        self.pose_y += linear_vel * math.sin(self.heading) * 0.01
        self.heading += angular_vel * 0.01

        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.heading)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.covariance[0, 0]
        odom.pose.covariance[7] = self.covariance[1, 1]
        odom.pose.covariance[35] = self.covariance[2, 2]
        odom.pose.covariance[1] = self.covariance[0, 1]
        odom.pose.covariance[6] = self.covariance[1, 0]
        odom.pose.covariance[5] = self.covariance[0, 2]
        odom.pose.covariance[30] = self.covariance[2, 0]
        odom.pose.covariance[11] = self.covariance[1, 2]
        odom.pose.covariance[31] = self.covariance[2, 1]
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
