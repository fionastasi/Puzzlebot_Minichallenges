import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import math

class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.r = 0.05
        self.l = 0.19

        self.wr = 0.0
        self.wl = 0.0

        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value

        self.ns_prefix = self.get_namespace().strip('/')

        self.v = 0.0
        self.w = 0.0

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_localisation)

    def frame_id(self, name):
        if self.ns_prefix:
            return f'{self.ns_prefix}/{name}'
        return name

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data
    
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
        odom_msg.header.frame_id = self.frame_id('odom')
        odom_msg.child_frame_id = self.frame_id('base_footprint')

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

        self.odom_pub.publish(odom_msg)

    def update_localisation(self):
        self.compute_robot_velocities()
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