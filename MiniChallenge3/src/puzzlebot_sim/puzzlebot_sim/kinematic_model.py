import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32

import math

class KinematicModel(Node):

    def __init__(self):
        super().__init__('puzzlebot_kinematic_model')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub = self.create_publisher(Float32, '/wl', 10)

        self.r = 0.05   # radio de rueda [m]
        self.l = 0.19   # distancia entre ruedas [m]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.wr = 0.0
        self.wl = 0.0

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_simulation)

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def compute_wheel_speeds(self):
        self.wr = (2.0 * self.v + self.w * self.l) / (2.0 * self.r)
        self.wl = (2.0 * self.v - self.w * self.l) / (2.0 * self.r)

    def integrate_model(self):
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        theta_dot = self.w

        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.theta += theta_dot * self.dt

    def publish_pose(self):
        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pose_pub.publish(pose_msg)

    def publish_wheel_speeds(self):
        wr_msg = Float32()
        wl_msg = Float32()

        wr_msg.data = self.wr
        wl_msg.data = self.wl

        self.wr_pub.publish(wr_msg)
        self.wl_pub.publish(wl_msg)

    def update_simulation(self):
        self.compute_wheel_speeds()
        self.integrate_model()
        self.publish_pose()
        self.publish_wheel_speeds()

def main(args=None):
    rclpy.init(args=args)
    node = KinematicModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()