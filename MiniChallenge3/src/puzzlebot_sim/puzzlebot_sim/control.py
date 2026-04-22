import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math

class Control(Node):

    def __init__(self):
        super().__init__('control')

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('k_rho', 0.8)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('goal_tolerance', 0.05)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.02, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)
    
    def control_loop(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        rho = math.sqrt(dx**2 + dy**2)
        alpha = math.atan2(dy, dx) - self.theta
        alpha = self.normalize_angle(alpha)

        cmd = Twist()

        if rho < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return
        
        v = self.k_rho * rho
        w = self.k_alpha * alpha

        v = self.clamp(v, -self.v_max, self.v_max)
        w = self.clamp(w, -self.w_max, self.w_max)

        cmd.linear.x = v
        cmd.angular.z = w

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()