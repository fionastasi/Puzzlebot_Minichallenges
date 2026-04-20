#!/usr/bin/env python3
"""
Localization Node - Dead Reckoning via Odometry
Calculates robot pose based on wheel velocities using kinematic model.
Subscribes to: wr, wl (wheel velocities)
Publishes: odom (nav_msgs/Odometry), pose_odom
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class LocalizationNode(Node):
    """Calculates odometry from wheel velocities"""
    
    def __init__(self):
        super().__init__('localization_node')
        
        # Robot parameters
        self.r = 0.05           # wheel radius [m]
        self.l = 0.19           # wheelbase [m]
        self.dt = 0.05          # time step [s]
        
        # State variables (odometry)
        self.x_odom = 0.0       # position x [m]
        self.y_odom = 0.0       # position y [m]
        self.theta_odom = 0.0   # orientation [rad]
        
        # Velocity buffers
        self.wr_current = 0.0   # right wheel velocity [rad/s]
        self.wl_current = 0.0   # left wheel velocity [rad/s]
        
        # For covariance estimation
        self.position_cov = 0.01
        self.orientation_cov = 0.1
        
        # Last measurement time
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.pub_odom = self.create_publisher(
            Odometry, 'odom', 10
        )
        self.pub_pose_odom = self.create_publisher(
            PoseStamped, 'pose_odom', 10
        )
        
        # Subscribers
        self.create_subscription(
            Float32, 'wr', self.wr_callback, 10
        )
        self.create_subscription(
            Float32, 'wl', self.wl_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for odometry calculation
        self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info(
            f'Localization node initialized\n'
            f'  wheel_radius: {self.r} m\n'
            f'  wheelbase: {self.l} m\n'
            f'  dt: {self.dt} s'
        )
    
    def wr_callback(self, msg: Float32):
        """Receive right wheel velocity"""
        self.wr_current = msg.data
    
    def wl_callback(self, msg: Float32):
        """Receive left wheel velocity"""
        self.wl_current = msg.data
    
    def timer_callback(self):
        """Calculate odometry and publish"""
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        # Calculate linear and angular velocities from wheel speeds
        # v = (r/2) * (wr + wl)
        # w = (r/l) * (wr - wl)
        v = (self.r / 2.0) * (self.wr_current + self.wl_current)
        w = (self.r / self.l) * (self.wr_current - self.wl_current)
        
        # Integrate kinematics using Euler method
        # ẋ = v * cos(θ)
        # ẏ = v * sin(θ)
        # θ̇ = ω
        
        self.x_odom += v * math.cos(self.theta_odom) * dt
        self.y_odom += v * math.sin(self.theta_odom) * dt
        self.theta_odom += w * dt
        
        # Normalize theta to [-pi, pi]
        self.theta_odom = math.atan2(math.sin(self.theta_odom), 
                                      math.cos(self.theta_odom))
        
        # Publish odometry
        self.publish_odometry(v, w, current_time)
        
        # Broadcast TF
        self.broadcast_tf(current_time)
    
    def publish_odometry(self, v, w, current_time):
        """Publish nav_msgs/Odometry message"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x_odom
        odom.pose.pose.position.y = self.y_odom
        odom.pose.pose.position.z = 0.0
        
        # Orientation (theta to quaternion)
        cy = math.cos(self.theta_odom * 0.5)
        sy = math.sin(self.theta_odom * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Pose covariance (6x6)
        # Order: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
        odom.pose.covariance[0] = self.position_cov       # x
        odom.pose.covariance[7] = self.position_cov       # y
        odom.pose.covariance[14] = 1e9                    # z (fixed)
        odom.pose.covariance[21] = 1e9                    # roll (fixed)
        odom.pose.covariance[28] = 1e9                    # pitch (fixed)
        odom.pose.covariance[35] = self.orientation_cov   # yaw
        
        # Twist (velocity)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w
        
        # Twist covariance
        for i in range(36):
            odom.twist.covariance[i] = 0.0  # Simplified
        
        self.pub_odom.publish(odom)
        
        # Also publish PoseStamped for visualization
        pose_msg = PoseStamped()
        pose_msg.header = odom.header
        pose_msg.pose = odom.pose.pose
        self.pub_pose_odom.publish(pose_msg)
    
    def broadcast_tf(self, current_time):
        """Broadcast transformation from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x_odom
        t.transform.translation.y = self.y_odom
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        cy = math.cos(self.theta_odom * 0.5)
        sy = math.sin(self.theta_odom * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
