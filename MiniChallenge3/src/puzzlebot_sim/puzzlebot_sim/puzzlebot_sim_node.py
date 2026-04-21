#!/usr/bin/env python3
"""
Puzzlebot Kinematic Simulator Node
Simulates the differential drive kinematics of the Puzzlebot robot.
Publishes: pose_sim, wr, wl
Subscribes to: cmd_vel
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PuzzlebotSimNode(Node):
    """Simulates Puzzlebot kinematics"""
    
    def __init__(self):
        super().__init__('puzzlebot_sim_node')
        
        # Robot parameters
        self.r = 0.05           # wheel radius [m]
        self.l = 0.19           # wheelbase [m]
        self.dt = 0.05          # time step [s]
        
        # State variables
        self.x = 0.0            # position x [m]
        self.y = 0.0            # position y [m]
        self.theta = 0.0        # orientation [rad]
        
        # Velocity commands
        self.v_cmd = 0.0        # linear velocity [m/s]
        self.w_cmd = 0.0        # angular velocity [rad/s]
        
        # Wheel velocities (rad/s)
        self.wr = 0.0
        self.wl = 0.0
        
        # Publishers
        self.pub_pose_sim = self.create_publisher(
            PoseStamped, 'pose_sim', 10
        )
        self.pub_wr = self.create_publisher(
            Float32, 'wr', 10
        )
        self.pub_wl = self.create_publisher(
            Float32, 'wl', 10
        )
        
        # Subscribers
        self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main loop
        self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info(
            f'Puzzlebot simulator initialized\n'
            f'  wheel_radius: {self.r} m\n'
            f'  wheelbase: {self.l} m\n'
            f'  dt: {self.dt} s'
        )
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands"""
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
    
    def timer_callback(self):
        """Main simulation loop"""
        # Limit velocities
        self.v_cmd = np.clip(self.v_cmd, -0.5, 0.5)
        self.w_cmd = np.clip(self.w_cmd, -2.0, 2.0)
        
        # Calculate wheel velocities from desired linear and angular velocities
        # v = (r/2) * (wr + wl)  =>  wr + wl = 2*v/r
        # w = (r/l) * (wr - wl)  =>  wr - wl = w*l/r
        # Solving:
        # wr = (2*v + w*l) / (2*r)
        # wl = (2*v - w*l) / (2*r)
        
        if self.v_cmd == 0.0 and self.w_cmd == 0.0:
            self.wr = 0.0
            self.wl = 0.0
        else:
            self.wr = (2.0 * self.v_cmd + self.w_cmd * self.l) / (2.0 * self.r)
            self.wl = (2.0 * self.v_cmd - self.w_cmd * self.l) / (2.0 * self.r)
        
        # Integrate kinematics using Euler method
        # ẋ = v * cos(θ)
        # ẏ = v * sin(θ)
        # θ̇ = ω
        
        # For numerical stability, use instantaneous velocities from wheel speeds
        v_actual = (self.r / 2.0) * (self.wr + self.wl)
        w_actual = (self.r / self.l) * (self.wr - self.wl)
        
        # Euler integration
        self.x += v_actual * math.cos(self.theta) * self.dt
        self.y += v_actual * math.sin(self.theta) * self.dt
        self.theta += w_actual * self.dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish pose
        self.publish_pose()
        
        # Publish wheel velocities
        wr_msg = Float32(data=self.wr)
        wl_msg = Float32(data=self.wl)
        self.pub_wr.publish(wr_msg)
        self.pub_wl.publish(wl_msg)
        
        # Broadcast TF
        self.broadcast_tf()
    
    def publish_pose(self):
        """Publish simulated pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = sy
        pose_msg.pose.orientation.w = cy
        
        self.pub_pose_sim.publish(pose_msg)
    
    def broadcast_tf(self):
        """Broadcast transformation from map to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSimNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
