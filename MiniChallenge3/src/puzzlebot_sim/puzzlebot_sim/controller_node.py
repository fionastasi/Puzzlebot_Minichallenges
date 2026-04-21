#!/usr/bin/env python3
"""
Controller Node - PD Controller for Path Tracking
Subscribes to: odom (current pose), target_pose (desired pose)
Publishes: cmd_vel (control commands to simulator)
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class ControllerNode(Node):
    """PD controller for Puzzlebot navigation"""
    
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters
        self.declare_parameter('kp_linear', 0.3)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 2.0)
        
        # Control gains
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        
        # Tolerances
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Velocity limits
        self.max_v = self.get_parameter('max_linear_velocity').value
        self.max_w = self.get_parameter('max_angular_velocity').value
        
        # State
        self.current_pose = None    # Current pose from odometry
        self.target_pose = None     # Target pose
        self.prev_v_error = 0.0
        self.prev_w_error = 0.0
        self.last_error_time = self.get_clock().now()
        
        self.waypoint_reached = False
        
        # Publisher
        self.pub_cmd_vel = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        
        # Subscribers
        self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.create_subscription(
            PoseStamped, 'target_pose', self.target_pose_callback, 10
        )
        
        # Control loop timer
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(
            f'Controller node initialized\n'
            f'  kp_linear: {self.kp_linear}, kd_linear: {self.kd_linear}\n'
            f'  kp_angular: {self.kp_angular}, kd_angular: {self.kd_angular}\n'
            f'  pos_tolerance: {self.pos_tolerance} m\n'
            f'  angle_tolerance: {self.angle_tolerance} rad'
        )
    
    def odom_callback(self, msg: Odometry):
        """Receive current odometry"""
        self.current_pose = msg
    
    def target_pose_callback(self, msg: PoseStamped):
        """Receive target pose"""
        self.target_pose = msg
        self.waypoint_reached = False
    
    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None or self.target_pose is None:
            # Send zero velocity
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
            return
        
        # Extract current pose
        x_current = self.current_pose.pose.pose.position.x
        y_current = self.current_pose.pose.pose.position.y
        quat = self.current_pose.pose.pose.orientation
        
        # Convert quaternion to theta
        theta_current = 2 * math.atan2(quat.z, quat.w)
        
        # Extract target pose
        x_target = self.target_pose.pose.position.x
        y_target = self.target_pose.pose.position.y
        quat_target = self.target_pose.pose.orientation
        
        # Convert target quaternion to theta
        theta_target = 2 * math.atan2(quat_target.z, quat_target.w)
        
        # Calculate error in target frame
        # First, vector from current to target in global frame
        dx = x_target - x_current
        dy = y_target - y_current
        
        # Distance to target
        dist_to_target = math.sqrt(dx**2 + dy**2)
        
        # Angle to target in global frame
        angle_to_target = math.atan2(dy, dx)
        
        # Error in orientation (normalize to [-pi, pi])
        angle_error = angle_to_target - theta_current
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Check if waypoint is reached
        angle_to_target_error = theta_target - theta_current
        angle_to_target_error = math.atan2(
            math.sin(angle_to_target_error), 
            math.cos(angle_to_target_error)
        )
        
        if (dist_to_target < self.pos_tolerance and 
            abs(angle_to_target_error) < self.angle_tolerance):
            
            if not self.waypoint_reached:
                self.get_logger().info(
                    f'Waypoint reached: ({x_target:.2f}, {y_target:.2f})'
                )
                self.waypoint_reached = True
            
            # Stop at waypoint
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
            return
        
        # PD control
        # Stage 1: Rotate to face target
        if abs(angle_error) > 0.2:  # More than ~12 degrees
            v_cmd = 0.0
            w_error = angle_error
        else:
            # Stage 2: Move forward
            v_error = dist_to_target
            v_cmd = self.kp_linear * v_error + self.kd_linear * (v_error - self.prev_v_error)
            
            # Also correct angle while moving
            w_error = angle_error
            w_cmd = self.kp_angular * w_error + self.kd_angular * (w_error - self.prev_w_error)
        
        if abs(angle_error) > 0.2:
            w_error = angle_error
            w_cmd = self.kp_angular * w_error + self.kd_angular * (w_error - self.prev_w_error)
        else:
            w_cmd = self.kp_angular * w_error + self.kd_angular * (w_error - self.prev_w_error)
        
        # Limit velocities
        v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd_vel.publish(cmd)
        
        # Store errors for derivative term
        self.prev_v_error = dist_to_target
        self.prev_w_error = w_error


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
