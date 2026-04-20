#!/usr/bin/env python3
"""
Trajectory Generator Node
Generates waypoint sequences for the robot to follow.
Publishes: target_pose (PoseStamped with next waypoint)
Implements: Square, Pentagon, etc.
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class TrajectoryGeneratorNode(Node):
    """Generates trajectory waypoints"""
    
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # Declare parameters
        self.declare_parameter('trajectory_shape', 'square')
        self.declare_parameter('trajectory_size', 1.0)
        self.declare_parameter('waypoint_tolerance', 0.15)
        
        # Get parameters
        shape = self.get_parameter('trajectory_shape').value
        size = self.get_parameter('trajectory_size').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Initialize trajectory
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.trajectory_active = False
        
        self.get_logger().info(
            f'Trajectory Generator initialized\n'
            f'  Shape: {shape}\n'
            f'  Size: {size} m\n'
            f'  Waypoint tolerance: {self.waypoint_tolerance} m'
        )
        
        # Generate waypoints based on shape
        self.generate_trajectory(shape, size)
        
        # Publisher
        self.pub_target_pose = self.create_publisher(
            PoseStamped, 'target_pose', 10
        )
        
        # Service to start trajectory
        self.srv_start = self.create_service(
            Empty, 'start_trajectory', self.start_trajectory_callback
        )
        
        # Timer to publish current target
        self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
        self.start_trajectory()
    
    def generate_trajectory(self, shape: str, size: float):
        """Generate waypoint list based on shape"""
        self.waypoints = []
        
        if shape == 'square':
            # Square with side length = size
            self.waypoints = [
                {'x': size,  'y': 0.0,   'theta': 0.0},
                {'x': size,  'y': size,  'theta': math.pi/2},
                {'x': 0.0,   'y': size,  'theta': math.pi},
                {'x': 0.0,   'y': 0.0,   'theta': 3*math.pi/2},
            ]
        
        elif shape == 'pentagon':
            # Regular pentagon
            for i in range(5):
                angle = 2 * math.pi * i / 5
                x = size * math.cos(angle)
                y = size * math.sin(angle)
                theta = angle + math.pi / 2
                self.waypoints.append({'x': x, 'y': y, 'theta': theta})
        
        elif shape == 'circle':
            # Circle (approximated with 12 waypoints)
            for i in range(12):
                angle = 2 * math.pi * i / 12
                x = size * math.cos(angle)
                y = size * math.sin(angle)
                theta = angle + math.pi / 2
                self.waypoints.append({'x': x, 'y': y, 'theta': theta})
        
        else:
            self.get_logger().warn(f'Unknown shape: {shape}, using square')
            self.generate_trajectory('square', size)
    
    def start_trajectory(self):
        """Start trajectory from beginning"""
        self.current_waypoint_idx = 0
        self.trajectory_active = True
        self.get_logger().info(f'Starting trajectory with {len(self.waypoints)} waypoints')
    
    def start_trajectory_callback(self, request, response):
        """Service callback to start trajectory"""
        self.start_trajectory()
        return response
    
    def timer_callback(self):
        """Publish current target waypoint"""
        if not self.trajectory_active or len(self.waypoints) == 0:
            return
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'odom'
        
        target.pose.position.x = waypoint['x']
        target.pose.position.y = waypoint['y']
        target.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta = waypoint['theta']
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = sy
        target.pose.orientation.w = cy
        
        self.pub_target_pose.publish(target)
    
    def advance_waypoint(self):
        """Move to next waypoint"""
        if self.trajectory_active and len(self.waypoints) > 0:
            self.current_waypoint_idx = (
                (self.current_waypoint_idx + 1) % len(self.waypoints)
            )
            self.get_logger().info(
                f'Advanced to waypoint {self.current_waypoint_idx}/'
                f'{len(self.waypoints)}'
            )
    
    def get_current_waypoint(self):
        """Get current target waypoint"""
        if not self.trajectory_active or len(self.waypoints) == 0:
            return None
        return self.waypoints[self.current_waypoint_idx]


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
