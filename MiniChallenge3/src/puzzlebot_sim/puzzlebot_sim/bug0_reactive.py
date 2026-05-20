#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def quaternion_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class Bug0Reactive(Node):
    def __init__(self):
        super().__init__('bug0_reactive')

        # Posición estimada del robot en el plano
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.heading = 0.0

        # Objetivo fijo de navegación
        self.goal_x = 8.0
        self.goal_y = 0.0

        # Datos del LiDAR agrupados en sectores
        self.sectors = {
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
            'right': 10.0,
            'fright': 10.0,
        }

        # Estados de la máquina: 0 = hacia objetivo, 1 = bordeando, 2 = detenido
        self.mode = 0
        self.obstacle_limit = 0.4
        self.turn_threshold = 0.2

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.cmd_out = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self._control_cycle)

    def _odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _scan_callback(self, msg):
        raw = msg.ranges
        raw = [v if not math.isinf(v) and not math.isnan(v) else 10.0 for v in raw]

        self.sectors = {
            'front': min(min(raw[0:15] + raw[-15:]), 10.0),
            'fleft': min(min(raw[16:75]), 10.0),
            'left': min(min(raw[76:105]), 10.0),
            'right': min(min(raw[-105:-76]), 10.0),
            'fright': min(min(raw[-75:-16]), 10.0),
        }

    def _angle_to_goal(self):
        return math.atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)

    def _distance_to_goal(self):
        return math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _set_mode(self, new_mode):
        if new_mode != self.mode:
            self.get_logger().info(f'Mode change: {self.mode} -> {new_mode}')
            self.mode = new_mode

    def _control_cycle(self):
        velocity = Twist()
        distance = self._distance_to_goal()
        heading_goal = self._angle_to_goal()
        heading_error = self._normalize_angle(heading_goal - self.heading)

        if distance < 0.1:
            self._set_mode(2)
            self.get_logger().info('Goal reached')
            self.cmd_out.publish(Twist())
            return

        if self.mode == 0:
            if self.sectors['front'] < self.obstacle_limit:
                self._set_mode(1)

        elif self.mode == 1:
            if self.sectors['front'] > self.obstacle_limit and abs(heading_error) < self.turn_threshold:
                self._set_mode(0)

        if self.mode == 0:
            if abs(heading_error) > self.turn_threshold:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.3 if heading_error > 0 else -0.3
            else:
                velocity.linear.x = 0.2
                velocity.angular.z = 0.0

        elif self.mode == 1:
            if self.sectors['front'] < self.obstacle_limit:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.5
            elif self.sectors['fright'] < self.obstacle_limit:
                velocity.linear.x = 0.1
                velocity.angular.z = 0.2
            elif self.sectors['right'] < self.obstacle_limit:
                velocity.linear.x = 0.2
                velocity.angular.z = 0.0
            else:
                velocity.linear.x = 0.1
                velocity.angular.z = -0.3

        self.cmd_out.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Reactive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
