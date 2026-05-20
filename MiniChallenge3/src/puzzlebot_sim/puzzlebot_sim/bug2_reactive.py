#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def quaternion_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class Bug2Reactive(Node):
    def __init__(self):
        super().__init__('bug2_reactive')

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.heading = 0.0

        self.reference_x = 0.0
        self.reference_y = 0.0
        self.reference_set = False

        self.goal_x = 8.0
        self.goal_y = 0.0

        self.closest_hit = float('inf')
        self.mode = 0
        self.obstacle_limit = 0.4

        self.sectors = {
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
            'right': 10.0,
            'fright': 10.0,
        }

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.cmd_out = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self._control_cycle)

    def _odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not self.reference_set:
            self.reference_x = self.pose_x
            self.reference_y = self.pose_y
            self.reference_set = True

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

    def _goal_heading(self):
        return math.atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)

    def _goal_distance(self):
        return math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y)

    def _cross_track_error(self):
        numerator = abs((self.goal_y - self.reference_y) * self.pose_x -
                        (self.goal_x - self.reference_x) * self.pose_y +
                        self.goal_x * self.reference_y - self.goal_y * self.reference_x)
        denominator = math.hypot(self.goal_y - self.reference_y, self.goal_x - self.reference_x)
        if denominator == 0.0:
            return 0.0
        return numerator / denominator

    def _change_mode(self, new_mode):
        if new_mode != self.mode:
            self.get_logger().info(f'Mode change: {self.mode} -> {new_mode}')
            self.mode = new_mode

    def _control_cycle(self):
        if not self.reference_set:
            return

        command = Twist()
        distance = self._goal_distance()
        heading_error = self._normalize_angle(self._goal_heading() - self.heading)

        if distance < 0.1:
            self._change_mode(2)
            self.get_logger().info('Goal reached')
            self.cmd_out.publish(Twist())
            return

        if self.mode == 0:
            if self.sectors['front'] < self.obstacle_limit:
                self.closest_hit = distance
                self._change_mode(1)
        elif self.mode == 1:
            cross_error = self._cross_track_error()
            if cross_error < 0.1 and distance < (self.closest_hit - 0.2):
                self._change_mode(0)

        if self.mode == 0:
            if abs(heading_error) > 0.2:
                command.linear.x = 0.0
                command.angular.z = 0.3 if heading_error > 0 else -0.3
            else:
                command.linear.x = 0.2
                command.angular.z = 0.0
        elif self.mode == 1:
            if self.sectors['front'] < self.obstacle_limit:
                command.linear.x = 0.0
                command.angular.z = 0.5
            elif self.sectors['fright'] < self.obstacle_limit:
                command.linear.x = 0.1
                command.angular.z = 0.2
            elif self.sectors['right'] < self.obstacle_limit:
                command.linear.x = 0.2
                command.angular.z = 0.0
            else:
                command.linear.x = 0.1
                command.angular.z = -0.3

        self.cmd_out.publish(command)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Reactive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
