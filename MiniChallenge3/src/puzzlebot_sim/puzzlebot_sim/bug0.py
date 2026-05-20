import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class Bug0(Node):
    GO_TO_GOAL = 'GO_TO_GOAL'
    WALL_FOLLOWING = 'WALL_FOLLOWING'
    GOAL_REACHED = 'GOAL_REACHED'

    def __init__(self):
        super().__init__('bug0')

        # Parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_rho', 0.8)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('obstacle_distance', 0.6)   # start wall following
        self.declare_parameter('clear_distance', 0.9)      # consider path clear
        self.declare_parameter('front_angle', math.pi/6.0) # +- angle to check direct path

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.clear_distance = self.get_parameter('clear_distance').value
        self.front_angle = self.get_parameter('front_angle').value

        # State
        self.state = self.GO_TO_GOAL

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Laser data
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

        # Timer
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Bug0 node started. Goal: x=%.2f y=%.2f' % (self.goal_x, self.goal_y))

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # yaw from quaternion (safe for planar)
        self.theta = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

    def scan_cb(self, msg: LaserScan):
        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def normalize(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def clamp(self, v, a, b):
        return max(min(v, b), a)

    def obstacle_blocking_path(self):
        # Check front sector for obstacle closer than obstacle_distance
        if len(self.ranges) == 0:
            return False
        half_span = int(self.front_angle / abs(self.angle_increment)) if self.angle_increment != 0 else 1
        center = len(self.ranges)//2
        sector = self.ranges[center - half_span: center + half_span + 1]
        if np.isfinite(sector).any():
            min_r = np.nanmin(np.where(np.isfinite(sector), sector, np.inf))
            return min_r < self.obstacle_distance
        return False

    def is_path_clear_to_goal(self):
        # Compute angle to goal in robot frame and check small cone along that direction
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        if dist < self.goal_tolerance:
            return True
        ang_to_goal = self.normalize(math.atan2(dy, dx) - self.theta)
        if abs(ang_to_goal) > (math.pi/2):
            return False
        # find index in scan corresponding to ang_to_goal
        if len(self.ranges) == 0 or self.angle_increment == 0:
            return False
        idx = int((ang_to_goal - self.angle_min) / self.angle_increment)
        window = max(1, int( (self.front_angle/2) / abs(self.angle_increment) ))
        lo = max(0, idx - window)
        hi = min(len(self.ranges)-1, idx + window)
        sector = self.ranges[lo:hi+1]
        if len(sector) == 0:
            return False
        min_r = np.nanmin(np.where(np.isfinite(sector), sector, np.inf))
        return min_r > self.clear_distance

    def control_loop(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        rho = math.hypot(dx, dy)
        alpha = self.normalize(math.atan2(dy, dx) - self.theta)

        cmd = Twist()

        # Goal reached
        if rho < self.goal_tolerance:
            if self.state != self.GOAL_REACHED:
                self.get_logger().info('GOAL_REACHED')
            self.state = self.GOAL_REACHED
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # FSM transitions
        if self.state == self.GO_TO_GOAL:
            if self.obstacle_blocking_path():
                self.get_logger().info('Obstacle detected: switching to WALL_FOLLOWING')
                self.state = self.WALL_FOLLOWING

        elif self.state == self.WALL_FOLLOWING:
            # If path to goal becomes clear, return to go_to_goal
            if self.is_path_clear_to_goal():
                self.get_logger().info('Path to goal clear: switching to GO_TO_GOAL')
                self.state = self.GO_TO_GOAL

        # State behaviors
        if self.state == self.GO_TO_GOAL:
            # rotate first then go forward
            if abs(alpha) > 0.12:
                cmd.linear.x = 0.0
                cmd.angular.z = self.clamp(self.k_alpha * alpha, -self.w_max, self.w_max)
            else:
                cmd.linear.x = self.clamp(self.k_rho * rho, -self.v_max, self.v_max)
                cmd.angular.z = 0.0

        elif self.state == self.WALL_FOLLOWING:
            # simple reactive wall following based on closest point in front semicircle
            if len(self.ranges) == 0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # consider front 180 deg for wall following
                n = len(self.ranges)
                mid = n // 2
                sector = self.ranges[mid - n//4: mid + n//4 + 1]  # front 90 deg approx
                angles = self.angle_min + (np.arange(len(self.ranges)) * self.angle_increment)
                sec_angles = angles[mid - n//4: mid + n//4 + 1]
                # find closest finite
                valid = np.isfinite(sector)
                if not valid.any():
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    rel_idx = np.argmin(np.where(valid, sector, np.inf))
                    d = float(sector[rel_idx])
                    theta_closest = float(sec_angles[rel_idx])
                    # compute avoidance direction: turn away from obstacle
                    theta_avoid = theta_closest + math.pi if theta_closest < 0 else theta_closest - math.pi
                    theta_avoid = self.normalize(theta_avoid)
                    # simple control: reduce forward speed when close, turn proportional to avoidance angle
                    v = 0.15 if d > 0.2 else 0.0
                    w = 1.2 * theta_avoid
                    cmd.linear.x = self.clamp(v, -self.v_max, self.v_max)
                    cmd.angular.z = self.clamp(w, -self.w_max, self.w_max)

        elif self.state == self.GOAL_REACHED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Bug0()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()