import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class URDF_TFS(Node):

    def __init__(self):
        super().__init__('URDF_tfs')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, 10)

        self.declare_parameter('tf_prefix', '')
        self.tf_prefix = self.get_parameter('tf_prefix').value

        self.x = 0.0
        self.y = 0.0

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.wr = 0.0
        self.wl = 0.0

        self.wheel_l_angle = 0.0
        self.wheel_r_angle = 0.0
        self.wheel_radius = 0.05
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.publish_static_tf()
        self.timer = self.create_timer(0.02, self.update_visualisation)
        
    def publish_static_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.tf_prefix}/odom'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)

    def publish_dynamic_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{self.tf_prefix}/odom'
        t.child_frame_id = f'{self.tf_prefix}/base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        self.tf_broadcaster.sendTransform(t)

    def publish_wheel_joints(self, current_time_sec):
        dt = current_time_sec - self.last_time
        self.last_time = current_time_sec

        if dt <= 0.0:
            return

        self.wheel_r_angle += self.wr * dt
        self.wheel_l_angle += self.wl * dt

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['wheel_l_joint', 'wheel_r_joint']
        joint_msg.position = [self.wheel_l_angle, self.wheel_r_angle]

        self.joint_pub.publish(joint_msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def update_visualisation(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.publish_dynamic_tf()
        self.publish_wheel_joints(current_time)


def main(args=None):
    rclpy.init(args=args)
    node = URDF_TFS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()