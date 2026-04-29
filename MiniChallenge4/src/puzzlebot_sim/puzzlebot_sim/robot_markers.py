import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class Markers(Node):

    def __init__(self):
        super().__init__('markers')
        self.marker_pub = self.create_publisher(Marker, "/sim_markers", 10)
        self.timer = self.create_timer(0.1, self.publish_markers) # 10 Hz
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.r = 0.5
        self.w = 0.5
        self.center_x = 0.0
        self.center_y = 0.0

    def base_complete(self, stamp):
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.frame_locked = True

        marker.ns = 'puzzlebot'
        marker.id = 0

        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://puzzlebot_sim/meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl"
        marker.mesh_use_embedded_materials = True

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        return marker
    
    def left_wheel(self, stamp):
        marker = Marker()

        marker.header.frame_id = 'wheel_l'
        marker.header.stamp = stamp
        marker.frame_locked = True

        marker.ns = 'puzzlebot'
        marker.id = 1

        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://puzzlebot_sim/meshes/Puzzlebot_Wheel.stl"
        marker.mesh_use_embedded_materials = True

        marker.pose.orientation.x = 0.7071
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.7071

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        return marker
    
    def right_wheel(self, stamp):
        marker = Marker()

        marker.header.frame_id = 'wheel_r'
        marker.header.stamp = stamp
        marker.frame_locked = True

        marker.ns = 'puzzlebot'
        marker.id = 2

        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://puzzlebot_sim/meshes/Puzzlebot_Wheel.stl"
        marker.mesh_use_embedded_materials = True

        marker.pose.orientation.x = 0.7071
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.7071

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        return marker
    
    def back_wheel(self, stamp):
        marker = Marker()

        marker.header.frame_id = 'caster'
        marker.header.stamp = stamp
        marker.frame_locked = True

        marker.ns = 'puzzlebot'
        marker.id = 3

        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://puzzlebot_sim/meshes/Puzzlebot_Caster_Wheel.stl"
        marker.mesh_use_embedded_materials = True

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        return marker
    
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        self.publish_dynamic_tf(stamp)

        self.marker_pub.publish(self.base_complete(stamp))
        self.marker_pub.publish(self.left_wheel(stamp))
        self.marker_pub.publish(self.right_wheel(stamp))
        self.marker_pub.publish(self.back_wheel(stamp))

    def publish_static_tf(self):
        static_transforms = []
        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        static_transforms.append(t)

        t_base_caster = TransformStamped()
        t_base_caster.header.stamp = stamp
        t_base_caster.header.frame_id = 'base_link'
        t_base_caster.child_frame_id = 'caster'

        t_base_caster.transform.translation.x = -0.095
        t_base_caster.transform.translation.y = 0.0
        t_base_caster.transform.translation.z = -0.03

        t_base_caster.transform.rotation.x = 0.0
        t_base_caster.transform.rotation.y = 0.0
        t_base_caster.transform.rotation.z = 0.0
        t_base_caster.transform.rotation.w = 1.0

        static_transforms.append(t_base_caster)
    
        self.static_broadcaster.sendTransform(static_transforms)

    def publish_dynamic_tf(self, stamp):
        dynamic_transforms = []
        actual_time = self.get_clock().now().nanoseconds / 1e9

        #  Odom to base
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        self.x = self.center_x + self.r * np.cos(self.w * actual_time)
        self.y = self.center_y + self.r * np.sin(self.w * actual_time)

        vx = -self.r * self.w * np.sin(self.w * actual_time)
        vy = self.r * self.w * np.cos(self.w * actual_time)

        self.theta = math.atan2(vy, vx)

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(self.theta / 2)
        t.transform.rotation.w = np.cos(self.theta / 2)

        dynamic_transforms.append(t)

        # Base to footprint
        t_base_footprint = TransformStamped()
        t_base_footprint.header.stamp = stamp
        t_base_footprint.header.frame_id = 'base_footprint'
        t_base_footprint.child_frame_id = 'base_link'

        t_base_footprint.transform.translation.x = 0.0
        t_base_footprint.transform.translation.y = 0.0
        t_base_footprint.transform.translation.z = 0.5

        t_base_footprint.transform.rotation.x = 0.0
        t_base_footprint.transform.rotation.y = 0.0
        t_base_footprint.transform.rotation.z = 0.0
        t_base_footprint.transform.rotation.w = 1.0

        dynamic_transforms.append(t_base_footprint)

        # Base to left wheel
        t_base_wheel_l = TransformStamped()
        t_base_wheel_l.header.stamp = stamp
        t_base_wheel_l.header.frame_id = 'base_link'
        t_base_wheel_l.child_frame_id = 'wheel_l'

        t_base_wheel_l.transform.translation.x = 0.052 #0.0
        t_base_wheel_l.transform.translation.y = 0.095 #0.085
        t_base_wheel_l.transform.translation.z = 0.0025 #-0.03

        t_base_wheel_l.transform.rotation.x = 0.0
        t_base_wheel_l.transform.rotation.y = 0.0
        t_base_wheel_l.transform.rotation.z = 0.0
        t_base_wheel_l.transform.rotation.w = 1.0

        dynamic_transforms.append(t_base_wheel_l)

        # Base to right wheel
        t_base_wheel_r = TransformStamped()
        t_base_wheel_r.header.stamp = stamp
        t_base_wheel_r.header.frame_id = 'base_link'
        t_base_wheel_r.child_frame_id = 'wheel_r'
        t_base_wheel_r.transform.translation.x = 0.052 #0.0
        t_base_wheel_r.transform.translation.y = -0.095 #-0.085
        t_base_wheel_r.transform.translation.z = 0.0025 #-0.03

        t_base_wheel_r.transform.rotation.x = 0.0
        t_base_wheel_r.transform.rotation.y = 0.0
        t_base_wheel_r.transform.rotation.z = 0.0
        t_base_wheel_r.transform.rotation.w = 1.0

        dynamic_transforms.append(t_base_wheel_r)

        self.broadcaster.sendTransform(dynamic_transforms)


def main(args=None):
    rclpy.init(args=args)

    node = Markers()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()