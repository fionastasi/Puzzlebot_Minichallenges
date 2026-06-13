#!/usr/bin/env python3
import math

import rclpy
from rclpy import qos
from rclpy.node import Node


KNOWN_MARKERS = {
    70:  (1.84, -0.295),
    705: (0.93, -1.23),
    706: (2.42, -1.27),
    708: (1.19, -1.25),
    703: (1.21, -2.09),
    702: (0.0, -1.82),
    75:  (2.72, -2.40),
    701: (2.77,  0.0),
    710: (1.86, -0.28),
    711: (3.02, -2.70),
    712: (0.00, -0.92),
    713: (0.37, -3.25),
    714: (0.00, -0.26),
}

CAMERA_TO_BASE_TRANSLATION = (0.1241, 0.0, 0.067)
CAMERA_TO_BASE_ROTATION_MATRIX = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)


class ArucoDetectionMonitor(Node):
    def __init__(self):
        super().__init__('aruco_detection_monitor')

        self.declare_parameter('detection_topic', '/aruco_detections')
        self.declare_parameter('detection_type', 'aruco_opencv')
        self.declare_parameter('diagnostic_period', 1.0)
        self.declare_parameter('known_marker_ids', [])
        self.declare_parameter('camera_offset_x', CAMERA_TO_BASE_TRANSLATION[0])
        self.declare_parameter('camera_offset_y', CAMERA_TO_BASE_TRANSLATION[1])
        self.declare_parameter('camera_offset_z', CAMERA_TO_BASE_TRANSLATION[2])

        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_type = self.get_parameter('detection_type').value
        self.diagnostic_period = self.get_parameter('diagnostic_period').value
        self.known_marker_ids = set(self.get_parameter('known_marker_ids').value)
        self.camera_to_base_translation = (
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value,
        )
        self.camera_to_base_rotation = self.quaternion_from_matrix(
            CAMERA_TO_BASE_ROTATION_MATRIX
        )

        self.last_detections = []
        self.last_detection_time = None
        self.received_message_count = 0
        self.last_raw_marker_count = 0
        msg_type = self.message_type()
        self.create_subscription(msg_type, self.detection_topic, self.detection_callback, 10)
        self.create_subscription(
            msg_type,
            self.detection_topic,
            self.detection_callback,
            qos.qos_profile_sensor_data,
        )
        self.create_timer(self.diagnostic_period, self.publish_diagnostics)

    def message_type(self):
        if self.detection_type == 'aruco_opencv':
            try:
                from aruco_opencv_msgs.msg import ArucoDetection
                return ArucoDetection
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_opencv_msgs. Sourcea el workspace/instalacion '
                    'donde existe aruco_opencv, o usa detection_type:=visualization_marker_array.'
                ) from exc

        if self.detection_type == 'visualization_marker_array':
            from visualization_msgs.msg import MarkerArray
            return MarkerArray

        if self.detection_type == 'aruco_msgs':
            try:
                from aruco_msgs.msg import MarkerArray
                return MarkerArray
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_msgs. Verifica el tipo real con: '
                    'ros2 topic type /marker_publisher/markers'
                ) from exc

        if self.detection_type == 'markers_list':
            from std_msgs.msg import Int32MultiArray
            return Int32MultiArray

        if self.detection_type == 'markers_list_u32':
            from std_msgs.msg import UInt32MultiArray
            return UInt32MultiArray

        raise RuntimeError(
            'detection_type debe ser aruco_opencv, aruco_msgs, visualization_marker_array, '
            'markers_list o markers_list_u32.'
        )

    def detection_callback(self, msg):
        self.last_detection_time = self.get_clock().now()
        self.received_message_count += 1

        if self.detection_type == 'aruco_opencv':
            self.last_raw_marker_count = len(msg.markers)
            self.last_detections = [
                self.marker_record(marker.marker_id, marker.pose)
                for marker in msg.markers
            ]
            return

        if self.detection_type == 'aruco_msgs':
            self.last_raw_marker_count = len(msg.markers)
            self.last_detections = [
                self.marker_record(marker.id, marker.pose.pose)
                for marker in msg.markers
            ]
            return

        if self.detection_type in ('markers_list', 'markers_list_u32'):
            self.last_raw_marker_count = len(msg.data)
            self.last_detections = [
                self.marker_record(marker_id, None)
                for marker_id in msg.data
            ]
            return

        self.last_raw_marker_count = len(msg.markers)
        self.last_detections = [
            self.marker_record(marker.id, marker.pose)
            for marker in msg.markers
        ]

    def marker_record(self, marker_id, pose):
        distance = None
        if pose is not None:
            distance = math.sqrt(
                pose.position.x ** 2 +
                pose.position.y ** 2 +
                pose.position.z ** 2
            )

        return {
            'id': int(marker_id),
            'pose': pose,
            'robot_pose': self.pose_camera_to_robot(pose) if pose is not None else None,
            'distance': distance,
            'known': not self.known_marker_ids or int(marker_id) in self.known_marker_ids,
            'map_position': KNOWN_MARKERS.get(int(marker_id)),
        }

    def pose_camera_to_robot(self, pose):
        position = pose.position
        orientation = pose.orientation
        tx, ty, tz = self.camera_to_base_translation

        robot_position = (
            position.z + tx,
            -position.x + ty,
            -position.y + tz,
        )
        robot_orientation = self.normalize_quaternion(
            self.multiply_quaternions(
                self.camera_to_base_rotation,
                (orientation.x, orientation.y, orientation.z, orientation.w),
            )
        )

        return {
            'position': robot_position,
            'orientation': robot_orientation,
        }

    def multiply_quaternions(self, first, second):
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def normalize_quaternion(self, quaternion):
        x, y, z, w = quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return x / norm, y / norm, z / norm, w / norm

    def quaternion_from_matrix(self, matrix):
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return (
                (m21 - m12) / s,
                (m02 - m20) / s,
                (m10 - m01) / s,
                0.25 * s,
            )

        if m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            return (
                0.25 * s,
                (m01 + m10) / s,
                (m02 + m20) / s,
                (m21 - m12) / s,
            )

        if m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            return (
                (m01 + m10) / s,
                0.25 * s,
                (m12 + m21) / s,
                (m02 - m20) / s,
            )

        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        return (
            (m02 + m20) / s,
            (m12 + m21) / s,
            0.25 * s,
            (m10 - m01) / s,
        )

    def publish_diagnostics(self):
        if self.last_detection_time is None:
            return

        if self.last_raw_marker_count == 0:
            return

        detections = [item for item in self.last_detections if item['known']]
        if not detections:
            return

        if all(item['distance'] is None for item in detections):
            return

        closest = min(
            detections,
            key=lambda item: item['distance'] if item['distance'] is not None else float('inf')
        )
        self.get_logger().info(self.format_robot_pose(closest))

    def format_robot_pose(self, marker):
        robot_pose = marker['robot_pose']
        if robot_pose is None:
            return 'aruco_en_robot=sin_pose'

        robot_x, robot_y, robot_z = robot_pose['position']
        robot_qx, robot_qy, robot_qz, robot_qw = robot_pose['orientation']
        return (
            f'aruco_en_robot=(x={robot_x:.3f}, y={robot_y:.3f}, z={robot_z:.3f}, '
            f'qx={robot_qx:.3f}, qy={robot_qy:.3f}, '
            f'qz={robot_qz:.3f}, qw={robot_qw:.3f})'
        )

    def format_marker(self, marker, selected=False):
        pose = marker['pose']
        robot_pose = marker['robot_pose']
        map_text = 'map=(desconocido)'
        if marker['map_position'] is not None:
            marker_x, marker_y = marker['map_position']
            map_text = f'map=({marker_x:.2f}, {marker_y:.2f})'

        selected_text = ' seleccionado' if selected else ''
        if pose is None:
            return f'id={marker["id"]}{selected_text}, {map_text}'

        distance_text = 'dist=sin_pose'
        if marker['distance'] is not None:
            distance_text = f'dist={marker["distance"]:.2f}m'

        camera_orientation_text = (
            'cam_q=('
            f'x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, '
            f'z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f})'
        )
        robot_pose_text = 'robot_pose=sin_pose'
        if robot_pose is not None:
            robot_x, robot_y, robot_z = robot_pose['position']
            robot_qx, robot_qy, robot_qz, robot_qw = robot_pose['orientation']
            robot_pose_text = (
                f'robot=(x={robot_x:.3f}, y={robot_y:.3f}, z={robot_z:.3f}, '
                f'qx={robot_qx:.3f}, qy={robot_qy:.3f}, '
                f'qz={robot_qz:.3f}, qw={robot_qw:.3f})'
            )

        return (
            f'id={marker["id"]}{selected_text}, {map_text}, '
            f'{distance_text}, '
            f'cam=(x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}), '
            f'{camera_orientation_text}, {robot_pose_text}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
