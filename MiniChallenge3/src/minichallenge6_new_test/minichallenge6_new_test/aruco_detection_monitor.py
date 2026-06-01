#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node


class ArucoDetectionMonitor(Node):
    def __init__(self):
        super().__init__('aruco_detection_monitor')

        self.declare_parameter('detection_topic', '/aruco_detections')
        self.declare_parameter('detection_type', 'aruco_opencv')
        self.declare_parameter('diagnostic_period', 1.0)
        self.declare_parameter('known_marker_ids', [])

        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_type = self.get_parameter('detection_type').value
        self.diagnostic_period = self.get_parameter('diagnostic_period').value
        self.known_marker_ids = set(self.get_parameter('known_marker_ids').value)

        self.last_detections = []
        self.last_detection_time = None
        self.create_subscription(self.message_type(), self.detection_topic, self.detection_callback, 10)
        self.create_timer(self.diagnostic_period, self.publish_diagnostics)

        self.get_logger().info(
            f'Monitor ArUco escuchando {self.detection_topic} como {self.detection_type}. '
            'Solo reporta detecciones; no corrige odometria.'
        )

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

        raise RuntimeError(
            'detection_type debe ser aruco_opencv o visualization_marker_array.'
        )

    def detection_callback(self, msg):
        self.last_detection_time = self.get_clock().now()

        if self.detection_type == 'aruco_opencv':
            self.last_detections = [
                self.marker_record(marker.marker_id, marker.pose)
                for marker in msg.markers
            ]
            return

        self.last_detections = [
            self.marker_record(marker.id, marker.pose)
            for marker in msg.markers
        ]

    def marker_record(self, marker_id, pose):
        distance = math.sqrt(
            pose.position.x ** 2 +
            pose.position.y ** 2 +
            pose.position.z ** 2
        )
        return {
            'id': int(marker_id),
            'pose': pose,
            'distance': distance,
            'known': not self.known_marker_ids or int(marker_id) in self.known_marker_ids,
        }

    def publish_diagnostics(self):
        if self.last_detection_time is None:
            self.get_logger().info('Aun no recibo detecciones ArUco.')
            return

        detections = [item for item in self.last_detections if item['known']]
        if not detections:
            self.get_logger().info('Recibi detecciones, pero ninguna coincide con known_marker_ids.')
            return

        closest = min(detections, key=lambda item: item['distance'])
        pose = closest['pose']
        all_ids = ','.join(str(item['id']) for item in detections)
        self.get_logger().info(
            f'aruco_cercano id={closest["id"]}, distancia={closest["distance"]:.2f} m, '
            f'camera_frame_pose: x={pose.position.x:.2f}, y={pose.position.y:.2f}, '
            f'z={pose.position.z:.2f}, ids=[{all_ids}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
