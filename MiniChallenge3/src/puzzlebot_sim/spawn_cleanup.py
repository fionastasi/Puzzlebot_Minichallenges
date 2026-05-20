#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity


class SpawnCleanup(Node):
    def __init__(self):
        super().__init__('spawn_cleanup')
        self.declare_parameter('entity_name', 'robot1')
        self.entity_name = self.get_parameter('entity_name').value
        self.client = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('Waiting for delete_entity service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning('delete_entity service did not appear. Skipping removal.')
            self._shutdown()
            return

        request = DeleteEntity.Request()
        request.name = self.entity_name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            try:
                future.result()
                self.get_logger().info(f'Deleted entity [{self.entity_name}] if present.')
            except Exception as exc:
                self.get_logger().warning(f'Deletion failed: {exc}')
        else:
            self.get_logger().warning('delete_entity service call timed out.')

        self._shutdown()

    def _shutdown(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SpawnCleanup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
