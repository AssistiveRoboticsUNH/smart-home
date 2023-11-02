import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from shr_msgs.action import DockingRequest


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = ActionClient(self, DockingRequest, 'docking')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        goal = DockingRequest.Goal()

        self.future = self.cli.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of '
        )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
