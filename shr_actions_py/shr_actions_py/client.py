import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose_with_localization')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.goal_pose = PoseStamped()

    def send_request(self, a, b):
        nav_goal = NavigateToPose.Goal()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = a
        self.goal_pose.pose.position.y = b
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        nav_goal.pose = self.goal_pose

        self.future = self.cli.send_goal_async(nav_goal)

        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(float(sys.argv[1]), float(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d ' %
        (float(sys.argv[1]), float(sys.argv[2])))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



