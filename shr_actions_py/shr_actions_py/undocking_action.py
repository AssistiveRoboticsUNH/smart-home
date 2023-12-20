import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from geometry_msgs.msg import Twist
import os
## TODO: check if its about to hit anything

class UnDockingActionServer(Node):

    def __init__(self):
        super().__init__('UnDocking_action_server')

        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'undocking',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )
        # print("working action")
        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)
        self.time_out = 2
    def goal_callback(self, goal_request):
        # You can add logic here to decide whether to accept or reject the goal.
        # For example, you might check if the goal is valid or if the robot is ready.

        # If you want to accept the goal, return GoalResponse.ACCEPT
        # If you want to reject the goal, return GoalResponse.REJECT
        self.get_logger().info("weblog="+'ACCEPTED undocking goal')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("executing callback")
        self.get_logger().info("weblog="+'starting undocking!')
        start_time = time.time()
        speed = 3.14 / 15.0
        msg = Twist()
        msg.linear.x = -speed
        while time.time() - start_time < self.time_out:
            self.vel_pub.publish(msg)
            print("Undocking")

        msg.linear.x = 0.0
        self.vel_pub.publish(msg)

        goal_handle.succeed()
        result = DockingRequest.Result()
        result.result = True
        self.get_logger().info("weblog="+'undocking is successful')
        return result

        # else:
        #     goal_handle.abort()
        #     result = DockingRequest.Result()
        #     result.result = False
        #     return result

    def feedback_callback(self, msg):
        # self.get_logger().info('Received action feedback message')
        self.feedback = msg.feedback
        return


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = UnDockingActionServer()
    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, timeout_sec=5.0)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
