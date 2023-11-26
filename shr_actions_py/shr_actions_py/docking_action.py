import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from shr_actions_py.docking_main import Docking


class DockingActionServer(Node):

    def __init__(self):
        super().__init__('Docking_action_server')
        self.docking = Docking()

        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'docking',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )
        # print("working action")

    def goal_callback(self, goal_request):
        # You can add logic here to decide whether to accept or reject the goal.
        # For example, you might check if the goal is valid or if the robot is ready.

        # If you want to accept the goal, return GoalResponse.ACCEPT
        # If you want to reject the goal, return GoalResponse.REJECT
        self.get_logger().info("weblog="+'ACCEPTED docking goal')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("working callback")
        print("working init", goal_handle)

        while not self.docking.bumped:
            self.docking.get_transformation_from_aptag_to_port()
            self.docking.move_towards_tag()

        print(self.docking.bumped)
        if self.docking.bumped:
            print("Bumped!!")
            self.get_logger().info("weblog="+'charger and port bumped!')
            goal_handle.succeed()
            result = DockingRequest.Result()
            result.result = True
            self.docking.bumped = False
            return result
        else:
            goal_handle.abort()
            result = DockingRequest.Result()
            self.get_logger().info("weblog="+' docking aborted!')
            result.result = False
            return result

    def feedback_callback(self, msg):
        #self.get_logger().info('Received action feedback message')
        self.feedback = msg.feedback
        return





def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockingActionServer()
    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, timeout_sec=5.0)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
