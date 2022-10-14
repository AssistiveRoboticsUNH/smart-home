import os
import time

from shr_msgs.action import FindPersonRequest, GatherInformationRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse
from rclpy.node import Node
import rclpy

from std_msgs.msg import Bool
from shr_msgs.msg import WorldState

import datetime
import time
class MyActionClient(Node):
    def __init__(self):
        super().__init__("my_action_client")
        self.goal_handle = None
        self._action_client = ActionClient(self, GatherInformationRequest, 'gather_information')

    def send_goal(self):
        #Send an action goal
        goal_msg = GatherInformationRequest.Goal()
        goal_msg.states = ['patient_location']
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # self._get_result_future = self.goal_handle.get_result_async()

    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_callback)

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
        else:
            self.get_logger().warning('Goal failed to cancel')



def main(args=None):
    rclpy.init(args=args)

    world_state_node = MyActionClient()
    world_state_node.send_goal()
    start = time.time()
    while world_state_node.goal_handle is None:
        rclpy.spin_once(world_state_node)

    world_state_node.cancel_goal()

    rclpy.spin(world_state_node)


if __name__ == '__main__':
    main()
