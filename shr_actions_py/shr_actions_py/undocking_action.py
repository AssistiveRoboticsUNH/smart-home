import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import os
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class UnDockingActionServer(Node):

    def __init__(self):
        super().__init__('UnDocking_action_server')

        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'undocking',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )
        # print("working action")
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)

        self.time_out = 100
        self.min_range = None

    def scan_callback(self, msg):
        print("Scan ***********")
        if msg:
            print("self.min_range", self.min_range)
            start_ind = int(3.5*(len(msg.ranges)/8)) #0
            end_ind = int(4.5*(len(msg.ranges)/8)-1)
            print("self.min_range", self.min_range)
            self.min_range = min(msg.ranges[start_ind:end_ind])


    def goal_callback(self, goal_request):
        # You can add logic here to decide whether to accept or reject the goal.
        # For example, you might check if the goal is valid or if the robot is ready.

        # If you want to accept the goal, return GoalResponse.ACCEPT
        # If you want to reject the goal, return GoalResponse.REJECT
        self.get_logger().info("weblog="+'ACCEPTED undocking goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Your cancellation logic here
        self.get_logger().info('Goal cancelled')
        self.goal_cancel = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("executing callback")
        self.get_logger().info("weblog="+'starting undocking!')
        start_time = time.time()
        speed = 3.14 / 15.0
        msg = Twist()

        while time.time() - start_time < self.time_out:
            print("&&&&&&&&& self.min_range in while #################")
            if self.min_range is not None and self.min_range > 0.7:
                msg.linear.x = -speed
                self.vel_pub.publish(msg)
                print("Undocking")
            else:
                msg.linear.x = 0.0
                self.vel_pub.publish(msg)
                print("Stop robot, obstacle close")

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
    executor = MultiThreadedExecutor()
    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, executor=executor, timeout_sec=5.0)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
