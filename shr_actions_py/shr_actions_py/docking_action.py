import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from shr_actions_py.docking_main import Docking
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import os


class DockingActionServer(Node):

    def __init__(self):
        super().__init__('Docking_action_server')
        self.docking = Docking()
        #self.goal_cancel = False

        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'docking',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # print("working action")
        self.pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 1)
        self.vel = Twist()

    def goal_callback(self, goal_request):
        self.get_logger().info("weblog="+'ACCEPTED docking goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        # Your cancellation logic here
        self.get_logger().info('Goal cancelled')
        self.goal_cancel = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("working callback")
        print("working init", goal_handle)

        while not (self.docking.bumped):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.abort()
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
                result = DockingRequest.Result()
                result.result = False
                return result
            
            self.docking.get_transformation_from_aptag_to_port()
            self.docking.move_towards_tag()

        print(self.docking.bumped)
        if self.docking.bumped:
            print("Bumped!!")
            self.vel.linear.x = 0.0
            self.vel.angular.z =0.0
            self.pub.publish(self.vel)
            self.get_logger().info("weblog="+'charger and port bumped!')
            goal_handle.succeed()
            result = DockingRequest.Result()
            result.result = True
            self.docking.bumped = False
            self.vel.linear.x = 0.0
            self.vel.angular.z =0.0
            self.pub.publish(self.vel)
            return result
        else:
            goal_handle.abort()
            self.vel.linear.x = 0.0
            self.vel.angular.z =0.0
            self.pub.publish(self.vel)
            
            result = DockingRequest.Result()
            self.get_logger().info("weblog="+' docking aborted!')
            result.result = False
            return result


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockingActionServer()
    executor = MultiThreadedExecutor()
    try:
        while rclpy.ok():
            rclpy.spin(subscriber_node, executor=executor)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
