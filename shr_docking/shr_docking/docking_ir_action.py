import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from shr_docking.docking_ir_main import Docking_IR
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import os
from std_msgs.msg import Float32, Int64, Int32, Bool
from rclpy.callback_groups import ReentrantCallbackGroup

#ros2 action send_goal /docking_ir shr_msgs/action/DockingRequest {}

class DockingIRActionServer(Node):

    def __init__(self):
        super().__init__('Docking_ir_action_server')
        self.docking_ir = Docking_IR()
        self.rate = self.docking_ir.create_rate(10)
        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'docking_ir',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )


    def goal_callback(self, goal_request):
        self.get_logger().info("weblog="+'ACCEPTED docking with IR goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        # Your cancellation logic here
        self.get_logger().info('Goal cancelled')
        self.goal_cancel = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("working callback")
        print("working init", goal_handle)

        while not (self.docking_ir.bumped):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.abort()
                self.docking_ir.move_robot(0, 0)
                result = DockingRequest.Result()
                result.result = False
                return result
            
            self.docking_ir.move_to_docking_station()
            # self.get_logger().info(str(self.docking_ir.bump))





        print(self.docking_ir.bumped)
        if self.docking_ir.bumped or self.docking_ir.is_charging:
            self.get_logger().info(f'Bump:{self.docking_ir.bump}')
            self.docking_ir.move_robot(0, 0)
            # print(self.docking_ir.is_charging)
            time.sleep(1)
            if(self.docking_ir.is_charging is not None and self.docking_ir.is_charging == True):
                goal_handle.succeed()
                result = DockingRequest.Result()
                result.result = True
                self.docking_ir.bumped = False
                self.docking_ir.is_charging = False
                self.docking_ir.move_robot(0, 0)
                self.rate.sleep()
                self.get_logger().info("weblog="+' docked and charging!')
                return result
            else:
                goal_handle.abort()
                result = DockingRequest.Result()
                result.result = False
                self.docking_ir.bumped = False
                self.docking_ir.is_charging = False
                self.docking_ir.move_robot(0, 0)
                self.rate.sleep()
                self.get_logger().info("weblog="+' docking aborted for not charging!')
                return result
            
        else:
            goal_handle.abort()
            self.docking_ir.move_robot(0, 0)
            
            result = DockingRequest.Result()
            self.get_logger().info("weblog="+' docking aborted!')
            result.result = False
            return result


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockingIRActionServer()
    executor = MultiThreadedExecutor()
    try:
        while rclpy.ok():
            rclpy.spin(subscriber_node, executor=executor)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
