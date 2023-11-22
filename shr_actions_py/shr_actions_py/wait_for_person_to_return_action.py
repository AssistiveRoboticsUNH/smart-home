import os
import time

from shr_msgs.action import WaitForPersonToReturnRequest
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class PersonReturnedToHouseActionServer(Node):
    def __init__(self):
        super().__init__('wait_for_person_to_return_action')
        self.action_server = ActionServer(self, WaitForPersonToReturnRequest, 'returned_to_house',
                                          self.callback, cancel_callback=self.cancel_callback)
        #     self.yolo_sub = self.create_subscription(Int32MultiArray, '/detect_human', self.yolo_sub_callback, 10)
        # self.human_loc = self.create_subscription(Bool, '/human_loc_from_cams', self.hum_loc_callback, 10)
        # self.human_outside = False
        self.human_loc = self.create_subscription(String, '/human_loc_from_cams', self.hum_loc_callback, 10)
        self.human_back = False

    def hum_loc_callback(self, msg):
        # if msg.data:
        #     self.human_coords = msg.data
        if msg.data != 'outside':
            self.human_back = True
            print("person_back")
        else:
            self.human_back = False
    #
    # def yolo_sub_callback(self, msg):
    #     if len(msg.data) > 0 and sum(msg.data) != 0:
    #         self.human_coords = msg.data

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def callback(self, goal_handle):
        self.get_logger().info('waiting for person to return ')
        # self.human_coords = ""
        result = WaitForPersonToReturnRequest.Result()
        timeout = goal_handle.request.timeout

        start_time = time.time()
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('person is not back ')
                print("person not back")
                self.get_logger().info('Goal canceled')
                return WaitForPersonToReturnRequest.Result()
            if self.human_back:
                print("person back")
                self.get_logger().info('person is back ')
                result.status = "success"
                goal_handle.succeed()
                return result

        result.status = "fail"
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = PersonReturnedToHouseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    #executor.spin()
    while True:
        executor.spin_once(timeout_sec=1.0)


if __name__ == '__main__':
    main()
