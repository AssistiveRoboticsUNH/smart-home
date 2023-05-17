import os
import time

from shr_msgs.action import DetectLeftHouseRequest
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class DetectLeftHouseActionServer(Node):
    def __init__(self):
        super().__init__('detect_person_action')
        self.action_server = ActionServer(self, DetectLeftHouseRequest, 'left_house',
                                          self.callback, cancel_callback=self.cancel_callback)

        self.human_loc = self.create_subscription(String, '/human_loc_from_cams', self.hum_loc_callback, 10)
        self.human_outside = False
        print('started')

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def hum_loc_callback(self, msg):
        if msg.data == 'outside':
            self.human_outside = True
            print('outside')

    def callback(self, goal_handle):
        print('callback')
        result = DetectLeftHouseRequest.Result()
        timeout = goal_handle.request.timeout

        start_time = time.time()
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                print('can')
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return DetectLeftHouseRequest.Result()
            if self.human_outside:
                print('fail')
                result.status = "success"
                goal_handle.succeed()
                return result

        result.status = "fail"
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = DetectLeftHouseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    executor.spin()


if __name__ == '__main__':
    main()
