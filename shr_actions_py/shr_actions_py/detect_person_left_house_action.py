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
        super().__init__('detect_person_left_house_action')
        self.action_server = ActionServer(self, DetectLeftHouseRequest, 'left_house',
                                          self.callback, cancel_callback=self.cancel_callback)

        self.human_loc = self.create_subscription(String, '/human_loc_from_cams', self.hum_loc_callback, 10)
        self.human_outside = False

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def hum_loc_callback(self, msg):
        print(msg.data, msg.data)
        if msg.data == 'outside':
            self.human_outside = True
            # print('outside')
        else:
            self.human_outside = False

    def callback(self, goal_handle):
        # print('callback')
        result = DetectLeftHouseRequest.Result()
        timeout = goal_handle.request.timeout

        start_time = time.time()
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                # print('can')
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return DetectLeftHouseRequest.Result()
            if self.human_outside:
                # print('fail')
                self.get_logger().info('Human left house was detected')
                result.status = "success"
                goal_handle.succeed()
                return result

        self.get_logger().info('Human left house was not detected')
        result.status = "fail"
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = DetectLeftHouseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    while True:
        executor.spin_once(timeout_sec=5.0)



if __name__ == '__main__':
    main()
