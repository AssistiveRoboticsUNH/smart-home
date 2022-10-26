import os
import time

from shr_msgs.action import DetectPersonRequest
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor


class DetectPersonActionServer(Node):
    def __init__(self):
        super().__init__('detect_person_action')
        self.action_server = ActionServer(self, DetectPersonRequest, 'detect_person',
                                          self.callback, cancel_callback=self.cancel_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.yolo_sub = self.create_subscription(String, '/detecthuman', self.yolo_sub_callback, 10)
        self.human_coords = ""

    def yolo_sub_callback(self, msg):
        if len(msg.data) > 0 and msg.data != '[0, 0, 0, 0]':
            self.human_coords = msg.data

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def callback(self, goal_handle):
        self.human_coords = ""
        result = DetectPersonRequest.Result()
        timeout = goal_handle.request.timeout

        start_time = time.time()
        speed = 3.14 / 5.0
        msg = Twist()
        msg.angular.z = speed
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                self.get_logger().info('Goal canceled')
                return DetectPersonRequest.Result()
            if self.human_coords != "":
                result.status = "success"
                goal_handle.succeed()
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                return result

            self.vel_pub.publish(msg)

        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        result.status = "fail"
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = DetectPersonActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    executor.spin()
    # while True:
    #     executor.spin_once()


if __name__ == '__main__':
    main()
