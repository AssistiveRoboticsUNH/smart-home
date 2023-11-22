import os
import time

from shr_msgs.action import RotateRequest
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor


class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action')
        self.read_script_action_server = ActionServer(self, RotateRequest, 'rotate',
                                                      self.rotate_callback, cancel_callback=self.cancel_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def rotate_callback(self, goal_handle):
        result = RotateRequest.Result()
        angle = goal_handle.request.angle
        total_time = goal_handle.request.total_time + 1  # account for ramp up

        start_time = time.time()
        speed = angle / total_time
        msg = Twist()
        msg.angular.z = speed
        while time.time() - start_time < total_time:
            if goal_handle.is_cancel_requested:
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return RotateRequest.Result()

            self.vel_pub.publish(msg)
            # rclpy.spin_once(self, timeout_sec=0)

        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    rotate_action_server = RotateActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(rotate_action_server)

    while True:
        executor.spin_once(timeout_sec=1.0)


if __name__ == '__main__':
    main()
