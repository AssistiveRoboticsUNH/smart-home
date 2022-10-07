import os
import time

from pioneer_shr_msg.action import RotateRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist


class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action')
        self.read_script_action_server = ActionServer(self, RotateRequest, 'rotate',
                                                      self.rotate_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.proc_dict = {}

    def rotate_callback(self, goal_handle):
        result = RotateRequest.Result()
        angle = goal_handle.request.angle
        total_time = goal_handle.request.total_time+1 # account for ramp up

        start_time = time.time()
        speed = angle/total_time
        msg = Twist()
        msg.angular.z = speed
        while time.time() - start_time < total_time:
            self.vel_pub.publish(msg)

        msg.angular.z = 0.0
        for i in range(10):
            self.vel_pub.publish(msg)
            time.sleep(0.01)

        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    rotate_action_server = RotateActionServer()

    while True:
        rclpy.spin_once(rotate_action_server)


if __name__ == '__main__':
    main()
