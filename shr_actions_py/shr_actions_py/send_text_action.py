import os
from shr_msgs.action import TextRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sound_play.libsoundplay import SoundClient
import rclpy
import requests

from twilio.rest import Client


class MakeCallActionServer(Node):
    def __init__(self):
        super().__init__('send_text_action')
        self.soundhandle = SoundClient(self, blocking=True)
        self.read_script_action_server = ActionServer(self, TextRequest, 'send_text',
                                                      self.make_call_callback)

    def make_call_callback(self, goal_handle):
        self.get_logger().info('Sending text...')
        result = TextRequest.Result()

        try:
            resp = requests.post('http://textbelt.com/text', {
                'phone': goal_handle.request.phone_number,
                'message': goal_handle.request.msg,
                'key': 'textbelt'
            })
        except Exception as e:
            self.get_logger().info('Making call failed')
            result.status = "failed: " + str(e)
            goal_handle.abort()
            return result

        self.get_logger().info('Making text succeeded')
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    make_call_action_server = MakeCallActionServer()

    while True:
        rclpy.spin_once(make_call_action_server, timeout_sec=1.0)


if __name__ == '__main__':
    main()
