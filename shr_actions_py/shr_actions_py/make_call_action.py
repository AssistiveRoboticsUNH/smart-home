import os
import time

from shr_msgs.action import CallRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sound_play.libsoundplay import SoundClient
import rclpy
from ament_index_python.packages import get_package_share_directory

from twilio.rest import Client


class MakeCallActionServer(Node):
    def __init__(self):
        super().__init__('make_call_action')
        self.soundhandle = SoundClient(self, blocking=True)
        self.make_call_action_server = ActionServer(self, CallRequest, 'make_call',
                                                      self.make_call_callback)

    def make_call_callback(self, goal_handle):
        self.get_logger().info('Making call...')
        result = CallRequest.Result()

        file_name = goal_handle.request.script_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', 'phoneApp', file_name)
        if not os.path.isfile(file_path):
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            self.get_logger().info('Reading script was aborted')
            return result

        account_sid = os.environ['TWILIO_ACCOUNT_SID']
        token = os.environ['TWILIO_TOKEN']

        client = Client(account_sid, token)
        try:
            with open(file_path) as f:
                call = client.calls.create(
                    twiml=f.read(),
                    to=goal_handle.request.phone_number,
                    from_='+18332447105')
        except Exception as e:
            self.get_logger().info('Making call failed')
            result.status = "failed: " + str(e)
            goal_handle.abort()
            return result

        self.get_logger().info('Making call succeeded')
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    make_call_action_server = MakeCallActionServer()

    while True:
        rclpy.spin_once(make_call_action_server)


if __name__ == '__main__':
    main()
