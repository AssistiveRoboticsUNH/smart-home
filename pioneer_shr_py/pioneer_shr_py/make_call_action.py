import os

from pioneer_shr_msg.action import CallRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sound_play.libsoundplay import SoundClient
import rclpy

from twilio.rest import Client


class MakeCallActionServer(Node):
    def __init__(self):
        super().__init__('make_call_action')
        self.soundhandle = SoundClient(self, blocking=True)
        self.read_script_action_server = ActionServer(self, CallRequest, 'make_call',
                                                      self.make_call_callback)

    def make_call_callback(self, goal_handle):
        self.get_logger().info('Reading script...')
        result = CallRequest.Result()

        account_sid = os.environ['TWILIO_ACCOUNT_SID']
        token = os.environ['TWIO_TOKEN']

        client = Client(account_sid, token)
        try:
            call = client.calls.create(
                url='https://mypages.unh.edu/sites/default/files/paulgesel/files/' + goal_handle.request.script_name,
                to=goal_handle.request.phone_number,
                from_='+14094074384')
        except Exception as e:
            result.status = "failed: " + str(e)
            goal_handle.abort()
            return result

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
