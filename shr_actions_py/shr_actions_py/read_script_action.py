import os
from ament_index_python.packages import get_package_share_directory
from shr_msg.action import ReadScriptRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sound_play.libsoundplay import SoundClient
import rclpy


class ReadScriptActionServer(Node):
    def __init__(self):
        super().__init__('read_script_action')
        self.declare_parameter('voice', 'voice_cmu_us_fem_cg')
        self.voice = self.get_parameter('voice').value
        self.soundhandle = SoundClient(self, blocking=True)
        self.read_script_action_server = ActionServer(self, ReadScriptRequest, 'read_script',
                                                      self.read_script_callback)

    def read_script_callback(self, goal_handle):
        self.get_logger().info('Reading script...')
        result = ReadScriptRequest.Result()

        file_name = goal_handle.request.script_name
        file_path = os.path.join(get_package_share_directory('shr_msg'), 'resources', file_name)

        if not os.path.isfile(file_path):
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            return result

        f = open(file_path, "r")
        text = f.read()
        self.soundhandle.say(text, self.voice, 1.0)
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    read_script_action_server = ReadScriptActionServer()

    while True:
        rclpy.spin_once(read_script_action_server)


if __name__ == '__main__':
    main()
