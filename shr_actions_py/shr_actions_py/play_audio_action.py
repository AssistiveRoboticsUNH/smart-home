import os
from ament_index_python.packages import get_package_share_directory
from shr_msgs.action import PlayAudioRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import rclpy


class PlayAudioActionServer(Node):
    def __init__(self):
        super().__init__('play_audio_action')
        self.read_script_action_server = ActionServer(self, PlayAudioRequest, 'play_audio',
                                                      self.play_audio_callback)

    def play_audio_callback(self, goal_handle):
        self.get_logger().info('Playing audio...')
        result = PlayAudioRequest.Result()

        file_name = goal_handle.request.file_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)

        if not os.path.isfile(file_path):
            self.get_logger().info('Playing audio was aborted')
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            return result

        command = 'mpg321 ' + file_path
        os.system(command)
        self.get_logger().info('Playing audio was successful')
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    play_audio_action_server = PlayAudioActionServer()

    while True:
        rclpy.spin_once(play_audio_action_server)


if __name__ == '__main__':
    main()