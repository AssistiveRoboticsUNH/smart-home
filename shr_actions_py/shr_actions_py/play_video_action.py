import os

from ament_index_python.packages import get_package_share_directory
from shr_msgs.action import PlayVideoRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import rclpy


class PlayVideoActionServer(Node):
    def __init__(self):
        super().__init__('play_video_action')
        self.play_video_action_server = ActionServer(self, PlayVideoRequest, 'play_video',
                                                      self.play_audio_callback)

    def play_audio_callback(self, goal_handle):
        self.get_logger().info("weblog="+'Playing video...')
        result = PlayVideoRequest.Result()

        file_name = goal_handle.request.file_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)
        self.get_logger().info("weblog="+"video_file"+file_path)
        if not os.path.isfile(file_path):
            result.status = "file '" + file_path + "' does not exist"
            self.get_logger().info("weblog="+'Playing video was aborted')
            goal_handle.abort()
            return result

        command = 'vlc ' + file_path + ' --fullscreen vlc://quit'
        os.system(command)
        self.get_logger().info("weblog="+'Playing video was successful')
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    play_video_action_server = PlayVideoActionServer()

    while True:
        rclpy.spin_once(play_video_action_server, timeout_sec=5.0)


if __name__ == '__main__':
    main()
