import os
import zmq
from ament_index_python.packages import get_package_share_directory
from shr_msgs.action import PlayAudioRequest
from rclpy.action import ActionServer
from rclpy.node import Node
import rclpy


class PlayAudioActionServer(Node):
    def __init__(self, zmq_socket):
        super().__init__('play_audio_action')
        self.play_audio_action_server = ActionServer(self, PlayAudioRequest, 'play_audio', self.play_audio_callback)
        self.zmq_socket = zmq_socket  # Use the shared ZeroMQ socket


    def play_audio_callback(self, goal_handle):
        self.get_logger().info("weblog="+'Playing audio...')
        result = PlayAudioRequest.Result()

        file_name = goal_handle.request.file_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)
        self.get_logger().info("weblog="+"audio_file_path:"+file_path)
        if not os.path.isfile(file_path):
            self.get_logger().info("weblog="+'Playing audio was aborted')
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            return result

        # Send "0" to ZeroMQ before starting audio playback
        self.zmq_socket.send_string("0")
        self.get_logger().info("weblog="+'Sent ZeroMQ message: 0')

        # Play the audio
        command = 'mpg321 ' + file_path
        os.system(command)

        # After audio playback, send "1" to ZeroMQ
        self.zmq_socket.send_string("1")
        self.get_logger().info("weblog="+'Sent ZeroMQ message: 1')

        self.get_logger().info("weblog="+'Playing audio was successful')
        result.status = "success"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    play_audio_action_server = PlayAudioActionServer()

    while True:
        rclpy.spin_once(play_audio_action_server, timeout_sec=5.0)


if __name__ == '__main__':
    main()