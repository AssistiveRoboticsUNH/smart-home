import os
from subprocess import Popen

from ament_index_python.packages import get_package_share_directory
from shr_msgs.action import OpenImageRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import rclpy


class OpenImageActionServer(Node):
    def __init__(self):
        super().__init__('open_image_action')
        self.read_script_action_server = ActionServer(self, OpenImageRequest, 'open_image',
                                                      self.open_image_callback)
        self.proc_dict = {}

    def open_image_callback(self, goal_handle):
        result = OpenImageRequest.Result()
        file_name = goal_handle.request.file_name

        if goal_handle.request.open:
            self.get_logger().info("weblog="+'Opening image...')
            file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)
            self.get_logger().info("weblog="+"image_file_path:"+file_path)
            if not os.path.isfile(file_path):
                self.get_logger().info("weblog="+'Opening image was abroad!')
                result.status = "file '" + file_path + "' does not exist"
                goal_handle.abort()
                return result

            self.proc_dict[file_name] = Popen(['eog', '--fullscreen', file_path])
            result.status = "success"
            self.get_logger().info("weblog="+'Opening image was successful')
            goal_handle.succeed()

        else:
            self.get_logger().info("weblog="+'Closing image...')

            if file_name in self.proc_dict:
                self.proc_dict[file_name].terminate()
            result.status = "success"
            self.get_logger().info("weblog="+'Closing image was successful')
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    open_image_action_server = OpenImageActionServer()

    while True:
        rclpy.spin_once(open_image_action_server,timeout_sec=5.0)


if __name__ == '__main__':
    main()
