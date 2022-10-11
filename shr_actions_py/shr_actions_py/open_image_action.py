import os
from subprocess import Popen

from ament_index_python.packages import get_package_share_directory
from shr_msg.action import OpenImageRequest
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
            self.get_logger().info('Opening image...')
            file_path = os.path.join(get_package_share_directory('shr_msg'), 'resources', file_name)

            if not os.path.isfile(file_path):
                result.status = "file '" + file_path + "' does not exist"
                goal_handle.abort()
                return result

            self.proc_dict[file_name] = Popen(['eog', '--fullscreen', file_path])
            result.status = "success"
            goal_handle.succeed()

        else:
            self.get_logger().info('Closing image...')

            if file_name in self.proc_dict:
                self.proc_dict[file_name].terminate()
            result.status = "success"
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    open_image_action_server = OpenImageActionServer()

    while True:
        rclpy.spin_once(open_image_action_server)


if __name__ == '__main__':
    main()
