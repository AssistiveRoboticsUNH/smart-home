import os
import glob

from ament_index_python.packages import get_package_share_directory
from shr_msgs.action import ReadScriptRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import rclpy
import tempfile
import threading
import functools
from gtts import gTTS


class ReadScriptActionServer(Node):
    def __init__(self):
        super().__init__('read_script_action')
        self.declare_parameter('voice', 'voice_cmu_us_fem_cg')
        self.voice = self.get_parameter('voice').value
        self.read_script_action_server = ActionServer(self, ReadScriptRequest, 'read_script',
                                                      self.read_script_callback)

    def read_script_callback(self, goal_handle):
        self.get_logger().info("weblog="+'Reading script...')
        result = ReadScriptRequest.Result()

        file_name = goal_handle.request.script_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)
        self.get_logger().info("weblog="+"script_file:"+file_path)

        if not os.path.isfile(file_path):
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            self.get_logger().info("weblog="+'Reading script was aborted')
            return result

        wavfilename = self.create_mp4_from_text(file_path)
        os.system('mpg321 -o alsa ' + wavfilename)
        self.get_logger().info("weblog="+'Reading script was successful')
        result.status = "success"
        goal_handle.succeed()

        return result
        
    @functools.cache
    def create_mp4_from_text(self, file_path):
        (mp4file, mp4filename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.mp4')

        # Create a gTTS object with the text and language
        with open(file_path, 'r') as f:
            mytext = f.read()
        tts_obj = gTTS(text=mytext, lang='en', slow=False)

        # Save the generated speech as an MP4 file
        with tempfile.NamedTemporaryFile(suffix='.mp4', delete=False) as f:
            mp4filename = f.name
            tts_obj.save(mp4filename)

        return mp4filename


    def thread_function(self):
        files = glob.glob(os.path.join(get_package_share_directory('shr_resources'), 'resources', '*.txt'))
        # for file in files:
        #     wavfilename = self.create_wav_from_text(file)


def main(args=None):
    rclpy.init(args=args)
    read_script_action_server = ReadScriptActionServer()

    x = threading.Thread(target=read_script_action_server.thread_function)
    x.start()

    while True:
        rclpy.spin_once(read_script_action_server, timeout_sec=5.0)


if __name__ == '__main__':
    main()
