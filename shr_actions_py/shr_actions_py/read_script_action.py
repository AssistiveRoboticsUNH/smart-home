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
        self.get_logger().info('Reading script...')
        result = ReadScriptRequest.Result()

        file_name = goal_handle.request.script_name
        file_path = os.path.join(get_package_share_directory('shr_resources'), 'resources', file_name)

        if not os.path.isfile(file_path):
            result.status = "file '" + file_path + "' does not exist"
            goal_handle.abort()
            self.get_logger().info('Reading script was aborted')
            return result

        wavfilename = self.create_wav_from_text(file_path)
        os.system('vlc ' + wavfilename + ' vlc://quit')
        self.get_logger().info('Reading script was successful')
        result.status = "success"
        goal_handle.succeed()

        return result

    def create_wav_from_text(self, file_path):
        (wavfile, wavfilename) = tempfile.mkstemp(
            prefix='sound_play', suffix='.wav')

        # Create a gTTS object with the text and language
        # Path to the file containing the text you want to convert
        # Open the text file and read its contents
        with open(file_path, 'r') as f:
            mytext = f.read()
        tts_obj = gTTS(text=mytext, lang='en', slow=False)

        # Save the generated speech as a WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            wavfilename = f.name
            tts_obj.save(wavfilename)
        # os.system("pico2wave -l en-US -w" + wavfilename + f' "{data.arg}"')
        wavfilename_new = wavfilename.replace('.wav', '')
        wavfilename_new += '_new.wav'
        os.system("ffmpeg -i " + wavfilename + " -af areverse,apad=pad_dur=500ms,areverse " + wavfilename_new)
        wavfilename = wavfilename_new
        return wavfilename

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
        rclpy.spin_once(read_script_action_server)


if __name__ == '__main__':
    main()
