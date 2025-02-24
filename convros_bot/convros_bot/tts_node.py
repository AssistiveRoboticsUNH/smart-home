'''
This text-to-speech node uses TTS class and speaks whatever published in /tts_text topic
e.g. ros2 topic pub /tts_text std_msgs/msg/String "data: 'Hello, this is a test message!'" --once
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import playsound
from google.oauth2 import service_account
from google.cloud import texttospeech
from convros_bot.tts_main import TTS
from ament_index_python.packages import get_package_share_directory

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('tts_model', "gTTS")
        self.declare_parameter('api_credential_location', "")
        self.tts_model = self.get_parameter('tts_model').get_parameter_value().string_value

        if self.tts_model == "googleAPI":
            api_credential_fileName = self.get_parameter('api_credential_file').get_parameter_value().string_value
            
            # Define text-to-speech object
            self.tts = TTS(self.tts_model, api_credential_fileName)

            self.get_logger().warn(f'Using Google Text-to-Speech API')
        elif self.tts_model == "gTTS":
            # Define text-to-speech object
            self.tts = TTS(self.tts_model)
            self.get_logger().warn(f'Using gTTS for speech to text service')

        # Subscribe to the speech_text topic
        self.subscription = self.create_subscription(String, 'tts_text', self.text_to_speech_callback, 10)

    def text_to_speech_callback(self, msg):
        text = msg.data
        if text:
            self.tts.speak(text)


def main(args=None):
    rclpy.init(args=args)
    
    tts_node = TTSNode()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        tts_node.get_logger().info('TTS Node stopped by user.')
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
