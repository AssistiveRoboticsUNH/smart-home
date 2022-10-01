import os
import cv2
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from pioneer_shr_msg.action import EmptyRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from sound_play.libsoundplay import SoundClient
from sound_play_msg.action import SoundRequest
import face_recognition
import rclpy


class RecognizeFaceActionServer(Node):

    def __init__(self):
        super().__init__('recognize_face_action')
        self._action_server = ActionServer(
            self,
            EmptyRequest,
            'recognize_face',
            self.recognize_callback)

        # Ordered this way to minimize wait time.
        soundhandle = SoundClient(self, blocking=True)

        # voice = 'voice_kal_diphone'
        voice = 'voice_cmu_us_fem_cg'
        # voice = 'voice_cmu_us_aew_cg'
        volume = 1.0

        s = 'Make sure you are centered in the camera frame'
        # s = 'Hi.    How are you today?'

        self.get_logger().info('Saying: %s' % s)
        self.get_logger().info('Voice: %s' % voice)
        self.get_logger().info('Volume: %s' % volume)

        soundhandle.say(s, voice, volume)

        get_package_share_directory('pioneer_shr_msg')

    def recognize_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = EmptyRequest.Result()
        return result

    def train_callback(self, goal_handle):
        pass
        # Load a sample picture and learn how to recognize it.
        # standing_person_image = face_recognition.load_image_file(image_path)
        # standing_person_face_encoding = face_recognition.face_encodings(standing_person_image)[0]

        # return standing_person_face_encoding


def main(args=None):
    rclpy.init(args=args)

    recognize_face_action_server = RecognizeFaceActionServer()

    rclpy.spin(recognize_face_action_server)


if __name__ == '__main__':
    main()
