import os
import cv2
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from shr_msg.action import RecognizeTrainRequest, RecognizeRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from sound_play.libsoundplay import SoundClient
from sound_play_msg.action import SoundRequest
import face_recognition
import rclpy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# recognize_train_timeout = 30
# recognize_timeout = 2
save_path = os.path.join(os.path.expanduser('~'), '.smart-home')


# camera_topic = '/camera/rgb/image_raw'
# voice = 'voice_cmu_us_fem_cg'


# voice = 'voice_kal_diphone'
# voice = 'voice_cmu_us_fem_cg'
# voice = 'voice_cmu_us_aew_cg'

class RecognizeFaceActionServer(Node):
    def __init__(self):
        super().__init__('recognize_face_action')
        # self.declare_parameter('camera_topic', '/camera/rgb/image_raw')
        # self.declare_parameter('camera_topic', 'unity_camera/color/image_raw')
        self.declare_parameter('camera_topic', 'smart_home/camera/color/image_raw')
        self.declare_parameter('voice', 'voice_cmu_us_fem_cg')
        self.declare_parameter('recognize_train_timeout', 30)
        self.declare_parameter('recognize_timeout', 10)

        self.voice = self.get_parameter('voice').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.recognize_train_timeout = self.get_parameter('recognize_train_timeout').value
        self.recognize_timeout = self.get_parameter('recognize_timeout').value

        self.training_action = False
        self.recognize_action = False
        self.latest_image = np.array([0])
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.br = CvBridge()

        self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 1)
        self.train_action_server = ActionServer(self, RecognizeTrainRequest, 'train_recognize_face',
                                                self.train_callback)
        self.recognize_action_server = ActionServer(self, RecognizeRequest, 'recognize_face', self.recognize_callback)

        self.soundhandle = SoundClient(self, blocking=True)

    def image_callback(self, data):
        if self.training_action or self.recognize_action:
            current_frame = self.br.imgmsg_to_cv2(data)
            self.latest_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
            self.latest_image = cv2.resize(self.latest_image, (0, 0), fx=2, fy=2)
        else:
            self.latest_image = np.array([0])

    def recognize_callback(self, goal_handle):
        self.get_logger().info('Recognize face...')
        feedback_msg = RecognizeRequest.Feedback()
        database = self.get_database()
        start_time = time.time()
        result = RecognizeRequest.Result()

        self.recognize_action = True
        # standing_person_face_encoding = None
        while time.time() - start_time < self.recognize_timeout:
            if self.latest_image.shape[0] > 1:
                standing_person_face_encoding = self.detect_face(self.latest_image)
                if standing_person_face_encoding is not None:
                    names = set()
                    for encoding in standing_person_face_encoding:
                        # min_val = np.finfo(float).max
                        # min_name = ""
                        for name in database:
                            known_encodings = database[name]
                            for known_encoding in known_encodings:
                                match = face_recognition.compare_faces(known_encoding, encoding, tolerance=0.4)
                                if match[0]:
                                    names.add(name)

                    result.names = list(names)
                    if len(result.names) > 0:
                        goal_handle.succeed()
                        return result

            feedback_msg.running = True
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self)

        self.recognize_action = False

        # if standing_person_face_encoding is not None:
        #     goal_handle.succeed()
        #     names = set()
        #     for encoding in standing_person_face_encoding:
        #         min_val = np.finfo(float).max
        #         min_name = ""
        #         for name in database:
        #             known_encodings = database[name]
        #             # for known_encoding in known_encodings:
        #             match = face_recognition.compare_faces(known_encodings, encoding, tolerance=0.6)
        #             if match:
        #                 names.add(min_name)
        # tmp = np.sum(np.abs(database[name] - encoding), axis=1)
        # if np.min(tmp) < min_val:
        #     min_val = np.min(tmp)
        #     min_name = name
        # result.names.append(min_name)
        # names.add(min_name)

        goal_handle.abort()
        result.names = [""]

        return result

    def train_callback(self, goal_handle):
        self.get_logger().info('Train recognize face...')
        start_time = time.time()
        feedback_msg = RecognizeTrainRequest.Feedback()

        self.training_action = True
        center_prompt_given = False
        only_person_prompt_given = False
        consecutive_identified = 0
        while time.time() - start_time < self.recognize_train_timeout:
            if time.time() - start_time > 10 and not center_prompt_given:
                self.soundhandle.say('Make sure you are centered in the camera frame', self.voice, 1.0)
                center_prompt_given = True

            if self.latest_image.shape[0] > 1:
                standing_person_face_encoding = self.detect_face(self.latest_image, gui=True)
                if standing_person_face_encoding is None:
                    consecutive_identified = 0
                elif len(standing_person_face_encoding) > 1:
                    consecutive_identified = 0
                    if not only_person_prompt_given:
                        self.soundhandle.say('Make sure you are the only person in the frame', self.voice, 1.0)
                        only_person_prompt_given = True
                else:
                    consecutive_identified += 1
                if consecutive_identified > 2:
                    self.add_to_database(goal_handle.request.name, standing_person_face_encoding)
                    cv2.waitKey(1)
                    break

            feedback_msg.running = True
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self)

        self.training_action = False
        cv2.destroyAllWindows()

        result = RecognizeTrainRequest.Result()
        if standing_person_face_encoding is not None:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False

        return result

    def detect_face(self, image, gui=False):
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(image_gray, 1.1, 4, minSize=(30, 30), )

        if gui:
            image_mod = image.copy()
            for (x, y, w, h) in faces:
                cv2.rectangle(image_mod, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.imshow('img', image_mod)
            cv2.waitKey(1)

        standing_person_face_encoding = None
        # box = face_recognition.api.face_locations(image, number_of_times_to_upsample=2) # (top, right, bottom, left)
        boxes = []
        for face in faces:
            boxes.append([face[1], face[0] + face[2], face[1] + face[3], face[0]])

        if len(boxes) > 0:
            tmp = face_recognition.face_encodings(image, known_face_locations=boxes)
            if len(tmp) > 0:
                standing_person_face_encoding = tmp

        # return [x.reshape((1, 128)) for x in
        #         standing_person_face_encoding]  # somehow this is messedup   standing_person_face_encoding
        return standing_person_face_encoding

    def add_to_database(self, name, face_encoding):
        path = os.path.join(save_path, name)
        if not os.path.exists(path):
            os.makedirs(path)
        path_images = os.path.join(path, 'images')
        if not os.path.exists(path_images):
            os.makedirs(path_images)
        path_encodings = os.path.join(path, 'encodings')
        if not os.path.exists(path_encodings):
            os.makedirs(path_encodings)
        num_files = len(os.listdir(path_encodings))
        with open(os.path.join(path_encodings, 'encoding_' + str(num_files) + '.npy'), 'wb') as f:
            np.save(f, face_encoding)

        cv2.imwrite(os.path.join(path_images, 'image_' + str(num_files) + '.jpg'), self.latest_image)

    def get_database(self):
        person_list = os.listdir(save_path)
        data_base = dict()
        for person in person_list:
            encoding_files = os.listdir(os.path.join(save_path, person, 'encodings'))
            for encoding_file in encoding_files:
                with open(os.path.join(save_path, person, 'encodings', encoding_file), 'rb') as f:
                    face_encoding = np.load(f)
                    if person in data_base:
                        # data_base[person] = np.concatenate((data_base[person], face_encoding), axis=0)
                        data_base[person].append(face_encoding)
                    else:
                        data_base[person] = [face_encoding]
        return data_base


def main(args=None):
    rclpy.init(args=args)

    recognize_face_action_server = RecognizeFaceActionServer()

    while True:
        rclpy.spin_once(recognize_face_action_server)


if __name__ == '__main__':
    main()
