import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, Bool
from cv_bridge import CvBridge
import os
import time

# from rclpy.exceptions import ParameterNotDeclaredException


class DetectEating(Node):
    def __init__(self):
        super().__init__('detect_eating_sim_node')

        self.pub_ = self.create_publisher(Bool, '/observe/eat_detection', 10)

        self.subscriber_eat = self.create_subscription(Bool, '/smartthings_sensors_motion_eat',
                                                       self.eat_callback, 10)

        self.eat_motion_sensor = False

        # self.declare_parameter('camera',
        #                        '/camera_dining_room/color/image_raw')  # '/smart_home/camera/color/image_raw')
        #
        # param_camera_topic = self.get_parameter('camera').value

        self.subscription = self.create_subscription(
            Image,
            '/camera_dining_room/color/image_raw',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.stage_left = None
        self.stage_right = None

        # Eating counter variables
        self.counter_right = 0
        self.counter_left = 0

    def eat_callback(self, msg):
        print('callllbackk', msg.data)
        self.eat_motion_sensor = msg.data

    def camera_callback(self, data):
        if self.eat_motion_sensor:
            frame = self.br.imgmsg_to_cv2(data)

            mpPose = mp.solutions.pose
            pose = mpPose.Pose()
            mpDraw = mp.solutions.drawing_utils

            # Setup mediapipe instance
            with mpPose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                # Recolor image to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Make detection
                results = pose.process(image)

                # Recolor back to BGR
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                print('before if')
                # Extract landmarks
                if results.pose_landmarks is not None:
                    print('in if')
                    landmarks = results.pose_landmarks.landmark

                    # Get coordinates
                    Left_wrist = np.array(
                        [landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].x,
                         landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].y])
                    Right_wrist = np.array([landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].x,
                                            landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].y])

                    Left_mouth = np.array(
                        [landmarks[mpPose.PoseLandmark.MOUTH_LEFT.value].x,
                         landmarks[mpPose.PoseLandmark.MOUTH_LEFT.value].y])
                    Right_mouth = np.array([landmarks[mpPose.PoseLandmark.MOUTH_RIGHT.value].x,
                                            landmarks[mpPose.PoseLandmark.MOUTH_RIGHT.value].y])
                    mid_mouth = abs(Left_mouth + Right_mouth) / 2

                    # Calculate distance
                    Left_wrist_mouth = np.linalg.norm(Left_wrist - mid_mouth)
                    Right_wrist_mouth = np.linalg.norm(Right_wrist - mid_mouth)

                    threshold = 0.05
                    # Curl counter logic
                    if Left_wrist_mouth < threshold:
                        self.stage_left = "near"
                    if Left_wrist_mouth > threshold and self.stage_left == 'near':
                        self.stage_left = "far"
                        self.counter_left += 1

                    if Right_wrist_mouth < threshold:
                        self.stage_right = "near"
                    if Right_wrist_mouth > threshold and self.stage_right == 'near':
                        self.stage_right = "far"
                        self.counter_right += 1

                    # Render detections

                    # mpDraw.draw_landmarks(image, results.pose_landmarks, mpPose.POSE_CONNECTIONS,
                    #                       mpDraw.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                    #                       mpDraw.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                    #                       )

                    specific_landmarks = [mpPose.PoseLandmark.MOUTH_RIGHT,
                                          mpPose.PoseLandmark.MOUTH_LEFT,
                                          mpPose.PoseLandmark.RIGHT_WRIST,
                                          mpPose.PoseLandmark.LEFT_WRIST,
                                          ]

                    # Render the specific landmark points
                    mp_drawing = mp.solutions.drawing_utils
                    drawing_spec = mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2)

                    for landmark in specific_landmarks:
                        landmark_point = results.pose_landmarks.landmark[landmark]
                        x_px, y_px = int(landmark_point.x * image.shape[1]), int(landmark_point.y * image.shape[0])
                        cv2.circle(image, (x_px, y_px), drawing_spec.circle_radius, drawing_spec.color, drawing_spec.thickness)

                        cv2.line(image, tuple(map(int, Left_wrist)), tuple(map(int, mid_mouth)), color=(0, 0, 255),
                                                 thickness=2)
                    cv2.line(image, tuple(map(int, Left_wrist)), tuple(map(int, mid_mouth)), color=(0, 225, 255),
                             thickness=2)

                    print('out if')
                    # Render eating counter
                    # Setup status box
                cv2.rectangle(image, (0, 0), (225, 73), (245, 117, 16), -1)

                # Rep data
                cv2.putText(image, 'REPS', (15, 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(image, str(self.counter_left),
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(self.counter_right),
                            (100, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow('Mediapipe Feed', image)
                print('media')
                cv2.waitKey(1)
                threshold_rep = 5
                msg = Bool()
                if self.counter_right > threshold_rep or self.counter_right > threshold_rep:
                    msg.data = True
                else:
                    msg.data = False
                self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=None)
    detection_eating = DetectEating()
    print('mains')
    rclpy.spin(detection_eating)


if __name__ == '__main__':
    main()
