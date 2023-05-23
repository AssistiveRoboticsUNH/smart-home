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


class DetectTakingPill(Node):
    def __init__(self):
        super().__init__('detect_taking_pill_sim_node')

        self.pub_ = self.create_publisher(Bool, '/observe/pill_detection', 10)

        self.subscriber_eat = self.create_subscription(Bool, '/smartthings_sensors_motion_pills',
                                                       self.pill_callback, 10)
        self.pill_motion_sensor = False

        self.declare_parameter('camera',
                               '/unity_camera_kitchen/color/image_raw')  # '/smart_home/camera/color/image_raw')

        param_camera_topic = self.get_parameter('camera').value

        self.subscription = self.create_subscription(
            Image,
            param_camera_topic,
            self.camera_sim_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()



    def pill_callback(self, msg):
        print('callllbackk', msg.data)
        self.pill_motion_sensor = msg.data

    def camera_sim_callback(self, data):
        if self.pill_motion_sensor:
            print(2)
            frame = self.br.imgmsg_to_cv2(data)

            mpPose = mp.solutions.pose
            pose = mpPose.Pose()
            mpDraw = mp.solutions.drawing_utils

            # frame = self.cap

            # pill taking counter variables
            counter_right = 0
            counter_left = 0

            stage_left = None
            stage_right = None

            # Setup mediapipe instance
            with mpPose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

                # print(frame)
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
                        stage_left = "near"
                    if Left_wrist_mouth > threshold and stage_left == 'near':
                        stage_left = "far"
                        counter_left += 1

                    if Right_wrist_mouth < threshold:
                        stage_right = "near"
                    if Right_wrist_mouth > threshold and stage_right == 'near':
                        stage_right = "far"
                        counter_right += 1
                        print(counter_right)

                    # Render detections

                    mpDraw.draw_landmarks(image, results.pose_landmarks, mpPose.POSE_CONNECTIONS,
                                          mpDraw.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                          mpDraw.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                          )

                    cv2.line(image, tuple(map(int, Left_wrist)), tuple(map(int, mid_mouth)), color=(0, 0, 255),
                             thickness=2)
                    cv2.line(image, tuple(map(int, Left_wrist)), tuple(map(int, mid_mouth)), color=(0, 225, 255),
                             thickness=2)

                    print('out if')
                    # Render pill taking counter
                    # Setup status box
                cv2.rectangle(image, (0, 0), (225, 73), (245, 117, 16), -1)

                # Rep data
                cv2.putText(image, 'REPS', (15, 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(image, str(counter_left),
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(counter_right),
                            (100, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

                # Stage data
                # cv2.putText(image, 'STAGE', (65, 12),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                # cv2.putText(image, stage,
                #             (60, 60),
                #             cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow('Mediapipe Feed', image)
                print('media')
                cv2.waitKey(1)
                threshold = 1
                msg = Bool()
                if counter_right > threshold or counter_right > threshold:
                    msg.data = True

                else:
                    msg.data = False
                self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=None)
    detection_pill_taking = DetectTakingPill()
    print('mains')
    rclpy.spin(detection_pill_taking)


if __name__ == '__main__':
    main()
