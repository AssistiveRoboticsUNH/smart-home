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
        super().__init__('detect_eating_real_node')
        print('#########################################')

        self.pub_ = self.create_publisher(Bool, '/eating_sensor', 10)

        self.subscriber_eat = self.create_subscription(Bool, '/smartthings_sensors_eat',
                                                       self.eat_callback, 10)
        self.eat_motion_sensor = False

        ip_address = "192.168.1.34"
        username = 'Dining'
        password = os.environ['TAPO_CAMERA_PASS']
        self.link = "rtsp://" + username + ":" + password + "@" + ip_address + "/stream2"

        timer_period = 2
        self.timer = self.create_timer(timer_period, self.camera_real)

    def eat_callback(self, msg):
        print('callllbackk', msg.data)
        self.eat_motion_sensor = msg.data

    def camera_real(self):
        if self.eat_motion_sensor:
            print('here', self.eat_motion_sensor)
            cap = cv2.VideoCapture(self.link, cv2.CAP_FFMPEG)
            print(2)

            mpPose = mp.solutions.pose
            pose = mpPose.Pose()
            mpDraw = mp.solutions.drawing_utils

            # frame = self.cap

            # Eating counter variables
            counter_right = 0
            counter_left = 0

            stage_left = None
            stage_right = None

            #does pose detection for 5 mins
            time_thre = 5  # in minutes

            # Setup mediapipe instance
            with mpPose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                start_time = time.time()
                while cap.isOpened():

                    execution_time = time.time() - start_time
                    print('fffff', execution_time)
                    if execution_time > time_thre * 60:
                        cap.release()
                        cv2.destroyAllWindows()
                        break

                    ret, frame = cap.read()
                    # Recolor image to RGB
                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image.flags.writeable = False

                    # Make detection
                    results = pose.process(image)

                    # Recolor back to BGR
                    image.flags.writeable = True
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                    # Extract landmarks
                    if results.pose_landmarks is not None:
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

                        threshold = 5
                        msg = Bool()
                        if counter_right > threshold or counter_right > threshold:
                            msg.data = True

                            # to make sure that it was picked up by the world states change this to a service
                            # Get the current time
                            start_time = time.time()

                            # Publish the message continuously for 10 seconds
                            while time.time() - start_time < 10.0:
                                self.pub_.publish(msg)
                                time.sleep(0.1)
                            break

                        else:
                            msg.data = False

                        self.pub_.publish(msg)

                    # Render eating counter
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
                    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=None)
    detection_eating = DetectEating()
    print('mains')
    rclpy.spin(detection_eating)


if __name__ == '__main__':
    main()
