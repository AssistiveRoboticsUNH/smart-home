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


class DetectPersonaAtBedSide(Node):
    def __init__(self):
        super().__init__('detect_person_at_bed_side_node')

        self.pub_ = self.create_publisher(Bool, '/observe/bed_after_returning', 10)

        self.subscriber_eat = self.create_subscription(Bool, '/smartthings_sensors_motion_bed_side',
                                                       self.bed_side_callback, 10)
        self.bed_side_motion_sensor = False

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def bed_side_callback(self, msg):
        print('callllbackk', msg.data)
        self.bed_side_motion_sensor = msg.data


def main(args=None):
    rclpy.init(args=None)
    detection_person_bed_side = DetectPersonaAtBedSide()
    print('mains')
    rclpy.spin(detection_person_bed_side)


if __name__ == '__main__':
    main()
