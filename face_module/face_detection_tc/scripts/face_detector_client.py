#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray

# Move base using navigation stack
class FaceDetectClient(object):

    def __init__(self):
        self.face_detect_subs = rospy.Subscriber(   "/face_detector/people_tracker_measurements_array",
                                                    PositionMeasurementArray,
                                                    self.face_detect_subs_callback)

    
        self.pos_mesurement_array = PositionMeasurementArray()
    
    def face_detect_subs_callback(self,msg):
        self.pos_mesurement_array = msg

    
def Face_DetectionClient_Start():
    # Create a node
    rospy.init_node("face_detection_client_start_node")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    face_detector_client = FaceDetectClient()

    rospy.spin()

if __name__ == "__main__":
    Face_DetectionClient_Start()

