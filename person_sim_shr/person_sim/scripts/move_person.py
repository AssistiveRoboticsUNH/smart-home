#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Twist


class MovePerson(object):
    def __init__(self, person_model_name):
        person_moving_topic = "/" + person_model_name + "/cmd_vel"
        self._move_base_publisher = rospy.Publisher(person_moving_topic, Twist, queue_size=1)

    def move_to(self, twist_object):
        self._move_base_publisher.publish(twist_object)
    
    def stop(self):
        stop_twist = Twist()
        self.move_to(stop_twist)
    
    def oscilate_left_right(self, n=10):
        twist_object = Twist()
        twist_object.angular.z = 2.0
        
        #Move left:
        self.move_to(twist_object)
        time.sleep(0.5)
        twist_object.angular.z *= -1
        self.stop()
        
        for i in range(n):
            self.move_to(twist_object)
            time.sleep(1.0)
            twist_object.angular.z *= -1
            rospy.loginfo("i ="+str(i)+", MAX="+str(n))
        
        self.stop()



def OscilatePerson():
    # Create a node
    rospy.init_node("move_person_oscilate_node")

    move_person = MovePerson()
    s = "Y"
    while not rospy.is_shutdown() and s == "Y":
        move_person.oscilate_left_right()
        s = raw_input("Continue..[Y,N]")
    
if __name__ == "__main__":
    OscilatePerson()

