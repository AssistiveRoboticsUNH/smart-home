#!/usr/bin/env python  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_fixed_carrot_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(3.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "fixed_carrot",
                         "turtle1")
        rate.sleep()