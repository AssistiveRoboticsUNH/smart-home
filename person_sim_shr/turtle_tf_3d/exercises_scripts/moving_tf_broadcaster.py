#!/usr/bin/env python  
import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('my_moving_carrot_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(5.0)
    turning_speed_rate = 0.1
    while not rospy.is_shutdown():
        t = (rospy.Time.now().to_sec() * math.pi)*turning_speed_rate
        # Map to only one turn maximum [0,2*pi)
        rad_var = t % (2*math.pi)
        br.sendTransform((1.0 * math.sin(rad_var), 1.0 * math.cos(rad_var), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "moving_carrot",
                         "turtle2")
        rate.sleep()