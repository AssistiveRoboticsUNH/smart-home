import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import transforms3d as tf
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations as tr
import numpy as np
import json
from geometry_msgs.msg import Quaternion

import os
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import TransformStamped

import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import yaml

import os
class PID:
    def __init__(self, Kp=0, Ki=0, Kd=0):
        '''
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def proportional_control(self, error):
        return self.Kp * error

    def integral_control(self, error, dt):
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        return self.Kd * (error - previous_error) / dt
        

class Docking(Node):
    def __init__(self):
        super().__init__('docking')
        self.tf_buffer = Buffer()
        self.tf_listenser = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 1)
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        self.proxi = None
        self.proxi_subscriber = self.create_subscription(
            Float32,
            'proximity_sensor',
            self.proxi_callback,
            10
        )
        self.bumped = False
        self.spin = True
        self.is_detect = False
        self.move = False
        self.vel = Twist()
        self.saved_time = self.get_clock().now()
        self.prev_error = 0
        self.prev_error_x = 0
        kp = 2
        ki = 0.4
        kd = 0
        kp_x = 0.4
        ki_x = 0.01
        self.controller = PID(kp, kd, ki)
        self.controller_x = PID(kp_x, kd, ki_x)

        self.translation = {}

    def get_transformation_from_aptag_to_port(self):
        frame = "port"
        source_frame = "charger"
        try:
            transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                            timeout=rclpy.duration.Duration(seconds=2.0))

            translation_values = {
                 "translation_x":transformation.transform.translation.x,
                 "translation_y":transformation.transform.translation.y,
                 "translation_z":transformation.transform.translation.z,
            }
            self.translation.update(translation_values)
            print("trans:", (translation_values))

        except Exception as e:
            print("found problem:", e)

    def velocity_control(self, error, dt, prev_error):
        max_vel = 1
        mv_p = self.controller.proportional_control(error)
        mv_i = self.controller.integral_control(error, dt)
        mv_d = self.controller.derivative_control(error, prev_error, dt)

        desired_vel = np.clip(mv_p + mv_i + mv_d, -max_vel, max_vel)
        return desired_vel
    
    def velocity_control_X(self, error, dt, prev_error):
        max_vel = 0.1
        mv_p = self.controller.proportional_control(error)
        mv_i = self.controller.integral_control(error, dt)
        mv_d = self.controller.derivative_control(error, prev_error, dt)

        desired_vel = np.clip(mv_p + mv_i + mv_d, 0.009, max_vel)
        return desired_vel

    def proxi_callback(self, msg):
        #self.get_logger().info(f'Received float: {msg.data}')
        self.proxi = msg.data
        print(self.proxi)

    def spin(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.2

    def apriltag_callback(self, msg):
        
        if msg.detections:           
            for at in msg.detections:
                if(at.id ==203):
                    self.is_detect = True 
                    print("True!")            
        else:
            # pass
            self.is_detect = False
            print('No aptags from callback')


    def move_towards_tag(self):
        if (self.is_detect is True and self.bumped is False):
            current_error = float(self.translation.get("translation_y", 0.0))
            transition_x = float(self.translation.get("translation_x", 0.0))
            error_x = (transition_x-0.02)
            modified_error_x = error_x*0.2
            #print("current_error", current_error)
            #print("x", transition_x)
            if (error_x>0.00 or (self.proxi is not None and self.proxi > 15.0)):
                
                current_time = self.get_clock().now()
                dt = (current_time - self.saved_time).nanoseconds / 1e9
                pid_output =self.velocity_control(current_error, dt, self.prev_error)
                pid_output_x = self.velocity_control_X(modified_error_x, dt, self.prev_error_x)
                self.vel.linear.x = pid_output_x
                print("pid_output_x", pid_output_x)
                print("pid_output ", pid_output)
                self.saved_time = current_time
                if(pid_output>0.3):
                    self.vel.angular.z = 0.2
                    #print("PID pos", self.vel.angular.z)
                elif(pid_output<-0.3):
                    self.vel.angular.z = -0.2
                    #print("PID neg", self.vel.angular.z)
                else:
                    self.vel.angular.z = pid_output
                    #print("PID", self.vel.angular.z)
                

                self.pub.publish(self.vel)
                self.prev_error = current_error
                self.prev_error_x = modified_error_x
                self.bumped = False
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
                self.bumped = True
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.2
            self.pub.publish(self.vel)
            self.bumped = False

def main(args=None):
    rclpy.init(args=args)
    tag_to_bar = Docking()
    
    while( rclpy.ok()):
        tag_to_bar.get_transformation_from_aptag_to_port()
        tag_to_bar.move_towards_tag()
        print("I am calling")
        rclpy.spin_once(tag_to_bar)
        
    
    tag_to_bar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
