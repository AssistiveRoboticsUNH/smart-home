import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time

try:
    # for motor state 1 command
    from p2os_msgs.msg import MotorState
except :
    print('cant import p2os')

class PioneerPub(Node):

    def __init__(self):
        super().__init__('pioneer_pub_nih')
        print('pub motor')
        self.motor_state_pub= self.create_publisher(MotorState, '/cmd_motor_state', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """
        send 1 to enable motor
        """
        self.i += 1
        ms=MotorState()
        ms.state=1
        self.motor_state_pub.publish(ms)



def main():
    print('Hi from nih.')
    rclpy.init(args=None)
    ros2_node = PioneerPub()

    rclpy.spin(ros2_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
