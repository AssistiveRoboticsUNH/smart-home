import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
import numpy as np

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        # from_frame_rel = 'map'
        # to_frame_rel = 'human_tf'
        to_frame_rel = 'map'
        from_frame_rel = 'human_tf'
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            print('no exception')

            x=t.transform.translation.x
            y=t.transform.translation.y
            d=np.sqrt(x**2 +  y**2)
            print(f'x: {x:.2f} y:{y:.2f} d:{d:.2f}')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
 


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('keyboard interrupt')
        pass

    print('shutting down')
    rclpy.shutdown()

if __name__ == '__main__':
  main()