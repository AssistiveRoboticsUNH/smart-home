import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from yolostate.yolo_human_detect import HumanDetector
from ament_index_python.packages import get_package_share_directory
import threading

'''
ROS2 node, 
subscribe to RGB image, 
detect human and 
publish x,y,width,height as bounding box

ros2 run yolostate detecthuman --ros-args -p camera:=/smart_home/camera/color/image_raw -p view_camera:=true
'''


class DetectHuman(Node):
    def __init__(self):
        super().__init__('detecthuman')
 
        self.declare_parameter('view_camera', True)
        self.declare_parameter('camera', '/camera/color/image_raw')
        self.declare_parameter('pub_human', '/detecthuman')

        param_camera_topic = self.get_parameter('camera').value
        self.view_camera=self.get_parameter('view_camera').value
        self.pub_human_topic=self.get_parameter('pub_human').value 

        self.subscription = self.create_subscription(
            Image, 
            param_camera_topic,
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning
 

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        current_dir = get_package_share_directory('yolostate')
        print('package_path: ', current_dir)
        data_path = current_dir + '/' + 'yolodata'
        print('data_path', data_path, '\n')
        # current_dir=os.getcwd()
        self.yh = HumanDetector(data_path)
        self.human_pos = [0, 0, 0, 0]

        self.publisher_ = self.create_publisher(Int32MultiArray, self.pub_human_topic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_human_pub_callback)
        self.image = None
        self.plotting = True
        self.names = None
        self.confidences = None
        self.boxes = None
        self.current_frame = None
        self.org = None


    def timer_human_pub_callback(self):
        msg = Int32MultiArray()
        msg.data = [int(self.human_pos[0]), int(self.human_pos[1]), int(self.human_pos[2]), int(self.human_pos[3])]

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        # reset data
        self.human_pos = [0, 0, 0, 0]

    def on_human_data(self, box):
        x, y, xd, yd = box
        self.human_pos = box

    def camera_callback(self, data): 
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame = current_frame[:, :, :3]  # channel 4 to 3
        image_r = current_frame[:, :, 0]
        image_g = current_frame[:, :, 1]
        image_b = current_frame[:, :, 2]
        self.current_frame = np.dstack((image_b, image_g, image_r))

        self.org = self.current_frame.copy()

        self.names, self.confidences, self.boxes = self.yh.detect_human(self.current_frame)
        if len(self.names) > 0:
            print(f'Human detected. total={len(self.names)}')
            self.on_human_data(self.boxes[0])                   #set human detected data for timer publisher.
        
        
            


def plot(image_subscriber):
    while image_subscriber.plotting:

        if image_subscriber.view_camera and image_subscriber.current_frame is not None and image_subscriber.names is not None:


            for name, conf, box in zip(image_subscriber.names, image_subscriber.confidences, image_subscriber.boxes):
                org = image_subscriber.yh.draw_box(image_subscriber.org, name, conf, box)
            cv2.imshow("human detection", org)
            cv2.waitKey(100)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = DetectHuman()

    t1 = threading.Thread(target=plot, args=(image_subscriber, ))
    t1.start()
    rclpy.spin(image_subscriber)
    image_subscriber.plotting = False

    t1.join()

    image_subscriber.destroy_node()
    rclpy.shutdown()







if __name__ == '__main__':
    main()
