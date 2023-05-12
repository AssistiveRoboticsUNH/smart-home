import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from shr_msgs.msg import LocationFromCamera
from std_msgs.msg import Float32MultiArray, String
from yolo_human_detect import HumanDetector
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

'''
ROS2 node, 
subscribe to RGB image, 
detect human and 
publish x,y,width,height as bounding box

ros2 run yolostate detecthuman --ros-args -p camera:=/smart_home/camera/color/image_raw -p view_camera:=true
'''


#### ASSUMES ONLY ONE PERSON IS IN THE HOUSE WILL GENERALIZE LATER

class DetectHumanLoc(Node):
    def __init__(self, yh, node_name, sub_topic_name):
        super().__init__('detect_human')

        self.node = rclpy.create_node(node_name)

        self.declare_parameter('view_camera', True)

        self.publisher_ = self.create_publisher(LocationFromCamera, node_name + 'tp', 10)

        self.view_camera = self.get_parameter('view_camera').value

        self.subscription = self.create_subscription(
            Image,
            sub_topic_name,
            self.camera_callback,
            10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


        self.human_pos = [0, 0, 0, 0]
        self.yh = yh

        timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_human_loc_callback)

        self.sub_topic_name = sub_topic_name
        # self.pub_topic_name = pub_topic_name
        self.node_name = node_name

    def timer_human_loc_callback(self):
        # reset data
        self.human_pos = [0, 0, 0, 0]

    def on_human_data(self, box):
        x, y, xd, yd = box
        self.human_pos = box

    def camera_callback(self, data):
        print('camera_callback', self.node_name)
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame = current_frame[:, :, :3]  # channel 4 to 3
        image_r = current_frame[:, :, 0]
        image_g = current_frame[:, :, 1]
        image_b = current_frame[:, :, 2]
        current_frame = np.dstack((image_b, image_g, image_r))

        org = current_frame.copy()

        names, confidences, boxes = self.yh.detect_human(current_frame)
        if len(names) > 0:
            # print('topic name', self.sub_topic_name)
            print(f'Human detected. total={len(names)}')
            self.on_human_data(boxes[0])  # set human detected data for timer publisher.

            message = LocationFromCamera()
            message.detected = True
            message.confidence = max(confidences)
            print('aaaaaaaaaaa', message)
            self.publisher_.publish(message)

        else:
            message = LocationFromCamera()
            message.detected = False
            message.confidence = 0.0
            self.publisher_.publish(message)


class GetLoc(Node):
    def __init__(self, node_name, pub_topic_name):
        super().__init__('get_loc')

        self.node = rclpy.create_node(node_name)

        self.publisher_ = self.create_publisher(String, pub_topic_name, 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_human_loc_callback)

        self.pub_topic_name = pub_topic_name
        self.node_name = node_name
        self.nodes = ['living_room_cam', 'dining_room_cam', 'bedroom_cam', 'kitchen_cam']

        self.subscription_0 = self.create_subscription(
            LocationFromCamera,
            self.nodes[0] + 'tp',
            self.living_room_callback,
            10)
        self.subscription_1 = self.create_subscription(
            LocationFromCamera,
            self.nodes[1] + 'tp',
            self.dining_room_callback,
            10)
        self.subscription_2 = self.create_subscription(
            LocationFromCamera,
            self.nodes[2] + 'tp',
            self.bedroom_callback,
            10)
        self.subscription_3 = self.create_subscription(
            LocationFromCamera,
            self.nodes[3] + 'tp',
            self.kitchen_callback,
            10)
        
        # self.loc = {location:confi}
        self.loc = {}

    def dining_room_callback(self, msg):
        self.loc['diningroom'] = msg.confidence

    def bedroom_callback(self, msg):
        self.loc['bedroom'] = msg.confidence

    def kitchen_callback(self, msg):
        self.loc['kitchen'] = msg.confidence

    def living_room_callback(self, msg):
        self.loc['livingroom'] = msg.confidence

    def timer_human_loc_callback(self):
        print('locval', self.loc.values())
        message = String()
        print('sum', sum(self.loc.values()))
        if sum(self.loc.values()) == 0:
            message.data = 'outside'
        else:
            max_conf = max(self.loc.values())
            for key, value in self.loc.items():
                if value == max_conf:
                    message.data = key

        print('aaaaaxxxxxxxxxxxxxxxxaaaaaa', message)
        self.publisher_.publish(message)
        # self.loc = []
        # self.conf = []


def main(args=None):
    rclpy.init(args=args)

    current_dir = get_package_share_directory('yolostate')
    data_path = os.path.join(current_dir, 'yolodata')
    yh = HumanDetector(data_path)

    node_living_room = DetectHumanLoc(yh, 'living_room_cam', '/unity_camera_living_room/color/image_raw')

    node_kitchen = DetectHumanLoc(yh, 'kitchen_cam', '/unity_camera_kitchen/color/image_raw')

    node_bedroom = DetectHumanLoc(yh, 'bedroom_cam', '/unity_camera_bedroom/color/image_raw')

    node_dining_room = DetectHumanLoc(yh, 'dining_room_cam', '/unity_camera_dining_room/color/image_raw')

    node_hum_loc = GetLoc('loc', 'human_loc_from_cams')

    executor = MultiThreadedExecutor()

    executor.add_node(node_living_room)
    executor.add_node(node_kitchen)
    executor.add_node(node_dining_room)
    executor.add_node(node_bedroom)

    executor.add_node(node_hum_loc)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
