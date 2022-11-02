import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from ament_index_python.packages import get_package_share_directory


class YoloHuman:
    def __init__(self, yolo_data_dir):
        """
        yolo_data_dir contains yolov3.cfg, yolov3.txt, yolov3.weights
        """
        print('loaidng yolo data from: ', yolo_data_dir)
        fn_class_names = yolo_data_dir + '/' + 'yolov3.txt'
        fn_cfg = yolo_data_dir + '/' + 'yolov3.cfg'
        fn_weights = yolo_data_dir + '/' + 'yolov3.weights'

        # load data from /home/user/.yolo directory
        home_path = os.path.expanduser('~')
        yolo_dir = os.path.join(home_path, '.yolo')
        fn_weights = yolo_dir + '/' + 'yolov3.weights'
        installed = os.path.exists(fn_weights)
        print('yolo weights path: ', fn_weights, 'installed?', installed)
        if not installed:
            print('yolov3.weights not found')
            print('please run: ros2 run yolostate downloadyolo')
            print('aborted')
            return

        with open(fn_class_names, 'r') as f:
            classes = [line.strip() for line in f.readlines()]
        self.classes = np.array(classes)

        self.net = cv2.dnn.readNet(fn_weights, fn_cfg)
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

    def run_inference(self, image, scale=0.00392):
        self.width = image.shape[1]
        self.height = image.shape[0]
        blob = cv2.dnn.blobFromImage(image, scale, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        return outs

    def parse_boxes(self, outs, conf_threshold=0.5, nms_threshold=0.4):
        '''
        outs= net.forward(---)
        '''
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                if self.classes[class_id] != 'person':  # we are interested only in human
                    continue
                confidence = scores[class_id]
                if confidence > conf_threshold:
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([int(x), int(y), int(w), int(h)])

        # apply non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

        boxes = np.array(boxes)[indices]
        confidences = np.array(confidences)[indices]
        class_ids = np.array(class_ids)[indices]
        names = []
        if len(indices) > 0:
            names = self.classes[class_ids]

        return names, confidences, boxes

    def detect_human(self, image):
        outs = self.run_inference(image)
        names, confidences, boxes = self.parse_boxes(outs)
        return names, confidences, boxes

    def draw_box(self, img, name, confidence, box):
        x, y, w, h = box
        x, y, xd, yd = round(x), round(y), round(x + w), round(y + h)
        color = (np.random.randint(255), np.random.randint(255), np.random.randint(255))
        color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)][np.random.randint(3)]
        cv2.rectangle(img, (x, y), (xd, yd), color, 2)
        cv2.putText(img, name, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

'''
ros2 run yolostate detecthuman --ros-args -p camera:=/depth_camera/image_raw -p view_camera:=true
'''
class DetectHuman(Node):
    def __init__(self):
        super().__init__('detect_human')

        self.view_camera = True
        
        self.declare_parameter('view_camera', False)
        self.declare_parameter('camera', '/smart_home/camera/color/image_raw')
        param_camera_topic = self.get_parameter('camera').value
        self.view_camera=self.get_parameter('view_camera').value

        self.subscription = self.create_subscription(
            Image, 
            param_camera_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
 

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        current_dir = get_package_share_directory('yolostate')
        print('package_path: ', current_dir)
        data_path = current_dir + '/' + 'yolodata'
        print('data_path', data_path, '\n')
        # current_dir=os.getcwd()
        self.yh = YoloHuman(data_path)
        self.human_pos = [0, 0, 0, 0]

        self.publisher_ = self.create_publisher(Int32MultiArray, '/detecthuman', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_human_pub_callback)

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

    def listener_callback(self, data): 
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
            print(f'Human detected. total={len(names)}')
            self.on_human_data(boxes[0])                   #set human detected data for timer publisher.
 
        if self.view_camera:
            for name, conf, box in zip(names, confidences, boxes):
                org = self.yh.draw_box(org, name, conf, box)

            # cv2.imshow("camera", current_frame)
            cv2.imshow("human detection", org)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = DetectHuman()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
