import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge  
import cv2  
import numpy as np
import os
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory


class YoloHuman:
    def __init__(self, yolo_data_dir):
        """
        yolo_data_dir contains yolov3.cfg, yolov3.txt, yolov3.weights
        """
        print('loaidng yolo data from: ', yolo_data_dir)
        fn_class_names=yolo_data_dir+'/'+'yolov3.txt'
        fn_cfg=yolo_data_dir+'/'+'yolov3.cfg'
        fn_weights=yolo_data_dir+'/'+'yolov3.weights'
 
        with open(fn_class_names, 'r') as f:
            classes = [line.strip() for line in f.readlines()]
        self.classes=np.array(classes)

        self.net = cv2.dnn.readNet(fn_weights, fn_cfg)
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
    def run_inference(self, image, scale = 0.00392):
        self.width = image.shape[1]
        self.height= image.shape[0]
        blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        return outs
    
    def parse_boxes(self, outs, conf_threshold = 0.5, nms_threshold = 0.4):
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
                if self.classes[class_id]!='person':  #we are interested only in human
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
                    boxes.append([x, y, w, h])
                    
        # apply non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
        
        boxes=np.array(boxes)[indices]
        confidences=np.array(confidences)[indices]
        class_ids=np.array(class_ids)[indices]
        names=[]
        if len(indices)>0:
            names=self.classes[class_ids]

        return names, confidences, boxes

    def detect_human(self, image):
        outs=self.run_inference(image)
        names, confidences, boxes=self.parse_boxes(outs)
        return names, confidences, boxes

    
    def draw_box(self, img, name, confidence, box):
        x,y,w,h=box 
        x,y,xd,yd=round(x), round(y), round(x+w), round(y+h)
        color=(np.random.randint(255), np.random.randint(255), np.random.randint(255))
        color=[(255,0,0), (0,255,0), (0,0,255)][np.random.randint(3)]
        cv2.rectangle(img, (x,y), (xd,yd), color, 2)
        cv2.putText(img, name, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

class DetectHuman(Node):
    def __init__(self):
        super().__init__('detect_human')

        self.view_camera=True
        # self.view_camera=False
        
        self.subscription = self.create_subscription(
        Image, 
        '/smart_home/camera/color/image_raw', 
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


        current_dir = get_package_share_directory('yolostate')
        print('package_path: ', current_dir)
        data_path=current_dir+'/'+'yolodata'
        print('data_path', data_path, '\n')
        # current_dir=os.getcwd()
        self.yh=YoloHuman(data_path)
        self.human_pos=[0, 0, 0, 0]

        self.publisher_ = self.create_publisher(String, '/detecthuman', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_human_pub_callback)


    def timer_human_pub_callback(self):
        msg = String()
        msg.data = str(self.human_pos)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        #reset data
        self.human_pos=[0, 0, 0, 0]


    def on_human_data(self, box):
        x,y,xd,yd=box
        self.human_pos=box

    def listener_callback(self, data):

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        org=current_frame.copy()

        current_frame=current_frame[:, :, :3]    #channel 4 to 3

        names, confidences, boxes=self.yh.detect_human(current_frame)
        if len(names)>0:
            print(f'Human detected. total={len(names)}')  
            self.on_human_data(boxes[0]) 
        
        if self.view_camera:
            for name, conf, box in zip(names, confidences, boxes):
                org=self.yh.draw_box(org, name, conf, box)

            # cv2.imshow("camera", current_frame)
            cv2.imshow("camera", org)
            cv2.waitKey(1)
  
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = DetectHuman()
    
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()