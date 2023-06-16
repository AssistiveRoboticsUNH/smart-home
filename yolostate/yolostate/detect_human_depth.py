import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ament_index_python.packages import get_package_share_directory
from yolostate.yolo_human_detect import HumanDetector
 


'''
ROS2 node, 
subscribe to both RGB image and depth camera+depth info
detect human and estimate 3d transform

publish transform
'''
class DetectHumanAndDepth(Node):
    def __init__(self):
        super().__init__('detect_human_depth')
 
        self.declare_parameter('view_camera', False)
        self.declare_parameter('view_depth_camera', True)
        self.declare_parameter('camera', '/camera/color/image_raw')
        self.declare_parameter('depth_camera', '/camera/depth/image_rect_raw')
        self.declare_parameter('depth_camera_info', '/camera/depth/camera_info')

        self.declare_parameter('pub_human', '/detecthuman')
        self.declare_parameter('pub_human_depth', '/detecthumandepth')


        param_camera_topic = self.get_parameter('camera').value
        param_depth_camera_topic = self.get_parameter('depth_camera').value
        param_depth_camera_info_topic = self.get_parameter('depth_camera_info').value
        self.view_camera=self.get_parameter('view_camera').value
        self.view_depth_camera=self.get_parameter('view_depth_camera').value
        self.pub_human_topic=self.get_parameter('pub_human').value 
        self.pub_human_depth_topic=self.get_parameter('pub_human_depth').value 
 
        self.subscription = self.create_subscription(
            Image, 
            param_camera_topic,
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning


        self.sub_depth_camera= self.create_subscription(
            Image, 
            param_depth_camera_topic,
            self.depth_callback,
            10)

        self.sub_depth_camera_infoHumanDetector = self.create_subscription(
            CameraInfo,
            param_depth_camera_info_topic,
            self.depth_info_callback,
            10)

        self.publisher_ = self.create_publisher(Int32MultiArray, self.pub_human_topic, 10)
        self.pub_hd= self.create_publisher(Float32MultiArray, self.pub_human_depth_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        self.last_depth_ros_image = None   #store the depth camera ros image
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        current_dir = get_package_share_directory('yolostate')
        print('package_path: ', current_dir)
        data_path = current_dir + '/' + 'yolodata'
        print('data_path', data_path, '\n')
        # current_dir=os.getcwd()
        self.yh = HumanDetector(data_path)
        self.human_pos = [0, 0, 0, 0]

        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_human_pub_callback)

    def timer_human_pub_callback(self):
        msg = Int32MultiArray()
        msg.data = [int(self.human_pos[0]), int(self.human_pos[1]), int(self.human_pos[2]), int(self.human_pos[3])]
       
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        # reset data
        self.human_pos = [0, 0, 0, 0]

    def calculate_xyz(self, cx, cy, depth):
        """
        use camera intrinsic
        given, center position of the object as cx,cy and distance from the camera
        calculate x,y,z 
        """
        constant_x=1/self.fxy[0]
        constant_y=1/self.fxy[1]
        px= (cx - self.cxy[0]) * depth * constant_x
        py = (cy - self.cxy[1]) * depth * constant_y
        pz=depth
        return [pz, -px, py]
        
    def draw_box_gray(self, img, name, box, thick=1):
        """
        draw box on gray image
        """
        x, y, w, h = box
        x, y, xd, yd = round(x), round(y), round(x + w), round(y + h)
        color = (np.random.randint(255), np.random.randint(255), np.random.randint(255))
        color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)][np.random.randint(3)]
        cv2.rectangle(img, (x, y), (xd, yd), color, thick)
        cv2.putText(img, name, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thick)
        return img

    def on_human_detected(self, box):
        """
        when bounding box found from rgb camera
        """
        x, y, xd, yd = box
        self.human_pos = box
        self.get_logger().info(f'on_human_detected {box} ')

        if self.last_depth_ros_image!=None: 
            ros_image=self.last_depth_ros_image
            # ros_image.step *=2  #solve unity issue  
            img = self.br.imgmsg_to_cv2(ros_image ,desired_encoding='passthrough')
            org=img.copy()
            org=(org-org.min() )    /(org.max()-org.min())  #correction vis for gazebo depth cam
            #img=(img-img.min() ) #/ (img.max()-img.min())  #correction vis for gazebo depth cam
            # img=(img-img.min() ) / (img.max() )  #correction vis for gazebo depth cam

            if(xd>5):   #if width of the box is >5
                # org=self.draw_box(org, "bbox", self.last_rect) 
        
                #crop middle
                if x<0: x=0
                if y<0: y=0 
                y=y+int(yd*0.2) 
                yd=int(yd*0.4)  #top 40%
                #end of crop


                cx = min(int( x+xd/2 ), img.shape[1]-2) #center of the box
                cy = min(int( y+yd/2 ),  img.shape[0]-2)#center of the box
                pv=img[cy,cx]     #center pixel value 
                # depth =pv*10      #unity depth image give 1 for 10 meter.
                depth=(pv -0.28 ) *10   #TODO: hack
                # depth=pv *10.0

                # depth -=None   #TODO: play

                org[cy,cx]=0  #drawing
                org[cy,cx+1]=0  #drawing
                org[cy,cx-1]=0  #drawing
                org[cy+1,cx]=0  #drawing
                org[cy-1,cx]=0  #drawing
                
                color = 2**16-1  # 65535 is white color for 16 bis image
                text='pv: '+str(pv)+' depth='+str(depth)
                org = cv2.putText(org, f"cx:{cx} cy:{cy}", (40, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                org = cv2.putText(org, text, (40, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    
                

                if self.fxy[0] != None: 
                    self.pxyz=self.calculate_xyz(cx, cy, depth)
                    msg = Float32MultiArray()
                    # msg.data=self.pxyz 
                    msg.data = [float(self.pxyz[0]), float(self.pxyz[1]), float(self.pxyz[2]) ]  #TODO: find problemj
                    self.pub_hd.publish(msg)


                
                # #cropping.
                # if x<0: x=0
                # if y<0: y=0 
                # y=y+int(yd*0.2) 
                # yd=int(yd*0.4)  #top 40%

                # img=img[y:y+yd, x:x+xd]       #crop the center of the person.

                org=self.draw_box_gray(org, "mid", [x,y,xd,yd]) 

                #publish tf of the human position. 
                # self.publish_tf(self.pxyz, "camera_link", ros_image.header.stamp) 
                self.publish_tf(self.pxyz, ros_image.header.frame_id, ros_image.header.stamp) 
 
            if self.view_depth_camera:
                cv2.imshow("depth camera 2: "+str(ros_image.header.frame_id) , org)
                cv2.waitKey(1)


    def publish_tf(self, xyz, parent_frame, header_stamp):
        """
        publish a tf for the human
        """
        t = TransformStamped()
        t.header.stamp = header_stamp
        # t.header.frame_id = ros_image.header.frame_id
        # t.header.frame_id = 'base_link'
        t.header.frame_id = parent_frame
        t.child_frame_id = 'human_tf'

        t.transform.translation.x = float(xyz[0] )
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])

        # q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = 0.
        t.transform.rotation.y = 0.
        t.transform.rotation.z = 0.
        t.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


    def camera_callback(self, data): 
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
            self.on_human_detected(boxes[0])                   #set human detected data for timer publisher.

        elif self.view_depth_camera and self.last_depth_ros_image!=None:  #to make sure depth viewer window doesn't get freeze.
                ros_image=self.last_depth_ros_image 
                dimg = self.br.imgmsg_to_cv2(ros_image ,desired_encoding='passthrough') 
                cv2.imshow("depth camera 2: "+str(ros_image.header.frame_id) , dimg)
                cv2.waitKey(1)

 
        if self.view_camera:
            for name, conf, box in zip(names, confidences, boxes):
                org = self.yh.draw_box(org, name, conf, box)

            # cv2.imshow("camera", current_frame)
            cv2.imshow("human detection", org)
            cv2.waitKey(1)


    def depth_info_callback(self, msg):
        """
        depth camera info
        """
        K=msg.k 
        fx=msg.k[0]
        cx=msg.k[2]
        fy=msg.k[4]
        cy=msg.k[5] 

        self.fxy=[fx, fy]
        self.cxy=[cx, cy]
        self.get_logger().info(f'depth info: fxy: {fx:.2f},{fy:.2f} cxy:{cx},{cy}')


    
    def depth_callback(self, ros_image):
        """
        store the depth image to be processed when human detected.
        """
        self.last_depth_ros_image=ros_image
        




def main(args=None):
    rclpy.init(args=args)
    image_subscriber = DetectHumanAndDepth()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
