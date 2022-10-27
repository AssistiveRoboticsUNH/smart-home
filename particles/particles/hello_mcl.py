import rclpy  
from rclpy.node import Node  
from nav2_msgs.msg import ParticleCloud, Particle
import numpy as np
import os
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import  qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from particles import util
# from particles.sensor_model import Map
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 
import yaml

class Map:
    def __init__(self, pkg_name, cfg_file, logger):
        package_dir = get_package_share_directory(pkg_name)
        self._map_cfg = None
        with open(os.path.join(package_dir, cfg_file)) as cfg:
            self._map_cfg = yaml.load(cfg, Loader=yaml.FullLoader)
        if self._map_cfg is None:
            raise RuntimeError(f"{cfg_file} not found")
        self.image_name = self._map_cfg['image']
        self.mode = self._map_cfg['mode']
        self.resolution = self._map_cfg['resolution']
        self.origin = self._map_cfg['origin']
        self.negate = self._map_cfg['negate']
        self.occupied_thresh = self._map_cfg['occupied_thresh']
        self.free_thresh = self._map_cfg['free_thresh']
        image_path = os.path.join(package_dir, self.image_name)
        print('image_path:', image_path)
        image_data = cv2.imread(image_path, -1)
        rotated = np.rot90(image_data)
        self.data = []
        for pixel in list(rotated.flatten()):
            self.data.append(1.0 - float(pixel) / 255.0)
        self.height = rotated.shape[1]
        self.width = rotated.shape[0]
        logger.debug(f"height:{self.height}, width:{self.width}, origin:{self.origin}")

        # cv2.imshow("image", image_data)
        # cv2.waitKey(0) 



# ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "map_url: '/home/ns/smart_home_ws/src/smart-home/external/unity_pioneer/pioneer_navigation2/map/map.yaml'" 

class HelloMCL(Node):
    def __init__(self):
        super().__init__('hello_mcl')
 
        self.create_subscription(Odometry, '/odom',
                                 self.odometry_callback, 1)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, 1)

        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_callback, 1)


        self.br = CvBridge()
        self.sub_camera= self.create_subscription(
            Image,
            '/smart_home/camera/depth/image_raw',
            self.depth_callback,
            10)


        self.last_odom= None
        self.last_scan = None 

        self.map_origin=(-8.28 , -6.33)
        self.map_resolution=0.05
        self.map_width=225
        self.map_height=178


        # self.subscription = self.create_subscription(
        # ParticleCloud,  '/particle_cloud',  self.listener_callback, qos_profile=qos_profile_system_default)
        # self.subscription # prevent unused variable warning

        self.particles= []
        self.num_of_particles=200
        self._initialize_pose()
        self._initialize_particles_gaussian()

        self._particle_pub = self.create_publisher(ParticleCloud, '/particlecloud', 10)
        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)

        
        print('loading map...')
        self._map = Map("pioneer_navigation2", 'map/map.yaml', self.get_logger())
        print('map size: ',self._map.width, self._map.height)   #w=225, h=178 using service load
 
        # self._map_publisher = self.create_publisher(
        #     OccupancyGrid,
        #     '/map2',
        #     qos_profile=QoSProfile(
        #         depth=1,
        #         durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #         history=HistoryPolicy.KEEP_LAST,
        #     )
        # )

        # self._publish_map()
 
    def depth_callback(self, ros_image):
        # ros_image.step *=2  #solve unity issue
        img = self.br.imgmsg_to_cv2(ros_image ,desired_encoding='passthrough')

      
        cv2.imshow("depth camera " , img)
        cv2.waitKey(1)


    def _publish_map(self):
        map = [-1] * self._map.width * self._map.height
        idx = 0
        for cell in self._map.data:
            map[idx] = int(cell * 100.0)
            idx += 1
        stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self._map.resolution
        msg.info.width = self._map.width
        msg.info.height = self._map.width
        msg.info.origin.position.x = self._map.origin[0]
        msg.info.origin.position.y = self._map.origin[1]
        msg.data = map
        self._map_publisher.publish(msg)

    def timer_callback(self): 
            # robot_pos=self.last_odom.pose.pose.position
            # rx=robot_pos.x
            # ry=robot_pos.y
            if self.last_odom!=None:
                self._initialize_particles_gaussian(pose=self.last_odom)
                
            self.pub_particles()
             


    def map_callback(self, msg):
        print('----map callback-----')
        header=msg.header
        data=msg.data
        info=msg.info
        origin=info.origin.position
        print('new map: ', info.width, info.height)
        print('origin:', origin.x, origin.y)            #origin: -8.28 -6.33

        print('robot pos:', self.last_odom.position.x, self.last_odom.position.x)

        data=np.array(data).reshape(info.height, info.width)
        print('data shape: ', data.shape)

        data=np.rot90(data)
        image = cv2.flip(data, 1)
        ts=set( image.ravel().tolist())
        print('ts=', ts)


        floodval=200
        # cv2.floodFill(image, None, (0,0), floodval) 
        # cv2.floodFill(image, None)
        # arena = ((image==floodval) * 255).astype(np.uint8) 

        # ts2=set( arena.ravel().tolist())
        # print('ts2=', ts2)
        # cv2.imwrite("problemfile.jpg", image)


        # self.create_uniform_particles()
        # self.pub_particles()


        # cv2.imshow("map_data", image)
        # cv2.waitKey(0) 
 
    def odometry_callback(self, msg: Odometry):
        self.last_odom = msg.pose.pose
         

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        # self.get_logger().info(f'scan total: {len(msg.ranges)}')
        dist_back = format(msg.ranges[180], '.2f')
        dist_left = format(msg.ranges[90], '.2f')
        dist_right = format(msg.ranges[270], '.2f')
        dist_head = format(msg.ranges[0], '.2f')
        self.get_logger().info(f'scan: {len(msg.ranges)} {dist_back} {dist_left} {dist_right} {dist_head}')




    def listener_callback(self, data):
        header=data.header
        particles=data.particles
        print('header:', header)
        print('Total: ', len(particles))
        pos=particles[0].pose.position
        print('pos=', pos)

    def pub_particles(self): 
        msg=ParticleCloud()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for particle in self.particles:
            msg.particles.append(particle)
        self._particle_pub.publish(msg)


    def _initialize_pose(self):
        position = Point(x=0.0,
                         y=0.0,
                         z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self.current_pose = Pose(position=position,orientation=orientation)
        self.last_odom = self.current_pose

    def _initialize_particles_gaussian(self, pose= None, scale= 0.05):
        self.particles=[]
        if pose is None:
            pose = self.current_pose

        
        x_min=self.map_origin[0]+0
        x_max=self.map_origin[0]+self.map_width*self.map_resolution

        y_min=self.map_origin[1]+0
        y_max=self.map_origin[1]+self.map_height*self.map_resolution

        x_list=list( np.random.uniform(x_min, x_max,  size=self.num_of_particles) )
        y_list=list( np.random.uniform(y_min, y_max,  size=self.num_of_particles) )
        scale=1
 
        current_yaw = util.yaw_from_quaternion(pose.orientation)
        yaw_list = list(np.random.normal(loc=current_yaw, scale=0.01, size=self.num_of_particles- 1))

        initial_weight = 1.0 / float(self.num_of_particles)

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            position = Point(x=x, y=y, z=0.0)
            orientation = util.euler_to_quaternion(yaw, 0.0, 0.0)
            temp_pose = Pose(position=position, orientation=orientation)
            p=Particle()
            p.pose=temp_pose
            p.weight=initial_weight
            self.particles.append(p) 

        p=Particle()
        p.pose= pose
        p.weight=initial_weight
        self.particles.append(p)
        


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = HelloMCL()
    
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()