import imp
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
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from copy import deepcopy
import geometry_msgs.msg as gm
import cv2 
import yaml

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.declare_parameter('odom', '/odom') 
        param_odom_topic = self.get_parameter('odom').value

        self.create_subscription(Odometry, param_odom_topic,
                                 self.odometry_callback, 1)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, 1)

        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_callback, 1)


        # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'human_tf').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_human_xy=[None, None]
   
        self.last_odom2=None  #real odom
        self.last_odom= None  #only Pose
        self.last_scan = None 
 
        self.map_origin=(-8.28 , -6.33)
        self.map_resolution=0.05
        self.map_width=225
        self.map_height=178
        
        
 
        self._particle_pub = self.create_publisher(ParticleCloud, '/particlecloud', 10)
        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)

        self.particles_xy=np.zeros((200, 3))
        self.particles= []
        self.num_of_particles=200
        self._initialize_pose()
        self._initialize_particles_gaussian()
        self.pub_particles()  #pub initial

        
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
        self.time_stamp=None 

 

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

    def parse_human_xy(self, timestamp=None):
        to_frame_rel = 'map'
        # from_frame_rel = 'depth_camera'
        from_frame_rel = 'human_tf'
        try: 
            timestamp=rclpy.time.Time()
            # if self.time_stamp!=None:
            #     timestamp=self.time_stamp

            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                timestamp)
            x=t.transform.translation.x
            y=t.transform.translation.y
            d=np.sqrt(x**2 +  y**2)
            return (x, y, d)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return (None, None, 0)

    def pdf(self, x, mu, sigma):
        scaling=1/(sigma**2 * 2 * np.pi)**0.5
        v=np.exp(-0.5* ( (x-mu)/sigma)**2 )
        return scaling*v

    def timer_callback(self): 

        #adding some noise to each particles
        # noise =np.random.normal(0, 0.1, len(self.particles)) 
        # for i in range(len(self.particles)):
        #     self.particles[i].pose.position.x+=noise[i]
        #     self.particles[i].pose.position.y+=noise[i]
        #end of adding noise. 

        x,y,d=self.parse_human_xy()
        
        noise_std=0.2
        if d>0:  #human detected
            noise_std=0.05
            self.last_human_xy=[x,y]
            self.get_logger().info(f"human_tf: {x}, {y} :: {d} ")

            pd=[]
            for i in range(len(self.particles_xy)):
                px=self.particles_xy[i,0]
                py=self.particles_xy[i,1]
                d=(x-px)**2 + (y-py)**2   #distance from the human to particle_i
                d=self.pdf(d, 0, 0.5)
                pd.append(d)

            pr=np.array( pd) +0.0001       #to prevent sum(pr)=0
            pr = pr / np.sum(pr)      #proba dist

            ni=np.random.choice(range(len(pr)), len(pr), p=pr)  #getting the indexes

   
            si=[]
            for i in ni:
                si.append(self.particles_xy[i])

            self.particles_xy=np.array( si )   #update  


 

        noise =np.random.normal(0, noise_std, len(self.particles_xy)) 
        noise2 =np.random.normal(0, noise_std, len(self.particles_xy)) 
        for i in range(len(self.particles_xy)):
            self.particles_xy[i,0]+=noise[i]
            self.particles_xy[i,1]+=noise2[i]


        self.check_boundary()
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
        self.last_odom2=msg
        self.time_stamp=msg.header.stamp
         

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        # self.get_logger().info(f'scan total: {len(msg.ranges)}')
        dist_back = format(msg.ranges[180], '.2f')
        dist_left = format(msg.ranges[90], '.2f')
        dist_right = format(msg.ranges[270], '.2f')
        dist_head = format(msg.ranges[0], '.2f')
        # self.get_logger().info(f'scan: {len(msg.ranges)} {dist_back} {dist_left} {dist_right} {dist_head}')

 

    def listener_callback(self, data):
        header=data.header
        particles=data.particles
        print('header:', header)
        print('Total: ', len(particles))
        pos=particles[0].pose.position
        print('pos=', pos)

    def pub_particles(self): 
        num_particles=200
        initial_weight = 1.0 / float(num_particles)
        particles=[]
        for x, y, yaw in self.particles_xy: 
            position = Point(x=x, y=y, z=0.0)
            orientation = util.euler_to_quaternion(yaw, 0.0, 0.0)
            temp_pose = Pose(position=position, orientation=orientation)
            p=Particle()
            p.pose=temp_pose
            p.weight=initial_weight
            particles.append(p)  
        #created Particle from x,y position

        msg=ParticleCloud()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for particle in particles:
            msg.particles.append(particle)
        self._particle_pub.publish(msg)
 

    def _initialize_pose(self):
        position = Point(x=0.0,
                         y=0.0,
                         z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self.current_pose = Pose(position=position,orientation=orientation)
        self.last_odom = self.current_pose


    def check_boundary(self):
        x_min=self.map_origin[0]+0
        x_max=self.map_origin[0]+self.map_width*self.map_resolution
        y_min=self.map_origin[1]+0
        y_max=self.map_origin[1]+self.map_height*self.map_resolution

        for i in range(len(self.particles_xy)):

            if self.particles_xy[i,0] <x_min or self.particles_xy[i,0] >x_max:
                self.particles_xy[i,0 ] =np.random.uniform(x_min, x_max,  size=1)[0]

            if self.particles_xy[i,1]<y_min or self.particles_xy[i,1] >y_max:
                self.particles_xy[i,1] =np.random.uniform(y_min, y_max,  size=1)[0]


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

        i=0
        for x, y, yaw in zip(x_list, y_list, yaw_list): 
            self.particles_xy[i,:]=[x,y,yaw]    #add x,y,z
            i+=1
 

    

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = HelloMCL()
    
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()