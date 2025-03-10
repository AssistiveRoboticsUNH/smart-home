import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import os
from std_msgs.msg import Float32, Int64, Int32, Bool


class Docking_IR(Node):

    def __init__(self):
        super().__init__('Docking_ir')
        self.tf_buffer = Buffer()
        self.tf_listenser = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.rate = self.create_rate(10)

        # class variables
        self.vel = Twist()
        self.bumped = False #Bump either by limit switch or is_charging condition check
        self.bump = None  # Bump by limit switch (Int64)
        self.is_charging = False
        self.voltage = None
        self.voltage_prev = None
        self.ir_sensor_weight = None
        self.obstacle_back = 10 #considering no obstacle initially
        self.isLidarActive = False

        # Subscribers
        self.bump_subscriber = self.create_subscription(Int64, 'bump', self.bump_callback, 10)
        self.charger_subscriber = self.create_subscription(Float32, 'charging_voltage', self.charger_callback, 10)
        self.ir_sensor_subscriber = self.create_subscription(Float32, 'docking/ir_weight', self.ir_sensor_callback, 10)

        self.subscription = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, 10)
        
        # Publishers
        self.pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 1)

        

        # Move to docking station method variables
        self.forward_speed = -0.05
        self.rotation_speed = 0
        self.sensor_data_receiving = False #check if ir sensor is active
        self.sensor_oriented = False # Check if sensor is oriented to docking station once
        self.close_counter = 0
        self.mode = 'far'
        self.centre_ir_weight = 3.5
        self.centre_ir_set = False

    def bump_callback(self, msg):
        #self.get_logger().info(f'Received float: {msg.data}')
        self.bump = msg.data
        #print(self.bump)

    def charger_callback(self, msg):
        # For first call initialize both voltage and previous voltage 
        if self.voltage == None or self.voltage == 0:
            self.voltage = msg.data
            self.voltage_prev = msg.data
        
        # Else check charging status
        else:
            self.voltage = msg.data
            spike_ratio = (self.voltage - self.voltage_prev) / self.voltage_prev
            if spike_ratio >  0.03 and self.voltage > 12:
                self.is_charging = True
                self.get_logger().info(f'Charger Connected!')
            # elif spike_ratio <  0:
            #     self.is_charging = False
            #     self.get_logger().info(f'Charger not connected and discharging.')
            # else:
            #     self.is_charging = False
            self.voltage_prev = self.voltage
    
    def ir_sensor_callback(self, msg):
        # self.get_logger().info(f'Received IR sensor: {msg.data}')
        self.ir_sensor_weight = msg.data
        self.sensor_data_receiving = True

    def scan_callback(self, msg):
        # Scans laser and returns obstacle distane towards the back of the robot.
        if msg:
            # print("self.min_range", self.min_range)
            start_ind = int(4 * (len(msg.ranges) / 8))  # 0
            end_ind = int(4.5 * (len(msg.ranges) / 8) - 1)

            truncated_ranges = msg.ranges[start_ind:end_ind]
            # print("tracated", truncated_ranges)
            self.obstacle_back = min(msg.ranges[start_ind:end_ind])
            self.isLidarActive = True
            # print("obstacle_back: ", self.obstacle_back)
            
            ## Set the mode, whethre robot is close to docking station or far away
            if 0 < self.obstacle_back < 0.6:
                self.close_counter += 1
                if self.close_counter > 15:
                    self.mode = 'close'
            elif self.obstacle_back > 0.65: # if enough amount of close proximity not detected consider station not close (handles null laser values)
                self.close_counter = 0
                self.mode = 'far'

    def move_robot(self, x, z):
        print("I am moving from IR")
        self.vel.linear.x = float(x)
        self.vel.angular.z = float(z)
        self.pub.publish(self.vel)

    def move_to_docking_station(self):
        if (self.bump == None):
            self.get_logger().info(f'Bump not avaialable, waiting...')
            time.sleep(0.5)
            return 1
            
        if not self.isLidarActive: self.get_logger().info(f'Lidar: Not Active')
        # if not self.sensor_oriented and not self.centre_ir_set:
        #     if self.ir_sensor_weight > 4:
        #         self.centre_ir_weight = 4
        #     else:
        #         self.centre_ir_weight = 4.5
        #     self.centre_ir_set = True

        if self.isLidarActive:
            # Move robot by limit swithc bump and charging condition check
            self.get_logger().info(f'Bump: {self.bump}, is_charging: {self.is_charging}')
            if (not self.bump and not self.is_charging):
                # print("Bump: ", self.bump)
                # print("is_charging: ", self.is_charging)
                # print("")
                self.bumped = False
                try:
                    # self.get_logger().info(f'IR Weight: {self.ir_sensor_weight}')

                    if self.ir_sensor_weight == -1:
                        self.sensor_data_receiving = False

                    # self.get_logger().info(f'Obstacle Back: {self.obstacle_back}')
                    
                    # ======= Limit velocity in case of close proximity of docking station
                    if self.mode == 'close':
                        self.forward_speed = -0.03

                    elif self.mode == 'far': # if enough amount of close proximity not detected consider station not close (handles null laser values)
                        self.forward_speed = -0.05
                    
                    if self.sensor_data_receiving:
                        # ======= Set robot velocity to dock
                        if(self.ir_sensor_weight < self.centre_ir_weight-0.1):
                            if not self.sensor_oriented:
                                # First roation for self orienting to station should be fast
                                # self.get_logger().info("Rotating Fast")
                                self.move_robot(0, 0.3)
                            elif self.mode == 'close': self.move_robot(forward_speed, 0.01) #slow rotation
                            else:
                                self.rotation_speed += 0.005
                                self.rotation_speed = min(max(self.rotation_speed, -0.16), 0.16) # clip between [-0.16, 0.16]
                                self.move_robot(self.forward_speed, self.rotation_speed) # if far, rotate fast

                        elif(self.ir_sensor_weight > self.centre_ir_weight +0.1):
                            if not self.sensor_oriented:
                                # First roation for self orienting to station should be fast
                                # self.get_logger().info("Rotating Fast")
                                self.move_robot(0, -0.3)
                            elif self.mode == 'close' :self.move_robot(forward_speed, -0.01)
                            else:
                                self.rotation_speed -= 0.005
                                self.rotation_speed = min(max(self.rotation_speed, -0.16), 0.16) # clip between [-0.16, 0.16]
                                self.move_robot(self.forward_speed, self.rotation_speed)

                        else: # Move to forward to docking
                            self.move_robot(self.forward_speed, 0)
                            self.sensor_oriented = True # Set True if ever oriented to docking station in the loop
                            
                    
                except:
                    # self.get_logger().info(e)
                    # continue
                    pass
            else:
                self.bumped = True
                self.sensor_oriented = False
                self.rotation_speed = 0
                self.centre_ir_set = False
                print("Bumped")

                # self.get_logger().info(f'Bumped')
                # self.is_charging = False # Reset it to false in action server to get ready for next docking

        return 0


def main(args=None):
    rclpy.init(args=args)
    docking_ir = Docking_IR()
    
    while( rclpy.ok() and not (docking_ir.bumped)):
        docking_ir.move_to_docking_station()
        rclpy.spin_once(docking_ir)

    if(docking_ir.is_charging):
        print("Charger Connected")
    else:
        print("Charger Not Connected")

    docking_ir.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()