import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from shr_msgs.action import DockingRequest
from shr_docking.docking_camera_main import Docking
from shr_docking.docking_ir_main import Docking_IR
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import os


class DockingMainActionServer(Node):

    def __init__(self):
        super().__init__('Docking_main_action_server')
        self.docking_camera = Docking()
        self.docking_ir = Docking_IR()
        #self.goal_cancel = False
        self.rate = self.docking_camera.create_rate(10)
        
        
        self.action_server = ActionServer(
            self,
            DockingRequest,  # Replace with the actual action type
            'docking',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # print("working action")
        self.pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 1)
        self.vel = Twist()
        
        self.current_mode = 'apriltag'  # Default mode
        self.last_detection_time = None
        self.tag_timeout = 2.0  # Timeout in seconds before switching to IR docking

        

    def goal_callback(self, goal_request):
        self.get_logger().info("weblog="+'ACCEPTED docking goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        # Your cancellation logic here
        self.get_logger().info('Goal cancelled')
        self.goal_cancel = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("working callback")
        print("working init", goal_handle)
        self.docking_ir.is_charging = self.docking_camera.charger_status==1
        self.docking_camera.bumped = False
        self.docking_ir.bumped = False
        self.failed_count = 0
        
        while not (self.docking_camera.bumped or self.docking_ir.bumped):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.abort()
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
                result = DockingRequest.Result()
                result.result = False
                return result
            
            # # **AprilTag Detection Handling with Debounce**
            # current_time = self.get_clock().now().nanoseconds / 1e9  # Convert time to seconds

            # if self.docking_camera.is_detect:
            #     self.current_mode = 'apriltag'
            #     self.last_detection_time = current_time  # Update last detection time
            # elif self.last_detection_time and (current_time - self.last_detection_time < self.tag_timeout):
            #     self.get_logger().info("AprilTag lost briefly, waiting before switching.")
            # else:
            #     self.current_mode = 'ir'  # Switch to IR docking

            # self.get_logger().info(f"Current mode: {self.current_mode}")

            
            # **Execute the Active Docking Mode**
            if self.docking_camera.is_detect:
                self.docking_camera.bumped = False
                self.docking_ir.bumped = False
                self.docking_camera.get_transformation_from_aptag_to_port()
                self.failed_count += self.docking_camera.move_towards_tag()
            else:
                self.failed_count += self.docking_ir.move_to_docking_station()
                 
            if self.failed_count > 10:
                self.get_logger().info(f'Docking aborted for no bump sensor data')
                break
                

        self.get_logger().info(f'Bumped from Camera: {self.docking_camera.bumped}')
        self.get_logger().info(f'Bumped from IR: {self.docking_ir.bumped}')
        if (self.docking_camera.bumped or self.docking_ir.bumped):
            print("Bumped!!")
            self.vel.linear.x = 0.0
            self.vel.angular.z =0.0
            self.pub.publish(self.vel)
            print(self.docking_camera.charger_status)
            time.sleep(15)
            if(self.docking_camera.charger_status is not None and (self.docking_camera.charger_status ==1)):
                goal_handle.succeed()
                result = DockingRequest.Result()
                result.result = True
                self.docking_camera.bumped = False
                self.docking_ir.bumped = False
                self.docking_ir.is_charging = False
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
                self.rate.sleep()
                self.get_logger().info("weblog="+' docked and charging!')
                return result
            else:
                goal_handle.abort()
                result = DockingRequest.Result()
                result.result = False
                self.docking_camera.bumped = False
                self.docking_ir.bumped = False
                
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
                self.rate.sleep()
                self.get_logger().info("weblog="+' docking aborted for not charging!')
                return result
            
        else:
            goal_handle.abort()
            self.vel.linear.x = 0.0
            self.vel.angular.z =0.0
            self.pub.publish(self.vel)
            
            result = DockingRequest.Result()
            self.get_logger().info("weblog="+' docking aborted!')
            result.result = False
            return result


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockingMainActionServer()
    executor = MultiThreadedExecutor()
    
    try:
        while rclpy.ok():
            rclpy.spin(subscriber_node, executor=executor)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



