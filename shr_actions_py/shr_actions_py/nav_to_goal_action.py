import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Point, Twist
from math import atan2
from rclpy.action import ActionServer
from shr_msgs.action import NavigateToGoal
# from nav2_msgs.action import NavigateToPose


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback)


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.subscription = self.create_subscription(Odometry, "/odom", self.new_odom, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.subscription = self.create_subscription(Point, 'waypoints', self.listener_callback, 10)

        self.speed = Twist()

        self.waypoints = []
        self.goal = Point()

        self.current_waypoint_count = 0

        self.timer = self.create_timer(0.25, self.control_loop)



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        print('goal_handle', goal_handle.request)

        if

        else:
            goal_handle.abort()

        result = NavigateToPose.Result()
        # result.error_code = result.NONE
        return result


    def listener_callback(self, point):
        self.waypoints.append(point)


    def new_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = self.euler_from_quaternion(rot_q)
        # tf2_ros.transformations.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def euler_from_quaternion(self, quaternion):

        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def control_loop(self):
        print(self.waypoints)
        if self.waypoints:
            if len(self.waypoints)-1 > self.current_waypoint_count:

                self.goal.x = self.waypoints[self.current_waypoint_count].x
                self.goal.y = self.waypoints[self.current_waypoint_count].y

                inc_x = self.goal.x - self.x
                inc_y = self.goal.y - self.y

                angle_to_goal = atan2(inc_y, inc_x)

                if abs(angle_to_goal - self.theta) > 0.1:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.3
                else:
                    self.speed.linear.x = 0.5
                    self.speed.angular.z = 0.0

                print('goal', self.goal)
                print('waypoint', self.waypoints[self.current_waypoint_count])
                print('speed', self.speed)
                self.publisher.publish(self.speed)
                self.current_waypoint_count += 1


def main(args=None):
    rclpy.init(args=None)
    nav_action_server = NavigationActionServer()
    rclpy.spin(nav_action_server)
    nav_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

