import rclpy
from rclpy.action import ActionServer, GoalResponse, ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile

import numpy as np
import math
import time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped, PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
import tf_transformations as tr
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection




class MoveToGoalwithLocalizationActionServer(Node):

    def __init__(self):
        super().__init__('MoveToGoalwithLocalization_action_server')
        self.nav2_to_goal_client = None
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose_with_localization',
            self.execute_callback,
            goal_callback=self.goal_callback)  # Add goal_callback

        ## get the pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # from dt_apriltags import Detector
        # at_detector = Detector(searchpath=['apriltags'],
        #                        families='tag36h11',
        #                        nthreads=1,
        #                        quad_decimate=1.0,
        #                        quad_sigma=0.0,
        #                        refine_edges=1,
        #                        decode_sharpening=0.25,
        #                        debug=0)

        # For localization
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(ParticleCloud, 'particle_cloud', self.particles_callback,
                                                   qos_profile)
        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

        # For April tags
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        self.aptags_detected = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.used_apriltags = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11 ,12 ,13, 14, 15, 16, 18]  # add the apriltag ids that you used
        self.transform_aptag_in_cam_dict = {}  # location of apriltags in camera frame
        self.transform_aptag_in_world_dict = {}  # global location of apriltags
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.closest_aptag = None
        # self.cam_to_base_link = None

        self.successfully_navigating = False
        self.result_future = None
        self.max_weight = 0
        self.time_out = 200
        self.get_tf_info = True

        self.get_transform_matrix_aptags_in_world_from_tf()
        self.aptags_detected_inside_callback = False

    ##### Localization Part #####
    def publish_tf(self, x, y, z, rot_mat, child_frame_id, frame_id):
        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rot_mat))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = frame_id
        static_transform.child_frame_id = child_frame_id

        static_transform.transform.translation.x = x  # Set translation values
        static_transform.transform.translation.y = y
        static_transform.transform.translation.z = z  # meters

        static_transform.transform.rotation = quat
        # print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        self.tf_broadcaster.sendTransform(static_transform)
        # print(f'publishing child {child_frame_id} from {frame_id}')

    def get_dist(self, x, y, z):
        return np.linalg.norm([x, y, z])

    def apriltag_callback(self, msg):
        ### THIS SHOULD HAVE A FLAG IF APRILTAG CALLBACK CALCULATE TF IS TRUE THEN DO THE CLACULATION BUT FIRST IJUST WANT TO CHECK IF THERE ARE TAGS DETECTED
        print("cjcsdcnjksac")
        if msg.detections:
            self.aptags_detected = True

            min_distance = np.inf
            if self.get_tf_info:
                self.transform_aptag_in_cam_dict = {}
                source_frame = msg.header.frame_id  # to

                try:
                    for at in msg.detections:
                        # if at.id != "203":
                        frame = "tag_" + str(at.id)  # from
                        # print(frame)
                        # try:
                        transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                         timeout=rclpy.duration.Duration(seconds=1000.0))

                        dist = self.get_dist(transformation.transform.translation.x, transformation.transform.translation.y,
                                             transformation.transform.translation.z)
                        if dist < min_distance:
                            min_distance = dist
                            self.closest_aptag = at.id

                        translation = tr.translation_matrix(
                            [transformation.transform.translation.x, transformation.transform.translation.y,
                             transformation.transform.translation.z])
                        rotation = tr.quaternion_matrix(
                            [transformation.transform.rotation.x, transformation.transform.rotation.y,
                             transformation.transform.rotation.z, transformation.transform.rotation.w])
                        # print('source', source_frame, 'frame', frame, 'rotation', rotation)
                        # Get the homogeneous transformation matrix
                        transform_aptag_in_cam = np.dot(translation, rotation)

                        self.transform_aptag_in_cam_dict[at.id] = transform_aptag_in_cam
                        # print('self.transform_aptag_in_cam_dict[at.id]', self.transform_aptag_in_cam_dict[at.id])
                        # self.get_logger().info(f'transform ready from {frame} to {source_frame}')
                        # print("aptag detected!!!!!!")
                        self.aptags_detected_inside_callback = True
                        print('********** apriltag detectedd: ' + str(at.id))

                except Exception as ex:
                    # pass
                    self.get_logger().info(f'Error ***************: {ex}')
                    # else:
                    #     print('id 203')

        else:
            # pass
            # print('No aptags from callback')
            self.aptags_detected = False

    def get_transform_matrix_aptags_in_world_from_tf(self):
        self.transform_aptag_in_cam_dict = {}
        for aptag in self.used_apriltags:

            str_aptag = str(aptag)
            source_frame = "map"  # to
            frame = "aptag_" + str_aptag  # from

            try:
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(seconds=5.0))

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # Get the homogeneous transformation matrix
                transform_aptag_in_world = np.dot(translation, rotation)

                self.transform_aptag_in_world_dict[aptag] = transform_aptag_in_world

                self.get_logger().info(f'transform ready from {frame} to {source_frame}')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

    def rotation_matrix_to_quaternion(self, R):
        # Convert a 3x3 rotation matrix to a Quaternion
        trace = np.trace(R)
        if trace > 0:
            S = 2.0 * math.sqrt(trace + 1.0)
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return [qx, qy, qz, qw]

    def get_transform_matrix_cam_to_base_link(self):

        frame = "camera_color_optical_frame"  # to
        source_frame = "base_link"  # from

        try:
            transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                             timeout=rclpy.duration.Duration(seconds=5.0))
            ## TODO check why trasnformation from tf broadcast is wrong
            translation = tr.translation_matrix(
                [transformation.transform.translation.x, transformation.transform.translation.y,
                 transformation.transform.translation.z])

            rotation = tr.quaternion_matrix(
                [transformation.transform.rotation.x, transformation.transform.rotation.y,
                 transformation.transform.rotation.z, transformation.transform.rotation.w])

            # Get the homogeneous transformation matrix
            # self.cam_to_base_link = np.dot(translation, rotation)

            # transform_cam_to_base_link = np.array([[0.0, 0.0, 1.0, 0.0],
            #                                        [1.0, 0.0, 0.0, 0.0],
            #                                        [0.0, -1.0, 0.0, 0.0],
            #                                        [0.0, 0.0, 0.0, 1.0]])
            # self.cam_to_base_link = transform_cam_to_base_link
            # quat = Quaternion()
            # quat.x = 0.5
            # quat.y = -0.5
            # quat.z = 0.5
            # quat.w = 0.5
            # print('cam_to_base_link', self.cam_to_base_link)
        except:
            print('No aptags from callback')

    def particles_callback(self, msg):
        max_weight = 0.0  # Initialize with a double value
        particles_count = 0.0

        for particle in msg.particles:
            weight = particle.weight

            if weight > max_weight:
                max_weight = weight
                particles_count += particles_count

        self.max_weight = max_weight

        if particles_count > 0:
            self.get_logger().info('no particles')

        print(self.max_weight, '**************************')

    def publish_pose(self, robot_pose_aptags, rotation_matrix):
        robot_pose = PoseWithCovarianceStamped()

        self.get_logger().info(f'Publishiungg g POSEEEEE')

        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rotation_matrix))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        robot_pose.header.stamp = self.get_clock().now().to_msg()

        robot_pose.header.frame_id = "map"  # or frame_link

        robot_pose.pose.pose.position.x = robot_pose_aptags[0]
        robot_pose.pose.pose.position.y = robot_pose_aptags[1]
        robot_pose.pose.pose.orientation = quat

        self.publisher_initial_pose.publish(robot_pose)
        self.get_logger().info(f'FINISH   Publishingg POSEEEEE')


    def transform_cam_world_frame(self):
        robot_position = []
        transform_aptag_in_cam_dict = self.transform_aptag_in_cam_dict
        transform_aptag_in_world_dict = self.transform_aptag_in_world_dict

        for aptag in transform_aptag_in_cam_dict.keys():
            # print(aptag)te
            t_apriltag_to_world = transform_aptag_in_world_dict[aptag]
            t_apriltag_in_camera = transform_aptag_in_cam_dict[aptag]

            t_cam_in_world = np.dot(t_apriltag_to_world, np.linalg.inv(t_apriltag_in_camera))

            transform_cam_to_base_link = np.array([[0.0, 0.0, 1.0, 0.0],
                                                   [-1.0, 0.0, 0.0, 0.0],
                                                   [0.0, -1.0, 0.0, 0.0],
                                                   [0.0, 0.0, 0.0, 1.0]])
            self.publish_tf(transform_cam_to_base_link[0, 3], transform_cam_to_base_link[1, 3], transform_cam_to_base_link[2, 3], transform_cam_to_base_link[:3,:3],
                            'camera_color_optical_frame', 'base_link')

            t_robot_in_world = np.dot(t_cam_in_world, transform_cam_to_base_link.T)

            robot_x = t_robot_in_world[0, 3]
            robot_y = t_robot_in_world[1, 3]
            robot_z = t_robot_in_world[2, 3]

            # Append the column vector to the robot_position array
            robot_position.append([robot_x, robot_y, robot_z])

            if aptag == self.closest_aptag:
                # no average for theta jst take the one of the closest aptag
                rotation_matrix = t_robot_in_world[:3, :3]

        # Convert the robot_position list to a NumPy array
        robot_position_array = np.array(robot_position)

        # Compute the mean for each row to get average of position computed from different aptags
        mean_values = np.mean(robot_position_array, axis=0)

        return mean_values, rotation_matrix

    def localize(self):
        # LOCALIZE
        start_time = time.time()
        speed = 3.14 / 8.0
        msg = Twist()
        msg.angular.z = speed

        # CHECK IF THERE ARE ANY APRILTAGS
        if not self.aptags_detected:
            # rotate until you find one
            while time.time() - start_time < self.time_out:
                # self.get_logger().info('no aptags detected will start looking for one')
                # change this to VECTOR FIELD HISTOGRAM exploration
                # TODO: uncomment
                self.vel_pub.publish(msg)
                if self.aptags_detected_inside_callback:
                    # STOP
                    msg.angular.z = 0.0
                    # TODO: uncomment
                    self.vel_pub.publish(msg)
                    # localize
                    self.get_tf_info = True
                    print("published_pose)))))))))))))))))))" )
                    if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                        # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                        robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                        self.publish_pose(robot_pose_aptags, rotation_matrix)
                        print("published_pose")
                        self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
                                        'base_link', 'map')
                        self.successfully_localized = True

                        return

                    else:
                        if not self.transform_aptag_in_cam_dict:
                            self.get_logger().info('NO apriltags detected')
                        if not self.transform_aptag_in_world_dict:
                            self.get_logger().info('NO transform_aptag_in_world_dict')
                        # if self.cam_to_base_link is None:
                        #     self.get_logger().info('NO transformation cam_to_base_link')
        else:
            # localize
            self.get_tf_info = True
            print("published_pose)))))))))))))))))))" )
            if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                self.publish_pose(robot_pose_aptags, rotation_matrix)
                print("published_pose")
                self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
                                'base_link', 'map')
                self.successfully_localized = True

                return

            else:
                if not self.transform_aptag_in_cam_dict:
                    self.get_logger().info('NO apriltags detected')
                if not self.transform_aptag_in_world_dict:
                    self.get_logger().info('NO transform_aptag_in_world_dict')
                # if self.cam_to_base_link is None:
                #     self.get_logger().info('NO transformation cam_to_base_link')

    ##### Action Server #####

    def goal_callback(self, goal_request):
        # You can add logic here to decide whether to accept or reject the goal.
        # For example, you might check if the goal is valid or if the robot is ready.

        # If you want to accept the goal, return GoalResponse.ACCEPT
        # If you want to reject the goal, return GoalResponse.REJECT
        self.get_logger().info(f'ACCEPTED navigation goal')
        return GoalResponse.ACCEPT

    def send_goal_nav2(self, pose):
        self.get_logger().info(f'Sending navigation goal')
        self.nav2_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        while not self.nav2_to_goal_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose action client not available, waiting...')

        nav_goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose = pose

        self.get_logger().info('Navigating to goal: ' + str(nav_goal.pose.pose.position.x) + ' ' +
                               str(nav_goal.pose.pose.position.y) + '...')

        # Send the goal to the action client
        send_goal_future = self.nav2_to_goal_client.send_goal_async(nav_goal, self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            # self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
            #            str(pose.pose.position.y) + ' was rejected!')
            self.get_logger().info('Goal was rejected!')

        self.result_future = goal_handle.get_result_async()

        while not self.is_nav_complete():
            self.get_logger().info('Goal accepted. Trying to reach goal')

    def feedback_callback(self, msg):
        # self.get_logger().info('Received action feedback message')
        self.feedback = msg.feedback
        return

    def is_nav_complete(self):
        # Check if the result future has a result (completed) or is None (timed out or canceled)
        if not self.result_future:
            self.get_logger().info('Goal failed')
            # task was cancelled or completed
            self.successfully_navigating = False
            return True

        # Spin until the result future is complete or a timeout is reached
        rclpy.spin_until_future_complete(self, self.result_future)  # , timeout_sec=100)

        # Extract the status from the result
        status = self.result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            self.successfully_navigating = True
            return True
        else:
            self.get_logger().info('Goal failed with status code: {0}'.format(status))
            self.successfully_navigating = False
            return True

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Perform the navigation and localization logic here.

        ### debugging
        self.max_weight = 0.0

        if (self.max_weight >= 0.001):
            self.get_logger().info('Robot is not lost; continuing without localizing')
        else:
            self.get_logger().info('Robot is lost; Localizing')
            self.localize()
            self.get_logger().info('Robot Localized')

        # send goal to nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = goal_handle.request.pose.pose.position
        goal_pose.pose.orientation = goal_handle.request.pose.pose.orientation
        # You can set the goal state to succeeded or aborted based on your logic.

        self.get_logger().info('Sending goal')
        # self.get_logger().info('Debugging localization so robot shouldn't move commnet this if the line below is uncommented')
        self.send_goal_nav2(goal_pose)

        result = NavigateToPose.Result()

        # Assuming you successfully navigated and localized, set the goal state to succeeded or abort since the code is already in executing state
        if self.successfully_navigating:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        # If you want to set the goal state to aborted in case of an error, use:
        # goal_handle.abort(result)
        self.get_logger().info('Goal Executed...')

        return result


def main(args=None):
    rclpy.init(args=args)

    nav_action_server = MoveToGoalwithLocalizationActionServer()

    exe = rclpy.executors.MultiThreadedExecutor()
    exe.add_node(nav_action_server)
    while True:
        exe.spin_once()


if __name__ == '__main__':
    main()
