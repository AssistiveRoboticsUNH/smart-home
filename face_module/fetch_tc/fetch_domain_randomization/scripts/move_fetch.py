#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from fetch_domain_randomization.srv import FetchMove, FetchMoveResponse


import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        result = self.client.wait_for_result()
        
        return result
        
        
class GripperActionClient(object):
    def __init__(self):
        
        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")
        
    def move_gripper(self, gripper_x, max_effort, timeout=5.0):
        
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x
        
        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))
        
        return result


class MoveFetch(object):
    
    def __init__(self):
        
        
        rospy.loginfo("In Move Fetch Calss init...")
        
        fetch_move_service = rospy.Service('/fetch_move_service', FetchMove , self.move_callback)
        
        # Init Torso Action
        self.torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
        
        # Gripper Action
        self.gripper_action = GripperActionClient()
        
        # Point Head action
        self.head_action = PointHeadClient()
        
        # MOVEIT
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")
        
        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")
        
        self.scene = moveit_commander.PlanningSceneInterface()  
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")
        
        
        rospy.logwarn("self.group TYPE==>"+str(type(self.group)))
        
        rospy.loginfo("FETCH ready to move!")

    def move_callback(self, request):
        
        move_response = FetchMoveResponse()
        
        pose_requested = request.pose
        joints_array_requested = request.joints_array
        movement_type_requested = request.movement_type.data
        
        if movement_type_requested == "TCP":
            move_response.success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            move_response.success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            move_response.success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            XYZ = [joints_array_requested[0],joints_array_requested[1],joints_array_requested[2]]
            move_response.success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]
            
            move_response.success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>"+str(movement_type_requested))
        
        return move_response.success
        
    def ee_traj(self, pose):
        
        pose_frame = self.group.get_pose_reference_frame()
        rospy.logerr("pose_frame==>"+str(pose_frame))
        
        
        if pose_frame != "/world":
            new_reference_frame = "/world"
            self.group.set_pose_reference_frame(new_reference_frame)
        
            pose_frame = self.group.get_pose_reference_frame()
            rospy.logerr("pose_frame==>"+str(pose_frame))
        else:
            rospy.logwarn("pose_frame OK, ==>"+str(pose_frame))
        
        
        
        self.group.set_pose_target(pose)
        
        result = self.execute_trajectory()
        
        return result
        
    def joint_traj(self, positions_array):
        
        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result =  self.execute_trajectory()
        
        return result
        
    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.torso_action.move_to([torso_height, ])
        
        return result
        
    
    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)
        
        return result
    
    def move_head_point(self, XYZ, frame="world"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)
        
        return result
        
    def execute_trajectory(self):
        
        self.plan = self.group.plan()
        result = self.group.go(wait=True)
        
        return result

    def ee_pose(self):
        
        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose
        
    def ee_rpy(self, request):
        
        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy
        

if __name__ == '__main__':
    rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)
    
    move_fetch_obj = MoveFetch()
    rospy.spin()