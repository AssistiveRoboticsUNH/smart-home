#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include "httpRequest.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AutoNavigation {
public:
    void navigateTo(const geometry_msgs::Pose& goalPose) const {
        // tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        // wait for the action server to come up
        while (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        // we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose = goalPose;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved to goal");
        else
            ROS_INFO("The base failed to move for some reason");
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  AutoNavigation autonav;

  geometry_msgs::Pose goalPose;
  goalPose.position.x = 1.72;
  goalPose.position.y = 1.52;
  goalPose.orientation.z = 0.73;
  goalPose.orientation.w = 0.67;

  HttpRequest httpReq;

  //we don't need this if there is a nodehandle be created
  ros::Time::init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();
  ros::spinOnce();

  bool doorOpen = false;

  while (ros::ok()) {
      if (doorOpen)
          break;

      ros::spinOnce();

      if (httpReq.isOpen()) {
          autonav.navigateTo(goalPose);
		  doorOpen = true;
      }
  }

  return 0;
}
