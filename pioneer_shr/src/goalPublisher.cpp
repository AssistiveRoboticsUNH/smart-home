#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include "httpRequest.hpp"
#include <string>

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

    void playAudio() {
        const char* ros_work_space = std::getenv("ROS_WORKSPACE");
        if (ros_work_space == 0) {
            std::cout << "ROS_WORKSPACE environment variable not found!"
                      << std::endl;
			return;
        }
        std::string resourcePath = ros_work_space;
        resourcePath += "/src/pioneer_shr/resource/playAudio.sh";
        std::system(resourcePath.c_str());
    }
};



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle nh("~");

  float x,y,z,w;

  nh.getParam("goalPosePositionX", x);
  nh.getParam("goalPosePositionY", y);
  nh.getParam("goalPoseOrientationZ", z);
  nh.getParam("goalPoseOrientationW", w);

  std::cout << "load in goalPosePositionX: " << x << std::endl;
  std::cout << "load in goalPosePositionY: " << y << std::endl;
  std::cout << "load in goalPoseOrientationZ: " << z << std::endl;
  std::cout << "load in goalPoseOrientationW: " << w << std::endl;

  AutoNavigation autonav;

  geometry_msgs::Pose goalPose;
  goalPose.position.x = x;
  goalPose.position.y = y;
  goalPose.orientation.z = z;
  goalPose.orientation.w = w;

  //goalPose.position.x = -0.6109;
  //goalPose.position.y = -1.7612;
  //goalPose.orientation.z = 0.9947;
  //goalPose.orientation.w = -0.1019;

  HttpRequest httpReq;

  //we don't need this if there is a nodehandle be created
  //ros::Time::init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();
  ros::spinOnce();

  /*bool doorOpen = false;*/

  //while (ros::ok()) {
	  //if (doorOpen)
		  //break;

	  //ros::spinOnce();

	  //if (httpReq.isOpen()) {
		  //autonav.navigateTo(goalPose);
		  //autonav.playAudio();
		  //doorOpen = true;
	  //}
  /*}*/

  autonav.navigateTo(goalPose);
  autonav.playAudio();

  return 0;
}
