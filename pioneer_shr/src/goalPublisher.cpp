#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include "httpRequest.hpp"
#include <string>
#include "skills.hpp"

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

  std::vector<PreDefinedPose> pdlist;
  std::string cameraTopic = "/camera";
  Skills autonav(nh, pdlist, cameraTopic);

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

  bool doorOpen = false;

  while (ros::ok()) {
      if (doorOpen)
          break;

      ros::spinOnce();

      if (httpReq.isOpen()) {
          autonav.navigateTo(PreDefinedPose(goalPose, "Door"));
          autonav.playAudio();
          doorOpen = true;
      }
  }

  //autonav.navigateTo(goalPose);
  //autonav.playAudio();

  return 0;
}
