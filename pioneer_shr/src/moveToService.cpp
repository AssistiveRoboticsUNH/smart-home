#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <unordered_map>
#include "skills.hpp"
#include "pioneer_shr_msg/Action_Move_To.h"

class ActionMoveTo {
public:
    ActionMoveTo() {
        nh = ros::NodeHandle("~");

        float x1, y1, z1, w1, x2, y2, z2, w2, x3, y3, z3, w3;

        std::string p1name, p2name, p3name;

        nh.getParam("predefined1PositionX", x1);
        nh.getParam("predefined1PositionY", y1);
        nh.getParam("predefined1OrientationZ", z1);
        nh.getParam("predefined1OrientationW", w1);
        nh.getParam("predefined1Name", p1name);

        nh.getParam("predefined2PositionX", x2);
        nh.getParam("predefined2PositionY", y2);
        nh.getParam("predefined2OrientationZ", z2);
        nh.getParam("predefined2OrientationW", w2);
        nh.getParam("predefined2Name", p2name);

        nh.getParam("predefined3PositionX", x3);
        nh.getParam("predefined3PositionY", y3);
        nh.getParam("predefined3OrientationZ", z3);
        nh.getParam("predefined3OrientationW", w3);
        nh.getParam("predefined3Name", p3name);


        ROS_INFO_STREAM("load in predefined1PositionX: " << x1);
        ROS_INFO_STREAM("load in predefined1PositionY: " << y1);
        ROS_INFO_STREAM("load in predefined1OrientationZ: " << z1);
        ROS_INFO_STREAM("load in predefined1OrientationW: " << w1);
        ROS_INFO_STREAM("load in predefined1Name: " + p1name);

        ROS_INFO_STREAM("load in predefined2PositionX: " << x2);
        ROS_INFO_STREAM("load in predefined2PositionY: " << y2);
        ROS_INFO_STREAM("load in predefined2OrientationZ: " << z2);
        ROS_INFO_STREAM("load in predefined2OrientationW: " << w2);
        ROS_INFO_STREAM("load in predefined2Name: " + p2name);


        ROS_INFO_STREAM("load in predefined3PositionX: " << x3);
        ROS_INFO_STREAM("load in predefined3PositionY: " << y3);
        ROS_INFO_STREAM("load in predefined3OrientationZ: " << z3);
        ROS_INFO_STREAM("load in predefined3OrientationW: " << w3);
        ROS_INFO_STREAM("load in predefined3Name: " + p3name);

        geometry_msgs::Pose pose1, pose2, pose3;
        pose1.position.x = x1;
        pose1.position.y = y1;
        pose1.orientation.z = z1;
        pose1.orientation.w = w1;

        pose2.position.x = x2;
        pose2.position.y = y2;
        pose2.orientation.z = z2;
        pose2.orientation.w = w2;

        pose3.position.x = x3;
        pose3.position.y = y3;
        pose3.orientation.z = z3;
        pose3.orientation.w = w3;

        std::shared_ptr<PreDefinedPose> pdpose1Ptr =
                std::make_shared<PreDefinedPose>(pose1, p1name);
        std::shared_ptr<PreDefinedPose> pdpose2Ptr=
                std::make_shared<PreDefinedPose>(pose2, p2name);
        std::shared_ptr<PreDefinedPose> pdpose3Ptr=
                std::make_shared<PreDefinedPose>(pose3, p3name);

        availablePlaces[p1name] = pdpose1Ptr;
        availablePlaces[p2name] = pdpose2Ptr;
        availablePlaces[p3name] = pdpose3Ptr;

        ros::ServiceServer service =
                nh.advertiseService("Action_Move_To",
                        &ActionMoveTo::doAction,
                        this);

        ROS_INFO_STREAM("Running action service run scipt");

        ros::spin();
    }

private:
    bool doAction(pioneer_shr_msg::Action_Move_To::Request& req,
            pioneer_shr_msg::Action_Move_To::Response& res) {
        Skills skills(nh);

        skills.navigateTo(*availablePlaces[req.destination_name]);

		res.success = true;

        ROS_INFO_STREAM(
                "Action service move to finished: " << req.destination_name);
        ROS_INFO("Keep Running action service move to");
        return true;
    }

    ros::NodeHandle nh;
    std::unordered_map<std::string, std::shared_ptr<PreDefinedPose>>
            availablePlaces;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "move_to_service");
  ActionMoveTo action;
}
