#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include "skills.hpp"
#include "pioneer_shr_msg/Action_Approach_Person.h"

class ActionApproachPerson {
public:
    ActionApproachPerson() {
        nh = ros::NodeHandle("~");

        float x1, y1, z1, w1, x2, y2, z2, w2;

        std::string p1name, p2name, tname;

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

        nh.getParam("targetPersonName", tname);

        nh.getParam("cameraFrameId", cameraFrameId);

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

        ROS_INFO_STREAM("load in target person name: " + tname);

        geometry_msgs::Pose pose1, pose2;
        pose1.position.x = x1;
        pose1.position.y = y1;
        pose1.orientation.z = z1;
        pose1.orientation.w = w1;

        pose2.position.x = x2;
        pose2.position.y = y2;
        pose2.orientation.z = z2;
        pose2.orientation.w = w2;

        PreDefinedPose pdpose1(pose1, p1name);
        PreDefinedPose pdpose2(pose2, p2name);

        landMarks.push_back(pdpose1);
        landMarks.push_back(pdpose2);

        ros::ServiceServer service =
                nh.advertiseService("Action_Approach_Person",
                        &ActionApproachPerson::doAction,
                        this);

        ROS_INFO("Running action service approach person");

        ros::spin();
    }

private:
    bool doAction(pioneer_shr_msg::Action_Approach_Person::Request& req,
            pioneer_shr_msg::Action_Approach_Person::Response& res) {
        Skills skills(nh, landMarks, cameraFrameId);

		std::thread faceDetect(&Skills::detectFace, &skills);
        std::thread cruise(&Skills::cruiseHouse, &skills);

		faceDetect.join();
        cruise.join();

        if (!skills.faceInHouse()) {
            res.success = false;
            //ROS_INFO_STREAM("call caregiver");
            ROS_INFO_STREAM("Action service approach person finished!");
            ROS_INFO("Keep Running action service approach person");
            return true;
        }


		std::thread faceRecognize(&Skills::recognizeFace, &skills);
        std::thread approachFace(&Skills::approachCurrentFace, &skills);

		faceRecognize.join();
		approachFace.join();

        if (!skills.targetInHouse()) {
            res.success = false;
            //ROS_INFO_STREAM("call caregiver");
            ROS_INFO_STREAM("Action service approach person finished!");
            ROS_INFO("Keep Running action service approach person");
            return true;
        }

        res.success = true;
        ROS_INFO_STREAM("Action service approach person finished!");
        ROS_INFO("Keep Running action service approach person");
        return true;
    }

    std::vector<PreDefinedPose> landMarks;
    ros::NodeHandle nh;
	std::string cameraFrameId;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "approach_person_service");
  ActionApproachPerson actionApproachPerson;
}
