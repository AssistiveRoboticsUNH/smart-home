#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "preDefinedPose.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const double BASE_DIAMETER = .32;
const int LOOP_RATE = 50;

class Skills {
public:
    void navigateTo(const PreDefinedPose& pdPose) const {
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

        goal.target_pose.pose = pdPose.getPose();

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        std::string msg;

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		    msg = "Hooray, the base moved to " + pdPose.getName();
        else
		    msg = "The base failed to reach " + pdPose.getName() + " for some reason";

        ROS_INFO("%s\n",msg.c_str());
    }

    void cancelMoveBaseGoal() const {
        // tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        // wait for the action server to come up
        while (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        ac.cancelAllGoals();

        ROS_INFO("current goal has been canceled");
    }

    void playAudio() { play("audio"); }

    void playVideo() { play("video"); }

    void rotate360(ros::Publisher& pub_vel) {
        double vl, vr;
        vl = -0.05;
        vr = 0.05;
        vel_from_wheels(vl, vr, 14, pub_vel);
    }

    void fullStop(ros::Publisher& pub_vel) {
		geometry_msgs::Twist curVel;
        curVel.linear.x = 0;
        curVel.angular.z = 0;
        ros::Rate loop_rate(LOOP_RATE);
        loop_rate.sleep();
        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(0.5);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
        while (ros::Time::now() < endTime && ros::ok()) {
            pub_vel.publish(curVel);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void play(std::string mediaType) {
        const char* ros_work_space = std::getenv("ROS_WORKSPACE");
        if (ros_work_space == 0) {
            std::cout << "ROS_WORKSPACE environment variable not found!"
                      << std::endl;
            return;
        }
        std::string resourcePath = ros_work_space;

        if (mediaType == "audio")
            resourcePath += "/src/pioneer_shr/resource/playAudio.sh";
        else if (mediaType == "video")
            resourcePath += "/src/pioneer_shr/resource/playVideo.sh";
        else
            ROS_ERROR("wrong mediao type !");

        std::system(resourcePath.c_str());
    }

    void vel_from_wheels(double vl, double vr, double sec, ros::Publisher& pub_vel) {
        double lin_vel = 0.5 * (vl + vr);
        double ang_vel = (vr - vl) / BASE_DIAMETER;
		geometry_msgs::Twist curVel;
        curVel.linear.x = lin_vel;
        curVel.angular.z = ang_vel;
        ros::Rate loop_rate(LOOP_RATE);
        loop_rate.sleep();
        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(sec);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
        while (ros::Time::now() < endTime && ros::ok()) {
            pub_vel.publish(curVel);
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("rotate 360...");
        }
    }
};
