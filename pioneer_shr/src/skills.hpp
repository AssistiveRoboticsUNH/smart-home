#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include "preDefinedPose.hpp"
#include <system_error>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const double BASE_DIAMETER = .32;
const int LOOP_RATE = 50;

class Skills {
public:
    Skills(ros::NodeHandle& nh) : nh(nh) {}

    Skills(ros::NodeHandle& nh,
            std::vector<PreDefinedPose>& landMarks,
            std::string& cameraFrameId)
            : nh(nh),
              faceFound(false),
              stopDetect(false),
              recognizedTarget(false),
              stopRecognize(false),
              landMarks(landMarks),
              cameraFrameId(cameraFrameId) {
        pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }

    void navigateTo(const PreDefinedPose& pdPose, const std::string& frameID = "map") const {
        // tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        // wait for the action server to come up
        while (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        // we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = frameID;
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose = pdPose.getPose();

        ROS_INFO_STREAM("Moving to "<< pdPose.getName()<<"...");
        //ROS_INFO_STREAM("pose "<< goal.target_pose);
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

    bool runScript(const std::string& scriptFile) const {
        try {
            std::system(scriptFile.c_str());
			return true;
        } catch (const std::system_error& e) {
            ROS_ERROR_STREAM("Caught system_error with code "
                    << e.code() << " meaning " << e.what());
			return false;
        }
    }

    void rotate360() const {
        double vl, vr;
        vl = -0.05;
        vr = 0.05;
        vel_from_wheels(vl, vr, 25);
    }

    void fullStop() const {
		geometry_msgs::Twist curVel;
        curVel.linear.x = 0;
        curVel.angular.z = 0;

        ros::Rate loop_rate(10);
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

    void cruiseHouse() {
        if (!faceFound)
            rotate360();

        for (auto p : landMarks) {
            if (!faceFound)
                navigateTo(p);
            if (!faceFound)
                rotate360();
            if (faceFound)
                break;
        }

        fullStop();

		if (!faceFound){
            ROS_INFO("No face detected in house!");
			stopDetect = true;
		}

    }

    void approachCurrentFace() {
        navigateTo(PreDefinedPose(detectedFacePose, "Current Face Pose"));

        fullStop();

        if (!recognizedTarget) {
            ROS_INFO("Not the right person!");
        }

        stopRecognize = true;
    }

    void detectFace() {
        ros::Subscriber sub_face_detector =
                nh.subscribe("/face_detector/people_tracker_measurements_array",
                        1,
                        &Skills::face_detector_callback,
                        this);

        ros::Rate loop_rate(10);
        loop_rate.sleep();

        ros::spinOnce();

        ROS_INFO("detecting face....");
        while (ros::ok() && !faceFound && !stopDetect) {
            ros::spinOnce();
        }
        ROS_INFO("stop detecting face");

		// whenever detected a face, stop moving
		cancelMoveBaseGoal(); 
		fullStop();
    }

    void recognizeFace() {
        ros::Subscriber sub_face_recognizer =
                nh.subscribe("/face_recognizer",
                        1,
                        &Skills::face_recognizer_callback,
                        this);

        ros::Rate loop_rate(10);
        loop_rate.sleep();

        ros::spinOnce();

        ROS_INFO("recognizing face....");
        while (ros::ok() && !recognizedTarget && !stopRecognize) {
            ros::spinOnce();
        }
        ROS_INFO("stop recognize face");

		// whenever recognize a face, stop moving
		cancelMoveBaseGoal(); 
		fullStop();
    }

    bool faceInHouse() const { return faceFound; }

    bool targetInHouse() const { return recognizedTarget; }

private:
    

    void vel_from_wheels(double vl, double vr, double sec) const {
        double lin_vel = 0.5 * (vl + vr);
        double ang_vel = (vr - vl) / BASE_DIAMETER;

		geometry_msgs::Twist curVel;
        curVel.linear.x = lin_vel;
        curVel.angular.z = ang_vel;

        ros::Rate loop_rate(10);
        loop_rate.sleep();

        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(sec);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        ROS_INFO_STREAM("rotate 360...");
        while (ros::Time::now() < endTime && ros::ok() && !faceFound) {
            pub_vel.publish(curVel);
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO_STREAM("finsihed rotate 360!");
    }

    void face_detector_callback(const people_msgs::PositionMeasurementArray& msg) {
		// we can add a filter here
		// filt all poses that are close to one of the exist no-target face
        ROS_INFO("Detect a new face!!!");

        geometry_msgs::Point msgPoint = msg.people[0].pos;
        detectedFacePose.position.x = msgPoint.x;
        detectedFacePose.position.y = msgPoint.y;
        detectedFacePose.position.z = msgPoint.z;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf2_listener(tfBuffer);
        geometry_msgs::TransformStamped camera_to_base_link;
		// might need set up same frame id for real world and gazebo here
        try {
            camera_to_base_link = tfBuffer.lookupTransform(
                    "map", cameraFrameId, ros::Time(0),ros::Duration(3.0));

            tf2::doTransform(
                    detectedFacePose, detectedFacePose, camera_to_base_link);

        } catch (tf2::TransformException& ex) {
            ROS_WARN("Could NOT transform camera_link to base_link: %s", ex.what());
        }


        detectedFacePose.position.z = 0;
        detectedFacePose.orientation.x = 0;
        detectedFacePose.orientation.y = 0;
        detectedFacePose.orientation.z = landMarks[0].getPose().orientation.z;
        detectedFacePose.orientation.w = landMarks[0].getPose().orientation.w;

        faceFound = true;
    }

    void face_recognizer_callback(const std_msgs::String& msg) {
		// we can add a filter here
		// filt all poses that are close to one of the exist no-target face
        ROS_INFO_STREAM(msg);
        recognizedTarget = true;
    }

    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    bool faceFound;
    bool stopDetect;
    bool recognizedTarget;
    bool stopRecognize;
    geometry_msgs::Pose detectedFacePose;
    std::vector<PreDefinedPose> landMarks;
	std::string cameraFrameId;
};
