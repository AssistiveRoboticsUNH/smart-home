#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include "skills.hpp"
#include "pioneer_shr_msg/Action_Run_Script.h"

class ActionRunScript {
public:
    ActionRunScript() {
        nh = ros::NodeHandle("~");

        ros::ServiceServer service =
                nh.advertiseService("Action_Run_Script",
                        &ActionRunScript::doAction,
                        this);

        ROS_INFO_STREAM("Running action service run scipt");

        ros::spin();
    }

private:
    bool doAction(pioneer_shr_msg::Action_Run_Script::Request& req,
            pioneer_shr_msg::Action_Run_Script::Response& res) {
        Skills skills(nh);

        res.success = skills.runScript(req.script_file_name);

        ROS_INFO_STREAM(
                "Action service run script finished: " << req.script_file_name);
        ROS_INFO("Keep Running action service run script");
        return true;
    }

    ros::NodeHandle nh;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "run_script_service");
  ActionRunScript action;
}
