#include <ros/ros.h>
#include "httpRequest.hpp"
#include "pioneer_shr_msg/Action_Monitoring_DB.h"

class DBMonitor {
public:
    DBMonitor() {
        nh = ros::NodeHandle("~");

        ros::ServiceServer service = nh.advertiseService(
                "Action_Monitoring_DB", &DBMonitor::getDBInfo, this);

        ROS_INFO_STREAM("Running action service monitoring DB");

        ros::spin();
    }

private:
    bool getDBInfo(pioneer_shr_msg::Action_Monitoring_DB::Request& req,
            pioneer_shr_msg::Action_Monitoring_DB::Response& res) {

        res.sensorMsg = HttpRequest::getSensorData();

        ROS_INFO_STREAM(
                "Action service monitoring DB finished: ");
        ROS_INFO("Keep Running service monitoring DB");

        return true;
    }

    ros::NodeHandle nh;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "db_monitor_service");

  DBMonitor action;
}
