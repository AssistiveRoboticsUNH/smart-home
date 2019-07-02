#include "ros/ros.h"
#include "pioneer_shr_msg/Action_Approach_Person.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "approach_person_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pioneer_shr_msg::Action_Approach_Person>("/approach_person_service/Action_Approach_Person");
  pioneer_shr_msg::Action_Approach_Person srv;
  if (client.call(srv))
  {
      //ROS_INFO_STREAM("Result: " << srv.response.success ? "Success" : "Fail");
      if (srv.response.success)
          ROS_INFO_STREAM("Result: "
                  << "Success");
      else
          ROS_INFO_STREAM("Result: "
                  << "Fail");

  }
  else
  {
    ROS_ERROR("Failed to call service Action_Approach_Person");
    return 1;
  }

  return 0;
}
