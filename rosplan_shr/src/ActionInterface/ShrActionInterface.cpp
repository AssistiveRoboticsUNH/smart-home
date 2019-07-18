#include "rosplan_action_interface/ShrActionInterface.h"

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

template <class ServiceType>
void ShrActionInterface::callService(ros::ServiceClient &client,
                                     ServiceType &service,
                                     const std::string &serviceName) const {
  if (!client.call(service))
    ROS_ERROR_STREAM("Failed to call service " << serviceName);
}

bool ShrActionInterface::moveTo(const std::string &destinationName){

    ros::ServiceClient move_to_client =
                n.serviceClient<pioneer_shr_msg::Action_Move_To>(
                        "/move_to_service/Action_Move_To");

        pioneer_shr_msg::Action_Move_To move_to_srv;

		move_to_srv.request.destination_name = destinationName;

        callService<pioneer_shr_msg::Action_Move_To>(move_to_client,
                move_to_srv,
                "Action_Move_To");

        return move_to_srv.response.success;
}

bool ShrActionInterface::playAudioWithMessageFile(const std::string &msgFile) {

  ros::ServiceClient run_script_client =
      n.serviceClient<pioneer_shr_msg::Action_Run_Script>(
          "/run_script_service/Action_Run_Script");

  std::string resourcePath = ros_work_space;
  resourcePath += "/src/pioneer_shr/resource/";

  pioneer_shr_msg::Action_Run_Script run_script_srv;
  run_script_srv.request.script_file_name =
      "rosrun sound_play say.py < " + resourcePath + msgFile;

  callService<pioneer_shr_msg::Action_Run_Script>(
      run_script_client, run_script_srv, "Action_Run_Script");

  return run_script_srv.response.success;
}

/* constructor */
ShrActionInterface::ShrActionInterface(ros::NodeHandle &nh):ros_work_space(std::getenv("ROS_WORKSPACE")) {
  // perform setup
  n = nh;
}

        /* action dispatch callback */
bool ShrActionInterface::concreteCallback(
    const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  if (msg->name == "moveto_landmark") {
    if (moveTo(msg->parameters[2].value)) {
      ROS_INFO("KCL: (%s) move to landmark (%s) action completing.",
               msg->name.c_str(), msg->parameters[2].value.c_str());
      return true;
    } else {
      ROS_ERROR("KCL: (%s) move to landmark (%s): call action service fail.",
                msg->name.c_str(), msg->parameters[2].value.c_str());
      return false;
    }
  } else if (msg->name == "notifyat") {
    if (playAudioWithMessageFile(msg->parameters[2].value + ".txt")) {
      ROS_INFO("KCL: (%s) notify msg: (%s) at (%s) action completing.",
               msg->name.c_str(), msg->parameters[1].value.c_str(),
               msg->parameters[2].value.c_str());
      return true;
    } else {
      ROS_ERROR(
          "KCL: (%s) notify msg: (%s) at (%s) : call action service fail.",
          msg->name.c_str(), msg->parameters[1].value.c_str(),
          msg->parameters[2].value.c_str());
      return false;
    }
  }

  return true;
}
} // namespace KCL_rosplan

        /*-------------*/
	/* Main method */
	/*-------------*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "rosplan_shr_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::ShrActionInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
