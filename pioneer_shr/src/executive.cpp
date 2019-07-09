#include "ros/ros.h"
#include "pioneer_shr_msg/Action_Approach_Person.h"
#include "pioneer_shr_msg/Action_Run_Script.h"
#include <cstdlib>

class Executive {
public:
    Executive(const char* ros_work_space) : ros_work_space(ros_work_space) {
        n = ros::NodeHandle("~");
    }

	int run(){
            /*ros::ServiceClient approach_person_client =
             * n.serviceClient<pioneer_shr_msg::Action_Approach_Person>("/approach_person_service/Action_Approach_Person");*/
            // pioneer_shr_msg::Action_Approach_Person approach_person_srv;
            // if (approach_person_client.call(approach_person_srv))
            //{
            ////ROS_INFO_STREAM("Result: " << srv.response.success ? "Success" :
            ///"Fail");
            // if (approach_person_srv.response.success)
            // ROS_INFO_STREAM("Result: "
            //<< "Success");
            // else
            // ROS_INFO_STREAM("Result: "
            //<< "Fail");

            //}
            // else
            //{
            // ROS_ERROR("Failed to call service Action_Approach_Person");
            // return 1;
            //}

            ros::ServiceClient run_script_client =
                    n.serviceClient<pioneer_shr_msg::Action_Run_Script>(
                            "/run_script_service/Action_Run_Script");

            std::string resourcePath = ros_work_space;

            //if (mediaType == "audio")
                //resourcePath += "/src/pioneer_shr/resource/playAudio.sh";
            resourcePath += "/src/pioneer_shr/resource/phoneApp/call.py";

            pioneer_shr_msg::Action_Run_Script run_script_srv;
            run_script_srv.request.script_file_name =
                    "python " + resourcePath + " call_msg_medical.xml";

            if (run_script_client.call(run_script_srv)) {
                if (run_script_srv.response.success)
                    ROS_INFO_STREAM("Result: "
                            << "Success");
                else
                    ROS_INFO_STREAM("Result: "
                            << "Fail");

            } else {
                ROS_ERROR("Failed to call service Action_Approach_Person");
                return 1;
            }

            return 0;
        }

private:
    ros::NodeHandle n;
	const std::string ros_work_space;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "executive");

  const char* ros_work_space = std::getenv("ROS_WORKSPACE");
  if (ros_work_space == 0) {
      ROS_ERROR_STREAM("ROS_WORKSPACE environment variable not found!");
      return 1;
  }

  Executive executive(ros_work_space);

  if(executive.run())
		  return 1;

  return 0;
}
