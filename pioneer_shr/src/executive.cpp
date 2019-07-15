#include "ros/ros.h"
#include "pioneer_shr_msg/Action_Approach_Person.h"
#include "pioneer_shr_msg/Action_Run_Script.h"
#include "pioneer_shr_msg/Action_Monitoring_DB.h"
#include <cstdlib>

class Executive {
public:
    Executive(const char* ros_work_space)
            : ros_work_space(ros_work_space), medicineTaken(false) {
        n = ros::NodeHandle("~");
    }

    int run() {
        ros::Rate loop_rate(0.5);
        loop_rate.sleep();

        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(10);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        pioneer_shr_msg::SmartSensor curSensorInfo;

        ROS_INFO_STREAM("Start DB monitoring...");
        ROS_INFO_STREAM(beginTime);
        ROS_INFO_STREAM(endTime);

        while (ros::Time::now() < endTime && ros::ok()) {
            if (medicineTaken)
                continue;
            curSensorInfo = monitoringDB();
            medicineTaken = curSensorInfo.motion1_is_on;
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Monitoring DB...");
        }
        ROS_INFO_STREAM(ros::Time::now());

        if (medicineTaken) {
            ROS_INFO_STREAM("Medicine is taken in the moring!");
            return 0;
        }

        ROS_INFO_STREAM("Medicine is not taken, approaching person...");

        if (approachPerson()) {

            //playMediaWithSciptFile("playMedicalNotify.sh");
			playAudioWithMessageFile("medicine_reminder.txt");

            beginTime = ros::Time::now();

            secondsIWantToSendMessagesFor = ros::Duration(20);

            endTime = beginTime + secondsIWantToSendMessagesFor;

            ROS_INFO_STREAM("Advice given, start DB monitoring again...");

            ROS_INFO_STREAM(beginTime);
            ROS_INFO_STREAM(endTime);

            while (ros::Time::now() < endTime && ros::ok()) {
                if (medicineTaken)
                    continue;
                curSensorInfo = monitoringDB();
                medicineTaken = curSensorInfo.motion1_is_on;
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO_STREAM("Monitoring DB...");
            }

            ROS_INFO_STREAM(ros::Time::now());

            if (medicineTaken) {
                ROS_INFO_STREAM("Medicine is taken after notify!");
                return 0;
            } else {
                phoneCallWithMessageFile("call_msg_medical.xml");
                ROS_INFO_STREAM(
                        "Medicine still not be take, hand over to caregiver!");
            }

            return 0;
        }

        phoneCallWithMessageFile("call_msg_alex_not_in_house.xml");
        ROS_INFO_STREAM("People is missing, hand over to caregiver!");
        return 0;
    }

    int runP2() {
        playAudioWithMessageFile("medicine_reminder.txt");
        return 0;
    }

private:
    template <class ServiceType>
    void callService(ros::ServiceClient& client,
            ServiceType& service,
            const std::string& serviceName) const {
        if (!client.call(service))
            ROS_ERROR_STREAM("Failed to call service " << serviceName);
    }

    bool phoneCallWithMessageFile(const std::string& msgFile) {
        ros::ServiceClient run_script_client =
                n.serviceClient<pioneer_shr_msg::Action_Run_Script>(
                        "/run_script_service/Action_Run_Script");

        std::string resourcePath = ros_work_space;

        resourcePath += "/src/pioneer_shr/resource/phoneApp/call.py";

        pioneer_shr_msg::Action_Run_Script run_script_srv;
        run_script_srv.request.script_file_name =
                "python " + resourcePath + " "+msgFile;

        callService<pioneer_shr_msg::Action_Run_Script>(
                run_script_client, run_script_srv, "Action_Run_Script");

		return run_script_srv.response.success;
    }

    bool playAudioWithMessageFile(const std::string& msgFile) {
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

    bool playMediaWithSciptFile(const std::string& scriptFile) {
        ros::ServiceClient run_script_client =
                n.serviceClient<pioneer_shr_msg::Action_Run_Script>(
                        "/run_script_service/Action_Run_Script");

        std::string resourcePath = ros_work_space;

        resourcePath += "/src/pioneer_shr/resource/" + scriptFile;

        pioneer_shr_msg::Action_Run_Script run_script_srv;

        run_script_srv.request.script_file_name = resourcePath;

        callService<pioneer_shr_msg::Action_Run_Script>(
                run_script_client, run_script_srv, "Action_Run_Script");

		return run_script_srv.response.success;
    }

    bool approachPerson() {
        ros::ServiceClient approach_person_client =
                n.serviceClient<pioneer_shr_msg::Action_Approach_Person>(
                        "/approach_person_service/Action_Approach_Person");
        pioneer_shr_msg::Action_Approach_Person approach_person_srv;

        callService<pioneer_shr_msg::Action_Approach_Person>(
                approach_person_client,
                approach_person_srv,
                "Action_Approach_Person");

        return approach_person_srv.response.success;
    }

    pioneer_shr_msg::SmartSensor monitoringDB() {
        ros::ServiceClient db_monitor_client =
                n.serviceClient<pioneer_shr_msg::Action_Monitoring_DB>(
                        "/db_monitor_service/Action_Monitoring_DB");
        pioneer_shr_msg::Action_Monitoring_DB monitoring_db_srv;

        callService<pioneer_shr_msg::Action_Monitoring_DB>(db_monitor_client,
                monitoring_db_srv,
                "Action_Monitoring_DB");

        return monitoring_db_srv.response.sensorMsg;
    }

    ros::NodeHandle n;
	const std::string ros_work_space;
	bool medicineTaken;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "executive");

  const char* ros_work_space = std::getenv("ROS_WORKSPACE");
  if (ros_work_space == 0) {
      ROS_ERROR_STREAM("ROS_WORKSPACE environment variable not found!");
      return 1;
  }

  Executive executive(ros_work_space);

  if(executive.runP2())
		  return 1;

  return 0;
}
