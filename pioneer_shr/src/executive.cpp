#include "ros/ros.h"
#include "pioneer_shr_msg/Action_Approach_Person.h"
#include "pioneer_shr_msg/Action_Run_Script.h"
#include "pioneer_shr_msg/Action_Monitoring_DB.h"
#include "pioneer_shr_msg/Action_Move_To.h"
#include "std_srvs/Empty.h"
#include "rosplan_dispatch_msgs/DispatchService.h"
#include <cstdlib>
#include <thread>

class Executive {
public:
    Executive(const char* ros_work_space)
            : ros_work_space(ros_work_space),
              medicineTaken(false),
              isDoorOpen(false),
              isPeopleBackToBed(false),
              isPeopleAtDoor(false) {
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
                phoneCallWithScriptandMessageFile(
                        "call.py", "call_msg_medical.xml");
                ROS_INFO_STREAM(
                        "Medicine still not be take, hand over to caregiver!");
            }

            return 0;
        }

        phoneCallWithScriptandMessageFile(
                "call.py", "call_msg_alex_not_in_house.xml");
        ROS_INFO_STREAM("People is missing, hand over to caregiver!");
        return 0;
    }

    int runP2() {
        ros::Rate loop_rate(10);
        loop_rate.sleep();

        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(15);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        pioneer_shr_msg::SmartSensor curSensorInfo;

        ROS_INFO_STREAM("Start DB monitoring M2...");
        ROS_INFO_STREAM(beginTime);
        ROS_INFO_STREAM(endTime);

        while (ros::Time::now() < endTime && ros::ok() && !isPeopleAtDoor) {
            curSensorInfo = monitoringDB();
            isPeopleAtDoor = curSensorInfo.motion2_is_on;
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Monitoring DB M2...");
        }

        if (!isPeopleAtDoor) {
            ROS_INFO_STREAM("No one approach the door during last night!");
            return 0;
        }

        ROS_INFO_STREAM("some one approach the door, approaching door...");
		std::thread yell(&Executive::playAudioWithMessageFile, this, "alex_wait.txt");
		std::thread move2door(&Executive::moveTo, this, "door");

		yell.join();
		move2door.join();

        // playMediaWithSciptFile("playMedicalNotify.sh");
        playAudioWithMessageFile("midnight_warning.txt");

        beginTime = ros::Time::now();

        secondsIWantToSendMessagesFor = ros::Duration(15);

        endTime = beginTime + secondsIWantToSendMessagesFor;

        ROS_INFO_STREAM("mid night warning given, start DB monitoring on Door "
                        "Sensor...");

        curSensorInfo = monitoringDB();
        isDoorOpen= curSensorInfo.door_is_open;

        while (ros::Time::now() < endTime && ros::ok() && isDoorOpen) {
            curSensorInfo = monitoringDB();
            isDoorOpen = curSensorInfo.door_is_open;
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Monitoring DB on Door Sensor...");
        }


        if (!isDoorOpen) {
            ROS_INFO_STREAM("Door closed after notify!");
			return 0;
        }

        ROS_INFO_STREAM(
                "Door is still open after voice notify, playing video!");
        playMediaWithSciptFile("playVideo.sh");
        phoneCallWithScriptandMessageFile(
                "call.py", "call_msg_leaving_house.xml");

        beginTime = ros::Time::now();

        secondsIWantToSendMessagesFor = ros::Duration(60);

        endTime = beginTime + secondsIWantToSendMessagesFor;

        ROS_INFO_STREAM("video warning given, start DB monitoring on Bed "
                        "Sensor...");

        while (ros::Time::now() < endTime && ros::ok() && !isPeopleBackToBed) {
            curSensorInfo = monitoringDB();
            isPeopleBackToBed = curSensorInfo.motion1_is_on;
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Monitoring DB on Bed Sensor...");
        }

        if (isPeopleBackToBed) {
            ROS_INFO_STREAM("Patient go back to bed...");
            phoneCallWithScriptandMessageFile(
                    "call.py", "call_msg_alarm_off.xml");
            return 0;
        }

        phoneCallWithScriptandMessageFile(
                    "call_emg.py","call_msg_911.xml");
        return 0;
    }

    int runpddl() {
        ros::Rate loop_rate(10);
        loop_rate.sleep();

        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(10);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        pioneer_shr_msg::SmartSensor curSensorInfo;

        ROS_INFO_STREAM("Start DB monitoring M2...");
        ROS_INFO_STREAM(beginTime);
        ROS_INFO_STREAM(endTime);

        while (ros::Time::now() < endTime && ros::ok() && !isPeopleAtDoor) {
            curSensorInfo = monitoringDB();
            isPeopleAtDoor = curSensorInfo.motion2_is_on;
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Monitoring DB M2...");
        }

		if (!isPeopleAtDoor) {
			ROS_INFO_STREAM("No one approach the door during last night!");
			return 0;
		}

        ROS_INFO_STREAM("some one approach the door, approaching door...");
        ROS_INFO_STREAM("yelling...");
        playAudioWithMessageFile("alex_wait.txt");


        ROS_INFO_STREAM("run rosplan");

        std_srvs::Empty srv;

        ROS_INFO_STREAM("Generating a Problem");
        ros::ServiceClient rosplan_client =
                n.serviceClient<std_srvs::Empty>(
                        "/rosplan_problem_interface/problem_generation_server");
        rosplan_client.call(srv);
		
        ROS_INFO_STREAM("Planning");
        rosplan_client = n.serviceClient<std_srvs::Empty>(
                "/rosplan_planner_interface/planning_server");
        rosplan_client.call(srv);

        ROS_INFO_STREAM("Parsing the Plan");
        rosplan_client = n.serviceClient<std_srvs::Empty>(
                "/rosplan_parsing_interface/parse_plan");
        rosplan_client.call(srv);

        ROS_INFO_STREAM("Executing the Plan");
		rosplan_dispatch_msgs::DispatchService dsrv;
        rosplan_client = n.serviceClient<rosplan_dispatch_msgs::DispatchService>(
                "/rosplan_plan_dispatcher/dispatch_plan");
        rosplan_client.call(dsrv);

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

    bool phoneCallWithScriptandMessageFile(const std::string& script,
            const std::string& msgFile) {
        ros::ServiceClient run_script_client =
                n.serviceClient<pioneer_shr_msg::Action_Run_Script>(
                        "/run_script_service/Action_Run_Script");

        std::string resourcePath = ros_work_space;

        resourcePath += "/src/pioneer_shr/resource/phoneApp/"+script;

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

    void moveTo(const std::string& destinationName) {
        ros::ServiceClient move_to_client =
                n.serviceClient<pioneer_shr_msg::Action_Move_To>(
                        "/move_to_service/Action_Move_To");

        pioneer_shr_msg::Action_Move_To move_to_srv;

		move_to_srv.request.destination_name = destinationName;

        callService<pioneer_shr_msg::Action_Move_To>(move_to_client,
                move_to_srv,
                "Action_Move_To");
    }

    ros::NodeHandle n;
	const std::string ros_work_space;
	bool medicineTaken;
	bool isDoorOpen;
	bool isPeopleBackToBed;
	bool isPeopleAtDoor;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "executive");

  const char* ros_work_space = std::getenv("ROS_WORKSPACE");
  if (ros_work_space == 0) {
      ROS_ERROR_STREAM("ROS_WORKSPACE environment variable not found!");
      return 1;
  }

  if (argc<2){
      ROS_ERROR_STREAM("usage: executive [p1|p2|pddl]");
      return 1;
  }

  Executive executive(ros_work_space);

  if (std::string(argv[1]) == "p1") {
      if (executive.run())
          return 1;
  } else if (std::string(argv[1]) == "p2") {
      if (executive.runP2())
          return 1;
  } else if (std::string(argv[1]) == "pddl") {
      if (executive.runpddl())
          return 1;
  } else {
      ROS_ERROR_STREAM("usage: executive [p1|p2|pddl]");
      return 1;
  }

    return 0;
}
