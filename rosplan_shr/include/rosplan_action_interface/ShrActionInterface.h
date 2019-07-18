#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"
#include "pioneer_shr_msg/Action_Move_To.h"
#include "pioneer_shr_msg/Action_Run_Script.h"
#include <cstdlib>

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

	class ShrActionInterface: public RPActionInterface
	{

	private:
          template <class ServiceType>
          void callService(ros::ServiceClient &client, ServiceType &service,
                           const std::string &serviceName) const;
          bool moveTo(const std::string &destinationName);

          bool playAudioWithMessageFile(const std::string &msgFile);

          ros::NodeHandle n;
          const char *ros_work_space;

        public:
          /* constructor */
          ShrActionInterface(ros::NodeHandle &nh);

          /* listen to and process action_dispatch topic */
          bool concreteCallback(
              const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
	};
}
