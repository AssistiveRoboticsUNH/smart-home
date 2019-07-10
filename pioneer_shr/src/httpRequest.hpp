#include <cpr/cpr.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "jasonParser.hpp"
#include "pioneer_shr_msg/SmartSensor.h"
#include <ros/ros.h>

class HttpRequest {
public:
    static pioneer_shr_msg::SmartSensor getSensorData() {
        auto r = cpr::Get(cpr::Url{"http://localhost:4567/getcontacts"});

        std::string webtext = r.text;

        std::string jasonListStr = webtext.substr(1, webtext.size() - 2);

        std::stringstream ss(jasonListStr);

        std::vector<std::string> jasonStrList;

        std::string jasonStr;

        int i = 1;
        while (getline(ss, jasonStr, ',')) {
            if (i % 2 != 0)
                jasonStrList.push_back(jasonStr);
            else
                jasonStrList[jasonStrList.size() - 1] += "," + jasonStr;

            i++;
        }


        std::vector<JasonParser> jasonList;

        ROS_INFO("parse sensor data:  ");

        for (auto str : jasonStrList) {
            jasonList.push_back(JasonParser(str));
            ROS_INFO_STREAM(str);
        }

        pioneer_shr_msg::SmartSensor sensorMsg;

        sensorMsg.door_is_open = jasonList[0].getValue("value") == "open";
        sensorMsg.motion1_is_on = jasonList[1].getValue("value") == "active";
        sensorMsg.motion2_is_on = jasonList[2].getValue("value") == "active";

		return sensorMsg;
    }
};
