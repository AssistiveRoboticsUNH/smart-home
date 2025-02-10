//
// Created by marzan on 2/10/25.
//

#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/call_request.hpp"
#include "shr_msgs/action/text_request.hpp"
#include "shr_msgs/action/docking_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "shr_msgs/action/read_script_request.hpp"
#include "shr_msgs/action/play_audio_request.hpp"
#include "shr_msgs/action/docking_request.hpp"
#include "shr_msgs/action/localize_request.hpp"
#include "shr_msgs/action/waypoint_request.hpp"
#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        InstantiatedParameter active_protocol;
        std::shared_ptr<WorldStateListener> world_state_converter;
        // change first to change time (x  before y after)
        const std::unordered_map<InstantiatedParameter, std::unordered_map<std::string, std::pair<int, int>>>
        // Msg in PDDL
        // name field should be the same as the name of the protocol in the high_level_problem
        // mak sure the txt files and mp3 are in shr_resources
        wait_times = {
                {{"am_meds",                           "MedicineProtocol"},                       {{"reminder_1_msg", {0, 1}},
                                                                                                          {"reminder_2_msg", {0, 1}},
                                                                                                          {"wait", {60, 0}},
                                                                                                  }},
                {{"pm_meds",                           "MedicineProtocol"},                       {{"reminder_1_msg", {0, 12}},
                                                                                                          {"reminder_2_msg", {0, 12}},
                                                                                                          {"wait", {60, 0}},
                                                                                                  }},
                {{"gym_reminder",                      "GymReminderProtocol"},                    {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},
                {{"medicine_refill_reminder",          "MedicineRefillReminderProtocol"},         {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},
                {{"medicine_refill_pharmacy_reminder", "MedicineRefillPharmacyReminderProtocol"}, {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},

        };

        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::string>> automated_reminder_msgs = {
                {{"am_meds",       "MedicineProtocol"},              {{"reminder_1_msg", "am_med_reminder.txt"},
                                                                     }},
                {{"pm_meds",       "MedicineProtocol"},              {{"reminder_1_msg", "pm_med_reminder.txt"},
                                                                     }},
                {{"gym_reminder",          "GymReminderProtocol"},          {{"reminder_1_msg", "move_reminder.txt"},
                                                                     }},
                {{"internal_check_reminder", "InternalCheckReminderProtocol"}, {{"reminder_1_msg", "internal_check_reminder.txt"},
                                                                     }},
                {{"medicine_refill_reminder",      "MedicineRefillReminderProtocol"},      {{"reminder_1_msg", "practice_reminder.txt"},
                                                                     }},
                {{"medicine_refill_pharmacy_reminder",      "MedicineRefillPharmacyReminderProtocol"},      {{"reminder_1_msg", "exercise_reminder.txt"},
                                                                     }},
        };
    };


}

