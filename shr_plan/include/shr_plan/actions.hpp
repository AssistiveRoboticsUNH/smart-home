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
        const std::unordered_map<InstantiatedParameter, std::unordered_map<std::string, std::pair<int, int>>> wait_times = {
                {{"daily_wand", "WanderingProtocol"}, {{"automated_msg", {0, 0}},
                                                              {"recorded_msg", {10, 0}},
                                                              {"call_caregiver_outside_msg", {0, 0}},
                                                              {"call_emergency_msg", {60, 0}},
                                                              {"call_caregiver_bed_msg",   {30, 0}},
                                                      }},
                {{"daily_med",  "MedicineProtocol"},  {{"guide_1_msg",   {0, 10}},
                                                              {"guide_2_msg",  {0,  10}},
                                                              {"automated_msg",              {0, 0}},
                                                              {"recorded_msg",       {10, 0}},
                                                              {"call_caregiver_guide_msg", {30, 0}},
                                                              {"call_caregiver_msg", {30, 0}},
                                                      }},
                {{"dinner",     "FoodProtocol"},      {{"guide_1_msg",   {0, 10}},
                                                              {"guide_2_msg",  {0,  10}},
                                                              {"automated_msg",              {0, 0}},
                                                              {"recorded_msg",       {10, 0}},
                                                              {"call_caregiver_guide_msg", {10, 0}},
                                                              {"call_caregiver_msg", {10, 0}},
                                                      }},
                {{"lunch",      "FoodProtocol"},      {{"guide_1_msg",   {0, 10}},
                                                              {"guide_2_msg",  {0,  10}},
                                                              {"automated_msg",              {0, 0}},
                                                              {"recorded_msg",       {10, 0}},
                                                              {"call_caregiver_guide_msg", {10, 0}},
                                                              {"call_caregiver_msg", {10, 0}},
                                                      }},
                {{"breakfast",  "FoodProtocol"},      {{"guide_1_msg",   {0, 10}},
                                                              {"guide_2_msg",  {0,  10}},
                                                              {"automated_msg",              {0, 0}},
                                                              {"recorded_msg",       {10, 0}},
                                                              {"call_caregiver_guide_msg", {10, 0}},
                                                              {"call_caregiver_msg", {10, 0}},
                                                      }}
        };

        const std::unordered_map<InstantiatedParameter, std::unordered_map<std::string, std::pair<std::string, std::string>>> call_msgs = {
                {{"daily_wand", "WanderingProtocol"}, {{"call_caregiver_outside_msg", {"call_msg_leaving_house.xml", "7163482782"}},
                                                              {"call_caregiver_bed_msg", {"call_msg_will_not_go_to_bed.xml", "7163482782"}},
                                                              {"call_emergency_msg", {"call_msg_911.xml", "7163482782"}},
                                                      }},
                {{"daily_med",  "MedicineProtocol"},  {{"call_caregiver_guide_msg",   {"call_msg_medical.xml",       "7163482782"}},
                                                              {"call_caregiver_msg",     {"call_msg_medical.xml",            "7163482782"}},
                                                      }},
                {{"dinner",     "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7163482782"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7163482782"}},
                                                      }},
                {{"lunch",      "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7163482782"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7163482782"}},
                                                      }},
                {{"breakfast",  "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7163482782"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7163482782"}},
                                                      }}
        };

        const std::unordered_map<InstantiatedParameter, std::unordered_map<std::string, std::string>> automated_reminder_msgs = {
                {{"daily_wand", "WanderingProtocol"}, {{"automated_msg", "midnight_reminder.txt"},
                                                      }},
                {{"daily_med",  "MedicineProtocol"},  {{"guide_1_msg",   "medicine_follow_me.txt"},
                                                              {"guide_2_msg", "medicine_follow_me.txt"},
                                                              {"automated_msg", "medicine_reminder.txt"},
                                                      }},
                {{"dinner",     "FoodProtocol"},      {{"guide_1_msg",   "food_follow_me.txt"},
                                                              {"guide_2_msg", "food_follow_me.txt"},
                                                              {"automated_msg", "food_reminder.txt"},
                                                      }},
                {{"lunch",      "FoodProtocol"},      {{"guide_1_msg",   "food_follow_me.txt"},
                                                              {"guide_2_msg", "food_follow_me.txt"},
                                                              {"automated_msg", "food_reminder.txt"},
                                                      }},
                {{"breakfast",  "FoodProtocol"},      {{"guide_1_msg",   "food_follow_me.txt"},
                                                              {"guide_2_msg", "food_follow_me.txt"},
                                                              {"automated_msg", "food_reminder.txt"},
                                                      }}
        };

        const std::unordered_map<InstantiatedParameter, std::unordered_map<std::string, std::string>> recorded_reminder_msgs = {
                {{"daily_wand", "WanderingProtocol"}, {{"recorded_msg", "midnight_reminder.mp3"},
                                                      }},
                {{"daily_med",  "MedicineProtocol"},  {{"recorded_msg", "medicine_reminder.mp3"},
                                                      }},
                {{"breakfast",     "FoodProtocol"},      {{"recorded_msg", "food_reminder.mp3"},
                                                      }},
                {{"lunch",     "FoodProtocol"},      {{"recorded_msg", "food_reminder.mp3"},
                                                      }},
                {{"dinner",     "FoodProtocol"},      {{"recorded_msg", "food_reminder.mp3"},
                                                      }}
        };

        // action servers
        rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr call_client_ = {};
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ = {};
        rclcpp_action::Client<shr_msgs::action::DockingRequest>::SharedPtr docking_ = {};
        rclcpp_action::Client<shr_msgs::action::DockingRequest>::SharedPtr undocking_ = {};
        rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SharedPtr localize_ = {};
        rclcpp_action::Client<shr_msgs::action::PlayAudioRequest>::SharedPtr audio_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::WaypointRequest>::SharedPtr waypoint_action_client_ = {};

        static InstantiatedParameter getActiveProtocol() {
            std::lock_guard<std::mutex> lock(getInstance().active_protocol_mtx);
            return getInstance().active_protocol;
        }

        static bool isRobotInUse() {
//            std::cout << "isRobotInUse:   " << getConcurrentInstance().first.robot_in_use << std::endl;
            return getConcurrentInstance().first.robot_in_use;
        }

        static std::pair<ProtocolState &, std::scoped_lock<std::mutex> &&> getConcurrentInstance() {
            std::scoped_lock<std::mutex> lock_guard(getInstance().mtx);
            return {getInstance(), std::move(lock_guard)};
        }

        struct RobotResource {
            ~RobotResource() {
                getConcurrentInstance().first.robot_in_use = false;
//                std::cout << "Destrcutor " << std::endl;
            }

            RobotResource() {
                getConcurrentInstance().first.robot_in_use = true;
//                std::cout << "Constructor " << std::endl;

            }
        };

        static RobotResource claimRobot() {
            RobotResource robot;
            //std::cout << "Claim Robot " << std::endl;
            return robot;
        }

    private:
        static ProtocolState &getInstance() {
            static ProtocolState instance;
            return instance;
        }

        ProtocolState() {} // Private constructor to prevent direct instantiation
        ~ProtocolState() {} // Private destructor to prevent deletion
        ProtocolState(const ProtocolState &) = delete; // Disable copy constructor
        ProtocolState &operator=(const ProtocolState &) = delete; // Disable assignment operator
        std::mutex mtx;
        std::mutex active_protocol_mtx;
        std::atomic<bool> robot_in_use = false;
    };

//    BT::NodeStatus observe_wait_for_cond(const InstantiatedAction &action,
//                                         std::chrono::steady_clock::time_point &init_time,
//                                         const std::function<bool()> &cond,
//                                         std::chrono::seconds wait_time) {
//        ProtocolState &ps = ProtocolState::getInstance();
//        auto startTime = std::chrono::steady_clock::now();
//        if (startTime > init_time) {
//            init_time = startTime + wait_time;
//        }
//
//        while (std::chrono::steady_clock::now() < init_time) {
//            auto active_protocol = get_active_protocol().value();
//            if (active_protocol.type != action.parameters[0].type) {
//                abort();
//            }
//            auto msg = ps.world_state_converter->get_world_state_msg();
//            if (cond()) {
//                return BT::NodeStatus::SUCCESS;
//            }
//            rclcpp::sleep_for(std::chrono::seconds(1));
//        }
//
//        return BT::NodeStatus::FAILURE;
//    }
    std::string find_robot_location(){
        std::vector<std::string> landmarks = {"bedroom", "door", "couch", "outside", "living_room", "kitchen", "bathroom", "dining_room", "hallway", "home"};
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        for(int i=0; i<landmarks.size(); i++){
            if(ps.world_state_converter->check_robot_at_loc(landmarks[i])){
                return landmarks[i];
            }
        }

    }

    int send_goal_blocking(const shr_msgs::action::CallRequest::Goal &goal, const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::CallRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::CallRequest>::WrappedResult result) {
            *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.call_client_->async_send_goal(goal, send_goal_options);
        rclcpp::sleep_for(std::chrono::seconds(15)); //automatically wait because call is not blocking
        auto tmp = ps.active_protocol;
        while (*success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.call_client_->async_cancel_all_goals();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return *success;
    }

   int send_goal_blocking(const nav2_msgs::action::NavigateToPose::Goal &goal, const InstantiatedAction &action) {

       auto [ps, lock] = ProtocolState::getConcurrentInstance();
       auto &kb = KnowledgeBase::getInstance();
       auto success = std::make_shared<std::atomic<int>>(-1);
       auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
       send_goal_options.result_callback = [&success](
               const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
           if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
               *success = 1;
               RCLCPP_INFO(rclcpp::get_logger(
                       std::string("weblog=") + " Navigation goal Succeeded."), "user...");
           } else {
               *success = 0;
               RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + " Navigation goal aborted."), "user...");
               std::cout << "Navigation goal aborted." << std::endl;
           }
       };
       ps.nav_client_->async_send_goal(goal, send_goal_options);
       auto tmp = ps.active_protocol;

       // prevent long navigation time
       int count = 0;
       int count_max = 50;

       while (*success == -1 && count_max > count) {
           if (!(tmp == ps.active_protocol)) {
               ps.nav_client_->async_cancel_all_goals();
               return false;
           }
           count++;
           rclcpp::sleep_for(std::chrono::seconds(1));
           if (count_max - 1 == count) {
               RCLCPP_INFO(rclcpp::get_logger(
                       std::string("weblog=") + " Navigation failed for exceed time."), "user...");
               ps.nav_client_->async_cancel_all_goals();
               std::cout << " Navigation failed for exceed time  " << std::endl;
               return false;
           }
       }
       return *success;
   }

    // int send_goal_blocking(const shr_msgs::action::WaypointRequest::Goal &goal, const InstantiatedAction &action) {
    //     auto [ps, lock] = ProtocolState::getConcurrentInstance();
    //     auto &kb = KnowledgeBase::getInstance();
    //     auto success = std::make_shared<std::atomic<int>>(-1);
    //     auto send_goal_options = rclcpp_action::Client<shr_msgs::action::WaypointRequest>::SendGoalOptions();
    //     send_goal_options.result_callback = [success](
    //             const rclcpp_action::ClientGoalHandle<shr_msgs::action::WaypointRequest>::WrappedResult result) {
    //         *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    //     };
    //     ps.waypoint_action_client_->async_send_goal(goal, send_goal_options);
    //     auto tmp = ps.active_protocol;
    //     while (*success == -1) {
    //         if (!(tmp == ps.active_protocol)) {
    //             ps.waypoint_action_client_->async_cancel_all_goals();
    //             return false;
    //         }
    //         rclcpp::sleep_for(std::chrono::seconds(1));
    //     }
    //     return *success;
    // }


    int send_goal_blocking(const shr_msgs::action::LocalizeRequest::Goal &goal, const InstantiatedAction &action) {

        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::LocalizeRequest>::WrappedResult result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                *success = 1;
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Localize goal Succeeded."), "user...");
            } else {
                *success = 0;
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + " Localize goal aborted."), "user...");
                std::cout << "Localize goal aborted." << std::endl;
            }
        };
        ps.localize_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;

        // prevent long navigation time
        int count = 0;
        int count_max = 50;

        while (*success == -1 && count_max > count) {
            if (!(tmp == ps.active_protocol)) {
                ps.localize_->async_cancel_all_goals();
                return false;
            }
            count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (count_max - 1 == count) {
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Localize failed for exceed time."), "user...");
                ps.localize_->async_cancel_all_goals();
                std::cout << " Localize failed for exceed time  " << std::endl;
                return false;
            }
        }
        return *success;
    }

    int send_goal_blocking(const shr_msgs::action::DockingRequest::Goal &goal, const InstantiatedAction &action) {

        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::DockingRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::DockingRequest>::WrappedResult result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                *success = 1;
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Docking goal Succeeded."), "user...");
            } else {
                *success = 0;
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + " Docking goal aborted."), "user...");
                std::cout << "Docking goal aborted." << std::endl;
            }
        };
        ps.docking_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;

        // prevent long navigation time
        int count = 0;
        int count_max = 50;

        while (*success == -1 && count_max > count) {
            if (!(tmp == ps.active_protocol)) {
                ps.docking_->async_cancel_all_goals();
                return false;
            }
            count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (count_max - 1 == count) {
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Docking failed for exceed time."), "user...");
                ps.docking_->async_cancel_all_goals();
                std::cout << " Docking failed for exceed time  " << std::endl;
                return false;
            }
        }
        return *success;
    }

    int send_goal_blocking(const shr_msgs::action::ReadScriptRequest::Goal &goal, const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SendGoalOptions();
        send_goal_options.result_callback = [success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::ReadScriptRequest>::WrappedResult result) {
            *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.read_action_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (*success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.read_action_client_->async_cancel_all_goals();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return *success;
    }

    int send_goal_blocking(const shr_msgs::action::PlayAudioRequest::Goal &goal, const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::PlayAudioRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::PlayAudioRequest>::WrappedResult result) {
            *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.audio_action_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;

//        while (*success == -1) {
//            if (!(tmp == ps.active_protocol)) {
//                ps.video_action_client_->async_cancel_all_goals();
//                return false;
//            }
//            rclcpp::sleep_for(std::chrono::seconds(1));
//        }
        int count = 0;
        int count_max = 50;

        while (*success == -1 && count_max > count) {
            if (!(tmp == ps.active_protocol)) {
                ps.audio_action_client_->async_cancel_all_goals();
                return false;
            }
            count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (count_max - 1 == count) {
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Recorded failed for exceed time."), "user...");
                ps.audio_action_client_->async_cancel_all_goals();
                std::cout << " Recorded failed for exceed time  " << std::endl;
                return false;
            }
        }
        return *success;
    }

    long get_inst_index_helper(const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto inst = action.parameters[0];
        auto params = ps.world_state_converter->get_params();
        return get_inst_index(inst, params).value();
    }

    std::string get_file_content(const std::string &file_name) {
        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
        auto pddl_path = pkg_dir / "pddl";
        auto problem_food_file = (pddl_path / file_name).string();
        std::ifstream f(problem_food_file);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    void instantiate_high_level_problem() {
        auto &kb = KnowledgeBase::getInstance();
        auto protocol_content = get_file_content("problem_high_level.pddl");
        auto domain_content = get_file_content("high_level_domain.pddl");
        auto prob = parse_problem(protocol_content, domain_content).value();
        kb.clear();
        kb.load_kb(prob);
    }

    void instantiate_protocol(const std::string &protocol_name,
                              const std::vector<std::pair<std::string, std::string>> &replacements = {}) {
        auto &kb = KnowledgeBase::getInstance();
        auto high_level_domain_content = get_file_content("high_level_domain.pddl");
        auto high_level_domain = parse_domain(high_level_domain_content).value();
        auto current_high_level = parse_problem(kb.convert_to_problem(high_level_domain),
                                                high_level_domain_content).value();

        auto protocol_content = get_file_content("problem_" + protocol_name);
        auto domain_content = get_file_content("low_level_domain.pddl");
        for (const auto &replacement: replacements) {
            protocol_content = replace_token(protocol_content, replacement.first, replacement.second);
        }
        auto prob = parse_problem(protocol_content, domain_content).value();

        kb.clear();
        kb.load_kb(current_high_level);
        kb.load_kb(prob);

    }

    class ProtocolActions : public pddl_lib::ActionInterface {
    public:

        BT::NodeStatus high_level_domain_Idle(const InstantiatedAction &action) override {

            // CHECKING IF ROBOT IS CHARGING FIRST
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
//            auto params = ps.world_state_converter->get_params();
            auto &kb = KnowledgeBase::getInstance();
            kb.insert_predicate({"abort", {}});
            kb.clear_unknowns();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(std::string("user=") + "high_level_domain_Idle" + "started"), "user...");


            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
                std::cout << "High level claim robot called " << std::endl;
                auto robot_resource = ps.claimRobot();
                ps.call_client_->async_cancel_all_goals();
                ps.read_action_client_->async_cancel_all_goals();
                ps.audio_action_client_->async_cancel_all_goals();
                ps.undocking_->async_cancel_all_goals();
                ps.docking_->async_cancel_all_goals();
                ps.localize_->async_cancel_all_goals();
                ps.docking_->async_cancel_all_goals();


               std::cout << "localize " << std::endl;
               RCLCPP_INFO(
                       rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "localizing started"),
                       "user...");

               shr_msgs::action::LocalizeRequest::Goal goal_msg_loc;
               goal_msg_loc.force_localize = true;

//
//               auto status_loc = send_goal_blocking(goal_msg_loc, action);
//               std::cout << "status: " << status_loc << std::endl;
//               if (!status_loc) {
//                   std::cout << "Fail: " << std::endl;
//                   return BT::NodeStatus::FAILURE;
//               }


                RCLCPP_INFO(
                        rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "Navigation started"),
                        "user...");
                std::cout << "navigate " << std::endl;

               nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
               navigation_goal_.pose.header.frame_id = "map";
               navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
               if (auto transform = ps.world_state_converter->get_tf("map", "home")) {
                   navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
                   navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
                   navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
                   navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
               }
               auto status_nav = send_goal_blocking(navigation_goal_, action);
               std::cout << "status: " << status_nav << std::endl;
               if (!status_nav) {
                   std::cout << "Fail: " << std::endl;
                   return BT::NodeStatus::FAILURE;
               }
               std::cout << "success navigation : " << std::endl;

                // std::string from_location = find_robot_location();
                // std::string to_location = "home";
                // if(from_location != to_location) {
                //     std::cout << "In Idle from: " << from_location << " to: " << to_location << std::endl;
                //     shr_msgs::action::WaypointRequest::Goal waypoint_goal_;
                //     waypoint_goal_.from_location = from_location;
                //     waypoint_goal_.to_location = to_location;
                //     auto status_nav = send_goal_blocking(waypoint_goal_, action);

                //     if (!status_nav) {
                //         std::cout << "Fail: " << std::endl;
                //         return BT::NodeStatus::FAILURE;
                //     }
                //     std::cout << "success navigation : " << std::endl;
                // }

//               std::cout << "dock " << std::endl;
//
//               shr_msgs::action::DockingRequest::Goal goal_msg_dock;
//               RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "docking started"),
//                           "user...");
//
//               auto status_dock = send_goal_blocking(goal_msg_dock, action);
//               std::cout << "status: " << status_dock << std::endl;
//               if (!status_dock) {
//                   std::cout << "Fail: " << std::endl;
//                   return BT::NodeStatus::FAILURE;
//               }
//               std::cout << "success: " << std::endl;



               // // sleep for 60 seconds to deal with the delay from //charging topic
               std::cout << " waiting  " << std::endl;
               rclcpp::sleep_for(std::chrono::seconds(10));

               std::cout << "High level ending " << std::endl;

            }
            ps.active_protocol = {};
            return BT::NodeStatus::SUCCESS;
        }

        void abort(const InstantiatedAction &action) override {
            std::cout << "abort: higher priority protocol detected\n";
            std::string currentDateTime = getCurrentDateTime();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"aborted"+"higher priority protocol detected"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"aborted"+"higher priority protocol detected"), "user...");
            auto &kb = KnowledgeBase::getInstance();
            kb.insert_predicate({"abort", {}});
        }

        BT::NodeStatus high_level_domain_StartWanderingProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"WanderingProtocol"+"started"), "user...");
            instantiate_protocol("midnight.pddl");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            ps.active_protocol = inst;
            return BT::NodeStatus::SUCCESS;
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartMedicineProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"MedicineProtocol"+"started"), "user...");
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            if (dest.name == cur.name) {
                cur.name = "living_room"; // cause we took out couch
            }
            instantiate_protocol("medicine.pddl", {{"current_loc", cur.name},
                                               {"dest_loc",    dest.name}});
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            ps.active_protocol = protocol;

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_MoveToLandmark(const InstantiatedAction &action) override {
            // if movetolandmark is called when the robot is docked it needs to undock first
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_MoveToLandmark"+" started"), "user...");

           if (ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
               std::cout << "Undock " << std::endl;

               shr_msgs::action::DockingRequest::Goal goal_msg;

               auto success_undock = std::make_shared<std::atomic<int>>(-1);
               auto send_goal_options_dock = rclcpp_action::Client<shr_msgs::action::DockingRequest>::SendGoalOptions();
               send_goal_options_dock.result_callback = [&success_undock](
                       const rclcpp_action::ClientGoalHandle<shr_msgs::action::DockingRequest>::WrappedResult result) {
                   *success_undock = result.code == rclcpp_action::ResultCode::SUCCEEDED;
                   if (*success_undock == 1) {
                       RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_MoveToLandmark"+"UnDocking goal Succeeded."), "user...");

                   } else {
                       RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_MoveToLandmark"+"UnDocking goal aborted!."), "user...");

                   }
               };

               ps.undocking_->async_send_goal(goal_msg, send_goal_options_dock);
               auto tmp_dock = ps.active_protocol;

               while (*success_undock == -1) {
                   if (!(tmp_dock == ps.active_protocol)) {
                       ps.undocking_->async_cancel_all_goals();
                       std::cout << " Failed " << std::endl;
                       RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_MoveToLandmark"+"UnDocking failed for protocol mismatched."), "user...");

                   }
                   rclcpp::sleep_for(std::chrono::seconds(1));
               }
               ps.undocking_->async_cancel_all_goals();
           }

            InstantiatedParameter from = action.parameters[0];
            InstantiatedParameter to = action.parameters[1];
            InstantiatedParameter t1 = {"t1", "Time"};
            InstantiatedAction action_inst = {"MoveToLandmark",
                                              {t1, from, to}};
            return shr_domain_MoveToLandmark(action_inst);
        }

        // food_protocol
        BT::NodeStatus high_level_domain_StartFoodProtocol(const InstantiatedAction &action) override {

            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartFoodProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"FoodProtocol"+"started"), "user...");
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            if (dest.name == cur.name) {
                cur.name = "living_room";  // cause we took out couch
            }
            instantiate_protocol("food.pddl", {{"current_loc", cur.name},
                                               {"dest_loc",    dest.name}});

            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            ps.active_protocol = protocol;

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_FoodEatenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate pred{"already_ate", {ps.active_protocol}};
            kb.insert_predicate(pred);
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto active_protocol = ps.active_protocol;
            std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_called_about_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "WanderingProtocol") {
                kb.erase_predicate({"wandering_protocol_enabled", {active_protocol}});
            }
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MessageGivenSuccess"+active_protocol.type), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime +std::string("user=")+"Message is given for: "+active_protocol.type), "user...");

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_PersonAtSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto active_protocol = ps.active_protocol;
            std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_called_about_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "WanderingProtocol") {
                kb.erase_predicate({"wandering_protocol_enabled", {active_protocol}});
            }
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_PersonAtSuccess"+active_protocol.type), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"active protocol"+active_protocol.type), "user...");
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto t = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate ate_food = {"person_eating", {t}};
            if (kb.find_predicate(ate_food)) {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectEatingFood" + "ate food success"), "user...");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectEatingFood"+"ate food failure!"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"person is not eating food!"), "user...");
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction &action) override {

            /// move robot to location
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MoveToLandmark"+"moving to land mark!"), "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            std::string location = action.parameters[2].name;

//            int count_max = 30;
//
//            std::cout << "localize " << std::endl ;
//            shr_msgs::action::LocalizeRequest::Goal goal_msg_loc;
//            goal_msg_loc.force_localize = false;
//
//            auto success_loc = std::make_shared<std::atomic<int>>(-1);
//            auto send_goal_options_loc = rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SendGoalOptions();
//            send_goal_options_loc.result_callback = [&success_loc](
//                    const rclcpp_action::ClientGoalHandle<shr_msgs::action::LocalizeRequest>::WrappedResult result) {
//                *success_loc = result.code == rclcpp_action::ResultCode::SUCCEEDED;
//            };
//
//            ps.localize_->async_send_goal(goal_msg_loc, send_goal_options_loc);
//            auto tmp_loc = ps.active_protocol;
//
//            int count__ = 0;
//            while (*success_loc == -1 && count_max > count__) {
//                if (!(tmp_loc == ps.active_protocol)) {
//                    ps.localize_->async_cancel_all_goals();
//                    std::cout << " Failed " << std::endl;
//                }
//                count__++;
//                rclcpp::sleep_for(std::chrono::seconds(1));
//            }


            nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
            navigation_goal_.pose.header.frame_id = "map";
            navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
            if (auto transform = ps.world_state_converter->get_tf("map", location)) {
                navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
                navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
                navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
                navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
            } else {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MoveToLandmark"+"moving to land mark failed!"), "user...");
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MoveToLandmark"+"moving to land mark succeed!"), "user...");

            return send_goal_blocking(navigation_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

        //    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MoveToLandmark"+"moving to land mark succeed!"), "user...");
        //     shr_msgs::action::WaypointRequest ::Goal waypoint_goal_;
        //     waypoint_goal_.from_location = action.parameters[1].name;
        //     waypoint_goal_.to_location = action.parameters[2].name;

           // return send_goal_blocking(navigation_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_GiveReminder(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            std::string currentDateTime = getCurrentDateTime();
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+"failed!"), "user...");
                    return BT::NodeStatus::FAILURE;
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
            std::string script_name_str;
            BT::NodeStatus ret;
            if (ps.automated_reminder_msgs.at(ps.active_protocol).find(msg) !=
                ps.automated_reminder_msgs.at(ps.active_protocol).end()) {
                shr_msgs::action::ReadScriptRequest::Goal read_goal_;
                read_goal_.script_name = ps.automated_reminder_msgs.at(ps.active_protocol).at(msg);
                script_name_str = std::string(read_goal_.script_name.begin(), read_goal_.script_name.end());

                ret = send_goal_blocking(read_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else {
                shr_msgs::action::PlayAudioRequest::Goal audio_goal_;
                audio_goal_.file_name = ps.recorded_reminder_msgs.at(ps.active_protocol).at(msg);
                script_name_str = std::string(audio_goal_.file_name.begin(), audio_goal_.file_name.end());

                ret = send_goal_blocking(audio_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            if (ret == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+script_name_str+"succeed!"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"GiveReminder"+script_name_str+"succeed!"), "user...");
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+script_name_str+"failed!"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"GiveReminder"+script_name_str+"failed!"), "user...");
            }

            return ret;
        }


        BT::NodeStatus shr_domain_MakeCall(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto params = ps.world_state_converter->get_params();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            std::string currentDateTime = getCurrentDateTime();
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MakeCall"+"failed"), "user...");
                    return BT::NodeStatus::FAILURE;
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            shr_msgs::action::CallRequest::Goal call_goal_;
            call_goal_.script_name = ps.call_msgs.at(ps.active_protocol).at(msg).first;
            std::string script_name_str(call_goal_.script_name.begin(), call_goal_.script_name.end());
            call_goal_.phone_number = ps.call_msgs.at(ps.active_protocol).at(msg).second;
            auto ret = send_goal_blocking(call_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            if (ret == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MakeCall"+script_name_str+"succeed"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"shr_domain_MakeCall"+script_name_str+"succeed"), "user...");
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MakeCall"+script_name_str+"failed"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"shr_domain_MakeCall"+script_name_str+"failed"), "user...");
            }
            return ret;
        }

        BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto t = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate took_medicine = {"person_taking_medicine", {t}};
            if (kb.find_predicate(took_medicine)) {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"succeeded"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"succeeded"), "user...");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"failed"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"failed"), "user...");
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            std::string currentDateTime = getCurrentDateTime();
            std::string lm = action.parameters[2].name;
            if (ps.world_state_converter->check_person_at_loc(lm)) {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectPersonLocation"+"succeeded"), "user...");
                RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"person location detection"+"succeeded"), "user...");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"failed"), "user...");

                return BT::NodeStatus::FAILURE;
            }
        }

        std::string getCurrentDateTime() {
            auto currentTimePoint = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
            std::tm* timeInfo = std::localtime(&currentTime);
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
            return buffer;
        }

    };
}
