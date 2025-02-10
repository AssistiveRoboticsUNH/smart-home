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

        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::string>> recorded_reminder_msgs = {
                {{"am_meds", "MedicineProtocol"}, {{"reminder_2_msg", "am_med_reminder.mp3"},
                                                  }},
                {{"pm_meds", "MedicineProtocol"}, {{"reminder_2_msg", "pm_med_reminder.mp3"},
                                                  }},

        };

        // action servers
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ = {};
        rclcpp_action::Client<shr_msgs::action::DockingRequest>::SharedPtr docking_ = {};
        rclcpp_action::Client<shr_msgs::action::DockingRequest>::SharedPtr undocking_ = {};
        rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::PlayAudioRequest>::SharedPtr audio_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SharedPtr localize_ = {};

        static InstantiatedParameter getActiveProtocol() {
            std::lock_guard <std::mutex> lock(getInstance().active_protocol_mtx);
            return getInstance().active_protocol;
        }

        static bool isRobotInUse() {
//            std::cout << "isRobotInUse:   " << getConcurrentInstance().first.robot_in_use << std::endl;
            return getConcurrentInstance().first.robot_in_use;
        }

        static bool IsLocked() {
            return getInstance().is_locked;
        }

        struct LockManager {
            std::mutex *mtx_;
            bool *is_locked_;

            void Lock() {
                mtx_->lock();
                *is_locked_ = true;
                // std::cout << " ****** LOCKING getInstance().active_protocol:   " << getInstance().active_protocol
                //   << std::endl;
            }

            LockManager(std::mutex &mtx, bool &is_locked) {
                mtx_ = &mtx;
//                mtx.lock();
//                assert(!is_locked);
//                is_locked = true;
                is_locked_ = &is_locked;
            }

            void UnLock() {
                mtx_->unlock();
                // std::cout << " $$$$$$$ UNLOCKING getInstance().active_protocol:   " << getInstance().active_protocol
                //           << std::endl;
                *is_locked_ = false;
            }
//            ~LockManager() {
//                mtx_->unlock();
//                std::cout << " $$$$$$$ UNLOCKING getInstance().active_protocol:   " <<  getInstance().active_protocol << std::endl;
//                *is_locked_ = false;
//            }
        };

        static std::pair<ProtocolState &, LockManager> getConcurrentInstance() {
            LockManager lock = LockManager(getInstance().mtx, getInstance().is_locked);
            return {getInstance(), lock};
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
        bool is_locked;




    };

    int send_goal_blocking(const nav2_msgs::action::NavigateToPose::Goal &goal, const InstantiatedAction &action,
                           ProtocolState &ps) {

        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared < std::atomic < int >> (-1);
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
        // int count = 0;
        // int count_max = 50;

        while (*success == -1) { // && count_max > count) {
            if (!(tmp == ps.active_protocol)) {
                ps.nav_client_->async_cancel_all_goals();
                return false;
            }
            // count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
            // if (count_max - 1 == count) {
            //     RCLCPP_INFO(rclcpp::get_logger(
            //             std::string("weblog=") + " Navigation failed for exceed time."), "user...");
            //     ps.nav_client_->async_cancel_all_goals();
            //     std::cout << " Navigation failed for exceed time  " << std::endl;
            //     return false;
            // }
        }
        return *success;
    }

    int send_goal_blocking(const shr_msgs::action::LocalizeRequest::Goal &goal, const InstantiatedAction &action,
                           ProtocolState &ps) {

        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared < std::atomic < int >> (-1);
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
                return *success; // we dont want to relocalize for now
            }
            count++;
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (count_max - 1 == count) {
                RCLCPP_INFO(rclcpp::get_logger(
                        std::string("weblog=") + " Localize failed for exceed time."), "user...");
                ps.localize_->async_cancel_all_goals();
                std::cout << " Localize failed for exceed time  " << std::endl;
                return *success; // we dont want to relocalize for now
            }
        }
        return *success;
    }

    int send_goal_blocking(const shr_msgs::action::DockingRequest::Goal &goal, const InstantiatedAction &action,
                           ProtocolState &ps) {

        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared < std::atomic < int >> (-1);
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
        int count_max = 150;

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

    int send_goal_blocking(const shr_msgs::action::ReadScriptRequest::Goal &goal, const InstantiatedAction &action,
                           ProtocolState &ps) {
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared < std::atomic < int >> (-1);
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

    int send_goal_blocking(const shr_msgs::action::PlayAudioRequest::Goal &goal, const InstantiatedAction &action,
                           ProtocolState &ps) {
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared < std::atomic < int >> (-1);
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
        lock.Lock();
        auto inst = action.parameters[0];
        auto params = ps.world_state_converter->get_params();
        return get_inst_index(inst, params).value();
        lock.UnLock();
    }

    std::string get_file_content(const std::string &file_name) {
        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
        auto pddl_path = pkg_dir / "pddl";
        auto problem_high_level_file = (pddl_path / file_name).string();
        std::ifstream f(problem_high_level_file);
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
                              const std::vector <std::pair<std::string, std::string>> &replacements = {}) {
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
        // Timeout for now doesnt do anything inrodere for the protocol to be retriggered
        BT::NodeStatus high_level_domain_Idle(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            kb.clear_unknowns();
            kb.insert_predicate({"abort", {}});

            // CHECKING IF ROBOT IS CHARGING FIRST
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            auto params = ps.world_state_converter->get_params();

            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(std::string("user=") + "high_level_domain_Idle" + "started"), "user...");

            // std::string currentDateTime = getCurrentDateTime();
            // std::string log_message =
            //         std::string("weblog=") + currentDateTime + " high_level_domain_Idle " + " started!";
            // RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
                std::cout << "High level claim robot called " << std::endl;
                auto robot_resource = ps.claimRobot();
                ps.read_action_client_->async_cancel_all_goals();
                ps.audio_action_client_->async_cancel_all_goals();
                ps.undocking_->async_cancel_all_goals();
                ps.docking_->async_cancel_all_goals();
                // ps.localize_->async_cancel_all_goals();


                std::cout << "localize " << std::endl;
//                RCLCPP_INFO(
//                        rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "localizing started"),
//                        "user...");
//
//                shr_msgs::action::LocalizeRequest::Goal goal_msg_loc;
//                goal_msg_loc.force_localize = false;
//
//
//                auto status_loc = send_goal_blocking(goal_msg_loc, action, ps);
//                std::cout << "status: " << status_loc << std::endl;
//                if (!status_loc) {
//                    std::cout << "Fail: " << std::endl;
//                    ps.localize_->async_cancel_all_goals();
//                    //lock.UnLock();
//                    //return BT::NodeStatus::FAILURE;
//                }
//                ps.localize_->async_cancel_all_goals();

                std::string currentDateTime = getCurrentDateTime();
                std::string log_message =
                        std::string("weblog=") + currentDateTime + " high_level_domain_Idle " + " started!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

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
                auto status_nav = send_goal_blocking(navigation_goal_, action, ps);
                std::cout << "status: " << status_nav << std::endl;
                if (!status_nav) {
                    std::cout << "Fail: " << std::endl;
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                std::cout << "success navigation : " << std::endl;


                std::cout << "dock " << std::endl;
// comment in sim
                shr_msgs::action::DockingRequest::Goal goal_msg_dock;
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "docking started"),
                            "user...");

                auto status_dock = send_goal_blocking(goal_msg_dock, action, ps);
                std::cout << "status: " << status_dock << std::endl;
                if (!status_dock) {
                    ps.docking_->async_cancel_all_goals();
                    std::cout << "Fail: " << std::endl;
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                ps.docking_->async_cancel_all_goals();
                std::cout << "success: " << std::endl;
// comment in sim

                // // sleep for 60 seconds to deal with the delay from //charging topic
                std::cout << " waiting  " << std::endl;
                rclcpp::sleep_for(std::chrono::seconds(60));

                std::cout << "High level ending " << std::endl;

            }

            if (ps.world_state_converter->get_world_state_msg()->robot_charging != 1) {
                std::cout << "Undock " << std::endl;

                shr_msgs::action::DockingRequest::Goal goal_msg;

                auto success_undock = std::make_shared<std::atomic<int >>(-1);
                auto send_goal_options_dock = rclcpp_action::Client<shr_msgs::action::DockingRequest>::SendGoalOptions();
                send_goal_options_dock.result_callback = [&success_undock](
                        const rclcpp_action::ClientGoalHandle<shr_msgs::action::DockingRequest>::WrappedResult result) {
                    *success_undock = result.code == rclcpp_action::ResultCode::SUCCEEDED;
                    if (*success_undock == 1) {
                        RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "low_level_domain_MoveToLandmark" +
                                                       "UnDocking goal Succeeded."), "user...");

                    } else {
                        RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "low_level_domain_MoveToLandmark" +
                                                       "UnDocking goal aborted!."), "user...");

                    }
                };

                ps.undocking_->async_send_goal(goal_msg, send_goal_options_dock);
                auto tmp_dock = ps.active_protocol;

                while (*success_undock == -1) {
                    if (!(tmp_dock == ps.active_protocol)) {
                        ps.undocking_->async_cancel_all_goals();
                        std::cout << " Failed " << std::endl;
                        RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "high_level_domain_MoveToLandmark" +
                                                       "UnDocking failed for protocol mismatched."), "user...");

                    }
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }
                ps.undocking_->async_cancel_all_goals();
            }
            ps.active_protocol = {};
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        void abort(const InstantiatedAction &action) override {
            std::cout << "abort: higher priority protocol detected\n";
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " aborted" + " higher priority protocol detected";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"aborted"+"higher priority protocol detected"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"aborted"+"higher priority protocol detected"), "user...");
            auto &kb = KnowledgeBase::getInstance();
            kb.insert_predicate({"abort", {}});
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];

            instantiate_protocol("medicine_reminder.pddl");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineProtocol" + " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            ps.active_protocol = protocol;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // Gym protocol
        BT::NodeStatus high_level_domain_StartGymReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartExerciseReminderProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartGymReminderProtocol" + "started"),
                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartGymReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            instantiate_protocol("gym_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }


        // StartMedicineRefill protocol
        BT::NodeStatus high_level_domain_StartMedicineRefillReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartMedicineRefillReminderProtocol" + "started"),
                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineRefillReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            instantiate_protocol("medicine_refill_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // internal check protocol
        BT::NodeStatus high_level_domain_StartMedicineRefillPharmacyReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartMedicineRefillPharmacyReminderProtocol" + "started"),
                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineRefillPharmacyReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            instantiate_protocol("medicine_pharmacy_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }


        std::string getCurrentDateTime() {
            auto currentTimePoint = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
            std::tm *timeInfo = std::localtime(&currentTime);
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
            return buffer;
        }


    };



}

