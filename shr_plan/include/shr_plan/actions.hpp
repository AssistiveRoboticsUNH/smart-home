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
#include <shr_plan/intersection_helpers.hpp>


namespace pddl_lib {

    class ProtocolState {
    public:
        InstantiatedParameter active_protocol;
        std::shared_ptr <WorldStateListener> world_state_converter;
        // change first to change time (x  before y after)
        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::pair < int, int>>>
        // Msg in PDDL
        // name field should be the same as the name of the protocol in the high_level_problem
        // mak sure the txt files and mp3 are in shr_resources
        wait_times = {
                {{"am_meds",                           "MedicineProtocol"},                       {{"reminder_1_msg", {0, 1}},
                                                                                                          {"reminder_2_msg", {0, 1}},
                                                                                                          {"wait", {2, 0}},
                                                                                                  }},
                {{"pm_meds",                           "MedicineProtocol"},                       {{"reminder_1_msg", {0, 12}},
                                                                                                          {"reminder_2_msg", {0, 12}},
                                                                                                          {"wait", {2, 0}},
                                                                                                  }},
                {{"gym_reminder",                      "GymReminderProtocol"},                    {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},
                {{"medicine_refill_reminder",          "MedicineRefillReminderProtocol"},         {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},
                {{"medicine_pharmacy_reminder", "MedicineRefillPharmacyReminderProtocol"}, {{"reminder_1_msg", {0, 1}},
                                                                                                          {"wait",           {0, 0}},

                                                                                                  }},

        };


        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::string>> automated_reminder_msgs = {
                {{"am_meds",       "MedicineProtocol"},              {{"reminder_1_msg", "am_med_reminder.txt"},
                                                                     }},
                {{"pm_meds",       "MedicineProtocol"},              {{"reminder_1_msg", "pm_med_reminder.txt"},
                                                                     }},
                {{"gym_reminder",          "GymReminderProtocol"},          {{"reminder_1_msg", "gym_reminder1.txt"},
                                                                     }},
                {{"medicine_refill_reminder",      "MedicineRefillReminderProtocol"},      {{"reminder_1_msg", "medicine_refill.txt"},
                                                                     }},
                {{"medicine_pharmacy_reminder",      "MedicineRefillPharmacyReminderProtocol"},      {{"reminder_1_msg", "pharmacy_refill.txt"},
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
        rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SharedPtr localize_ = {};
        rclcpp_action::Client<shr_msgs::action::PlayAudioRequest>::SharedPtr audio_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr call_client_ = {};

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

        BT::NodeStatus charge_robot(ProtocolState &ps, const InstantiatedAction &action, bool pred_started){
            std::cout << "ps.world_state_converter->get_world_state_msg()->robot_charging" << ps.world_state_converter->get_world_state_msg()->robot_charging  << std::endl;
            std::cout << "pred_started" << pred_started << std::endl;

            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1 && pred_started ) {
                std::cout << "High level claim robot called " << std::endl;
                auto robot_resource = ps.claimRobot();
                ps.read_action_client_->async_cancel_all_goals();
                ps.audio_action_client_->async_cancel_all_goals();
                ps.undocking_->async_cancel_all_goals();
                ps.docking_->async_cancel_all_goals();


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
                    // lock.UnLock();
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
                    //lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                ps.docking_->async_cancel_all_goals();
                std::cout << "success: " << std::endl;
                // comment in sim

                // // sleep for 60 seconds to deal with the delay from //charging topic
                std::cout << " waiting  " << std::endl;
                rclcpp::sleep_for(std::chrono::seconds(30));

                std::cout << "High level ending " << std::endl;

            }
            // for safety have it undock so that nav2 doesnt have to move when the robot is sp close to the docking
            if (ps.world_state_converter->get_world_state_msg()->robot_charging != 1){
                std::cout << "Undock " << std::endl;

                shr_msgs::action::DockingRequest::Goal goal_msg;

                auto success_undock = std::make_shared < std::atomic < int >> (-1);
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

                // indicating that robot didnt charge itself and needs ot start again
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::SUCCESS;

        }

        // Timeout for now doesnt do anything inrodere for the protocol to be retriggered
        BT::NodeStatus high_level_domain_Idle(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            kb.clear_unknowns();
            kb.insert_predicate({"abort", {}});

            bool pred_started = kb.find_predicate({"started", {}});
            std::cout <<  "kb.find_predicate " << pred_started << std::endl;

            // CHECKING IF ROBOT IS CHARGING FIRST
            auto [ps, lock] = ProtocolState::getConcurrentInstance();

            RCLCPP_INFO(rclcpp::get_logger(std::string("user=") + "high_level_domain_Idle" + "started"), "user...");

            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_Idle " + " started!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            RCLCPP_INFO(
                    rclcpp::get_logger(std::string("weblog=") + "high_level_domain_Idle" + "Navigation started"),
                    "user...");

            lock.Lock();
            BT::NodeStatus status = charge_robot(ps, action, pred_started);

            std::cout << "%%%%%%%  IDLE %%%%%%%  IDLE " << std::endl;

            ps.active_protocol = {};
            lock.UnLock();
            return status;
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
        BT::NodeStatus high_level_domain_StartMedReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];


            // instantiate_protocol("medicine_reminder.pddl", {{"current_loc", cur.name},
            //                                                 {"dest_loc",    dest.name}});
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineProtocol" + " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            if (dest.name == cur.name) {
                RCLCPP_INFO(rclcpp::get_logger("debug"),
                            "StartMedicineProtocol: Robot is already at %s. Skipping movement.", cur.name.c_str());
                // Just proceed with the protocol without moving
                instantiate_protocol("medicine_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", "bedroom"}});
            } else {
                // Move to the medicine location if not already there
                instantiate_protocol("medicine_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", dest.name}});
            }
            ps.active_protocol = protocol;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // Gym protocol
        BT::NodeStatus high_level_domain_StartGymReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();

            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartExerciseReminderProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartGymReminderProtocol" + "started"),
                        "user...");

            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartGymReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());



            if (dest.name == cur.name) {
                RCLCPP_INFO(rclcpp::get_logger("debug"),
                            "StartGymReminderProtocol: Robot is already at %s. Skipping movement.", cur.name.c_str());
                // Just proceed with the protocol without moving
                instantiate_protocol("gym_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", "bedroom"}});
            } else {
                // Move to the medicine location if not already there
                instantiate_protocol("gym_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", dest.name}});
            }

            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }


        // StartMedicineRefill protocol
        BT::NodeStatus high_level_domain_StartMedicineRefillReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];

            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();

            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartMedicineRefillReminderProtocol" + "started"),
                        "user...");

            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineRefillReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            if (dest.name == cur.name) {
                RCLCPP_INFO(rclcpp::get_logger("debug"),
                            "StartGymReminderProtocol: Robot is already at %s. Skipping movement.", cur.name.c_str());
                // Just proceed with the protocol without moving
                instantiate_protocol("medicine_refill_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", "bedroom"}});
            } else {
                // Move to the medicine location if not already there
                instantiate_protocol("medicine_refill_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", dest.name}});
            }

            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // MedicineRefillPharmacy check protocol
        BT::NodeStatus high_level_domain_StartMedicineRefillPharmacyReminderProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];

            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();

            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
            RCLCPP_INFO(rclcpp::get_logger(
                    currentDateTime + std::string("user=") + "StartMedicineRefillPharmacyReminderProtocol" + "started"),
                        "user...");

            std::string log_message =
                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineRefillPharmacyReminderProtocol" +
                    " started";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            if (dest.name == cur.name) {
                RCLCPP_INFO(rclcpp::get_logger("debug"),
                            "StartGymReminderProtocol: Robot is already at %s. Skipping movement.", cur.name.c_str());
                // Just proceed with the protocol without moving
                instantiate_protocol("medicine_pharmacy_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", "bedroom"}});
            } else {
                // Move to the medicine location if not already there
                instantiate_protocol("medicine_pharmacy_reminder.pddl", {{"current_loc", cur.name}, {"dest_loc", dest.name}});
            }


            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_MoveToLandmark(const InstantiatedAction &action) override {
            InstantiatedParameter from = action.parameters[0];
            InstantiatedParameter to = action.parameters[1];
            InstantiatedParameter t1 = {"t1", "Time"};
            InstantiatedAction action_inst = {"MoveToLandmark",
                                              {t1, from, to}};
            return shr_domain_MoveToLandmark(action_inst);
        }
        BT::NodeStatus high_level_domain_Shutdown(const InstantiatedAction &action) override {
            std::cout << " ------ Shutdown  ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();

            BT::NodeStatus status = BT::NodeStatus::FAILURE;
            auto [ps, lock] = ProtocolState::getConcurrentInstance();

            // dock the robot if it is not charging
            while (status !=BT::NodeStatus::SUCCESS){
                /// TODO: IF IT RUNS FOR TOO LONG ISSUE MIGHT BE IN THE CHARGER
                /// TODO: DISPLAY A WARNING ON THE SCREEN THAT IT NEEDS HELP
                lock.Lock();
                status = charge_robot(ps, action, true);
                lock.UnLock();
            }

            // Get keyword predicates to load them in next protocol
            std::cout << " RUNNING MATCH " << std::endl;
            std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
            std::filesystem::path keywordsFile = pkg_dir / "include" / "shr_plan" / "keywords.txt";

            const char* homeDir = std::getenv("HOME");

            std::filesystem::path outputFile = pkg_dir / "include" / "shr_plan" / "intersection.txt";
            const std::unordered_map<std::string, std::string> protocol_type_ = {
                    {"am_meds", "MedicineProtocol"},
                    {"pm_meds", "MedicineProtocol"},
                    {"gym_reminder", "GymReminderProtocol"},
                    {"medicine_refill_reminder", "MedicineRefillReminderProtocol"},
                    {"medicine_pharmacy_reminder", "MedicineRefillPharmacyReminderProtocol"}
            };

            const std::unordered_map<std::string, std::vector<std::string>> keyword_protocol_ = {
                    {"already_took_medicine", {"am_meds", "pm_meds"}},
                    {"already_reminded_medicine", {"am_meds", "pm_meds"}},
                    {"already_called_about_medicine", {"am_meds", "pm_meds"}},
                    {"already_reminded_gym",{"gym_reminder"}},
                    {"already_reminded_medicine_refill",{"medicine_refill_reminder"}},
                    {"already_reminded_medicine_pharmacy",{"medicine_pharmacy_reminder"}}
            };

            std::ifstream ifs(keywordsFile);
            if (!ifs) {
                std::cerr << "Failed to open keywords file: " << keywordsFile << std::endl;
//                return BT::NodeStatus::FAILURE;
            }

            std::vector<std::tuple<std::string, std::string, std::string>> keyword_protocol_list;
            std::string line;

            while (std::getline(ifs, line)) {
                // Here, 'line' is the keyword
                // make sure no leading space
                // TODO: trim leading space
                std::string keyword = line;

                // Check if the keyword exists in keyword_protocol_.
                auto keywordIt = keyword_protocol_.find(keyword);
                if (keywordIt != keyword_protocol_.end()) {

                    // For each protocol name associated with this keyword...
                    for (const auto& protocolName : keywordIt->second) {
                        // Look up the protocol type using protocol_type_.
                        auto typeIt = protocol_type_.find(protocolName);
                        if (typeIt != protocol_type_.end()) {
                            // Create an InstantiatedParameter with the protocol name and its type.
                            InstantiatedParameter active_protocol { protocolName, typeIt->second };
                            InstantiatedPredicate pred{keyword, {active_protocol}};

                            // "Find" the predicate in the knowledge base.
                            if (kb.find_predicate(pred)){
                                // add to the list
                                keyword_protocol_list.emplace_back(keyword, protocolName, typeIt->second);
                            }

                        } else {
                            std::cerr << "Protocol name '" << protocolName
                                      << "' not found in protocol_type_." << std::endl;
                        }
                    }


                } else {
                    std::cout << "Keyword '" << keyword << "' not associated with any protocol." << std::endl;
                }
            }
            ifs.close();

            write_to_intersection(outputFile.c_str(), keyword_protocol_list);

            
            // KILING ROS2 

            std::system("python3 /home/hello-robot/kill_ros.py");
            
            rclcpp::sleep_for(std::chrono::seconds(120));
        
            // reboot
            std::cout << " RUNNING REBOOT " << std::endl;

           const char* password = std::getenv("robot_pass");

           if (!password) {
               std::cerr << "Environment variable 'robot_pass' not set!" << std::endl;
               BT::NodeStatus::FAILURE;
           }

           std::string cmd_reboot = "echo '" + std::string(password) + "' | sudo -S reboot";
           std::system(cmd_reboot.c_str());



            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_StartROS(const InstantiatedAction &action) override {
            std::cout << " ------ Start ros ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();

            RCLCPP_INFO(rclcpp::get_logger("########## STARTT #################"), "Your message here");

            const char* homeDir = std::getenv("HOME");
            std::string cmd_startros = std::string(homeDir);
            cmd_startros += "/start_nav.sh";
            std::system(cmd_startros.c_str());

            std::cout << " ------ finish start ----" << std::endl;

            // start actions servers and navigation

//            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartMoveReminderProtocol" + "started"),
//                        "user...");
//            auto [ps, lock] = ProtocolState::getConcurrentInstance();
//            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMoveReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

//            instantiate_protocol("move_reminder.pddl");
//            instantiate_protocol("move_reminder.pddl", {{"current_loc", cur.name},
//                                                        {"dest_loc",    dest.name}});
//            ps.active_protocol = inst;
//            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MedicineTakenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate pred{"already_took_medicine", {ps.active_protocol}};
            kb.insert_predicate(pred);
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " Patient took medicine!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_NoActionUsed(const InstantiatedAction &action) override {
            // if person doesn't go to the visible area within 5 mins it
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
//            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {


            auto start_time = std::chrono::steady_clock::now();
            auto timeout = std::chrono::minutes(1);
            std::cout << "************** Noaction **************" << std::endl;
            while (std::chrono::steady_clock::now() - start_time < timeout) {
                if (ps.world_state_converter->check_person_at_loc("visible_area")) {
                    std::string currentDateTime = getCurrentDateTime();
                    std::string log_message = std::string("weblog=") + currentDateTime + " No action!";
                    RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                    std::this_thread::sleep_for(std::chrono::seconds(20));
                    lock.UnLock();
                    return BT::NodeStatus::SUCCESS;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Check every second
            }


            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " No action!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_FoodEatenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate pred{"already_ate", {ps.active_protocol}};
            kb.insert_predicate(pred);
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " Patient finished food!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_TimeOut(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            kb.insert_predicate({"abort", {}});
            std::cout << "TIMEout" << std::endl;

            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " Abort!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto active_protocol = ps.active_protocol;
            //std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_reminded_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_reminder_enabled", {active_protocol}});
            }else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            }else if (active_protocol.type == "MoveReminderProtocol") {
                kb.insert_predicate({"already_reminded_move", {active_protocol}});
                kb.erase_predicate({"move_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "InternalCheckReminderProtocol") {
                kb.insert_predicate({"already_reminded_internal_check", {active_protocol}});
                kb.erase_predicate({"internal_check_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "PracticeReminderProtocol") {
                kb.insert_predicate({"already_reminded_practice", {active_protocol}});
                kb.erase_predicate({"practice_reminder_enabled", {active_protocol}});
            }else if (active_protocol.type == "ExerciseReminderProtocol") {
                kb.insert_predicate({"already_reminded_exercise", {active_protocol}});
                kb.erase_predicate({"exercise_reminder_enabled", {active_protocol}});
            }

            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MessageGivenSuccess"+active_protocol.type), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime +std::string("user=")+"Message is given for: "+active_protocol.type), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " Message is given for: " + active_protocol.type;
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_PersonAtSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto active_protocol = ps.active_protocol;
            //std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_reminded_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_reminder_enabled", {active_protocol}});
            }else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            }else if (active_protocol.type == "MoveReminderProtocol") {
                kb.insert_predicate({"already_reminded_move", {active_protocol}});
                kb.erase_predicate({"move_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "InternalCheckReminderProtocol") {
                kb.insert_predicate({"already_reminded_internal_check", {active_protocol}});
                kb.erase_predicate({"internal_check_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "PracticeReminderProtocol") {
                kb.insert_predicate({"already_reminded_practice", {active_protocol}});
                kb.erase_predicate({"practice_reminder_enabled", {active_protocol}});
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_PersonAtSuccess"+active_protocol.type), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"active protocol"+active_protocol.type), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " shr_domain_PersonAtSuccess " + active_protocol.type;
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_Wait(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = "wait";
            //std::string currentDateTime = getCurrentDateTime();
            //  fix for all 
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;

            for (int i = 0; i < wait_time; i++) {
                if (ps.world_state_converter->get_world_state_msg()->person_taking_medicine == 1 && ps.active_protocol.type == "MedicineProtocol"){
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_Wait " + "medicine taken!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::SUCCESS;
                }
                if (ps.world_state_converter->get_world_state_msg()->person_eating == 1 && ps.active_protocol.type == "FoodProtocol"){
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_Wait " + "food eaten!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::SUCCESS;
                }
                rclcpp::sleep_for(std::chrono::seconds(10));
            }

            lock.UnLock();
            return BT::NodeStatus::SUCCESS;;
        }

        BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto t = action.parameters[0];
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate ate_food = {"person_eating", {t}};
            if (kb.find_predicate(ate_food)) {
                //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectEatingFood" + "ate food success"), "user...");
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message = std::string("weblog=") + currentDateTime + " person is eating food";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectEatingFood"+"ate food failure!"), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"person is not eating food!"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " person is not eating food";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::FAILURE;
        }



        BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction &action) override {
            /// move robot to location
            RCLCPP_INFO(
                    rclcpp::get_logger(std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark!"),
                    "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string location = action.parameters[2].name;


            if (ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
                std::cout << "Undock " << std::endl;

                shr_msgs::action::DockingRequest::Goal goal_msg;

                auto success_undock = std::make_shared < std::atomic < int >> (-1);
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

                nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
                navigation_goal_.pose.header.frame_id = "map";
                navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
                if (auto transform = ps.world_state_converter->get_tf("map", location)) {
                    navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
                    navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
                    navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
                    navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
                } else {
                    RCLCPP_INFO(rclcpp::get_logger(
                                        std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark failed!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }

                RCLCPP_INFO(rclcpp::get_logger(
                                    std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark succeed!"),
                            "user...");
                lock.UnLock();
                return send_goal_blocking(navigation_goal_, action, ps) ? BT::NodeStatus::SUCCESS
                                                                        : BT::NodeStatus::FAILURE;
            } else {

                int count_max = 30;

                std::cout << "localize " << std::endl;
//                shr_msgs::action::LocalizeRequest::Goal goal_msg_loc;
//                goal_msg_loc.force_localize = false;
//
//                auto success_loc = std::make_shared < std::atomic < int >> (-1);
//                auto send_goal_options_loc = rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SendGoalOptions();
//                send_goal_options_loc.result_callback = [&success_loc](
//                        const rclcpp_action::ClientGoalHandle<shr_msgs::action::LocalizeRequest>::WrappedResult result) {
//                    *success_loc = result.code == rclcpp_action::ResultCode::SUCCEEDED;
//                };
//
//                ps.localize_->async_send_goal(goal_msg_loc, send_goal_options_loc);
//                auto tmp_loc = ps.active_protocol;
//
//                int count__ = 0;
//                while (*success_loc == -1 && count_max > count__) {
//                    if (!(tmp_loc == ps.active_protocol)) {
//                        ps.localize_->async_cancel_all_goals();
//                        std::cout << " Failed " << std::endl;
//                    }
//                    count__++;
//                    rclcpp::sleep_for(std::chrono::seconds(1));
//                }


                nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
                navigation_goal_.pose.header.frame_id = "map";
                navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
                if (auto transform = ps.world_state_converter->get_tf("map", location)) {
                    std::cout << "degug location moveto landmark" << location << std::endl;
                    navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
                    navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
                    navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
                    navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
                } else {
                    RCLCPP_INFO(rclcpp::get_logger(
                                        std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark failed!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }

                RCLCPP_INFO(rclcpp::get_logger(
                                    std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark succeed!"),
                            "user...");
                lock.UnLock();
                return send_goal_blocking(navigation_goal_, action, ps) ? BT::NodeStatus::SUCCESS
                                                                        : BT::NodeStatus::FAILURE;
            }

            //    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MoveToLandmark"+"moving to land mark succeed!"), "user...");
            //     shr_msgs::action::WaypointRequest ::Goal waypoint_goal_;
            //     waypoint_goal_.from_location = action.parameters[1].name;
            //     waypoint_goal_.to_location = action.parameters[2].name;

            // return send_goal_blocking(navigation_goal_, action, ps) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_GiveReminder(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            //std::string currentDateTime = getCurrentDateTime();
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_GiveReminder" + "failed!"),
                                "user...");
                    lock.UnLock();
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

                ret = send_goal_blocking(read_goal_, action, ps) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else {
                shr_msgs::action::PlayAudioRequest::Goal audio_goal_;
                audio_goal_.file_name = ps.recorded_reminder_msgs.at(ps.active_protocol).at(msg);
                script_name_str = std::string(audio_goal_.file_name.begin(), audio_goal_.file_name.end());

                ret = send_goal_blocking(audio_goal_, action, ps) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            if (ret == BT::NodeStatus::SUCCESS) {
                // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+script_name_str+"succeed!"), "user...");
                // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"GiveReminder"+script_name_str+"succeed!"), "user...");
                // rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message =
                        std::string("weblog=") + currentDateTime + " GiveReminder" + script_name_str + " succeed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
                // wait_time = ps.wait_times.at(ps.active_protocol).at(msg).second;
                // for (int i = 0; i < wait_time; i++) {
                // if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::TRUE) {
                //     abort(action);
                //     RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_GiveReminder" + "succeeded during wait time after reminder!"),
                //                 "user...");
                //     lock.UnLock();
                //     return BT::NodeStatus::SUCCESS;
                // }
                //     rclcpp::sleep_for(std::chrono::seconds(1));
                // }

            } else {
                // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+script_name_str+"failed!"), "user...");
                // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"GiveReminder"+script_name_str+"failed!"), "user...");
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message =
                        std::string("weblog=") + currentDateTime + " GiveReminder" + script_name_str + " failed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            }
            lock.UnLock();
            return ret;
        }

        BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto t = action.parameters[0];
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate took_medicine = {"person_taking_medicine", {t}};
            if (kb.find_predicate(took_medicine)) {
                // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"succeeded"), "user...");
                // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"succeeded"), "user...");
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message = std::string("weblog=") + currentDateTime + " Taking Medicine" + " succeed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"failed"), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"failed"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " Taking Medicine" + " succeed!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string currentDateTime = getCurrentDateTime();
            std::string lm = action.parameters[2].name;
            if (ps.world_state_converter->check_person_at_loc(lm)) {
                RCLCPP_INFO(
                        rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectPersonLocation" + "succeeded"),
                        "user...");
                RCLCPP_INFO(rclcpp::get_logger(
                        currentDateTime + std::string("user=") + "person location detection" + "succeeded"), "user...");
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectTakingMedicine" + "failed"),
                            "user...");

                lock.UnLock();
                return BT::NodeStatus::FAILURE;
            }
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
