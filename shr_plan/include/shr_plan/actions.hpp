#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/call_request.hpp"
#include "shr_msgs/action/text_request.hpp"
#include "shr_msgs/action/docking_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "shr_msgs/action/read_script_request.hpp"
#include "shr_msgs/action/play_video_request.hpp"
#include "shr_msgs/action/docking_request.hpp"
#include "shr_msgs/action/localize_request.hpp"

#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        InstantiatedParameter active_protocol;
        std::shared_ptr<WorldStateListener> world_state_converter;
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
                {{"daily_wand", "WanderingProtocol"}, {{"call_caregiver_outside_msg", {"call_msg_leaving_house.xml", "7742257735"}},
                                                              {"call_caregiver_bed_msg", {"call_msg_will_not_go_to_bed.xml", "7742257735"}},
                                                              {"call_emergency_msg", {"call_msg_911.xml", "7742257735"}},
                                                      }},
                {{"daily_med",  "MedicineProtocol"},  {{"call_caregiver_guide_msg",   {"call_msg_medical.xml",       "7742257735"}},
                                                              {"call_caregiver_msg",     {"call_msg_medical.xml",            "7742257735"}},
                                                      }},
                {{"dinner",     "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7742257735"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7742257735"}},
                                                      }},
                {{"lunch",      "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7742257735"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7742257735"}},
                                                      }},
                {{"breakfast",  "FoodProtocol"},      {{"call_caregiver_guide_msg",   {"call_msg_food.xml",          "7742257735"}},
                                                              {"call_caregiver_msg",     {"call_msg_food.xml",               "7742257735"}},
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
                {{"dinner",     "FoodProtocol"},      {{"recorded_msg", "food_reminder.mp3"},
                                                      }}
        };

        // action servers
        rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr call_client_ = {};
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ = {};
        rclcpp_action::Client<shr_msgs::action::DockingRequest>::SharedPtr docking_ = {};
        rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SharedPtr localize_ = {};
        rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SharedPtr video_action_client_ = {};


        static InstantiatedParameter getActiveProtocol() {
            std::lock_guard<std::mutex> lock(getInstance().active_protocol_mtx);
            return getInstance().active_protocol;
        }

        static std::pair<ProtocolState &, std::scoped_lock<std::mutex> &&> getConcurrentInstance() {
            std::scoped_lock<std::mutex> lock_guard(getInstance().mtx);
            return {getInstance(), std::move(lock_guard)};
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
            *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.nav_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (*success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.nav_client_->async_cancel_all_goals();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
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

    int send_goal_blocking(const shr_msgs::action::PlayVideoRequest::Goal &goal, const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        auto &kb = KnowledgeBase::getInstance();
        auto success = std::make_shared<std::atomic<int>>(-1);
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::PlayVideoRequest>::WrappedResult result) {
            *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.video_action_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (*success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.video_action_client_->async_cancel_all_goals();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
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

            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1){
                ps.call_client_->async_cancel_all_goals();
                ps.read_action_client_->async_cancel_all_goals();
                ps.video_action_client_->async_cancel_all_goals();
//            ps.docking_->async_cancel_all_goals();
//            ps.localize_->async_cancel_all_goals();

                int count_max = 30;

                std::cout << "localize " << std::endl;
                shr_msgs::action::LocalizeRequest::Goal goal_msg_loc;
                goal_msg_loc.force_localize = false;

                auto success_loc = std::make_shared<std::atomic<int>>(-1);
                auto send_goal_options_loc = rclcpp_action::Client<shr_msgs::action::LocalizeRequest>::SendGoalOptions();
                send_goal_options_loc.result_callback = [&success_loc](
                        const rclcpp_action::ClientGoalHandle<shr_msgs::action::LocalizeRequest>::WrappedResult result) {
                    *success_loc = result.code == rclcpp_action::ResultCode::SUCCEEDED;
                };

                ps.localize_->async_send_goal(goal_msg_loc, send_goal_options_loc);
                auto tmp_loc = ps.active_protocol;

                int count__ = 0;
                while (*success_loc == -1 && count_max > count__) {
                    if (!(tmp_loc == ps.active_protocol)) {
                        ps.docking_->async_cancel_all_goals();
                        std::cout << " Failed " << std::endl;
                    }
                    count__++;
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }


                std::cout << "navigate " << std::endl;
                auto success = std::make_shared<std::atomic<int>>(-1);
                auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                send_goal_options.result_callback = [&success](
                        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
                    *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
                };

                nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
                navigation_goal_.pose.header.frame_id = "map";
                navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
                if (auto transform = ps.world_state_converter->get_tf("map", "home")) {
                    navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
                    navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
                    navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
                    navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
                }

                ps.nav_client_->async_send_goal(navigation_goal_, send_goal_options);
                auto tmp = ps.active_protocol;
                // TODO take this out // it is causing an infinite loop Add docking

                int count = 0;
                while (*success == -1 && count_max > count) {
                    if (!(tmp == ps.active_protocol)) {
                        ps.nav_client_->async_cancel_all_goals();
                        // goal not reached TODO change to retry going to goal
                    }
                    count++;
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }

                std::cout << "dock " << std::endl;

                //// TODO :: stop if not docked
                shr_msgs::action::DockingRequest::Goal goal_msg;

                auto success_dock = std::make_shared<std::atomic<int>>(-1);
                auto send_goal_options_dock = rclcpp_action::Client<shr_msgs::action::DockingRequest>::SendGoalOptions();
                send_goal_options_dock.result_callback = [&success_dock](
                        const rclcpp_action::ClientGoalHandle<shr_msgs::action::DockingRequest>::WrappedResult result) {
                    *success_dock = result.code == rclcpp_action::ResultCode::SUCCEEDED;
                };

                ps.docking_->async_send_goal(goal_msg, send_goal_options_dock);
                auto tmp_dock = ps.active_protocol;

                int count_ = 0;
                while (*success_dock == -1 && count_max > count_) {
                    if (!(tmp_dock == ps.active_protocol)) {
                        ps.docking_->async_cancel_all_goals();
                        std::cout << " Failed " << std::endl;
                    }
                    count_++;
                    rclcpp::sleep_for(std::chrono::seconds(1));
                }

                ps.active_protocol = {};

            }

            return BT::NodeStatus::SUCCESS;
        }

        void abort(const InstantiatedAction &action) override {
            std::cout << "abort: higher priority protocol detected\n";
            auto &kb = KnowledgeBase::getInstance();
            kb.insert_predicate({"abort", {}});
        }

        BT::NodeStatus high_level_domain_StartWanderingProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            instantiate_protocol("midnight.pddl");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            ps.active_protocol = inst;
            return BT::NodeStatus::SUCCESS;
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            if (dest.name == cur.name) {
                cur.name = "couch";
            }
            instantiate_protocol("food.pddl", {{"current_loc", cur.name},
                                               {"dest_loc",    dest.name}});
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            ps.active_protocol = protocol;

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

        // food_protocol
        BT::NodeStatus high_level_domain_StartFoodProtocol(const InstantiatedAction &action) override {

            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            if (dest.name == cur.name) {
                cur.name = "couch";
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
            InstantiatedPredicate pred{"already_ate", {ps.active_protocol}};
            kb.insert_predicate(pred);

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto active_protocol = ps.active_protocol;
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_called_about_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "WanderingProtocol") {
                kb.erase_predicate({"wandering_protocol_enabled", {active_protocol}});
            }

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_PersonAtSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto active_protocol = ps.active_protocol;
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_called_about_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "FoodProtocol") {
                kb.insert_predicate({"already_called_about_eating", {active_protocol}});
                kb.erase_predicate({"food_protocol_enabled", {active_protocol}});
            } else if (active_protocol.type == "WanderingProtocol") {
                kb.erase_predicate({"wandering_protocol_enabled", {active_protocol}});
            }

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto t = action.parameters[0];
            InstantiatedPredicate ate_food = {"person_eating", {t}};
            if (kb.find_predicate(ate_food)) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction &action) override {

            /// move robot to location
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
//                    ps.docking_->async_cancel_all_goals();
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
                return BT::NodeStatus::FAILURE;
            }

            return send_goal_blocking(navigation_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_GiveReminder(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    return BT::NodeStatus::FAILURE;
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            BT::NodeStatus ret;
            if (ps.automated_reminder_msgs.at(ps.active_protocol).find(msg) !=
                ps.automated_reminder_msgs.at(ps.active_protocol).end()) {
                shr_msgs::action::ReadScriptRequest::Goal read_goal_;
                read_goal_.script_name = ps.automated_reminder_msgs.at(ps.active_protocol).at(msg);
                ret = send_goal_blocking(read_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else {
                shr_msgs::action::PlayVideoRequest::Goal video_goal_;
                video_goal_.file_name = ps.recorded_reminder_msgs.at(ps.active_protocol).at(msg);
                ret = send_goal_blocking(video_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            if (ret == BT::NodeStatus::SUCCESS) {
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
            }
            return ret;
        }

        BT::NodeStatus shr_domain_MakeCall(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            auto params = ps.world_state_converter->get_params();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    return BT::NodeStatus::FAILURE;
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            shr_msgs::action::CallRequest::Goal call_goal_;
            call_goal_.script_name = ps.call_msgs.at(ps.active_protocol).at(msg).first;
            call_goal_.phone_number = ps.call_msgs.at(ps.active_protocol).at(msg).second;
            auto ret = send_goal_blocking(call_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            if (ret == BT::NodeStatus::SUCCESS) {
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));
            }
            return ret;
        }

        BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto t = action.parameters[0];
            InstantiatedPredicate took_medicine = {"person_taking_medicine", {t}};
            if (kb.find_predicate(took_medicine)) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction &action) override {
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            std::string lm = action.parameters[2].name;
            if (ps.world_state_converter->check_person_at_loc(lm)) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }

    };
}
