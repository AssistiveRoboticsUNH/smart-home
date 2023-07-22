#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/call_request.hpp"
#include "shr_msgs/action/text_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "shr_msgs/action/read_script_request.hpp"
#include "shr_msgs/action/play_video_request.hpp"

#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        InstantiatedParameter active_protocol;
        std::shared_ptr<WorldStateListener> world_state_converter;
        std::unordered_map<std::string, std::unordered_map<std::string, int>> wait_times = {
                {"WanderingProtocol", {{"automated_msg", 0},
                                       {"recorded_msg", 30},
                                       {"call_caregiver_outside_msg", 0},
                                       {"call_emergency_msg", 60},
                                       {"call_caregiver_bed_msg", 30},
                }
                }
        };

        // action servers
        rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr call_client_ = {};
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ = {};
        rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_ = {};
        rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SharedPtr video_action_client_ = {};


        static ProtocolState &getInstance() {
            static ProtocolState instance;
            return instance;
        }

    private:
        ProtocolState() {} // Private constructor to prevent direct instantiation
        ~ProtocolState() {} // Private destructor to prevent deletion
        ProtocolState(const ProtocolState &) = delete; // Disable copy constructor
        ProtocolState &operator=(const ProtocolState &) = delete; // Disable assignment operator
    };

    BT::NodeStatus observe_wait_for_cond(const InstantiatedAction &action,
                                         std::chrono::steady_clock::time_point &init_time,
                                         const std::function<bool()> &cond,
                                         std::chrono::seconds wait_time) {
        ProtocolState &ps = ProtocolState::getInstance();
        auto startTime = std::chrono::steady_clock::now();
        if (startTime > init_time) {
            init_time = startTime + wait_time;
        }

        while (std::chrono::steady_clock::now() < init_time) {
            auto active_protocol = get_active_protocol().value();
            if (active_protocol.type != action.parameters[0].type) {
                abort();
            }
            auto msg = ps.world_state_converter->get_world_state_msg();
            if (cond()) {
                return BT::NodeStatus::SUCCESS;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        return BT::NodeStatus::FAILURE;
    }

    int send_goal_blocking(const shr_msgs::action::CallRequest::Goal &goal, const InstantiatedAction &action) {
        auto &ps = ProtocolState::getInstance();
        auto &kb = KnowledgeBase::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::CallRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::CallRequest>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.call_client_->async_send_goal(goal, send_goal_options);
        rclcpp::sleep_for(std::chrono::seconds(15)); //automatically wait because call is not blocking
        auto tmp = ps.active_protocol;
        while (success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.call_client_->async_cancel_all_goals().wait();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    int send_goal_blocking(const nav2_msgs::action::NavigateToPose::Goal &goal, const InstantiatedAction &action) {
        auto &ps = ProtocolState::getInstance();
        auto &kb = KnowledgeBase::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.nav_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.nav_client_->async_cancel_all_goals().wait();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    int send_goal_blocking(const shr_msgs::action::ReadScriptRequest::Goal &goal, const InstantiatedAction &action) {
        auto &ps = ProtocolState::getInstance();
        auto &kb = KnowledgeBase::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::ReadScriptRequest>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.read_action_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.read_action_client_->async_cancel_all_goals().wait();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    int send_goal_blocking(const shr_msgs::action::PlayVideoRequest::Goal &goal, const InstantiatedAction &action) {
        auto &ps = ProtocolState::getInstance();
        auto &kb = KnowledgeBase::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::PlayVideoRequest>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.video_action_client_->async_send_goal(goal, send_goal_options);
        auto tmp = ps.active_protocol;
        while (success == -1) {
            if (!(tmp == ps.active_protocol)) {
                ps.video_action_client_->async_cancel_all_goals().wait();
                return false;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    long get_inst_index_helper(const InstantiatedAction &action) {
        auto &ps = ProtocolState::getInstance();
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

    void instantiate_protocol(const std::string &protocol_name) {
        auto &kb = KnowledgeBase::getInstance();
        if (protocol_name != "high_level.pddl") {
            auto high_level_domain_content = get_file_content("high_level_domain.pddl");
            auto high_level_domain = parse_domain(high_level_domain_content).value();
            auto current_high_level = parse_problem(kb.convert_to_problem(high_level_domain),
                                                    high_level_domain_content).value();

            auto protocol_content = get_file_content("problem_" + protocol_name);
            auto domain_content = get_file_content("low_level_domain.pddl");
            auto prob = parse_problem(protocol_content, domain_content).value();

            kb.clear();
            kb.load_kb(current_high_level);
            kb.load_kb(prob);
        } else {
            auto protocol_content = get_file_content("problem_" + protocol_name);
            auto domain_content = get_file_content("high_level_domain.pddl");
            auto prob = parse_problem(protocol_content, domain_content).value();

            kb.clear();
            kb.load_kb(prob);
        }

    }

    class ProtocolActions : public pddl_lib::ActionInterface {
    public:
        BT::NodeStatus high_level_domain_Idle(const InstantiatedAction &action) override {
            auto &ps = ProtocolState::getInstance();
            ps.call_client_->async_cancel_all_goals().wait();
            ps.read_action_client_->async_cancel_all_goals().wait();
            ps.video_action_client_->async_cancel_all_goals().wait();
            ps.nav_client_->async_cancel_all_goals().wait();

            instantiate_protocol("high_level.pddl");
            ProtocolState::getInstance().active_protocol = {};
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_StartWanderingProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            instantiate_protocol("midnight.pddl");
            ProtocolState::getInstance().active_protocol = inst;
            return BT::NodeStatus::SUCCESS;
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            instantiate_protocol("medicine.pddl");
            ProtocolState::getInstance().active_protocol = inst;
            return BT::NodeStatus::SUCCESS;
        }

        // food_protocol
        BT::NodeStatus high_level_domain_StartFoodProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            instantiate_protocol("food.pddl");
            ProtocolState::getInstance().active_protocol = inst;
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_FoodEatenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedPredicate pred{"already_ate", {ProtocolState::getInstance().active_protocol}};
            kb.insert_predicate(pred);

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            auto active_protocol = ProtocolState::getInstance().active_protocol;
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
            auto active_protocol = ProtocolState::getInstance().active_protocol;
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
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction &action) override {
            auto &ps = ProtocolState::getInstance();
            std::string location = action.parameters[2].name;

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
            auto &ps = ProtocolState::getInstance();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            int wait_time = ps.wait_times.at(ps.active_protocol.type).at(msg);
            for (int i = 0; i < wait_time; i++) {
                if (!kb.check_conditions(action.precondtions)) {
                    abort();
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            if (msg == "automated_msg") {
                shr_msgs::action::ReadScriptRequest::Goal read_goal_;
                read_goal_.script_name = "midnight_reminder.txt";
                return send_goal_blocking(read_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else if (msg == "recorded_msg") {
                shr_msgs::action::PlayVideoRequest::Goal video_goal_;
                video_goal_.file_name = "midnight_reminder.mp3";
                return send_goal_blocking(video_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::FAILURE;;
            }
        }

        BT::NodeStatus shr_domain_MakeCall(const InstantiatedAction &action) override {
            auto &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            int wait_time = ps.wait_times.at(ps.active_protocol.type).at(msg);
            for (int i = 0; i < wait_time; i++) {
                if (!kb.check_conditions(action.precondtions)) {
                    abort();
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            shr_msgs::action::CallRequest::Goal call_goal_;
            if (msg == "call_caregiver_outside_msg") {
                call_goal_.script_name = "call_msg_leaving_house.xml";
                call_goal_.phone_number = params.caregiver_phone_number;
                return send_goal_blocking(call_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else if (msg == "call_caregiver_bed_msg") {
                call_goal_.script_name = "call_msg_will_not_go_to_bed.xml";
                call_goal_.phone_number = params.caregiver_phone_number;
                return send_goal_blocking(call_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else if (msg == "call_emergency_msg") {
                call_goal_.script_name = "call_msg_911.xml";
                call_goal_.phone_number = params.emergency_phone_number;
                return send_goal_blocking(call_goal_, action) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::FAILURE;;
            }
        }

        BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction &action) override {
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction &action) override {
            auto &ps = ProtocolState::getInstance();
            std::string lm = action.parameters[2].name;
            if (ps.world_state_converter->check_person_at_loc(lm)) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }

    };
}
