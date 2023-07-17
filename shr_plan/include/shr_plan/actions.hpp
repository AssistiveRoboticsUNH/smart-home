#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/call_request.hpp"
#include "shr_msgs/action/text_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "shr_msgs/action/read_script_request.hpp"

#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        std::shared_ptr<WorldStateListener> world_state_converter;
        // midnight
        std::chrono::steady_clock::time_point time_wandering_protocol_detectPersonLeftHouse1 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_detectPersonLeftHouse2 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_CheckBedAfterReturn1 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_CheckBedAfterReturn2 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_checkIfPersonWentToBed1 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_checkIfPersonWentToBed2 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_waitForPersonToReturn1 = {};
        std::chrono::steady_clock::time_point time_wandering_protocol_waitForPersonToReturn2 = {};
        // medicine
        std::chrono::steady_clock::time_point time_medicine_protocol_checkGuideToSucceeded1 = {};
        std::chrono::steady_clock::time_point time_medicine_protocol_checkGuideToSucceeded2 = {};
        std::chrono::steady_clock::time_point time_notifyAutomatedMedicineAt = {};
        std::chrono::steady_clock::time_point time_notifyRecordedMedicineAt = {};
        // food
        std::chrono::steady_clock::time_point time_food_protocol_checkGuideToSucceeded1 = {};
        std::chrono::steady_clock::time_point time_food_protocol_checkGuideToSucceeded2 = {};
        std::chrono::steady_clock::time_point time_food_protocol_remindAutomatedFoodAt = {};
        std::chrono::steady_clock::time_point time_food_protocol_remindAutomatedFoodAt2 = {};
        // action servers
        rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr call_client_ = {};
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ = {};
        rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_ = {};


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
            auto active_protocol = get_active_protocol();
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

    int send_goal_blocking(const shr_msgs::action::CallRequest::Goal &goal) {
        auto &ps = ProtocolState::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::CallRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::CallRequest>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.call_client_->async_send_goal(goal, send_goal_options);

        while (success == -1) {
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    int send_goal_blocking(const nav2_msgs::action::NavigateToPose::Goal &goal) {
        auto &ps = ProtocolState::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.nav_client_->async_send_goal(goal, send_goal_options);

        while (success == -1) {
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        return success;
    }

    int send_goal_blocking(const shr_msgs::action::ReadScriptRequest::Goal &goal) {
        auto &ps = ProtocolState::getInstance();
        std::atomic<int> success = -1;
        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SendGoalOptions();
        send_goal_options.result_callback = [&success](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::ReadScriptRequest>::WrappedResult result) {
            success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        };
        ps.read_action_client_->async_send_goal(goal, send_goal_options);

        while (success == -1) {
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

    class ProtocolActions : public pddl_lib::ActionInterface {
    public:

        BT::NodeStatus high_level_domain_StartWanderingProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];

            kb.unknownPredicates.concurrent_insert({"person_decides_to_go_outside_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_decides_to_go_outside_2", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_decides_to_go_to_bed_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_decides_to_go_to_bed_2", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_decides_to_return_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_decides_to_return_2", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_goes_to_bed_after_return_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"person_goes_to_bed_after_return_2", {inst}});

            kb.knownPredicates.concurrent_erase({"person_decides_to_go_outside_1", {inst}});
            kb.knownPredicates.concurrent_erase({"person_decides_to_go_outside_2", {inst}});
            kb.knownPredicates.concurrent_erase({"person_decides_to_go_to_bed_1", {inst}});
            kb.knownPredicates.concurrent_erase({"person_decides_to_go_to_bed_2", {inst}});
            kb.knownPredicates.concurrent_erase({"person_decides_to_return_1", {inst}});
            kb.knownPredicates.concurrent_erase({"person_decides_to_return_2", {inst}});
            kb.knownPredicates.concurrent_erase({"person_goes_to_bed_after_return_1", {inst}});
            kb.knownPredicates.concurrent_erase({"person_goes_to_bed_after_return_2", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];

            kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
            kb.unknownPredicates.concurrent_insert({"notify_automated_succeeded", {inst}});
            kb.unknownPredicates.concurrent_insert({"notify_recorded_succeeded", {inst}});

            kb.knownPredicates.concurrent_erase({"guide_to_succeeded_attempt_1", {inst}});
            kb.knownPredicates.concurrent_erase({"guide_to_succeeded_attempt_2", {inst}});
            kb.knownPredicates.concurrent_erase({"notify_automated_succeeded", {inst}});
            kb.knownPredicates.concurrent_erase({"notify_recorded_succeeded", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

        // food_protocol
        BT::NodeStatus high_level_domain_StartFoodProtocol(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];

            kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
            kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
            kb.unknownPredicates.concurrent_insert({"remind_food_succeeded", {inst}});
            kb.unknownPredicates.concurrent_insert({"remind_food_succeeded2", {inst}});

            kb.knownPredicates.concurrent_erase({"guide_to_succeeded_attempt_1", {inst}});
            kb.knownPredicates.concurrent_erase({"guide_to_succeeded_attempt_2", {inst}});
            kb.knownPredicates.concurrent_erase({"remind_food_succeeded", {inst}});
            kb.knownPredicates.concurrent_erase({"remind_food_succeeded2", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

    };
}
