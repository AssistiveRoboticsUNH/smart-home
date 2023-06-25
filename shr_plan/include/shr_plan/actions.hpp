#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/call_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "shr_msgs/action/deep_fake_request.hpp"

#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        std::shared_ptr<WorldStateListener> world_state_converter;
        // midnight
        std::chrono::steady_clock::time_point time_wondering_protocol_detectPersonLeftHouse1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_detectPersonLeftHouse2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_CheckBedAfterReturn1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_CheckBedAfterReturn2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_checkIfPersonWentToBed1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_checkIfPersonWentToBed2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_waitForPersonToReturn1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_waitForPersonToReturn2 = {};
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
        rclcpp_action::Client<shr_msgs::action::DeepFakeRequest>::SharedPtr read_action_client_ = {};


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

    class ProtocolActions : public pddl_lib::ActionInterface {
    public:

        BT::NodeStatus high_level_domain_StartWonderingProtocol(const InstantiatedAction &action) override {
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

        BT::NodeStatus wondering_protocol_CheckBedAfterReturn1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.bedroom_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.check_bed_after_return_wait_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_CheckBedAfterReturn1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_CheckBedAfterReturn2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.bedroom_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.check_bed_after_return_wait_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_CheckBedAfterReturn2, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_detectPersonLeftHouse1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.outside_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.detect_person_left_house_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_detectPersonLeftHouse1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_detectPersonLeftHouse2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.outside_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.detect_person_left_house_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_detectPersonLeftHouse2, cond,
                                         std::chrono::seconds(wait_time));
        }


        BT::NodeStatus wondering_protocol_checkIfPersonWentToBed1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.bedroom_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.check_if_person_went_to_bed_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_checkIfPersonWentToBed1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_checkIfPersonWentToBed2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == params.pddl.WonderingProtocols.bedroom_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.check_if_person_went_to_bed_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_checkIfPersonWentToBed2, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_waitForPersonToReturn1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.WonderingProtocols.outside_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.wait_for_person_to_return_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_waitForPersonToReturn1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus wondering_protocol_waitForPersonToReturn2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.WonderingProtocols.outside_location[index];
            };
            auto wait_time = get_seconds(params.pddl.WonderingProtocols.wait_for_person_to_return_times[index]);
            return observe_wait_for_cond(action, ps.time_wondering_protocol_waitForPersonToReturn2, cond,
                                         std::chrono::seconds(wait_time));
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

        BT::NodeStatus medicine_protocol_checkGuideToSucceeded1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.MedicineProtocols.medicine_location[index];
            };
            auto wait_time = get_seconds(params.pddl.MedicineProtocols.check_guide_to_succeeded_times[index]);
            return observe_wait_for_cond(action, ps.time_medicine_protocol_checkGuideToSucceeded1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus medicine_protocol_checkGuideToSucceeded2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.MedicineProtocols.medicine_location[index];
            };
            auto wait_time = get_seconds(params.pddl.MedicineProtocols.check_guide_to_succeeded_times[index]);
            return observe_wait_for_cond(action, ps.time_medicine_protocol_checkGuideToSucceeded2, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus medicine_protocol_notifyAutomatedMedicineAt(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->medicine_sensor == 1;
            };
            auto wait_time = get_seconds(params.pddl.MedicineProtocols.notify_automated_medicine_at_times[index]);
            return observe_wait_for_cond(action, ps.time_notifyAutomatedMedicineAt, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus medicine_protocol_notifyRecordedMedicineAt(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->medicine_sensor == 1;
            };
            auto wait_time = get_seconds(params.pddl.MedicineProtocols.notify_automated_medicine_at_times[index]);
            return observe_wait_for_cond(action, ps.time_notifyRecordedMedicineAt, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus medicine_protocol_askCaregiverHelpMedicine1(const InstantiatedAction &action) override {
            //
            return ActionInterface::medicine_protocol_askCaregiverHelpMedicine2(action);
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

        BT::NodeStatus food_protocol_checkGuideToSucceeded1(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.FoodProtocols.eat_locations[index];
            };
            auto wait_time = get_seconds(params.pddl.FoodProtocols.check_guide_to_succeeded_times[index]);
            return observe_wait_for_cond(action, ps.time_food_protocol_checkGuideToSucceeded1, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus food_protocol_checkGuideToSucceeded2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != params.pddl.FoodProtocols.eat_locations[index];
            };
            auto wait_time = get_seconds(params.pddl.FoodProtocols.check_guide_to_succeeded_times[index]);
            return observe_wait_for_cond(action, ps.time_food_protocol_checkGuideToSucceeded2, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus food_protocol_remindAutomatedFoodAt(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->eating_sensor == 1;
            };
            auto wait_time = get_seconds(params.pddl.FoodProtocols.remind_automated_food_at_times[index]);
            return observe_wait_for_cond(action, ps.time_food_protocol_remindAutomatedFoodAt, cond,
                                         std::chrono::seconds(wait_time));
        }

        BT::NodeStatus food_protocol_remindAutomatedFoodAt2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto params = ps.world_state_converter->get_params();
            auto inst = action.parameters[0];
            auto index = get_inst_index(inst, params).value();
            auto cond = [params, index]() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->eating_sensor == 1;
            };
            auto wait_time = get_seconds(params.pddl.FoodProtocols.remind_automated_food_at_2_times[index]);
            return observe_wait_for_cond(action, ps.time_food_protocol_remindAutomatedFoodAt2, cond,
                                         std::chrono::seconds(wait_time));
        }

    };

}
