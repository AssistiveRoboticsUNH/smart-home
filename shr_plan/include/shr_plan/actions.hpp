#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include <shr_plan/world_state_converter.hpp>

namespace pddl_lib {
    void abort() {
        throw std::runtime_error("abort: higher priority protocol detected");
    }

    InstantiatedParameter get_active_protocol() {
        InstantiatedParameter out = {"idle", ""};
        auto &kb = KnowledgeBase::getInstance();
        kb.knownPredicates.lock();
        for (const auto &pred: kb.knownPredicates) {
            if ((pred.name == "fall_protocol_enabled") || (pred.name == "medicine_protocol_enabled")
                || (pred.name == "wondering_protocol_enabled") || (pred.name == "food_protocol_enabled")) {
                out = pred.parameters[0];
            }
        }
        kb.knownPredicates.unlock();
        return out;
    }

    class ProtocolState {
    public:
        std::shared_ptr<WorldStatePDDLConverter> world_state_converter;

        std::chrono::steady_clock::time_point time_wondering_protocol_detectPersonLeftHouse1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_detectPersonLeftHouse2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_CheckBedAfterReturn1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_CheckBedAfterReturn2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_checkIfPersonWentToBed1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_checkIfPersonWentToBed2 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_waitForPersonToReturn1 = {};
        std::chrono::steady_clock::time_point time_wondering_protocol_waitForPersonToReturn2 = {};

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
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->bedroom_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_CheckBedAfterReturn1, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_CheckBedAfterReturn2(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->bedroom_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_CheckBedAfterReturn2, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_detectPersonLeftHouse1(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->outside_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_detectPersonLeftHouse1, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_detectPersonLeftHouse2(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->outside_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_detectPersonLeftHouse2, cond, std::chrono::minutes(10));
        }


        BT::NodeStatus wondering_protocol_checkIfPersonWentToBed1(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->bedroom_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_checkIfPersonWentToBed1, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_checkIfPersonWentToBed2(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location == msg->bedroom_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_checkIfPersonWentToBed2, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_waitForPersonToReturn1(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != msg->outside_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_waitForPersonToReturn1, cond, std::chrono::minutes(10));
        }

        BT::NodeStatus wondering_protocol_waitForPersonToReturn2(const InstantiatedAction &action) override {
            auto cond = []() {
                ProtocolState &ps = ProtocolState::getInstance();
                auto msg = ps.world_state_converter->get_world_state_msg();
                return msg->patient_location != msg->outside_location;
            };
            ProtocolState &ps = ProtocolState::getInstance();
            return observe_wait_for_cond(action, ps.time_wondering_protocol_waitForPersonToReturn2, cond, std::chrono::minutes(10));
        }





    };

}
