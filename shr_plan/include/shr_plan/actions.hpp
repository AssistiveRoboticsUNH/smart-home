#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"


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
        std::chrono::steady_clock::time_point CheckBedAfterReturn1_ = {};
        std::chrono::steady_clock::time_point CheckBedAfterReturn2_ = {};

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
            auto startTime = std::chrono::steady_clock::now();
            if (startTime > ps.CheckBedAfterReturn1_) {
                ps.CheckBedAfterReturn1_ = startTime + std::chrono::minutes(10);
            }
            auto n = std::make_shared<rclcpp::Node>("wondering_protocol_CheckBedAfterReturn1");

            while (std::chrono::steady_clock::now() < ps.CheckBedAfterReturn1_) {
                auto active_protocol = get_active_protocol();
                if (active_protocol.type != "WonderingProtocol") {
                    abort();
                }
                rclcpp::spin_some(n);

            }

            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus wondering_protocol_InitCheckBedAfterReturn2(const InstantiatedAction &action) override {
            ProtocolState &ps = ProtocolState::getInstance();
            auto startTime = std::chrono::steady_clock::now();
            if (startTime > ps.CheckBedAfterReturn2_) {
                ps.CheckBedAfterReturn2_ = startTime + std::chrono::minutes(10);
            }
            auto n = std::make_shared<rclcpp::Node>("wondering_protocol_CheckBedAfterReturn1");
            while (std::chrono::steady_clock::now() < ps.CheckBedAfterReturn2_) {
                auto active_protocol = get_active_protocol();
                if (active_protocol.type != "WonderingProtocol") {
                    abort();
                }
                rclcpp::spin_some(n);
            }

            return BT::NodeStatus::FAILURE;
        }


    };

}
