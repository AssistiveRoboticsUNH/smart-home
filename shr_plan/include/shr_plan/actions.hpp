#include "bt_shr_actions.hpp"


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

    class ProtocolActions : public pddl_lib::ActionInterface {
        std::chrono::steady_clock::time_point CheckBedAfterReturn1_ = {};
        std::chrono::steady_clock::time_point CheckBedAfterReturn2_ = {};

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
            auto startTime = std::chrono::steady_clock::now();
            if (startTime > CheckBedAfterReturn1_){
                CheckBedAfterReturn1_ = startTime + std::chrono::minutes(10);
            }

            while (std::chrono::steady_clock::now() < CheckBedAfterReturn1_) {
                auto active_protocol = get_active_protocol();
                if (active_protocol.type != "WonderingProtocol") {
                    abort();
                }

            }

            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus wondering_protocol_InitCheckBedAfterReturn2(const InstantiatedAction &action) override {
            auto startTime = std::chrono::steady_clock::now();
            if (startTime > CheckBedAfterReturn2_){
                CheckBedAfterReturn2_ = startTime + std::chrono::minutes(10);
            }

            while (std::chrono::steady_clock::now() < CheckBedAfterReturn2_) {
                auto active_protocol = get_active_protocol();
                if (active_protocol.type != "WonderingProtocol") {
                    abort();
                }
            }

            return BT::NodeStatus::FAILURE;
        }


    };

}
