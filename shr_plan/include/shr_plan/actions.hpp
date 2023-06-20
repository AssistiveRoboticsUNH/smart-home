#include "bt_shr_actions.hpp"

std::string active_domain;

BT::NodeStatus
StartFallProtocol::tick_action(const InstantiatedAction &action) {
    active_domain = "fall_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
StartWonderingProtocol::tick_action(const InstantiatedAction &action) {
    active_domain = "wondering_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
StartMedicineProtocol::tick_action(const InstantiatedAction &action) {
    active_domain = "medicine_domain.pddl";
    auto &kb = KnowledgeBase::getInstance();
    InstantiatedParameter inst = {action.parameters[0].name, "MedicineProtocol"};
    kb.unknownPredicates.insert({"guide_to_succeeded_attempt_1", {inst}});
    kb.unknownPredicates.insert({"guide_to_succeeded_attempt_2", {inst}});
    kb.unknownPredicates.insert({"notify_automated_succeeded", {inst}});
    kb.unknownPredicates.insert({"notify_recorded_succeeded", {inst}});
    kb.knownPredicates.insert({"enabled", {inst}});

    kb.objects.push_back(inst);

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
StartFoodProtocol::tick_action(const InstantiatedAction &action) {
    active_domain = "food_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}



BT::NodeStatus
StartIdle::tick_action(const InstantiatedAction &action) {
    active_domain = "high_level_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
ChangePriority_1_2::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
ChangePriority_2_3::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
ChangePriority_3_4::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
ChangePriority_4_5::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
detectPerson::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus initMoveToLandmark::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus moveToLandmark::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus InitguidePersonToLandmarkAttempt::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus guidePersonToLandmarkAttempt1::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus guidePersonToLandmarkAttempt2::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus checkGuideToSucceeded1::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus checkGuideToSucceeded2::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus UpdatePersonLoc1::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdatePersonLoc2::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateSuccess1::tick_action(const InstantiatedAction &action) {
    active_domain = "high_level_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateSuccess2::tick_action(const InstantiatedAction &action) {
    active_domain = "high_level_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateSuccess3::tick_action(const InstantiatedAction &action) {
    active_domain = "high_level_domain.pddl";
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus notifyAutomatedMedicineAt::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus notifyRecordedMedicineAt::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus askCaregiverHelpMedicine1::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus askCaregiverHelpMedicine2::tick_action(const InstantiatedAction &action) {
    return BT::NodeStatus::SUCCESS;
}
