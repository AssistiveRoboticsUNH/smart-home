#include "bt_shr_actions.hpp"

std::string active_domain;

BT::NodeStatus
high_level_domain::StartFallProtocol::tick_action(const InstantiatedAction &action) {
  active_domain = "fall_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::StartWonderingProtocol::tick_action(const InstantiatedAction &action) {
  active_domain = "wondering_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::StartMedicineProtocol::tick_action(const InstantiatedAction &action) {
  active_domain = "medicine_domain.pddl";
  auto &kb = KnowledgeBase::getInstance();
  InstantiatedParameter inst = {action.parameters[0].name, "MedicineProtocol"};
  kb.knownPredicates.insert({"enabled", {inst}});

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::StartFoodProtocol::tick_action(const InstantiatedAction &action) {
  active_domain = "food_domain.pddl";
  auto &kb = KnowledgeBase::getInstance();
  InstantiatedParameter inst = {action.parameters[0].name, "FoodProtocol"};
  kb.knownPredicates.insert({"enabled", {inst}});

  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus
high_level_domain::StartIdle::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::ChangePriority_1_2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::ChangePriority_2_3::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::ChangePriority_3_4::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
high_level_domain::ChangePriority_4_5::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
medicine_protocol::detectPerson::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::initMoveToLandmark::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::moveToLandmark::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::InitguidePersonToLandmarkAttempt::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::guidePersonToLandmarkAttempt1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::guidePersonToLandmarkAttempt2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::checkGuideToSucceeded1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus medicine_protocol::checkGuideToSucceeded2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus medicine_protocol::UpdatePersonLoc1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::UpdatePersonLoc2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::UpdateSuccess1::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::UpdateSuccess2::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::UpdateSuccess3::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::notifyAutomatedMedicineAt::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus medicine_protocol::notifyRecordedMedicineAt::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus medicine_protocol::askCaregiverHelpMedicine1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus medicine_protocol::askCaregiverHelpMedicine2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus
food_protocol::detectPerson::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::initMoveToLandmark::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::moveToLandmark::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::InitguidePersonToLandmarkAttempt::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::guidePersonToLandmarkAttempt1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::guidePersonToLandmarkAttempt2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::checkGuideToSucceeded1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus food_protocol::checkGuideToSucceeded2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus food_protocol::UpdatePersonLoc1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::UpdatePersonLoc2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::UpdateSuccess1::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::UpdateSuccess2::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus food_protocol::UpdateSuccess3::tick_action(const InstantiatedAction &action) {
  active_domain = "high_level_domain.pddl";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
food_protocol::remindAutomatedFoodAt::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
food_protocol::remindAutomatedFoodAt2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
food_protocol::askCaregiverHelpFood1::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
food_protocol::askCaregiverHelpFood2::tick_action(const InstantiatedAction &action) {
  return BT::NodeStatus::SUCCESS;
}



