#include "bt_shr_actions.hpp"

std::string active_domain;

namespace pddl_lib {

    namespace high_level_domain {
        BT::NodeStatus
        StartFallProtocol::tick_action(const InstantiatedAction &action) {
            active_domain = "fall_domain.pddl";
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = {action.parameters[0].name, "FallProtocol"};
            kb.knownPredicates.insert({"enabled", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        StartWonderingProtocol::tick_action(const InstantiatedAction &action) {
            active_domain = "wondering_domain.pddl";
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = {action.parameters[0].name, "WonderingProtocol"};
            kb.knownPredicates.insert({"enabled", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        StartMedicineProtocol::tick_action(const InstantiatedAction &action) {
            active_domain = "medicine_domain.pddl";
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = {action.parameters[0].name, "MedicineProtocol"};
            kb.knownPredicates.insert({"enabled", {inst}});

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        StartFoodProtocol::tick_action(const InstantiatedAction &action) {
            active_domain = "food_domain.pddl";
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = {action.parameters[0].name, "FoodProtocol"};
            kb.knownPredicates.insert({"enabled", {inst}});

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
    }

    namespace medicine_protocol {
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

        BT::NodeStatus
        InitguidePersonToLandmarkAttempt::tick_action(const InstantiatedAction &action) {
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
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus askCaregiverHelpMedicine1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus askCaregiverHelpMedicine2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }
    }


    BT::NodeStatus
    food_protocol::detectPerson::tick_action(const InstantiatedAction &action) {
        return BT::NodeStatus::SUCCESS;
    }

    namespace food_protocol {
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

        BT::NodeStatus
        remindAutomatedFoodAt::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        remindAutomatedFoodAt2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        askCaregiverHelpFood1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        askCaregiverHelpFood2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }
    }


    namespace wondering_protocol {
        BT::NodeStatus initMoveToLandmark::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus moveToLandmark::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus updatePersonLocation1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus updatePersonLocation2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        InitCheckBedAfterReturn2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        notifyAutomatedMidnightAt::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        notifyRecordedMidnightAt::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        DetectPerson::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        initDetectPersonLeftHouse1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        detectPersonLeftHouse1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        initDetectPersonLeftHouse2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        detectPersonLeftHouse2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        personGoOutside1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        personGoOutside2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        finishDetectPerson1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        finishDetectPerson2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        InitCheckBedAfterReturn1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        checkIfPersonWentToBed1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        checkIfPersonWentToBed2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        callCaregiverAskToGoToBed::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        callCaregiverAskToGoToBedAfterReturn1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        callCaregiverAskToGoToBedAfterReturn2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        CheckBedAfterReturn1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        CheckBedAfterReturn2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        waitForPersonToReturn1::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        waitForPersonToReturn2::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        callCaregiverWondering::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus
        callEmergency::tick_action(const InstantiatedAction &action) {
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus UpdateSuccess0::tick_action(const InstantiatedAction &action) {
            active_domain = "high_level_domain.pddl";
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

        BT::NodeStatus UpdateSuccess4::tick_action(const InstantiatedAction &action) {
            active_domain = "high_level_domain.pddl";
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus UpdateSuccess5::tick_action(const InstantiatedAction &action) {
            active_domain = "high_level_domain.pddl";
            return BT::NodeStatus::SUCCESS;
        }
    }

}
