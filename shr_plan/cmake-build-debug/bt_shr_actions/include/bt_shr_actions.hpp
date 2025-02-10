#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "pddl_parser/pddl_parser.hpp"

namespace pddl_lib {

class Time : public std::string {
public:
    using std::string::string;
    explicit Time(const std::string& str) : std::string(str) {}
};
class Person : public std::string {
public:
    using std::string::string;
    explicit Person(const std::string& str) : std::string(str) {}
};
class MedicineRefillReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit MedicineRefillReminderProtocol(const std::string& str) : std::string(str) {}
};
class WaitAction : public std::string {
public:
    using std::string::string;
    explicit WaitAction(const std::string& str) : std::string(str) {}
};
class MedicineProtocol : public std::string {
public:
    using std::string::string;
    explicit MedicineProtocol(const std::string& str) : std::string(str) {}
};
class ReminderAction : public std::string {
public:
    using std::string::string;
    explicit ReminderAction(const std::string& str) : std::string(str) {}
};
class Msg : public std::string {
public:
    using std::string::string;
    explicit Msg(const std::string& str) : std::string(str) {}
};
class CallAction : public std::string {
public:
    using std::string::string;
    explicit CallAction(const std::string& str) : std::string(str) {}
};
class MedicineRefillPharmacyReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit MedicineRefillPharmacyReminderProtocol(const std::string& str) : std::string(str) {}
};
class GymReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit GymReminderProtocol(const std::string& str) : std::string(str) {}
};
class NoAction : public std::string {
public:
    using std::string::string;
    explicit NoAction(const std::string& str) : std::string(str) {}
};
class Landmark : public std::string {
public:
    using std::string::string;
    explicit Landmark(const std::string& str) : std::string(str) {}
};


class UpdatePredicates {
public:
  void update() const {
      auto & kb = KnowledgeBase::getInstance();
      std::vector<InstantiatedParameter> Time_instances;
      std::vector<InstantiatedParameter> Person_instances;
      std::vector<InstantiatedParameter> MedicineRefillReminderProtocol_instances;
      std::vector<InstantiatedParameter> WaitAction_instances;
      std::vector<InstantiatedParameter> MedicineProtocol_instances;
      std::vector<InstantiatedParameter> ReminderAction_instances;
      std::vector<InstantiatedParameter> Msg_instances;
      std::vector<InstantiatedParameter> CallAction_instances;
      std::vector<InstantiatedParameter> MedicineRefillPharmacyReminderProtocol_instances;
      std::vector<InstantiatedParameter> GymReminderProtocol_instances;
      std::vector<InstantiatedParameter> NoAction_instances;
      std::vector<InstantiatedParameter> Landmark_instances;

      for (const auto object : kb.get_objects()){
          if (object.type == "Time"){
              Time_instances.push_back(object);
          }
          if (object.type == "Person"){
              Person_instances.push_back(object);
          }
          if (object.type == "MedicineRefillReminderProtocol"){
              MedicineRefillReminderProtocol_instances.push_back(object);
          }
          if (object.type == "WaitAction"){
              WaitAction_instances.push_back(object);
          }
          if (object.type == "MedicineProtocol"){
              MedicineProtocol_instances.push_back(object);
          }
          if (object.type == "ReminderAction"){
              ReminderAction_instances.push_back(object);
          }
          if (object.type == "Msg"){
              Msg_instances.push_back(object);
          }
          if (object.type == "CallAction"){
              CallAction_instances.push_back(object);
          }
          if (object.type == "MedicineRefillPharmacyReminderProtocol"){
              MedicineRefillPharmacyReminderProtocol_instances.push_back(object);
          }
          if (object.type == "GymReminderProtocol"){
              GymReminderProtocol_instances.push_back(object);
          }
          if (object.type == "NoAction"){
              NoAction_instances.push_back(object);
          }
          if (object.type == "Landmark"){
              Landmark_instances.push_back(object);
          }
      }

      
        {
            for (auto MedicineProtocol_instance_1 : MedicineProtocol_instances ) {
              InstantiatedPredicate pred = {"already_called_about_medicine", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_called_about_medicine(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"not_visible_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = not_visible_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"gym_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = gym_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineRefillReminderProtocol_instance_1 : MedicineRefillReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"medicine_refill_reminder_enabled", {MedicineRefillReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_refill_reminder_enabled(old_val, MedicineRefillReminderProtocol(MedicineRefillReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto GymReminderProtocol_instance_1 : GymReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_gym_reminder", {GymReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_gym_reminder(old_val, GymReminderProtocol(GymReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineProtocol_instance_1 : MedicineProtocol_instances ) {
              InstantiatedPredicate pred = {"already_took_medicine", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_took_medicine(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"visible_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = visible_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"medicine_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineProtocol_instance_1 : MedicineProtocol_instances ) {
              InstantiatedPredicate pred = {"time_to_take_medicine", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_to_take_medicine(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto GymReminderProtocol_instance_1 : GymReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_gym", {GymReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_gym(old_val, GymReminderProtocol(GymReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Person_instance_1 : Person_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"person_currently_at", {Person_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = person_currently_at(old_val, Person(Person_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto MedicineProtocol_instance_1 : MedicineProtocol_instances ) {
              InstantiatedPredicate pred = {"medicine_protocol_enabled", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_protocol_enabled(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineRefillPharmacyReminderProtocol_instance_1 : MedicineRefillPharmacyReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"medicine_pharmacy_reminder_enabled", {MedicineRefillPharmacyReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_pharmacy_reminder_enabled(old_val, MedicineRefillPharmacyReminderProtocol(MedicineRefillPharmacyReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineRefillPharmacyReminderProtocol_instance_1 : MedicineRefillPharmacyReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_medicine_pharmacy_reminder", {MedicineRefillPharmacyReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_medicine_pharmacy_reminder(old_val, MedicineRefillPharmacyReminderProtocol(MedicineRefillPharmacyReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"person_at", {Time_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = person_at(old_val, Time(Time_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"robot_at", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = robot_at(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"medicine_refill_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_refill_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineRefillReminderProtocol_instance_1 : MedicineRefillReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_medicine_refill_reminder", {MedicineRefillReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_medicine_refill_reminder(old_val, MedicineRefillReminderProtocol(MedicineRefillReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"priority_3", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = priority_3(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"success", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = success(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto MedicineRefillReminderProtocol_instance_1 : MedicineRefillReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_medicine_refill", {MedicineRefillReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_medicine_refill(old_val, MedicineRefillReminderProtocol(MedicineRefillReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"priority_5", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = priority_5(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto GymReminderProtocol_instance_1 : GymReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"gym_reminder_enabled", {GymReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = gym_reminder_enabled(old_val, GymReminderProtocol(GymReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto MedicineRefillPharmacyReminderProtocol_instance_1 : MedicineRefillPharmacyReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_medicine_pharmacy", {MedicineRefillPharmacyReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_medicine_pharmacy(old_val, MedicineRefillPharmacyReminderProtocol(MedicineRefillPharmacyReminderProtocol_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"medicine_pharmacy_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_pharmacy_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"priority_1", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = priority_1(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"priority_2", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = priority_2(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"low_level_failed", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = low_level_failed(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"priority_4", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = priority_4(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto CallAction_instance_2 : CallAction_instances ) {
              InstantiatedPredicate pred = {"call_blocks_call", {CallAction_instance_1, CallAction_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = call_blocks_call(old_val, CallAction(CallAction_instance_1.name), CallAction(CallAction_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto WaitAction_instance_1 : WaitAction_instances ) {
              InstantiatedPredicate pred = {"executed_wait", {WaitAction_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = executed_wait(old_val, WaitAction(WaitAction_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"medicine_taken_success", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_taken_success(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"same_location_constraint", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = same_location_constraint(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"abort", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = abort(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
              InstantiatedPredicate pred = {"call_person_not_taking_medicine_constraint", {CallAction_instance_1, Person_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = call_person_not_taking_medicine_constraint(old_val, CallAction(CallAction_instance_1.name), Person(Person_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
              InstantiatedPredicate pred = {"executed_reminder", {ReminderAction_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = executed_reminder(old_val, ReminderAction(ReminderAction_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"call_person_location_constraint", {CallAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = call_person_location_constraint(old_val, CallAction(CallAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"executed_wait", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = executed_wait(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"DetectEatingFood_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = DetectEatingFood_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto NoAction_instance_1 : NoAction_instances ) {
              InstantiatedPredicate pred = {"na_used", {NoAction_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = na_used(old_val, NoAction(NoAction_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
              InstantiatedPredicate pred = {"reminder_person_not_eating_food_constraint", {ReminderAction_instance_1, Person_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_person_not_eating_food_constraint(old_val, ReminderAction(ReminderAction_instance_1.name), Person(Person_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"DetectPerson_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = DetectPerson_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
              InstantiatedPredicate pred = {"executed_call", {CallAction_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = executed_call(old_val, CallAction(CallAction_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"used_call", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = used_call(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto NoAction_instance_1 : NoAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"noaction_not_person_location_constraint", {NoAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = noaction_not_person_location_constraint(old_val, NoAction(NoAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"move_to_home_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = move_to_home_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto Msg_instance_1 : Msg_instances ) {
              InstantiatedPredicate pred = {"message_given", {Msg_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = message_given(old_val, Msg(Msg_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto CallAction_instance_2 : CallAction_instances ) {
              InstantiatedPredicate pred = {"reminder_blocks_call", {ReminderAction_instance_1, CallAction_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_blocks_call(old_val, ReminderAction(ReminderAction_instance_1.name), CallAction(CallAction_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"wait_not_person_location_constraint", {Time_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = wait_not_person_location_constraint(old_val, Time(Time_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto ReminderAction_instance_2 : ReminderAction_instances ) {
              InstantiatedPredicate pred = {"reminder_blocks_reminder", {ReminderAction_instance_1, ReminderAction_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_blocks_reminder(old_val, ReminderAction(ReminderAction_instance_1.name), ReminderAction(ReminderAction_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"GiveReminder_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = GiveReminder_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"MakeCall_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = MakeCall_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto Msg_instance_1 : Msg_instances ) {
              InstantiatedPredicate pred = {"message_given_success", {Msg_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = message_given_success(old_val, Msg(Msg_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto NoAction_instance_1 : NoAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"noaction_person_location_constraint", {NoAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = noaction_person_location_constraint(old_val, NoAction(NoAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto Msg_instance_2 : Msg_instances ) {
              InstantiatedPredicate pred = {"valid_reminder_message", {ReminderAction_instance_1, Msg_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = valid_reminder_message(old_val, ReminderAction(ReminderAction_instance_1.name), Msg(Msg_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Person_instance_1 : Person_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"person_at_success", {Person_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = person_at_success(old_val, Person(Person_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"robot_at_time", {Time_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = robot_at_time(old_val, Time(Time_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"food_eaten_success", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = food_eaten_success(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
              InstantiatedPredicate pred = {"not_same_location_constraint", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = not_same_location_constraint(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto WaitAction_instance_1 : WaitAction_instances ) {
            for (auto WaitAction_instance_2 : WaitAction_instances ) {
              InstantiatedPredicate pred = {"wait_blocks_wait", {WaitAction_instance_1, WaitAction_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = wait_blocks_wait(old_val, WaitAction(WaitAction_instance_1.name), WaitAction(WaitAction_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"time_critical", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_critical(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
              InstantiatedPredicate pred = {"reminder_person_not_taking_medicine_constraint", {ReminderAction_instance_1, Person_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_person_not_taking_medicine_constraint(old_val, ReminderAction(ReminderAction_instance_1.name), Person(Person_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"success_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = success_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"reminder_person_not_location_constraint", {ReminderAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_person_not_location_constraint(old_val, ReminderAction(ReminderAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto ReminderAction_instance_1 : ReminderAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"reminder_person_location_constraint", {ReminderAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_person_location_constraint(old_val, ReminderAction(ReminderAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
              InstantiatedPredicate pred = {"DetectTakingMedicine_enabled", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = DetectTakingMedicine_enabled(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"same_location", {Landmark_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = same_location(old_val, Landmark(Landmark_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"used_move", {Time_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = used_move(old_val, Time(Time_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
              InstantiatedPredicate pred = {"home_location", {Landmark_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = home_location(old_val, Landmark(Landmark_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"person_eating_food", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = person_eating_food(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"wait_robot_location_constraint", {Time_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = wait_robot_location_constraint(old_val, Time(Time_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"used_reminder", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = used_reminder(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"person_taking_medicine", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = person_taking_medicine(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto Msg_instance_2 : Msg_instances ) {
              InstantiatedPredicate pred = {"valid_call_message", {CallAction_instance_1, Msg_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = valid_call_message(old_val, CallAction(CallAction_instance_1.name), Msg(Msg_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
              InstantiatedPredicate pred = {"call_person_not_eating_food_constraint", {CallAction_instance_1, Person_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = call_person_not_eating_food_constraint(old_val, CallAction(CallAction_instance_1.name), Person(Person_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
              InstantiatedPredicate pred = {"current_time", {Time_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = current_time(old_val, Time(Time_instance_1.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
        }
        {
              InstantiatedPredicate pred = {"no_action", { } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = no_action(old_val);
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
        }
        {
            for (auto CallAction_instance_1 : CallAction_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"call_not_person_location_constraint", {CallAction_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = call_not_person_location_constraint(old_val, CallAction(CallAction_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }
        {
            for (auto Landmark_instance_1 : Landmark_instances ) {
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"traversable", {Landmark_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = traversable(old_val, Landmark(Landmark_instance_1.name), Landmark(Landmark_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Time_instance_2 : Time_instances ) {
              InstantiatedPredicate pred = {"next_time", {Time_instance_1, Time_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = next_time(old_val, Time(Time_instance_1.name), Time(Time_instance_2.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
        }
        {
            for (auto Time_instance_1 : Time_instances ) {
            for (auto Person_instance_2 : Person_instances ) {
            for (auto Landmark_instance_3 : Landmark_instances ) {
              InstantiatedPredicate pred = {"wait_person_location_constraint", {Time_instance_1, Person_instance_2, Landmark_instance_3 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = wait_person_location_constraint(old_val, Time(Time_instance_1.name), Person(Person_instance_2.name), Landmark(Landmark_instance_3.name));
            if (new_val==TRUTH_VALUE::TRUE){
                kb.insert_predicate(pred);
                kb.erase_unknown_predicate(pred);
            } else if(new_val==TRUTH_VALUE::UNKNOWN){
                kb.erase_predicate(pred);
                kb.insert_unknown_predicate(pred);
            } else {
                kb.erase_predicate(pred);
                kb.erase_unknown_predicate(pred);
            }
             }
             }
             }
        }

  }

    virtual TRUTH_VALUE already_called_about_medicine(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE not_visible_location(TRUTH_VALUE val, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE gym_location(TRUTH_VALUE val, Landmark lm) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_refill_reminder_enabled(TRUTH_VALUE val, MedicineRefillReminderProtocol mdrf) const {
        return val;
    }
    virtual TRUTH_VALUE time_for_gym_reminder(TRUTH_VALUE val, GymReminderProtocol gy) const {
        return val;
    }
    virtual TRUTH_VALUE already_took_medicine(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE visible_location(TRUTH_VALUE val, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_location(TRUTH_VALUE val, Landmark lm) const {
        return val;
    }
    virtual TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_gym(TRUTH_VALUE val, GymReminderProtocol gy) const {
        return val;
    }
    virtual TRUTH_VALUE person_currently_at(TRUTH_VALUE val, Person p, Landmark lm) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_protocol_enabled(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_pharmacy_reminder_enabled(TRUTH_VALUE val, MedicineRefillPharmacyReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE time_for_medicine_pharmacy_reminder(TRUTH_VALUE val, MedicineRefillPharmacyReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE person_at(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE robot_at(TRUTH_VALUE val, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_refill_location(TRUTH_VALUE val, Landmark lm) const {
        return val;
    }
    virtual TRUTH_VALUE time_for_medicine_refill_reminder(TRUTH_VALUE val, MedicineRefillReminderProtocol mdrf) const {
        return val;
    }
    virtual TRUTH_VALUE priority_3(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_medicine_refill(TRUTH_VALUE val, MedicineRefillReminderProtocol mdrf) const {
        return val;
    }
    virtual TRUTH_VALUE priority_5(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE gym_reminder_enabled(TRUTH_VALUE val, GymReminderProtocol gy) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_medicine_pharmacy(TRUTH_VALUE val, MedicineRefillPharmacyReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_pharmacy_location(TRUTH_VALUE val, Landmark lm) const {
        return val;
    }
    virtual TRUTH_VALUE priority_1(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE priority_2(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE low_level_failed(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE priority_4(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE call_blocks_call(TRUTH_VALUE val, CallAction a1, CallAction a2) const {
        return val;
    }
    virtual TRUTH_VALUE executed_wait(TRUTH_VALUE val, WaitAction a) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_taken_success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE same_location_constraint(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE abort(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE call_person_not_taking_medicine_constraint(TRUTH_VALUE val, CallAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE executed_reminder(TRUTH_VALUE val, ReminderAction a) const {
        return val;
    }
    virtual TRUTH_VALUE call_person_location_constraint(TRUTH_VALUE val, CallAction a, Person p, Landmark loc) const {
        return val;
    }
    virtual TRUTH_VALUE executed_wait(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE DetectEatingFood_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE na_used(TRUTH_VALUE val, NoAction na) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_eating_food_constraint(TRUTH_VALUE val, ReminderAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE DetectPerson_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE executed_call(TRUTH_VALUE val, CallAction c) const {
        return val;
    }
    virtual TRUTH_VALUE used_call(TRUTH_VALUE val, Time tc) const {
        return val;
    }
    virtual TRUTH_VALUE noaction_not_person_location_constraint(TRUTH_VALUE val, NoAction na, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE move_to_home_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE message_given(TRUTH_VALUE val, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_blocks_call(TRUTH_VALUE val, ReminderAction a1, CallAction a2) const {
        return val;
    }
    virtual TRUTH_VALUE wait_not_person_location_constraint(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_blocks_reminder(TRUTH_VALUE val, ReminderAction a1, ReminderAction a2) const {
        return val;
    }
    virtual TRUTH_VALUE GiveReminder_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE MakeCall_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE message_given_success(TRUTH_VALUE val, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE noaction_person_location_constraint(TRUTH_VALUE val, NoAction na, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE valid_reminder_message(TRUTH_VALUE val, ReminderAction a, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE person_at_success(TRUTH_VALUE val, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE robot_at_time(TRUTH_VALUE val, Time t, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE food_eaten_success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE not_same_location_constraint(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE wait_blocks_wait(TRUTH_VALUE val, WaitAction a1, WaitAction a2) const {
        return val;
    }
    virtual TRUTH_VALUE time_critical(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_taking_medicine_constraint(TRUTH_VALUE val, ReminderAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE success_location(TRUTH_VALUE val, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_location_constraint(TRUTH_VALUE val, ReminderAction a, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_location_constraint(TRUTH_VALUE val, ReminderAction a, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE DetectTakingMedicine_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE same_location(TRUTH_VALUE val, Landmark l1, Landmark l2) const {
        return val;
    }
    virtual TRUTH_VALUE used_move(TRUTH_VALUE val, Time tc, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE home_location(TRUTH_VALUE val, Landmark l) const {
        return val;
    }
    virtual TRUTH_VALUE person_eating_food(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE wait_robot_location_constraint(TRUTH_VALUE val, Time t, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE used_reminder(TRUTH_VALUE val, Time tc) const {
        return val;
    }
    virtual TRUTH_VALUE person_taking_medicine(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE valid_call_message(TRUTH_VALUE val, CallAction a, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE call_person_not_eating_food_constraint(TRUTH_VALUE val, CallAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE current_time(TRUTH_VALUE val, Time tc) const {
        return val;
    }
    virtual TRUTH_VALUE no_action(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE call_not_person_location_constraint(TRUTH_VALUE val, CallAction a, Person p, Landmark loc) const {
        return val;
    }
    virtual TRUTH_VALUE traversable(TRUTH_VALUE val, Landmark from, Landmark to) const {
        return val;
    }
    virtual TRUTH_VALUE next_time(TRUTH_VALUE val, Time tc, Time tn) const {
        return val;
    }
    virtual TRUTH_VALUE wait_person_location_constraint(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    };

class ActionInterface {
public:
    virtual BT::NodeStatus high_level_domain_MoveToLandmark(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_1_2(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_2_3(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_3_4(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_4_5(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartMedicineProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueMedicineProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartGymReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueGymReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartMedicineRefillReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueMedicineRefillReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartMedicineRefillPharmacyReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueMedicineRefillPharmacyReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_Idle(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_MakeCall(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_GiveReminder(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_Wait(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_PersonAtSuccess(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_NoActionUsed(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_TimeOut(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_MedicineTakenSuccess(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_FoodEatenSuccess(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual void abort(const InstantiatedAction & action){} // do nothing default
};

namespace high_level_domain {
template<typename T>
class MoveToLandmark : public BT::SyncActionNode {
public:
    MoveToLandmark(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("from"), BT::InputPort<std::string>("to"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> from = getInput<std::string>("from");
        BT::Optional <std::string> to = getInput<std::string>("to");
        
        if (!from) {
            throw BT::RuntimeError("missing required input from");
        }
        param_subs["from"] = from.value();
        if (!to) {
            throw BT::RuntimeError("missing required input to");
        }
        param_subs["to"] = to.value();

        std::string tmp(R"((:action MoveToLandmark
	:parameters ( ?from - Landmark ?to - Landmark)
	:precondition (and
(robot_at ?from)
(visible_location ?from)
(visible_location ?to)
(not (not_visible_location ?from) )
(not (not_visible_location ?to) )
)
	:effect (and
(robot_at ?to)
(not (robot_at ?from) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for MoveToLandmark");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_MoveToLandmark(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ChangePriority_1_2 : public BT::SyncActionNode {
public:
    ChangePriority_1_2(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action ChangePriority_1_2
	:parameters ()
	:precondition (priority_1)
	:effect (and
(priority_2)
(not (priority_1) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ChangePriority_1_2");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ChangePriority_1_2(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ChangePriority_2_3 : public BT::SyncActionNode {
public:
    ChangePriority_2_3(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action ChangePriority_2_3
	:parameters ()
	:precondition (priority_2)
	:effect (and
(priority_3)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ChangePriority_2_3");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ChangePriority_2_3(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ChangePriority_3_4 : public BT::SyncActionNode {
public:
    ChangePriority_3_4(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action ChangePriority_3_4
	:parameters ()
	:precondition (priority_3)
	:effect (and
(priority_4)
(not (priority_3) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ChangePriority_3_4");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ChangePriority_3_4(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ChangePriority_4_5 : public BT::SyncActionNode {
public:
    ChangePriority_4_5(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action ChangePriority_4_5
	:parameters ()
	:precondition (priority_4)
	:effect (and
(priority_5)
(not (priority_4) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ChangePriority_4_5");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ChangePriority_4_5(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class StartMedicineProtocol : public BT::SyncActionNode {
public:
    StartMedicineProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("m"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("cur"), BT::InputPort<std::string>("dest"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> m = getInput<std::string>("m");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> cur = getInput<std::string>("cur");
        BT::Optional <std::string> dest = getInput<std::string>("dest");
        
        if (!m) {
            throw BT::RuntimeError("missing required input m");
        }
        param_subs["m"] = m.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!cur) {
            throw BT::RuntimeError("missing required input cur");
        }
        param_subs["cur"] = cur.value();
        if (!dest) {
            throw BT::RuntimeError("missing required input dest");
        }
        param_subs["dest"] = dest.value();

        std::string tmp(R"((:action StartMedicineProtocol
	:parameters ( ?m - MedicineProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
(priority_2)
(time_to_take_medicine ?m)
(visible_location ?dest)
(not (not_visible_location ?dest) )
(visible_location ?cur)
(not (not_visible_location ?cur) )
(person_currently_at ?p ?cur)
(robot_at ?cur)
(medicine_location ?dest)
(not (already_took_medicine ?m) )
(not (already_called_about_medicine ?m) )
(forall (?med - MedicineProtocol)
(not (medicine_protocol_enabled ?med) )
)
)
	:effect (and
(success)
(not (priority_2) )
(medicine_protocol_enabled ?m)
(not (low_level_failed) )
(forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol)
(not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy) )
)
(forall (?mdrf - MedicineRefillReminderProtocol)
(not (medicine_refill_reminder_enabled ?mdrf) )
)
(forall (?gy - GymReminderProtocol)
(not (gym_reminder_enabled ?gy) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartMedicineProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_StartMedicineProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ContinueMedicineProtocol : public BT::SyncActionNode {
public:
    ContinueMedicineProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("m"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> m = getInput<std::string>("m");
        
        if (!m) {
            throw BT::RuntimeError("missing required input m");
        }
        param_subs["m"] = m.value();

        std::string tmp(R"((:action ContinueMedicineProtocol
	:parameters ( ?m - MedicineProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(time_to_take_medicine ?m)
(not (already_took_medicine ?m) )
(not (already_called_about_medicine ?m) )
(medicine_protocol_enabled ?m)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueMedicineProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ContinueMedicineProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class StartGymReminderProtocol : public BT::SyncActionNode {
public:
    StartGymReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("gy"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("cur"), BT::InputPort<std::string>("dest"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> gy = getInput<std::string>("gy");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> cur = getInput<std::string>("cur");
        BT::Optional <std::string> dest = getInput<std::string>("dest");
        
        if (!gy) {
            throw BT::RuntimeError("missing required input gy");
        }
        param_subs["gy"] = gy.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!cur) {
            throw BT::RuntimeError("missing required input cur");
        }
        param_subs["cur"] = cur.value();
        if (!dest) {
            throw BT::RuntimeError("missing required input dest");
        }
        param_subs["dest"] = dest.value();

        std::string tmp(R"((:action StartGymReminderProtocol
	:parameters ( ?gy - GymReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
(priority_2)
(robot_at ?cur)
(gym_location ?dest)
(time_for_gym_reminder ?gy)
(not (already_reminded_gym ?gy) )
(forall (?gy - GymReminderProtocol)
(not (gym_reminder_enabled ?gy) )
)
(person_currently_at ?p ?cur)
(visible_location ?dest)
(not (not_visible_location ?dest) )
(visible_location ?cur)
(not (not_visible_location ?cur) )
)
	:effect (and
(success)
(not (priority_2) )
(gym_reminder_enabled ?gy)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_protocol_enabled ?med) )
)
(forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol)
(not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy) )
)
(forall (?mdrf - MedicineRefillReminderProtocol)
(not (medicine_refill_reminder_enabled ?mdrf) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartGymReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_StartGymReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ContinueGymReminderProtocol : public BT::SyncActionNode {
public:
    ContinueGymReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("gy"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> gy = getInput<std::string>("gy");
        
        if (!gy) {
            throw BT::RuntimeError("missing required input gy");
        }
        param_subs["gy"] = gy.value();

        std::string tmp(R"((:action ContinueGymReminderProtocol
	:parameters ( ?gy - GymReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_gym ?gy) )
(gym_reminder_enabled ?gy)
(time_for_gym_reminder ?gy)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueGymReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ContinueGymReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class StartMedicineRefillReminderProtocol : public BT::SyncActionNode {
public:
    StartMedicineRefillReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("mdrf"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("cur"), BT::InputPort<std::string>("dest"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> mdrf = getInput<std::string>("mdrf");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> cur = getInput<std::string>("cur");
        BT::Optional <std::string> dest = getInput<std::string>("dest");
        
        if (!mdrf) {
            throw BT::RuntimeError("missing required input mdrf");
        }
        param_subs["mdrf"] = mdrf.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!cur) {
            throw BT::RuntimeError("missing required input cur");
        }
        param_subs["cur"] = cur.value();
        if (!dest) {
            throw BT::RuntimeError("missing required input dest");
        }
        param_subs["dest"] = dest.value();

        std::string tmp(R"((:action StartMedicineRefillReminderProtocol
	:parameters ( ?mdrf - MedicineRefillReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
(priority_2)
(robot_at ?cur)
(medicine_refill_location ?dest)
(time_for_medicine_refill_reminder ?mdrf)
(not (already_reminded_medicine_refill ?mdrf) )
(forall (?mdrf - MedicineRefillReminderProtocol)
(not (medicine_refill_reminder_enabled ?mdrf) )
)
(person_currently_at ?p ?cur)
(visible_location ?dest)
(not (not_visible_location ?dest) )
(visible_location ?cur)
(not (not_visible_location ?cur) )
)
	:effect (and
(success)
(not (priority_2) )
(medicine_refill_reminder_enabled ?mdrf)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_protocol_enabled ?med) )
)
(forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol)
(not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy) )
)
(forall (?gy - GymReminderProtocol)
(not (gym_reminder_enabled ?gy) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartMedicineRefillReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_StartMedicineRefillReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ContinueMedicineRefillReminderProtocol : public BT::SyncActionNode {
public:
    ContinueMedicineRefillReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("mdrf"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> mdrf = getInput<std::string>("mdrf");
        
        if (!mdrf) {
            throw BT::RuntimeError("missing required input mdrf");
        }
        param_subs["mdrf"] = mdrf.value();

        std::string tmp(R"((:action ContinueMedicineRefillReminderProtocol
	:parameters ( ?mdrf - MedicineRefillReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_medicine_refill ?mdrf) )
(medicine_refill_reminder_enabled ?mdrf)
(time_for_medicine_refill_reminder ?mdrf)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueMedicineRefillReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ContinueMedicineRefillReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class StartMedicineRefillPharmacyReminderProtocol : public BT::SyncActionNode {
public:
    StartMedicineRefillPharmacyReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ic"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("cur"), BT::InputPort<std::string>("dest"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> ic = getInput<std::string>("ic");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> cur = getInput<std::string>("cur");
        BT::Optional <std::string> dest = getInput<std::string>("dest");
        
        if (!ic) {
            throw BT::RuntimeError("missing required input ic");
        }
        param_subs["ic"] = ic.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!cur) {
            throw BT::RuntimeError("missing required input cur");
        }
        param_subs["cur"] = cur.value();
        if (!dest) {
            throw BT::RuntimeError("missing required input dest");
        }
        param_subs["dest"] = dest.value();

        std::string tmp(R"((:action StartMedicineRefillPharmacyReminderProtocol
	:parameters ( ?ic - MedicineRefillPharmacyReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
(priority_2)
(robot_at ?cur)
(medicine_pharmacy_location ?dest)
(time_for_medicine_pharmacy_reminder ?ic)
(not (already_reminded_medicine_pharmacy ?ic) )
(forall (?ic - MedicineRefillPharmacyReminderProtocol)
(not (medicine_pharmacy_reminder_enabled ?ic) )
)
(person_currently_at ?p ?cur)
(visible_location ?dest)
(not (not_visible_location ?dest) )
)
	:effect (and
(success)
(not (priority_2) )
(medicine_pharmacy_reminder_enabled ?ic)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_protocol_enabled ?med) )
)
(forall (?mdrf - MedicineRefillReminderProtocol)
(not (medicine_refill_reminder_enabled ?mdrf) )
)
(forall (?gy - GymReminderProtocol)
(not (gym_reminder_enabled ?gy) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartMedicineRefillPharmacyReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_StartMedicineRefillPharmacyReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class ContinueMedicineRefillPharmacyReminderProtocol : public BT::SyncActionNode {
public:
    ContinueMedicineRefillPharmacyReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ic"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> ic = getInput<std::string>("ic");
        
        if (!ic) {
            throw BT::RuntimeError("missing required input ic");
        }
        param_subs["ic"] = ic.value();

        std::string tmp(R"((:action ContinueMedicineRefillPharmacyReminderProtocol
	:parameters ( ?ic - MedicineRefillPharmacyReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_medicine_pharmacy ?ic) )
(medicine_pharmacy_reminder_enabled ?ic)
(time_for_medicine_pharmacy_reminder ?ic)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueMedicineRefillPharmacyReminderProtocol");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_ContinueMedicineRefillPharmacyReminderProtocol(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace high_level_domain {
template<typename T>
class Idle : public BT::SyncActionNode {
public:
    Idle(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action Idle
	:parameters ()
	:precondition (priority_5)
	:effect (and
(success)
(not (priority_5) )
(forall (?med - MedicineProtocol)
(not (medicine_protocol_enabled ?med) )
)
(forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocoli)
(not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy) )
)
(forall (?mdrf - MedicineRefillReminderProtocol)
(not (medicine_refill_reminder_enabled ?mdrf) )
)
(forall (?gy - GymReminderProtocol)
(not (gym_reminder_enabled ?gy) )
)
(not (low_level_failed) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for Idle");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.high_level_domain_Idle(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // high_level_domain

namespace shr_domain {
template<typename T>
class DetectTakingMedicine : public BT::SyncActionNode {
public:
    DetectTakingMedicine(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();

        std::string tmp(R"((:action DetectTakingMedicine
	:parameters ( ?t - Time)
	:precondition (and
(DetectTakingMedicine_enabled)
(current_time ?t)
(not (abort) )
)
	:observe (person_taking_medicine ?t)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for DetectTakingMedicine");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_DetectTakingMedicine(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class DetectEatingFood : public BT::SyncActionNode {
public:
    DetectEatingFood(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();

        std::string tmp(R"((:action DetectEatingFood
	:parameters ( ?t - Time)
	:precondition (and
(DetectEatingFood_enabled)
(current_time ?t)
(not (abort) )
)
	:observe (person_eating_food ?t)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for DetectEatingFood");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_DetectEatingFood(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class DetectPersonLocation : public BT::SyncActionNode {
public:
    DetectPersonLocation(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("lmp"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();

        std::string tmp(R"((:action DetectPersonLocation
	:parameters ( ?t - Time ?p - Person ?lmp - Landmark)
	:precondition (and
(current_time ?t)
(DetectPerson_enabled)
(not (abort) )
)
	:observe (person_at ?t ?p ?lmp)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for DetectPersonLocation");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_DetectPersonLocation(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class MoveToLandmark : public BT::SyncActionNode {
public:
    MoveToLandmark(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"), BT::InputPort<std::string>("from"), BT::InputPort<std::string>("to"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> from = getInput<std::string>("from");
        BT::Optional <std::string> to = getInput<std::string>("to");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!from) {
            throw BT::RuntimeError("missing required input from");
        }
        param_subs["from"] = from.value();
        if (!to) {
            throw BT::RuntimeError("missing required input to");
        }
        param_subs["to"] = to.value();

        std::string tmp(R"((:action MoveToLandmark
	:parameters ( ?t - Time ?from - Landmark ?to - Landmark)
	:precondition (and
(current_time ?t)
(not (used_move ?t ?to) )
(robot_at ?from)
(traversable ?from ?to)
(not (abort) )
)
	:effect (and
(robot_at ?to)
(not (robot_at ?from) )
(used_move ?t ?to)
(when (time_critical) (forall (?tn - Time)
(when (next_time ?t ?tn) (and
(not (current_time ?t) )
(current_time ?tn)
(robot_at_time ?tn ?to)
) )
) )
(when (not (time_critical) ) (robot_at_time ?t ?to) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for MoveToLandmark");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_MoveToLandmark(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class MakeCall : public BT::SyncActionNode {
public:
    MakeCall(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("a"), BT::InputPort<std::string>("t"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("m"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> a = getInput<std::string>("a");
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> m = getInput<std::string>("m");
        
        if (!a) {
            throw BT::RuntimeError("missing required input a");
        }
        param_subs["a"] = a.value();
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!m) {
            throw BT::RuntimeError("missing required input m");
        }
        param_subs["m"] = m.value();

        std::string tmp(R"((:action MakeCall
	:parameters ( ?a - CallAction ?t - Time ?p - Person ?m - Msg)
	:precondition (and
(MakeCall_enabled)
(current_time ?t)
(not (used_reminder ?t) )
(not (executed_call ?a) )
(valid_call_message ?a ?m)
(not (and
(call_person_not_taking_medicine_constraint ?a ?p)
(not (not (person_taking_medicine ?t) ) )
) )
(not (and
(call_person_not_eating_food_constraint ?a ?p)
(not (not (person_eating_food ?t) ) )
) )
(forall (?ai - CallAction)
(not (and
(call_blocks_call ?ai ?a)
(not (executed_call ?ai) )
) )
)
(forall (?ai - ReminderAction)
(not (and
(reminder_blocks_call ?ai ?a)
(not (executed_reminder ?ai) )
) )
)
(same_location_constraint)
(not (forall (?loc - Landmark)
(not (and
(person_at ?t ?p ?loc)
(robot_at ?loc)
) )
) )
(not (abort) )
)
	:effect (and
(message_given ?m)
(executed_call ?a)
(forall (?tn - Time)
(when (next_time ?t ?tn) (and
(not (current_time ?t) )
(current_time ?tn)
) )
)
(used_reminder ?t)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for MakeCall");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_MakeCall(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class GiveReminder : public BT::SyncActionNode {
public:
    GiveReminder(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("a"), BT::InputPort<std::string>("t"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("m"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> a = getInput<std::string>("a");
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> m = getInput<std::string>("m");
        
        if (!a) {
            throw BT::RuntimeError("missing required input a");
        }
        param_subs["a"] = a.value();
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!m) {
            throw BT::RuntimeError("missing required input m");
        }
        param_subs["m"] = m.value();

        std::string tmp(R"((:action GiveReminder
	:parameters ( ?a - ReminderAction ?t - Time ?p - Person ?m - Msg)
	:precondition (and
(GiveReminder_enabled)
(current_time ?t)
(not (used_reminder ?t) )
(valid_reminder_message ?a ?m)
(not (executed_reminder ?a) )
(not (and
(reminder_person_not_taking_medicine_constraint ?a ?p)
(not (not (person_taking_medicine ?t) ) )
) )
(not (and
(reminder_person_not_eating_food_constraint ?a ?p)
(not (not (person_eating_food ?t) ) )
) )
(forall (?ai - ReminderAction)
(not (and
(reminder_blocks_reminder ?ai ?a)
(not (executed_reminder ?ai) )
) )
)
(same_location_constraint)
(not (forall (?loc - Landmark)
(not (and
(person_at ?t ?p ?loc)
(robot_at ?loc)
) )
) )
(not (abort) )
)
	:effect (and
(message_given ?m)
(executed_reminder ?a)
(used_reminder ?t)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for GiveReminder");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_GiveReminder(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class Wait : public BT::SyncActionNode {
public:
    Wait(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"), BT::InputPort<std::string>("p"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> p = getInput<std::string>("p");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();

        std::string tmp(R"((:action Wait
	:parameters ( ?t - Time ?p - Person)
	:precondition (and
(forall (?lmr - Landmark)
(not (and
(not (robot_at ?lmr) )
(wait_robot_location_constraint ?t ?lmr)
) )
)
(current_time ?t)
(not (executed_wait ?t) )
(not (abort) )
(forall (?lmp - Landmark)
(not (and
(not (person_at ?t ?p ?lmp) )
(wait_person_location_constraint ?t ?p ?lmp)
) )
)
(forall (?lmp - Landmark)
(not (and
(person_at ?t ?p ?lmp)
(wait_not_person_location_constraint ?t ?p ?lmp)
) )
)
)
	:effect (and
(executed_wait ?t)
(forall (?tn - Time)
(when (next_time ?t ?tn) (and
(not (current_time ?t) )
(current_time ?tn)
) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for Wait");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_Wait(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class MessageGivenSuccess : public BT::SyncActionNode {
public:
    MessageGivenSuccess(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action MessageGivenSuccess
	:parameters ()
	:precondition (and
(not (forall (?m - Msg)
(not (and
(message_given_success ?m)
(message_given ?m)
) )
) )
(not (abort) )
)
	:effect (success)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for MessageGivenSuccess");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_MessageGivenSuccess(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class PersonAtSuccess : public BT::SyncActionNode {
public:
    PersonAtSuccess(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("p"), BT::InputPort<std::string>("t"), BT::InputPort<std::string>("lmp"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();

        std::string tmp(R"((:action PersonAtSuccess
	:parameters ( ?p - Person ?t - Time ?lmp - Landmark)
	:precondition (and
(person_at_success ?p ?lmp)
(success_location ?lmp)
(not (abort) )
)
	:effect (success)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for PersonAtSuccess");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_PersonAtSuccess(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class NoActionUsed : public BT::SyncActionNode {
public:
    NoActionUsed(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("t"), BT::InputPort<std::string>("p"), BT::InputPort<std::string>("na"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> t = getInput<std::string>("t");
        BT::Optional <std::string> p = getInput<std::string>("p");
        BT::Optional <std::string> na = getInput<std::string>("na");
        
        if (!t) {
            throw BT::RuntimeError("missing required input t");
        }
        param_subs["t"] = t.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();
        if (!na) {
            throw BT::RuntimeError("missing required input na");
        }
        param_subs["na"] = na.value();

        std::string tmp(R"((:action NoActionUsed
	:parameters ( ?t - Time ?p - Person ?na - NoAction)
	:precondition (and
(not (no_action) )
(forall (?loc - Landmark)
(not (and
(not (person_at ?t ?p ?loc) )
(noaction_person_location_constraint ?na ?p ?loc)
) )
)
(forall (?lmr - Landmark)
(not (and
(not (robot_at ?lmr) )
(wait_robot_location_constraint ?t ?lmr)
) )
)
(not (na_used ?na) )
(forall (?loc - Landmark)
(not (and
(person_at ?t ?p ?loc)
(noaction_not_person_location_constraint ?na ?p ?loc)
) )
)
(current_time ?t)
(not (abort) )
)
	:effect (and
(na_used ?na)
(forall (?tn - Time)
(when (next_time ?t ?tn) (and
(not (current_time ?t) )
(current_time ?tn)
) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for NoActionUsed");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_NoActionUsed(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class TimeOut : public BT::SyncActionNode {
public:
    TimeOut(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action TimeOut
	:parameters ()
	:precondition (and
(forall (?na - NoAction)
(na_used ?na)
)
(not (abort) )
)
	:effect (success)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for TimeOut");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_TimeOut(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class MedicineTakenSuccess : public BT::SyncActionNode {
public:
    MedicineTakenSuccess(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action MedicineTakenSuccess
	:parameters ()
	:precondition (and
(not (forall (?t - Time)
(not (and
(medicine_taken_success)
(person_taking_medicine ?t)
) )
) )
(not (abort) )
)
	:effect (success)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for MedicineTakenSuccess");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_MedicineTakenSuccess(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

namespace shr_domain {
template<typename T>
class FoodEatenSuccess : public BT::SyncActionNode {
public:
    FoodEatenSuccess(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        

        std::string tmp(R"((:action FoodEatenSuccess
	:parameters ()
	:precondition (and
(not (forall (?t - Time)
(not (and
(food_eaten_success)
(person_eating_food ?t)
) )
) )
(not (abort) )
)
	:effect (success)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for FoodEatenSuccess");
        }
        auto action = action_opt.value();

        auto & kb = KnowledgeBase::getInstance();
        InstantiatedAction inst_action = instantiate_action(action, param_subs, kb.get_objects());
        auto precondition_met = kb.check_conditions(inst_action.precondtions);
//        KnowledgeBase::getInstance().print_predicate();
        std::cout << "ACTION: " << inst_action.name << " params: ";
        for (const auto & p : inst_action.parameters){
            std::cout << p.name << " ";
        }
        std::cout << std::endl;

        T inst;
        if (precondition_met == TRUTH_VALUE::FALSE){
            std::cout << "\t" << "abort: preconditions violated" << std::endl;
//            throw std::runtime_error("abort: preconditions violated");
            inst.abort(inst_action);
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus status = inst.shr_domain_FoodEatenSuccess(inst_action);
        if (status == BT::NodeStatus::SUCCESS){
            kb.apply_conditions(inst_action.effect);
            kb.apply_conditions(inst_action.observe);
            kb.apply_constraints();

        } else if (status == BT::NodeStatus::FAILURE){
            InstantiatedCondition cond;
            cond.op = OPERATION::NOT;
            cond.conditions.push_back(inst_action.observe);
            kb.apply_conditions(cond);
            kb.apply_constraints();

            std::cout << "\t" << "action failed" << std::endl;
        }
        return status;
    }

};
} // shr_domain

template<typename T>
BT::BehaviorTreeFactory create_tree_factory(){
    static_assert(
            std::is_base_of<ActionInterface, T>::value,
            "template is not derived from ActionInterface"
    );

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<high_level_domain::MoveToLandmark<T>>("high_level_domain_MoveToLandmark");
    factory.registerNodeType<high_level_domain::ChangePriority_1_2<T>>("high_level_domain_ChangePriority_1_2");
    factory.registerNodeType<high_level_domain::ChangePriority_2_3<T>>("high_level_domain_ChangePriority_2_3");
    factory.registerNodeType<high_level_domain::ChangePriority_3_4<T>>("high_level_domain_ChangePriority_3_4");
    factory.registerNodeType<high_level_domain::ChangePriority_4_5<T>>("high_level_domain_ChangePriority_4_5");
    factory.registerNodeType<high_level_domain::StartMedicineProtocol<T>>("high_level_domain_StartMedicineProtocol");
    factory.registerNodeType<high_level_domain::ContinueMedicineProtocol<T>>("high_level_domain_ContinueMedicineProtocol");
    factory.registerNodeType<high_level_domain::StartGymReminderProtocol<T>>("high_level_domain_StartGymReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueGymReminderProtocol<T>>("high_level_domain_ContinueGymReminderProtocol");
    factory.registerNodeType<high_level_domain::StartMedicineRefillReminderProtocol<T>>("high_level_domain_StartMedicineRefillReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueMedicineRefillReminderProtocol<T>>("high_level_domain_ContinueMedicineRefillReminderProtocol");
    factory.registerNodeType<high_level_domain::StartMedicineRefillPharmacyReminderProtocol<T>>("high_level_domain_StartMedicineRefillPharmacyReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueMedicineRefillPharmacyReminderProtocol<T>>("high_level_domain_ContinueMedicineRefillPharmacyReminderProtocol");
    factory.registerNodeType<high_level_domain::Idle<T>>("high_level_domain_Idle");
    factory.registerNodeType<shr_domain::DetectTakingMedicine<T>>("shr_domain_DetectTakingMedicine");
    factory.registerNodeType<shr_domain::DetectEatingFood<T>>("shr_domain_DetectEatingFood");
    factory.registerNodeType<shr_domain::DetectPersonLocation<T>>("shr_domain_DetectPersonLocation");
    factory.registerNodeType<shr_domain::MoveToLandmark<T>>("shr_domain_MoveToLandmark");
    factory.registerNodeType<shr_domain::MakeCall<T>>("shr_domain_MakeCall");
    factory.registerNodeType<shr_domain::GiveReminder<T>>("shr_domain_GiveReminder");
    factory.registerNodeType<shr_domain::Wait<T>>("shr_domain_Wait");
    factory.registerNodeType<shr_domain::MessageGivenSuccess<T>>("shr_domain_MessageGivenSuccess");
    factory.registerNodeType<shr_domain::PersonAtSuccess<T>>("shr_domain_PersonAtSuccess");
    factory.registerNodeType<shr_domain::NoActionUsed<T>>("shr_domain_NoActionUsed");
    factory.registerNodeType<shr_domain::TimeOut<T>>("shr_domain_TimeOut");
    factory.registerNodeType<shr_domain::MedicineTakenSuccess<T>>("shr_domain_MedicineTakenSuccess");
    factory.registerNodeType<shr_domain::FoodEatenSuccess<T>>("shr_domain_FoodEatenSuccess");

    return factory;
}


} // pddl_lib