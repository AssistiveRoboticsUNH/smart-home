#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "pddl_parser/pddl_parser.hpp"

namespace pddl_lib {

class MoveReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit MoveReminderProtocol(const std::string& str) : std::string(str) {}
};
class Msg : public std::string {
public:
    using std::string::string;
    explicit Msg(const std::string& str) : std::string(str) {}
};
class InternalCheckReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit InternalCheckReminderProtocol(const std::string& str) : std::string(str) {}
};
class Landmark : public std::string {
public:
    using std::string::string;
    explicit Landmark(const std::string& str) : std::string(str) {}
};
class Time : public std::string {
public:
    using std::string::string;
    explicit Time(const std::string& str) : std::string(str) {}
};
class ReminderAction : public std::string {
public:
    using std::string::string;
    explicit ReminderAction(const std::string& str) : std::string(str) {}
};
class Person : public std::string {
public:
    using std::string::string;
    explicit Person(const std::string& str) : std::string(str) {}
};
class NoAction : public std::string {
public:
    using std::string::string;
    explicit NoAction(const std::string& str) : std::string(str) {}
};
class MedicineProtocol : public std::string {
public:
    using std::string::string;
    explicit MedicineProtocol(const std::string& str) : std::string(str) {}
};
class PracticeReminderProtocol : public std::string {
public:
    using std::string::string;
    explicit PracticeReminderProtocol(const std::string& str) : std::string(str) {}
};
class WaitAction : public std::string {
public:
    using std::string::string;
    explicit WaitAction(const std::string& str) : std::string(str) {}
};


class UpdatePredicates {
public:
  void update() const {
      auto & kb = KnowledgeBase::getInstance();
      std::vector<InstantiatedParameter> MoveReminderProtocol_instances;
      std::vector<InstantiatedParameter> Msg_instances;
      std::vector<InstantiatedParameter> InternalCheckReminderProtocol_instances;
      std::vector<InstantiatedParameter> Landmark_instances;
      std::vector<InstantiatedParameter> Time_instances;
      std::vector<InstantiatedParameter> ReminderAction_instances;
      std::vector<InstantiatedParameter> Person_instances;
      std::vector<InstantiatedParameter> NoAction_instances;
      std::vector<InstantiatedParameter> MedicineProtocol_instances;
      std::vector<InstantiatedParameter> PracticeReminderProtocol_instances;
      std::vector<InstantiatedParameter> WaitAction_instances;

      for (const auto object : kb.get_objects()){
          if (object.type == "MoveReminderProtocol"){
              MoveReminderProtocol_instances.push_back(object);
          }
          if (object.type == "Msg"){
              Msg_instances.push_back(object);
          }
          if (object.type == "InternalCheckReminderProtocol"){
              InternalCheckReminderProtocol_instances.push_back(object);
          }
          if (object.type == "Landmark"){
              Landmark_instances.push_back(object);
          }
          if (object.type == "Time"){
              Time_instances.push_back(object);
          }
          if (object.type == "ReminderAction"){
              ReminderAction_instances.push_back(object);
          }
          if (object.type == "Person"){
              Person_instances.push_back(object);
          }
          if (object.type == "NoAction"){
              NoAction_instances.push_back(object);
          }
          if (object.type == "MedicineProtocol"){
              MedicineProtocol_instances.push_back(object);
          }
          if (object.type == "PracticeReminderProtocol"){
              PracticeReminderProtocol_instances.push_back(object);
          }
          if (object.type == "WaitAction"){
              WaitAction_instances.push_back(object);
          }
      }

      
        {
            for (auto PracticeReminderProtocol_instance_1 : PracticeReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_practice_reminder", {PracticeReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_practice_reminder(old_val, PracticeReminderProtocol(PracticeReminderProtocol_instance_1.name));
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
              InstantiatedPredicate pred = {"already_reminded_medicine", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_medicine(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
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
            for (auto MoveReminderProtocol_instance_1 : MoveReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_move", {MoveReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_move(old_val, MoveReminderProtocol(MoveReminderProtocol_instance_1.name));
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
            for (auto PracticeReminderProtocol_instance_1 : PracticeReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_practice", {PracticeReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_practice(old_val, PracticeReminderProtocol(PracticeReminderProtocol_instance_1.name));
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
            for (auto PracticeReminderProtocol_instance_1 : PracticeReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"practice_reminder_enabled", {PracticeReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = practice_reminder_enabled(old_val, PracticeReminderProtocol(PracticeReminderProtocol_instance_1.name));
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
            for (auto InternalCheckReminderProtocol_instance_1 : InternalCheckReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_internal_check_reminder", {InternalCheckReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_internal_check_reminder(old_val, InternalCheckReminderProtocol(InternalCheckReminderProtocol_instance_1.name));
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
            for (auto MoveReminderProtocol_instance_1 : MoveReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"move_reminder_enabled", {MoveReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = move_reminder_enabled(old_val, MoveReminderProtocol(MoveReminderProtocol_instance_1.name));
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
            for (auto MoveReminderProtocol_instance_1 : MoveReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"time_for_move_reminder", {MoveReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = time_for_move_reminder(old_val, MoveReminderProtocol(MoveReminderProtocol_instance_1.name));
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
            for (auto MedicineProtocol_instance_1 : MedicineProtocol_instances ) {
              InstantiatedPredicate pred = {"medicine_reminder_enabled", {MedicineProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = medicine_reminder_enabled(old_val, MedicineProtocol(MedicineProtocol_instance_1.name));
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
            for (auto InternalCheckReminderProtocol_instance_1 : InternalCheckReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"internal_check_reminder_enabled", {InternalCheckReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = internal_check_reminder_enabled(old_val, InternalCheckReminderProtocol(InternalCheckReminderProtocol_instance_1.name));
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
            for (auto InternalCheckReminderProtocol_instance_1 : InternalCheckReminderProtocol_instances ) {
              InstantiatedPredicate pred = {"already_reminded_internal_check", {InternalCheckReminderProtocol_instance_1 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = already_reminded_internal_check(old_val, InternalCheckReminderProtocol(InternalCheckReminderProtocol_instance_1.name));
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
            for (auto Landmark_instance_2 : Landmark_instances ) {
              InstantiatedPredicate pred = {"reminder_robot_location_constraint", {ReminderAction_instance_1, Landmark_instance_2 } };
              TRUTH_VALUE old_val;
              if (kb.find_predicate(pred)){
                  old_val = TRUTH_VALUE::TRUE;
              } else if(kb.find_unknown_predicate(pred)){
                  old_val = TRUTH_VALUE::UNKNOWN;
              } else{
                  old_val = TRUTH_VALUE::FALSE;
              }
            auto new_val = reminder_robot_location_constraint(old_val, ReminderAction(ReminderAction_instance_1.name), Landmark(Landmark_instance_2.name));
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

  }

    virtual TRUTH_VALUE time_for_practice_reminder(TRUTH_VALUE val, PracticeReminderProtocol pra) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_medicine(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE priority_1(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE priority_4(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE already_took_medicine(TRUTH_VALUE val, MedicineProtocol m) const {
        return val;
    }
    virtual TRUTH_VALUE priority_5(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_move(TRUTH_VALUE val, MoveReminderProtocol mv) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_practice(TRUTH_VALUE val, PracticeReminderProtocol pra) const {
        return val;
    }
    virtual TRUTH_VALUE priority_2(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE practice_reminder_enabled(TRUTH_VALUE val, PracticeReminderProtocol pra) const {
        return val;
    }
    virtual TRUTH_VALUE low_level_failed(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE time_for_internal_check_reminder(TRUTH_VALUE val, InternalCheckReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE move_reminder_enabled(TRUTH_VALUE val, MoveReminderProtocol mv) const {
        return val;
    }
    virtual TRUTH_VALUE time_for_move_reminder(TRUTH_VALUE val, MoveReminderProtocol mv) const {
        return val;
    }
    virtual TRUTH_VALUE priority_3(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_reminder_enabled(TRUTH_VALUE val, MedicineProtocol med) const {
        return val;
    }
    virtual TRUTH_VALUE success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol med) const {
        return val;
    }
    virtual TRUTH_VALUE visible_location(TRUTH_VALUE val, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE robot_at(TRUTH_VALUE val, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE internal_check_reminder_enabled(TRUTH_VALUE val, InternalCheckReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE person_currently_at(TRUTH_VALUE val, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE person_at(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE already_reminded_internal_check(TRUTH_VALUE val, InternalCheckReminderProtocol ic) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_robot_location_constraint(TRUTH_VALUE val, ReminderAction a, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE used_reminder(TRUTH_VALUE val, Time tc) const {
        return val;
    }
    virtual TRUTH_VALUE noaction_not_person_location_constraint(TRUTH_VALUE val, NoAction na, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE DetectPerson_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE DetectTakingMedicine_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE executed_wait(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE time_critical(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE wait_person_location_constraint(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_blocks_reminder(TRUTH_VALUE val, ReminderAction a1, ReminderAction a2) const {
        return val;
    }
    virtual TRUTH_VALUE message_given_success(TRUTH_VALUE val, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE noaction_person_location_constraint(TRUTH_VALUE val, NoAction na, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE robot_at_time(TRUTH_VALUE val, Time t, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE food_eaten_success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE na_used(TRUTH_VALUE val, NoAction na) const {
        return val;
    }
    virtual TRUTH_VALUE used_move(TRUTH_VALUE val, Time tc, Landmark lmr) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_taking_medicine_constraint(TRUTH_VALUE val, ReminderAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE person_taking_medicine(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE person_eating_food(TRUTH_VALUE val, Time t) const {
        return val;
    }
    virtual TRUTH_VALUE move_to_home_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE no_action(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE next_time(TRUTH_VALUE val, Time tc, Time tn) const {
        return val;
    }
    virtual TRUTH_VALUE traversable(TRUTH_VALUE val, Landmark from, Landmark to) const {
        return val;
    }
    virtual TRUTH_VALUE not_same_location_constraint(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE current_time(TRUTH_VALUE val, Time tc) const {
        return val;
    }
    virtual TRUTH_VALUE medicine_taken_success(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE wait_robot_location_constraint(TRUTH_VALUE val, Time t, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE person_at_success(TRUTH_VALUE val, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE message_given(TRUTH_VALUE val, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_eating_food_constraint(TRUTH_VALUE val, ReminderAction a, Person p) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_not_location_constraint(TRUTH_VALUE val, ReminderAction a, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE executed_reminder(TRUTH_VALUE val, ReminderAction a) const {
        return val;
    }
    virtual TRUTH_VALUE same_location_constraint(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE success_location(TRUTH_VALUE val, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE abort(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE reminder_person_location_constraint(TRUTH_VALUE val, ReminderAction a, Person p, Landmark lmp) const {
        return val;
    }
    virtual TRUTH_VALUE valid_reminder_message(TRUTH_VALUE val, ReminderAction a, Msg m) const {
        return val;
    }
    virtual TRUTH_VALUE DetectEatingFood_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE GiveReminder_enabled(TRUTH_VALUE val) const {
        return val;
    }
    virtual TRUTH_VALUE wait_not_person_location_constraint(TRUTH_VALUE val, Time t, Person p, Landmark lmp) const {
        return val;
    }
    };

class ActionInterface {
public:
    virtual BT::NodeStatus high_level_domain_ChangePriority_1_2(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_2_3(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_3_4(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ChangePriority_4_5(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartMedReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueMedReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartMoveReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueMoveReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartInternalCheckReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinueInternalCheckReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_StartPracticeReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_ContinuePracticeReminderProtocol(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus high_level_domain_Idle(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
    virtual BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction & action){return BT::NodeStatus::SUCCESS;} // do nothing default
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
class StartMedReminderProtocol : public BT::SyncActionNode {
public:
    StartMedReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("m"), BT::InputPort<std::string>("lmp"), BT::InputPort<std::string>("p"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> m = getInput<std::string>("m");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        BT::Optional <std::string> p = getInput<std::string>("p");
        
        if (!m) {
            throw BT::RuntimeError("missing required input m");
        }
        param_subs["m"] = m.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();

        std::string tmp(R"((:action StartMedReminderProtocol
	:parameters ( ?m - MedicineProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
(priority_2)
(time_to_take_medicine ?m)
(not (already_took_medicine ?m) )
(not (already_reminded_medicine ?m) )
(forall (?med - MedicineProtocol)
(not (medicine_reminder_enabled ?med) )
)
(person_currently_at ?p ?lmp)
(visible_location ?lmp)
)
	:effect (and
(success)
(not (priority_2) )
(medicine_reminder_enabled ?m)
(not (low_level_failed) )
(forall (?internal - InternalCheckReminderProtocol)
(not (internal_check_reminder_enabled ?internal) )
)
(forall (?practice - PracticeReminderProtocol)
(not (practice_reminder_enabled ?practice) )
)
(forall (?mv - MoveReminderProtocol)
(not (move_reminder_enabled ?mv) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartMedReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_StartMedReminderProtocol(inst_action);
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
class ContinueMedReminderProtocol : public BT::SyncActionNode {
public:
    ContinueMedReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
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

        std::string tmp(R"((:action ContinueMedReminderProtocol
	:parameters ( ?m - MedicineProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(medicine_reminder_enabled ?m)
(not (already_took_medicine ?m) )
(not (already_reminded_medicine ?m) )
(time_to_take_medicine ?m)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueMedReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_ContinueMedReminderProtocol(inst_action);
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
class StartMoveReminderProtocol : public BT::SyncActionNode {
public:
    StartMoveReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("mv"), BT::InputPort<std::string>("lmp"), BT::InputPort<std::string>("p"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> mv = getInput<std::string>("mv");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        BT::Optional <std::string> p = getInput<std::string>("p");
        
        if (!mv) {
            throw BT::RuntimeError("missing required input mv");
        }
        param_subs["mv"] = mv.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();

        std::string tmp(R"((:action StartMoveReminderProtocol
	:parameters ( ?mv - MoveReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
(priority_2)
(time_for_move_reminder ?mv)
(not (already_reminded_move ?mv) )
(forall (?mv - MoveReminderProtocol)
(not (move_reminder_enabled ?mv) )
)
(person_currently_at ?p ?lmp)
(visible_location ?lmp)
)
	:effect (and
(success)
(not (priority_2) )
(move_reminder_enabled ?mv)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_reminder_enabled ?med) )
)
(forall (?internal - InternalCheckReminderProtocol)
(not (internal_check_reminder_enabled ?internal) )
)
(forall (?practice - PracticeReminderProtocol)
(not (practice_reminder_enabled ?practice) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartMoveReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_StartMoveReminderProtocol(inst_action);
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
class ContinueMoveReminderProtocol : public BT::SyncActionNode {
public:
    ContinueMoveReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("mv"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> mv = getInput<std::string>("mv");
        
        if (!mv) {
            throw BT::RuntimeError("missing required input mv");
        }
        param_subs["mv"] = mv.value();

        std::string tmp(R"((:action ContinueMoveReminderProtocol
	:parameters ( ?mv - MoveReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_move ?mv) )
(move_reminder_enabled ?mv)
(time_for_move_reminder ?mv)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueMoveReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_ContinueMoveReminderProtocol(inst_action);
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
class StartInternalCheckReminderProtocol : public BT::SyncActionNode {
public:
    StartInternalCheckReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ic"), BT::InputPort<std::string>("lmp"), BT::InputPort<std::string>("p"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> ic = getInput<std::string>("ic");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        BT::Optional <std::string> p = getInput<std::string>("p");
        
        if (!ic) {
            throw BT::RuntimeError("missing required input ic");
        }
        param_subs["ic"] = ic.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();

        std::string tmp(R"((:action StartInternalCheckReminderProtocol
	:parameters ( ?ic - InternalCheckReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
(priority_2)
(time_for_internal_check_reminder ?ic)
(not (already_reminded_internal_check ?ic) )
(forall (?ic - InternalCheckReminderProtocol)
(not (internal_check_reminder_enabled ?ic) )
)
(person_currently_at ?p ?lmp)
(visible_location ?lmp)
)
	:effect (and
(success)
(not (priority_2) )
(internal_check_reminder_enabled ?ic)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_reminder_enabled ?med) )
)
(forall (?move - MoveReminderProtocol)
(not (move_reminder_enabled ?move) )
)
(forall (?practice - PracticeReminderProtocol)
(not (practice_reminder_enabled ?practice) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartInternalCheckReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_StartInternalCheckReminderProtocol(inst_action);
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
class ContinueInternalCheckReminderProtocol : public BT::SyncActionNode {
public:
    ContinueInternalCheckReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
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

        std::string tmp(R"((:action ContinueInternalCheckReminderProtocol
	:parameters ( ?ic - InternalCheckReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_internal_check ?ic) )
(internal_check_reminder_enabled ?ic)
(time_for_internal_check_reminder ?ic)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinueInternalCheckReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_ContinueInternalCheckReminderProtocol(inst_action);
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
class StartPracticeReminderProtocol : public BT::SyncActionNode {
public:
    StartPracticeReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("pra"), BT::InputPort<std::string>("lmp"), BT::InputPort<std::string>("p"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> pra = getInput<std::string>("pra");
        BT::Optional <std::string> lmp = getInput<std::string>("lmp");
        BT::Optional <std::string> p = getInput<std::string>("p");
        
        if (!pra) {
            throw BT::RuntimeError("missing required input pra");
        }
        param_subs["pra"] = pra.value();
        if (!lmp) {
            throw BT::RuntimeError("missing required input lmp");
        }
        param_subs["lmp"] = lmp.value();
        if (!p) {
            throw BT::RuntimeError("missing required input p");
        }
        param_subs["p"] = p.value();

        std::string tmp(R"((:action StartPracticeReminderProtocol
	:parameters ( ?pra - PracticeReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
(priority_2)
(time_for_practice_reminder ?pra)
(not (already_reminded_practice ?pra) )
(forall (?pra - PracticeReminderProtocol)
(not (practice_reminder_enabled ?pra) )
)
(person_currently_at ?p ?lmp)
(visible_location ?lmp)
)
	:effect (and
(success)
(not (priority_2) )
(practice_reminder_enabled ?pra)
(not (low_level_failed) )
(forall (?med - MedicineProtocol)
(not (medicine_reminder_enabled ?med) )
)
(forall (?move - MoveReminderProtocol)
(not (move_reminder_enabled ?move) )
)
(forall (?internal - InternalCheckReminderProtocol)
(not (internal_check_reminder_enabled ?internal) )
)
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for StartPracticeReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_StartPracticeReminderProtocol(inst_action);
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
class ContinuePracticeReminderProtocol : public BT::SyncActionNode {
public:
    ContinuePracticeReminderProtocol(const std::string &name, const BT::NodeConfiguration &config)
            : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("pra"),  };
    }

    BT::NodeStatus tick() override {
        std::unordered_map<std::string, std::string> param_subs;
        BT::Optional <std::string> pra = getInput<std::string>("pra");
        
        if (!pra) {
            throw BT::RuntimeError("missing required input pra");
        }
        param_subs["pra"] = pra.value();

        std::string tmp(R"((:action ContinuePracticeReminderProtocol
	:parameters ( ?pra - PracticeReminderProtocol)
	:precondition (and
(priority_2)
(not (low_level_failed) )
(not (already_reminded_practice ?pra) )
(practice_reminder_enabled ?pra)
(time_for_practice_reminder ?pra)
)
	:effect (and
(success)
(not (priority_2) )
)
)
)");
        tl::expected<Action, std::string> action_opt = parse_action(tmp);
        if (!action_opt.has_value()) {
            throw BT::RuntimeError("failed to parse action string for ContinuePracticeReminderProtocol");
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
        BT::NodeStatus status = inst.high_level_domain_ContinuePracticeReminderProtocol(inst_action);
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
(not (medicine_reminder_enabled ?med) )
)
(forall (?internal - InternalCheckReminderProtocol)
(not (internal_check_reminder_enabled ?internal) )
)
(forall (?practice - PracticeReminderProtocol)
(not (practice_reminder_enabled ?practice) )
)
(forall (?move - MoveReminderProtocol)
(not (move_reminder_enabled ?move) )
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
    
    factory.registerNodeType<high_level_domain::ChangePriority_1_2<T>>("high_level_domain_ChangePriority_1_2");
    factory.registerNodeType<high_level_domain::ChangePriority_2_3<T>>("high_level_domain_ChangePriority_2_3");
    factory.registerNodeType<high_level_domain::ChangePriority_3_4<T>>("high_level_domain_ChangePriority_3_4");
    factory.registerNodeType<high_level_domain::ChangePriority_4_5<T>>("high_level_domain_ChangePriority_4_5");
    factory.registerNodeType<high_level_domain::StartMedReminderProtocol<T>>("high_level_domain_StartMedReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueMedReminderProtocol<T>>("high_level_domain_ContinueMedReminderProtocol");
    factory.registerNodeType<high_level_domain::StartMoveReminderProtocol<T>>("high_level_domain_StartMoveReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueMoveReminderProtocol<T>>("high_level_domain_ContinueMoveReminderProtocol");
    factory.registerNodeType<high_level_domain::StartInternalCheckReminderProtocol<T>>("high_level_domain_StartInternalCheckReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinueInternalCheckReminderProtocol<T>>("high_level_domain_ContinueInternalCheckReminderProtocol");
    factory.registerNodeType<high_level_domain::StartPracticeReminderProtocol<T>>("high_level_domain_StartPracticeReminderProtocol");
    factory.registerNodeType<high_level_domain::ContinuePracticeReminderProtocol<T>>("high_level_domain_ContinuePracticeReminderProtocol");
    factory.registerNodeType<high_level_domain::Idle<T>>("high_level_domain_Idle");
    factory.registerNodeType<shr_domain::DetectTakingMedicine<T>>("shr_domain_DetectTakingMedicine");
    factory.registerNodeType<shr_domain::DetectEatingFood<T>>("shr_domain_DetectEatingFood");
    factory.registerNodeType<shr_domain::DetectPersonLocation<T>>("shr_domain_DetectPersonLocation");
    factory.registerNodeType<shr_domain::MoveToLandmark<T>>("shr_domain_MoveToLandmark");
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