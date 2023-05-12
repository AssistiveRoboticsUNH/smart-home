// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/bool.hpp"
#include "shr_msgs/action/gather_information_request.hpp"
#include "shr_msgs/msg/midnight_warning_protocol.hpp"
#include "shr_msgs/msg/medicine_reminder_protocol.hpp"
#include "shr_msgs/msg/food_reminder_protocol.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include <rclcpp_action/client.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <shr_plan_parameters.hpp>


namespace planning_controller {

  typedef enum {
    IDLE, GATHERING_INFO, PLANNING, EXECUTING
  } StateType;


  class PlanningControllerSpin : public rclcpp::Node {
  public:
    PlanningControllerSpin()
        : rclcpp::Node("planing_controller_spin") {
      using std::placeholders::_1;
      param_listener_ = std::make_shared<shr_plan_parameters::ParamListener>(get_node_parameters_interface());
      params_ = param_listener_->get_params();

      world_state_sub_ = this->create_subscription<shr_msgs::msg::WorldState>(
          params_.world_state_topic, 10,
          std::bind(&PlanningControllerSpin::update_world_state_callback, this, _1));

      action_hub_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
          "actions_hub", rclcpp::QoS(100).reliable(),
          std::bind(&PlanningControllerSpin::action_hub_callback, this, _1));
    }

    shr_msgs::msg::WorldState get_world_state() {
      std::lock_guard<std::mutex> lock(mutex_);
      new_world_ = false;
      return world_state_;
    }

    std::vector<std::string> get_completed_actions() {
      std::lock_guard<std::mutex> lock(mutex_);
      auto tmp = completed_actions_;
      completed_actions_.clear();
      return tmp;
    }

    bool has_new_completed_actions() {
      return !completed_actions_.empty();
    }


  private:

    void update_world_state_callback(const shr_msgs::msg::WorldState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (world_state_ != *msg) {
        world_state_ = *msg;
        new_world_ = true;
      }
    }


    void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      if (msg->type == plansys2_msgs::msg::ActionExecution::FINISH) {
        std::cout << "Action: " << msg->action << " has completed!!!!!!!!!!" << std::endl;
        completed_actions_.push_back(msg->action);
      }
    }

    shr_msgs::msg::WorldState world_state_;
    std::vector<std::string> completed_actions_;
    bool new_world_;
    rclcpp::Subscription<shr_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;
    std::shared_ptr<shr_plan_parameters::ParamListener> param_listener_;
    shr_plan_parameters::Params params_;
    std::mutex mutable mutex_;

  };


  class PlanningController : public rclcpp::Node {
  public:
    PlanningController()
        : rclcpp::Node("planing_controller"), state_(IDLE) {

      param_listener_ = std::make_shared<shr_plan_parameters::ParamListener>(get_node_parameters_interface());
      params_ = param_listener_->get_params();

      using std::placeholders::_1;

      gathering_info_client_ = rclcpp_action::create_client<shr_msgs::action::GatherInformationRequest>(
          this, "gather_information");

    }

    void set_world_state(const shr_msgs::msg::WorldState &world_state) {
      if (world_state_ != world_state) {
        world_state_ = world_state;
      }
    }

    void init() {
      domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
      planner_client_ = std::make_shared<plansys2::PlannerClient>();
      problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
      executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    }

    void reset_protocol_state() {
      active_protocol = "";
      midnight_warning_state_.reset();
      medicine_reminder_state.reset();
      food_reminder_state.reset();
    }

    StateType get_transition() {
      if (world_state_.too_late_to_leave && world_state_.door_motion_sensor) {
        active_protocol = "midnight_reminder";
      }
      if (world_state_.time_to_take_medicine && !world_state_.took_medicine) {
        active_protocol = "medicine_reminder";
      }
      if (world_state_.time_to_eat   && !world_state_.ate) {
          active_protocol = "food_reminder";
      }
      if (active_protocol.empty()) {
        return IDLE;
      }
      if (state_ == IDLE) {
        if (!active_protocol.empty()) {
          return PLANNING;
        }
      }

      return state_;
    }

    void step() {

      switch (state_) {
        case IDLE: {
          state_ = get_transition();
          break;
        }
        case GATHERING_INFO: {
          state_ = get_transition();
          if (state_ != GATHERING_INFO) {
            gathering_info_client_->async_cancel_all_goals();
            break;
          }

          if (!gathering_info) {
            auto result_callback = [this](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::GatherInformationRequest>::WrappedResult &res) {
              gathering_info = false;
              if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
                world_state_ = res.result->world_state;
              }
              state_ = IDLE;
            };
            auto send_goal_options = rclcpp_action::Client<shr_msgs::action::GatherInformationRequest>::SendGoalOptions();
            send_goal_options.result_callback = result_callback;
            auto request = shr_msgs::action::GatherInformationRequest::Goal();
            request.states = requested_states;
            gathering_info_client_->async_send_goal(request, send_goal_options);
            gathering_info = true;
          }
          break;
        }
        case PLANNING: {
          if (!init_plan()) {
            state_ = IDLE;
            break;
          }

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
                      parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = EXECUTING;
          }

          break;
        }
        case EXECUTING: {
          state_ = get_transition();
          if (state_ != EXECUTING) {
            executor_client_->cancel_plan_execution();
            break;
          }

          auto feedback = executor_client_->getFeedBack();

//          for (const auto &action_feedback: feedback.action_execution_status) {
//            std::cout << "[" << action_feedback.action << " " <<
//                      action_feedback.completion * 100.0 << "%]";
//          }
//          if (!feedback.action_execution_status.empty()) std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;
              state_ = IDLE;
              active_protocol = "";
            } else {
              state_ = PLANNING;
            }
          }

          break;
        }

      }
    }

  private:


    bool init_plan() {

        if (active_protocol == "midnight_reminder") {
//          problem_expert_->clearKnowledge();
            std::string domain = pkgpath + "/pddl/midnight_domain.pddl";
            bool domain_result = domain_expert_->setDomain(domain);

            if (domain_result) {
                std::cout << "Midnight Domain set successfully: " << domain_result << std::endl;
            } else {
                std::cout << "Failed to set midnight domain" << std::endl;
            }
            bool problem_result = problem_expert_->setDomain(domain);
            if (problem_result) {
                std::cout << "Midnight problem set successfully: " << problem_result << std::endl;
            } else {
                std::cout << "Failed to set midnight problem" << std::endl;
            }

//            problem_expert_->addInstance(plansys2::Instance{world_state_.door_location, "landmark"});
            std::vector<std::string> search_locations = {"bedroom_robot_pos", "kitchen_robot_pos", "couch_robot_pos", "door_robot_pos"};
            for (const auto& loc : search_locations){
                problem_expert_->addInstance(plansys2::Instance{loc, "landmark"});
            }

            problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
            problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});

            problem_expert_->addInstance(plansys2::Instance{params_.patient_name, "person"});
//            problem_expert_->addInstance(plansys2::Instance{"midnight_warning", "automated_message"});
//            problem_expert_->addInstance(plansys2::Instance{"midnight_warning_video", "recorded_message"});


            problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
            problem_expert_->addPredicate(plansys2::Predicate(
                    "(person_at " + params_.patient_name + " " + "door_robot_pos" + ")"));
            problem_expert_->addPredicate(
                    plansys2::Predicate("(door_location " + world_state_.door_location + ")"));
//            if (world_state_.door_open == 1) {
//                problem_expert_->addPredicate(plansys2::Predicate("(automated_message_given midnight_warning)"));
//            }

            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_go_outside_1 ))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_go_outside_2))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_return_1))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_return_2))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_go_to_bed_1))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (person_decides_to_go_to_bed_2))"));

            
            problem_expert_->setGoal(plansys2::Goal("(and (success) )"));

            return true;
        }


      if (active_protocol == "medicine_reminder") {
//          problem_expert_->clearKnowledge();
          std::string domain = pkgpath + "/pddl/medicine_domain.pddl";
          bool domain_result = domain_expert_->setDomain(domain);

          if (domain_result) {
              std::cout << "Medicine Domain set successfully: " << domain_result << std::endl;
          } else {
              std::cout << "Failed to set medicine domain" << std::endl;
          }
          bool problem_result = problem_expert_->setDomain(domain);
          if (problem_result) {
              std::cout << "Medicine problem set successfully: " << problem_result << std::endl;
          } else {
              std::cout << "Failed to set medicine problem" << std::endl;
          }

        problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
        std::vector<std::string> search_locations = {"bedroom_robot_pos", "kitchen_robot_pos", "couch_robot_pos"};
        for (const auto& loc : search_locations){
          problem_expert_->addInstance(plansys2::Instance{loc, "landmark"});
        }
        problem_expert_->addInstance(plansys2::Instance{world_state_.patient_name, "person"});

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
        problem_expert_->addPredicate(plansys2::Predicate("(medicine_location " + world_state_.medicine_location + ")"));
        if (!world_state_.patient_location.empty()){
          problem_expert_->addPredicate(plansys2::Predicate("(person_at " + world_state_.patient_name + " " + world_state_.patient_location + ")"));
        } else{
          std::string oneof_pred = "(oneof ";
          for (const auto& loc : search_locations){
            auto pred = "(person_at " + world_state_.patient_name + " " + loc + ")";
            oneof_pred += pred;
            problem_expert_->addConditional(plansys2::Unknown("(unknown " + pred + ")"));
          }
          oneof_pred += ")";
          problem_expert_->addConditional(plansys2::OneOf(oneof_pred));
        }

        problem_expert_->addConditional(plansys2::Unknown("(unknown (guide_to_succeeded_attempt_1 ))"));
        problem_expert_->addConditional(plansys2::Unknown("(unknown (guide_to_succeeded_attempt_2 ))"));
        problem_expert_->addConditional(plansys2::Unknown("(unknown (notify_automated_succeeded ))"));
        problem_expert_->addConditional(plansys2::Unknown("(unknown (notify_recorded_succeeded ))"));


        problem_expert_->setGoal(plansys2::Goal("(and (success) )"));

        return true;
      }

        if (active_protocol == "food_reminder") {
            //          problem_expert_->clearKnowledge();
            std::string domain = pkgpath + "/pddl/food_domain.pddl";
            bool domain_result = domain_expert_->setDomain(domain);

            if (domain_result) {
                std::cout << "food Domain set successfully: " << domain_result << std::endl;
            } else {
                std::cout << "Failed to set food domain" << std::endl;
            }
            bool problem_result = problem_expert_->setDomain(domain);
            if (problem_result) {
                std::cout << "Food problem set successfully: " << problem_result << std::endl;
            } else {
                std::cout << "Failed to set food problem" << std::endl;
            }

            problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});
            problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
            std::vector<std::string> search_locations = {"bedroom_robot_pos", "kitchen_robot_pos", "couch_robot_pos"};
            for (const auto& loc : search_locations){
                problem_expert_->addInstance(plansys2::Instance{loc, "landmark"});
            }
            problem_expert_->addInstance(plansys2::Instance{world_state_.patient_name, "person"});

            problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
            problem_expert_->addPredicate(plansys2::Predicate("(food_location " + world_state_.eat_location + ")"));
            if (!world_state_.patient_location.empty()){
                problem_expert_->addPredicate(plansys2::Predicate("(person_at " + world_state_.patient_name + " " + world_state_.patient_location + ")"));
            } else{
                std::string oneof_pred = "(oneof ";
                for (const auto& loc : search_locations){
                    auto pred = "(person_at " + world_state_.patient_name + " " + loc + ")";
                    oneof_pred += pred;
                    problem_expert_->addConditional(plansys2::Unknown("(unknown " + pred + ")"));
                }
                oneof_pred += ")";
                problem_expert_->addConditional(plansys2::OneOf(oneof_pred));
            }

            problem_expert_->addConditional(plansys2::Unknown("(unknown (guide_to_succeeded_attempt_1 ))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (guide_to_succeeded_attempt_2 ))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (remind_food_succeeded ))"));
            problem_expert_->addConditional(plansys2::Unknown("(unknown (remind_food_succeeded2 ))"));


            problem_expert_->setGoal(plansys2::Goal("(and (success) )"));

            return true;
        }

        return false;

    }

    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;


    std::string pkgpath = ament_index_cpp::get_package_share_directory("shr_plan");


    std::shared_ptr<shr_plan_parameters::ParamListener> param_listener_;
    shr_plan_parameters::Params params_;


    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    rclcpp_action::Client<shr_msgs::action::GatherInformationRequest>::SharedPtr gathering_info_client_;


    shr_msgs::msg::WorldState world_state_;
    std::shared_ptr<shr_msgs::msg::MidnightWarningProtocol> midnight_warning_state_;
    std::shared_ptr<shr_msgs::msg::MedicineReminderProtocol> medicine_reminder_state;
    std::shared_ptr<shr_msgs::msg::FoodReminderProtocol> food_reminder_state;
    bool gathering_info = false;
    std::string active_protocol;
    std::vector<std::string> requested_states;
  };

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<planning_controller::PlanningController>();
  auto spin_node = std::make_shared<planning_controller::PlanningControllerSpin>();
  node->init();

  rclcpp::Rate rate(5);
  while (spin_node->get_world_state() == shr_msgs::msg::WorldState()) {
    std::cout << "Waiting for world update" << std::endl;
    rate.sleep();
    rclcpp::spin_some(spin_node->get_node_base_interface());
  }

  std::thread thread(
      [spin_node]() {
        rclcpp::spin(spin_node->get_node_base_interface());
      }
  );

  while (rclcpp::ok()) {
    node->set_world_state(spin_node->get_world_state());
    node->step();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  thread.join();

  rclcpp::shutdown();

  return 0;
}