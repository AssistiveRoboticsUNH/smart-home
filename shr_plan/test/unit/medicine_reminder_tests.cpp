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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <fstream>
#include <map>
#include <set>
#include <list>
#include <tuple>
#include <plansys2_executor/ExecutorNodeContingent.hpp>
#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/bt_builder_plugins/contingent_bt_builder.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <shr_plan_parameters.hpp>

#include "gtest/gtest.h"


using namespace std::chrono_literals;

class FakeAction : public plansys2::ActionExecutorClient {
public:
  FakeAction(const std::string &action_name)
      : plansys2::ActionExecutorClient(action_name, 500ms) {
    action_ = action_name;
    succeed_ = true;
    set_parameter(rclcpp::Parameter("action_name", action_name));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    start_time_ = now();
    return ActionExecutorClient::on_activate(previous_state);
  }

  void set_fail() {
    succeed_ = false;
  }


protected:
  void do_work() {
    std::cout << "executing: " << action_ << std::endl;
    auto time_diff = now() - start_time_;
    if (now() - start_time_ > rclcpp::Duration(0, 2E8)) {
      if (succeed_) {
        finish(true, 1.0, "Complete action: " + action_);
      } else {
        finish(false, 1.0, "Failed action: " + action_);
      }
    }
  }

  rclcpp::Time start_time_;
  std::string action_;
  bool succeed_;

};


TEST(medicine_reminder_tests, all_success) {
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 16);

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  std::string pkgpath = ament_index_cpp::get_package_share_directory("shr_plan");
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  std::vector<std::string> planner_plugins = {"CFF"};
  planner_node->set_parameter({"plan_solver_plugins", planner_plugins});
  planner_node->declare_parameter("CFF.plugin", "");
  planner_node->set_parameter({"CFF.plugin", "plansys2/CFFPlanSolver"});

  auto executor_node = std::make_shared<plansys2::ExecutorNodeContingent>();
  executor_node->set_parameter({"bt_builder_plugin", "ContingentBTBuilder"});

  auto parameter_node = std::make_shared<rclcpp::Node>("parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

  std::vector<std::shared_ptr<FakeAction>> all_nodes;
  for (const auto &action: params.none_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.call_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.detect_person_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.guide_to_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  for (const auto &action: params.notify_automated_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.notify_recorded_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  auto ind = all_nodes.size();
  all_nodes.push_back(std::make_shared<FakeAction>(params.move_robot_action));
  all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.add_node(all_nodes[ind]->get_node_base_interface());


  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  bool finish = false;
  bool updated = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
      updated = true;
    }
  });
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  std::ifstream problem_ifs(pkgpath + "/pddl/medicine_reminder_problem.pddl");
  std::string problem_str((std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  problem_client->addProblem(problem_str);
  updated = false;
  while (!updated) {}

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (true) {
    auto feedback = executor_client->getFeedBack();

    bool has_feedback = false;
    for (const auto &action_feedback: feedback.action_execution_status) {
      std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
      has_feedback = true;
    }
    if (has_feedback) {
      std::cout << std::endl;
    }


    if (!executor_client->execute_and_check_plan() && executor_client->getResult()) {
      ASSERT_TRUE(executor_client->getResult().value().success);
      std::cout << "Successful finished " << std::endl;
      break;
    }

    rate.sleep();
  }

  finish = true;
  t.join();

}

TEST(medicine_reminder_tests, patient_wont_move) {
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 16);

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  std::string pkgpath = ament_index_cpp::get_package_share_directory("shr_plan");
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  std::vector<std::string> planner_plugins = {"CFF"};
  planner_node->set_parameter({"plan_solver_plugins", planner_plugins});
  planner_node->declare_parameter("CFF.plugin", "");
  planner_node->set_parameter({"CFF.plugin", "plansys2/CFFPlanSolver"});

  auto executor_node = std::make_shared<plansys2::ExecutorNodeContingent>();
  executor_node->set_parameter({"bt_builder_plugin", "ContingentBTBuilder"});

  auto parameter_node = std::make_shared<rclcpp::Node>("parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

  std::vector<std::shared_ptr<FakeAction>> all_nodes;
  for (const auto &action: params.none_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.call_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.detect_person_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    if (action == "checkguidetosucceeded1" || action == "checkguidetosucceeded2") {
      all_nodes[ind]->set_fail();
    }
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.guide_to_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  for (const auto &action: params.notify_automated_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.notify_recorded_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  auto ind = all_nodes.size();
  all_nodes.push_back(std::make_shared<FakeAction>(params.move_robot_action));
  all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.add_node(all_nodes[ind]->get_node_base_interface());


  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  bool finish = false;
  bool updated = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
      updated = true;
    }
  });
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  std::ifstream problem_ifs(pkgpath + "/pddl/medicine_reminder_problem.pddl");
  std::string problem_str((std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  problem_client->addProblem(problem_str);
  updated = false;
  while (!updated) {}

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (true) {
    auto feedback = executor_client->getFeedBack();

    bool has_feedback = false;
    for (const auto &action_feedback: feedback.action_execution_status) {
      std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
      has_feedback = true;
    }
    if (has_feedback) {
      std::cout << std::endl;
    }


    if (!executor_client->execute_and_check_plan() && executor_client->getResult()) {
      ASSERT_TRUE(executor_client->getResult().value().success);
      std::cout << "Successful finished " << std::endl;
      break;
    }

    rate.sleep();
  }

  finish = true;
  t.join();

}


TEST(medicine_reminder_tests, patient_already_at_kitchen) {
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 16);

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  std::string pkgpath = ament_index_cpp::get_package_share_directory("shr_plan");
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/medicine_reminder_domain.pddl"});

  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  std::vector<std::string> planner_plugins = {"CFF"};
  planner_node->set_parameter({"plan_solver_plugins", planner_plugins});
  planner_node->declare_parameter("CFF.plugin", "");
  planner_node->set_parameter({"CFF.plugin", "plansys2/CFFPlanSolver"});

  auto executor_node = std::make_shared<plansys2::ExecutorNodeContingent>();
  executor_node->set_parameter({"bt_builder_plugin", "ContingentBTBuilder"});

  auto parameter_node = std::make_shared<rclcpp::Node>("parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

  std::vector<std::shared_ptr<FakeAction>> all_nodes;
  for (const auto &action: params.none_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.call_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.detect_person_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->set_fail();
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.guide_to_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  for (const auto &action: params.notify_automated_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  for (const auto &action: params.notify_recorded_actions.actions) {
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<FakeAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }
  auto ind = all_nodes.size();
  all_nodes.push_back(std::make_shared<FakeAction>(params.move_robot_action));
  all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.add_node(all_nodes[ind]->get_node_base_interface());


  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  bool finish = false;
  bool updated = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
      updated = true;
    }
  });
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  std::ifstream problem_ifs(pkgpath + "/pddl/medicine_reminder_problem.pddl");
  std::string problem_str((std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  problem_client->addProblem(problem_str);
  updated = false;
  while (!updated) {}

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (true) {
    auto feedback = executor_client->getFeedBack();

    bool has_feedback = false;
    for (const auto &action_feedback: feedback.action_execution_status) {
      std::cout << "[" << action_feedback.action << " " <<
                action_feedback.completion * 100.0 << "%]";
      has_feedback = true;
    }
    if (has_feedback) {
      std::cout << std::endl;
    }


    if (!executor_client->execute_and_check_plan() && executor_client->getResult()) {
      ASSERT_TRUE(executor_client->getResult().value().success);
      std::cout << "Successful finished " << std::endl;
      break;
    }

    rate.sleep();
  }

  finish = true;
  t.join();

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
