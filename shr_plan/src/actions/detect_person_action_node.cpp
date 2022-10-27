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


#include <memory>
#include <string>
#include <map>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "shr_msgs/action/detect_person_request.hpp"

#include <shr_plan_parameters.hpp>

using namespace std::chrono_literals;

class DetectPersonAction : public plansys2::ActionExecutorClient {
public:
  DetectPersonAction(const std::string &action_name, double timeout)
      : plansys2::ActionExecutorClient(action_name, 500ms) {
    set_parameter(rclcpp::Parameter("action_name", action_name));
    timeout_ = timeout;

  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    send_feedback(0.0, "Begin detect person");

    action_client_ = rclcpp_action::create_client<shr_msgs::action::DetectPersonRequest>(shared_from_this(),
                                                                                         "detect_person");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for /detect_person action server...");

      is_action_server_ready =
          action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "/detect_person action server ready");


    send_goal_options_ = rclcpp_action::Client<shr_msgs::action::DetectPersonRequest>::SendGoalOptions();

    send_goal_options_.result_callback = [this](
        const rclcpp_action::ClientGoalHandle<shr_msgs::action::DetectPersonRequest>::WrappedResult &response) {
      if (response.code == rclcpp_action::ResultCode::SUCCEEDED) {
        finish(true, 1.0, "Detected person!");
      } else {
        finish(false, 1.0, "Failed to detect person");
      }
    };

    goal_.timeout = timeout_;
    future_goal_handle_ = action_client_->async_send_goal(goal_, send_goal_options_);

    return ActionExecutorClient::on_activate(previous_state);
  }


protected:
  void do_work() {}

  using GoalHandle = rclcpp_action::ClientGoalHandle<shr_msgs::action::DetectPersonRequest>;

  GoalHandle::SharedPtr goal_handle_;
  rclcpp_action::Client<shr_msgs::action::DetectPersonRequest>::SharedPtr action_client_;
  shr_msgs::action::DetectPersonRequest::Goal goal_;
  std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
  rclcpp_action::Client<shr_msgs::action::DetectPersonRequest>::SendGoalOptions send_goal_options_;

  double timeout_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 4);

  auto parameter_node = std::make_shared<rclcpp::Node>("detect_person_parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

//  for (auto i = 0ul; i < params.detect_person_actions.actions.size(); i++) {
//    auto action = params.detect_person_actions.actions[i];
//    auto timeout = params.detect_person_actions.timeouts[i];
//    auto call_node = std::make_shared<DetectPersonAction>(action, timeout);
//    call_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//
//    exe.add_node(call_node->get_node_base_interface());
//  }

  std::vector<std::shared_ptr<DetectPersonAction>> all_nodes;
  for (auto i = 0ul; i < params.detect_person_actions.actions.size(); i++) {
    auto action = params.detect_person_actions.actions[i];
    auto timeout = params.detect_person_actions.timeouts[i];
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<DetectPersonAction>(action, timeout));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  exe.spin();
  rclcpp::shutdown();

  return 0;
}