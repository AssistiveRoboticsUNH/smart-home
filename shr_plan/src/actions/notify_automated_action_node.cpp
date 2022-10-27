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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/read_script_request.hpp"

#include <shr_plan_parameters.hpp>


using namespace std::chrono_literals;

class NotifyAutomated : public plansys2::ActionExecutorClient {
public:
  NotifyAutomated(const std::string &action, const std::string & script_names)
      : plansys2::ActionExecutorClient(action, 500ms) {
    set_parameter(rclcpp::Parameter("action_name", action));
    script_names_ = script_names;
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    send_feedback(0.0, "Begin audio");

    read_action_client_ =
        rclcpp_action::create_client<shr_msgs::action::ReadScriptRequest>(shared_from_this(), "read_script");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for /read_script action server...");

      is_action_server_ready =
          read_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "/read_script action server ready");

    auto send_goal_options = rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SendGoalOptions();

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Message completed");
    };

    read_goal_.script_name = script_names_;
    future_read_goal_handle_ = read_action_client_->async_send_goal(read_goal_, send_goal_options);


    return ActionExecutorClient::on_activate(previous_state);
  }

private:

  void do_work() {
  }

  using AudioGoalHandle = rclcpp_action::ClientGoalHandle<shr_msgs::action::ReadScriptRequest>;
  rclcpp_action::Client<shr_msgs::action::ReadScriptRequest>::SharedPtr read_action_client_;
  std::shared_future<AudioGoalHandle::SharedPtr> future_read_goal_handle_;
  AudioGoalHandle::SharedPtr read_goal_handle_;
  shr_msgs::action::ReadScriptRequest::Goal read_goal_;

  std::string script_names_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 4);

  auto parameter_node = std::make_shared<rclcpp::Node>("notify_automated_parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

//  for (auto i = 0ul ; i < params.notify_automated_actions.actions.size(); i++){
//    auto action = params.notify_automated_actions.actions[i];
//    auto script_names = params.notify_automated_actions.script_names[i];
//    auto none_node = std::make_shared<NotifyAutomated>(action, script_names);
//    none_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//    exe.add_node(none_node->get_node_base_interface());
//  }

  std::vector<std::shared_ptr<NotifyAutomated>> all_nodes;
  for (auto i = 0ul ; i < params.notify_automated_actions.actions.size(); i++){
    auto action = params.notify_automated_actions.actions[i];
    auto script_names = params.notify_automated_actions.script_names[i];
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<NotifyAutomated>(action, script_names));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  exe.spin();
  rclcpp::shutdown();

  return 0;
}