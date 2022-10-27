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

#include "shr_msgs/action/call_request.hpp"

#include <shr_plan_parameters.hpp>

using namespace std::chrono_literals;

class NoneAction : public plansys2::ActionExecutorClient {
public:
  NoneAction(const std::string& action_name)
      : plansys2::ActionExecutorClient(action_name, 500ms) {
    set_parameter(rclcpp::Parameter("action_name", action_name));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {

    finish(true, 1.0, "None completed");

    return ActionExecutorClient::on_activate(previous_state);
  }


protected:
  void do_work() {}


};




int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 4);

  auto parameter_node = std::make_shared<rclcpp::Node>("none_parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

  std::vector<std::shared_ptr<NoneAction>> all_nodes;
  for (const auto & action : params.none_actions.actions){
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<NoneAction>(action));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  exe.spin();
  rclcpp::shutdown();

  return 0;
}