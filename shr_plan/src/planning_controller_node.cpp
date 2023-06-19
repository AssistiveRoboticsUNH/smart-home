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


#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/bool.hpp"
#include "shr_msgs/action/gather_information_request.hpp"
#include "shr_msgs/msg/midnight_warning_protocol.hpp"
#include "shr_msgs/msg/medicine_reminder_protocol.hpp"
#include "shr_msgs/msg/food_reminder_protocol.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include "shr_msgs/msg/success_protocol.hpp"

#include <rclcpp_action/client.hpp>
#include "shr_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <shr_plan_parameters.hpp>
#include <shr_plan/actions.hpp>



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("PlanningControllerNode");
  auto param_listener_ = shr_plan_parameters::ParamListener(node);
  auto params_ = param_listener_.get_params();

  shr_msgs::msg::WorldState world_state_;
  auto world_state_sub_ = node->create_subscription<shr_msgs::msg::WorldState>(
      params_.world_state_topic, 10, [&world_state_](const shr_msgs::msg::WorldState::SharedPtr msg) {
        world_state_ = *msg;
      });


  rclcpp::shutdown();

  return 0;
}