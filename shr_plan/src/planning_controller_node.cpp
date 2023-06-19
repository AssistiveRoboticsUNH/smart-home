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


TRUTH_VALUE person_at(TRUTH_VALUE val, person p, landmark lm) {

    return val;
}

TRUTH_VALUE robot_at(TRUTH_VALUE val, robot p, landmark lm) {

    return val;
}

TRUTH_VALUE medicine_location(TRUTH_VALUE val, landmark lm) {

    return val;
}

TRUTH_VALUE time_to_eat_dinner(TRUTH_VALUE val) {

    return val;
}

TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val) {

    return val;
}

TRUTH_VALUE time_to_eat_breakfast(TRUTH_VALUE val) {

    return val;
}

TRUTH_VALUE time_to_eat_lunch(TRUTH_VALUE val) {

    return val;
}

TRUTH_VALUE person_on_ground(TRUTH_VALUE val) {

    return val;
}

TRUTH_VALUE too_late_to_go_outside(TRUTH_VALUE val) {

    return val;
}




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("PlanningControllerNode");
    auto param_listener_ = shr_plan_parameters::ParamListener(node);
    auto params = param_listener_.get_params();

    shr_msgs::msg::WorldState world_state_;
    auto world_state_sub_ = node->create_subscription<shr_msgs::msg::WorldState>(
            params.world_state_topic, 10, [&world_state_](const shr_msgs::msg::WorldState::SharedPtr msg) {
                world_state_ = *msg;
            });

    UpdatePredicates updater;
    updater.set_person_at(person_at);
    updater.set_robot_at(robot_at);
    updater.set_medicine_location(medicine_location);
    updater.set_time_to_eat_breakfast(time_to_eat_breakfast);
    updater.set_time_to_eat_lunch(time_to_eat_lunch);
    updater.set_time_to_eat_dinner(time_to_eat_dinner);
    updater.set_time_to_take_medicine(time_to_take_medicine);
    updater.set_person_on_ground(person_on_ground);
    updater.set_too_late_to_go_outside(too_late_to_go_outside);

    auto &kb = KnowledgeBase::getInstance();
    for (const auto& landmark : params.pddl_instances.landmarks){
        kb.objects.push_back({landmark, "landmark"});
    }
    for (const auto& person : params.pddl_instances.people){
        kb.objects.push_back({person, "person"});
    }
    for (const auto& robot : params.pddl_instances.robots){
        kb.objects.push_back({robot, "robot"});
    }

//    kb.knownPredicates.concurrent_insert({"person_at", {nathan, kitchen}} ) ;
//    kb.unknownPredicates.concurrent_insert({"person_at", {nathan, couch}} ) ;

    updater.update();

    rclcpp::shutdown();

    return 0;
}