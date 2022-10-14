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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>


#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "shr_utils/utils.hpp"


namespace move_action {
    using namespace std::chrono_literals;

    class MoveAction : public plansys2::ActionExecutorClient {
    public:
        MoveAction()
                : plansys2::ActionExecutorClient("moveto_landmark", 500ms) {

            tf_buffer_ =
                    std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            using namespace std::placeholders;
            pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
                                                                                          std::bind(
                                                                                                  &MoveAction::current_pos_callback,
                                                                                                  this, _1));
        }

        void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_pos_ = msg->pose.pose;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) {
            send_feedback(0.0, "Move starting");

            navigation_action_client_ =
                    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                            this,
                            "navigate_to_pose");

            if (!navigation_action_client_->wait_for_action_server()) {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            }
//            bool is_action_server_ready = false;
//            do {
//                RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
//
//                is_action_server_ready =
//                        navigation_action_client_->wait_for_action_server();//std::chrono::seconds(5)
//            } while (!is_action_server_ready);

            RCLCPP_INFO(get_logger(), "Navigation action server ready");

            auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
            RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

            auto point = shr_utils::get_tf_as_point(*tf_buffer_, "map", wp_to_navigate);
            dist_to_move = getDistance(point, current_pos_);
            auto feedback_callback = [this](
                    shr_utils::NavigationGoalHandle::SharedPtr,
                    shr_utils::NavigationFeedback feedback) {
                send_feedback(std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
                              "Move running");
            };
            auto result_callback = [this](auto) {
                finish(true, 1.0, "Move completed");
            };

            shr_utils::send_nav_request(*tf_buffer_, wp_to_navigate, now(),
            navigation_action_client_, std::nullopt, feedback_callback, result_callback);

            return ActionExecutorClient::on_activate(previous_state);
        }

    private:
        double getDistance(const geometry_msgs::msg::Pose &pos1, const geometry_msgs::msg::Pose &pos2) {
            return sqrt(
                    (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
                    (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
        }

        void do_work() {
        }

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
        std::shared_future<shr_utils::NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
        shr_utils::NavigationGoalHandle::SharedPtr navigation_goal_handle_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
        geometry_msgs::msg::Pose current_pos_;
        nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

        double dist_to_move;
    };
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<move_action::MoveAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "moveto_landmark"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}