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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace move_action {
    using namespace std::chrono_literals;

    geometry_msgs::msg::Pose
    get_tf_as_point(const tf2_ros::Buffer &tf_buffer, const std::string &parent_id, const std::string &child_id) {
        auto transformStamped = tf_buffer.lookupTransform(
                parent_id,
                child_id,
                tf2::TimePointZero);

        geometry_msgs::msg::Pose point;
        point.position.x = transformStamped.transform.translation.x;
        point.position.y = transformStamped.transform.translation.y;
        point.position.z = transformStamped.transform.translation.z;
        point.orientation.x = transformStamped.transform.rotation.x;
        point.orientation.y = transformStamped.transform.rotation.y;
        point.orientation.z = transformStamped.transform.rotation.z;
        point.orientation.w = transformStamped.transform.rotation.w;

        return point;
    }


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
                            shared_from_this(),
                            "navigate_to_pose");

            bool is_action_server_ready = false;
            do {
                RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

                is_action_server_ready =
                        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
            } while (!is_action_server_ready);

            RCLCPP_INFO(get_logger(), "Navigation action server ready");

            auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
            RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

            navigation_goal_.pose.header.frame_id = "map";
            navigation_goal_.pose.header.stamp = now();
            auto point = get_tf_as_point(*tf_buffer_, "map", wp_to_navigate + "_robot_pos");
            navigation_goal_.pose.pose = point;

            dist_to_move = getDistance(goal_pos_.pose, current_pos_);

            auto send_goal_options =
                    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

            send_goal_options.feedback_callback = [this](
                    NavigationGoalHandle::SharedPtr,
                    NavigationFeedback feedback) {
                send_feedback(std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
                        "Move running");
            };

            send_goal_options.result_callback = [this](auto) {
                finish(true, 1.0, "Move completed");
            };

            future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

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

        using NavigationGoalHandle =
                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
        using NavigationFeedback =
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
        std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
        NavigationGoalHandle::SharedPtr navigation_goal_handle_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
        geometry_msgs::msg::Pose current_pos_;
        geometry_msgs::msg::PoseStamped goal_pos_;
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