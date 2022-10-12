//
// Created by pac48 on 10/7/22.
//
#pragma once

#include <rclcpp_action/client.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace shr_utils {
    using NavigationGoalHandle =
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using NavigationFeedback =
            const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

    geometry_msgs::msg::Pose
    get_tf_as_point(const tf2_ros::Buffer &tf_buffer, const std::string &parent_id, const std::string &child_id);

    void send_nav_request(const tf2_ros::Buffer &tf_buffer, const std::string &goal_tf, const rclcpp::Time &cur_time,
                          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client,
                          std::optional <std::function<void(NavigationGoalHandle::SharedPtr,
                                                            NavigationFeedback)>> feedback_callback = std::nullopt,
                          std::optional <std::function<void(
                                  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &)>> result_callback = std::nullopt);

}