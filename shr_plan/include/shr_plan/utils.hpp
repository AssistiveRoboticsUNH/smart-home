//
// Created by pac48 on 10/7/22.
//
#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using NavigationGoalHandle =
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavigationFeedback =
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

namespace shr_plan {
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

    void send_nav_request(const tf2_ros::Buffer &tf_buffer, const std::string &goal_tf, const rclcpp::Time &cur_time,
                          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client,
                          std::optional<std::function<void(NavigationGoalHandle::SharedPtr,
                                                           NavigationFeedback)>> feedback_callback = std::nullopt,
                          std::optional<std::function<void(
                                  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &)>> result_callback = std::nullopt) {
        nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
        navigation_goal_.pose.header.frame_id = "map";
        navigation_goal_.pose.header.stamp = cur_time;
        auto goal_point = shr_plan::get_tf_as_point(tf_buffer, "map", goal_tf);
        auto current_point = shr_plan::get_tf_as_point(tf_buffer, "map", "base_link");

        navigation_goal_.pose.pose = goal_point;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        if (feedback_callback.has_value()) {
            send_goal_options.feedback_callback = feedback_callback.value();
        }
        if (result_callback.has_value()) {
            send_goal_options.result_callback = result_callback.value();

        }

        auto future_navigation_goal_handle_ = navigation_action_client->async_send_goal(navigation_goal_,
                                                                                        send_goal_options);

    }

}