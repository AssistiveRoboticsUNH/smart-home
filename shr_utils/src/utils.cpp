//
// Created by pac48 on 10/11/22.
//
#include "shr_utils/utils.hpp"

namespace shr_utils {


    geometry_msgs::msg::Pose
    get_tf_as_point(tf2_ros::Buffer &tf_buffer, const std::string &parent_id, const std::string &child_id) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer.lookupTransform(parent_id, child_id,
                                                  tf2::TimePointZero, std::chrono::seconds(10));
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("shr_utils"), "transform exception '%s'", e.what());
        }


        geometry_msgs::msg::Pose point;
        point.position.x = transform.transform.translation.x;
        point.position.y = transform.transform.translation.y;
        point.position.z = transform.transform.translation.z;
        point.orientation.x = transform.transform.rotation.x;
        point.orientation.y = transform.transform.rotation.y;
        point.orientation.z = transform.transform.rotation.z;
        point.orientation.w = transform.transform.rotation.w;
        return point;
    }

    double point_dist2(geometry_msgs::msg::Pose point1, geometry_msgs::msg::Pose point2) {
        return pow(point1.position.x - point2.position.x, 2) + pow(point1.position.y - point2.position.y, 2) +
               pow(point1.position.z - point2.position.z, 2);
    }

    int get_nearest_location(tf2_ros::Buffer &tf_buffer, const std::vector<std::string> &locations) {
        if (locations.empty()) return -1;

        auto cur_pose = get_tf_as_point(tf_buffer, "map", "base_link");
        double min_dist = 1E10;
        int min_ind = -1;
        for (auto i = 0; i < locations.size(); i++) {
            const auto &location = locations[i];
            auto point = get_tf_as_point(tf_buffer, "map", location);
            auto tmp = point_dist2(cur_pose, point);
            if (tmp < min_dist) {
                min_dist = tmp;
                min_ind = i;
            }
        }

        return min_ind;

    }


//    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
    void send_nav_request_sim(tf2_ros::Buffer &tf_buffer, const std::string &goal_tf, const rclcpp::Time &cur_time,
                          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client,
                          std::optional<const std::function<void(
                                  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>)>> goal_response_callback,
                          std::optional<std::function<void(NavigationGoalHandle_sim::SharedPtr,
                                                           NavigationFeedback_sim)>> feedback_callback,
                          std::optional<std::function<void(
                                  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &)>> result_callback) {
        nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
        navigation_goal_.pose.header.frame_id = "map";
        navigation_goal_.pose.header.stamp = cur_time;
        auto goal_point = get_tf_as_point(tf_buffer, "map", goal_tf);
        auto current_point = get_tf_as_point(tf_buffer, "map", "base_link");

        navigation_goal_.pose.pose = goal_point;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        if (goal_response_callback.has_value()) {
            send_goal_options.goal_response_callback = goal_response_callback.value();
        }
        if (feedback_callback.has_value()) {
            send_goal_options.feedback_callback = feedback_callback.value();
        }
        if (result_callback.has_value()) {
            send_goal_options.result_callback = result_callback.value();

        }
        if (!navigation_action_client->wait_for_action_server()) {
            RCLCPP_ERROR(rclcpp::get_logger("shr_utils"), "Action server not available after waiting");
        }
        auto navigation_goal_handle = navigation_action_client->async_send_goal(navigation_goal_,
                                                                                send_goal_options);
//        return navigation_goal_handle;

    }

    void send_nav_request(tf2_ros::Buffer &tf_buffer, const std::string &goal_tf, const rclcpp::Time &cur_time,
                          rclcpp_action::Client<shr_msgs::action::NavigateToPose>::SharedPtr navigation_action_client,
                          std::optional<const std::function<void(
                                  std::shared_ptr<rclcpp_action::ClientGoalHandle<shr_msgs::action::NavigateToPose>>)>> goal_response_callback,
                          std::optional<std::function<void(NavigationGoalHandle::SharedPtr,
                                                           NavigationFeedback)>> feedback_callback,
                          std::optional<std::function<void(
                                  const rclcpp_action::ClientGoalHandle<shr_msgs::action::NavigateToPose>::WrappedResult &)>> result_callback) {
        shr_msgs::action::NavigateToPose::Goal navigation_goal_;
        navigation_goal_.pose.header.frame_id = "map";
        navigation_goal_.pose.header.stamp = cur_time;
        auto goal_point = get_tf_as_point(tf_buffer, "map", goal_tf);
        auto current_point = get_tf_as_point(tf_buffer, "map", "base_link");

        navigation_goal_.pose.pose = goal_point;

        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::NavigateToPose>::SendGoalOptions();

        if (goal_response_callback.has_value()) {
            send_goal_options.goal_response_callback = goal_response_callback.value();
        }
        if (feedback_callback.has_value()) {
            send_goal_options.feedback_callback = feedback_callback.value();
        }
        if (result_callback.has_value()) {
            send_goal_options.result_callback = result_callback.value();

        }
        if (!navigation_action_client->wait_for_action_server()) {
            RCLCPP_ERROR(rclcpp::get_logger("shr_utils"), "Action server not available after waiting");
        }
        auto navigation_goal_handle = navigation_action_client->async_send_goal(navigation_goal_,
                                                                                send_goal_options);
//        return navigation_goal_handle;

    }

}
