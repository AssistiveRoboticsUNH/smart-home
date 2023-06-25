#include "rclcpp/rclcpp.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include <memory>
#include <shr_parameters.hpp>

#pragma once

class WorldStateListener : public rclcpp::Node {
private:
    rclcpp::Subscription<shr_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    std::shared_ptr<shr_msgs::msg::WorldState> world_state_;
    std::mutex mtx;
    bool terminate_node_;
    std::shared_ptr<shr_parameters::ParamListener> param_listener_;
public:

    WorldStateListener(const std::string &node_name, std::shared_ptr<shr_parameters::ParamListener> param_listener)
            : rclcpp::Node(
            node_name) {
        terminate_node_ = false;
        world_state_ = nullptr;
        param_listener_ = param_listener;
        auto params = param_listener->get_params();
        world_state_sub_ = create_subscription<shr_msgs::msg::WorldState>(
                params.world_state_topic, 10, [this](const shr_msgs::msg::WorldState::SharedPtr msg) {
                    set_world_state_msg(msg);
                });
    }

    shr_parameters::Params get_params() {
        std::lock_guard<std::mutex> lock(mtx);
        return param_listener_->get_params();
    }

    void terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        terminate_node_ = true;
    }

    bool should_terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        return terminate_node_;
    }

    void set_world_state_msg(const std::shared_ptr<shr_msgs::msg::WorldState> &msg) {
        std::lock_guard<std::mutex> lock(mtx);
        world_state_ = msg;
    }

    std::shared_ptr<shr_msgs::msg::WorldState> get_world_state_msg() {
        std::lock_guard<std::mutex> lock(mtx);
        return world_state_;
    }
};
