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

#include "shr_msgs/action/detect_left_house_request.hpp"

#include <shr_plan_parameters.hpp>

using namespace std::chrono_literals;

class DetectLeftHouseAction : public plansys2::ActionExecutorClient {
public:
    DetectLeftHouseAction(const std::string &action_name, double timeouts)
            : plansys2::ActionExecutorClient(action_name, 500ms) {
        set_parameter(rclcpp::Parameter("action_name", action_name));
        timeouts_ = timeouts;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) {
        send_feedback(0.0, "Begin detecting if person left");

        action_client_ = rclcpp_action::create_client<shr_msgs::action::DetectLeftHouseRequest>(shared_from_this(),
                                                                                                "left_house");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for /left_house action server...");

            is_action_server_ready =
                    action_client_->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "/left_house action server ready");


        send_goal_options_ = rclcpp_action::Client<shr_msgs::action::DetectLeftHouseRequest>::SendGoalOptions();

        send_goal_options_.result_callback = [this](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::DetectLeftHouseRequest>::WrappedResult &response) {
            if (response.code == rclcpp_action::ResultCode::SUCCEEDED) {
                finish(true, 1.0, "Detected person left house!");
            } else {
                finish(false, 1.0, "Detect person is in house");
            }
        };

        goal_.timeout = timeouts_;
        future_goal_handle_ = action_client_->async_send_goal(goal_, send_goal_options_);

        return ActionExecutorClient::on_activate(previous_state);
    }


protected:
    void do_work() {}

    using GoalHandle = rclcpp_action::ClientGoalHandle<shr_msgs::action::DetectLeftHouseRequest>;

    GoalHandle::SharedPtr goal_handle_;
    rclcpp_action::Client<shr_msgs::action::DetectLeftHouseRequest>::SharedPtr action_client_;
    shr_msgs::action::DetectLeftHouseRequest::Goal goal_;
    std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
    rclcpp_action::Client<shr_msgs::action::DetectLeftHouseRequest>::SendGoalOptions send_goal_options_;

    double timeouts_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    auto parameter_node = std::make_shared<rclcpp::Node>("detect_left_house_parameter_node");
    auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
    auto params = param_listener->get_params();


    std::vector<std::shared_ptr<DetectLeftHouseAction>> all_nodes;
    for (auto i = 0ul; i < params.call_actions.actions.size(); i++) {
        auto action = params.detect_person_left_house.actions[i];
        auto timeouts = params.detect_person_left_house.timeouts[i];
        auto ind = all_nodes.size();
        all_nodes.push_back(std::make_shared<DetectLeftHouseAction>(action, timeouts));
        all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        exe.add_node(all_nodes[ind]->get_node_base_interface());
    }

    exe.spin();
    rclcpp::shutdown();

    return 0;
}