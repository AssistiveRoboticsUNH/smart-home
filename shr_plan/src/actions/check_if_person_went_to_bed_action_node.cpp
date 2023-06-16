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

#include "shr_msgs/action/check_person_in_bed_request.hpp"

#include <shr_plan_parameters.hpp>

using namespace std::chrono_literals;

class CheckPersonInBedAction : public plansys2::ActionExecutorClient {
public:
    CheckPersonInBedAction(const std::string &action_name, double timeouts)
            : plansys2::ActionExecutorClient(action_name, 500ms) {
        set_parameter(rclcpp::Parameter("action_name", action_name));
        timeouts_ = timeouts;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) {
        send_feedback(0.0, "Begin checking if person is in bed");

        action_client_ = rclcpp_action::create_client<shr_msgs::action::CheckPersonInBedRequest>(shared_from_this(),
                                                                                                "in_bed");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for /in_bed action server...");

            is_action_server_ready =
                    action_client_->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "/in_bed action server ready");


        send_goal_options_ = rclcpp_action::Client<shr_msgs::action::CheckPersonInBedRequest>::SendGoalOptions();

        send_goal_options_.result_callback = [this](
                const rclcpp_action::ClientGoalHandle<shr_msgs::action::CheckPersonInBedRequest>::WrappedResult &response) {
            if (response.code == rclcpp_action::ResultCode::SUCCEEDED) {
                finish(true, 1.0, "Person went back to bed!");
            } else {
                finish(false, 1.0, "Person out of bed");
            }
        };
        goal_.timeout = timeouts_;
        future_goal_handle_ = action_client_->async_send_goal(goal_, send_goal_options_);

        return ActionExecutorClient::on_activate(previous_state);
    }


protected:
    void do_work() {}

    using GoalHandle = rclcpp_action::ClientGoalHandle<shr_msgs::action::CheckPersonInBedRequest>;

    GoalHandle::SharedPtr goal_handle_;
    rclcpp_action::Client<shr_msgs::action::CheckPersonInBedRequest>::SharedPtr action_client_;
    shr_msgs::action::CheckPersonInBedRequest::Goal goal_;
    std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
    rclcpp_action::Client<shr_msgs::action::CheckPersonInBedRequest>::SendGoalOptions send_goal_options_;

    double timeouts_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    auto parameter_node = std::make_shared<rclcpp::Node>("check_person_in_bed_parameter_node");
    auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
    auto params = param_listener->get_params();


    std::vector<std::shared_ptr<CheckPersonInBedAction>> all_nodes;
    for (auto i = 0ul; i < params.check_if_person_went_to_bed.actions.size(); i++) {
        auto action = params.check_if_person_went_to_bed.actions[i];
        auto timeouts = params.check_if_person_went_to_bed.timeouts[i];
        auto ind = all_nodes.size();
        all_nodes.push_back(std::make_shared<CheckPersonInBedAction>(action, timeouts));
        all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        exe.add_node(all_nodes[ind]->get_node_base_interface());
    }

    exe.spin();
    rclcpp::shutdown();

    return 0;
}