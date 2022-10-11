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

#include "shr_msg/action/read_script_request.hpp"

using namespace std::chrono_literals;

class NotifyAutomated : public plansys2::ActionExecutorClient {
public:
    NotifyAutomated()
            : plansys2::ActionExecutorClient("notifyautomatedat", 500ms) {

    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) {
        send_feedback(0.0, "Begin audio");

        navigation_action_client_ =
                rclcpp_action::create_client<shr_msg::action::ReadScriptRequest>(shared_from_this(), "read_script");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for /read_script action server...");

            is_action_server_ready =
                    navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "/read_script action server ready");

        auto message = get_arguments()[3];  // The goal is in the 3rd argument of the action
        auto location = get_arguments()[2];  // The goal is in the 3rd argument of the action
        RCLCPP_INFO(get_logger(), "Saying message [%s] at [%s]", message.c_str(), location.c_str());


        auto send_goal_options = rclcpp_action::Client<shr_msg::action::ReadScriptRequest>::SendGoalOptions();

        send_goal_options.result_callback = [this](auto) {
            finish(true, 1.0, "Message completed");
        };

        if (message =="midnight_warning"){
            navigation_goal_.script_name = "midnight_warning.txt";
        }

        future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);


        return ActionExecutorClient::on_activate(previous_state);
    }

private:

    void do_work() {
    }

    using AudioGoalHandle =
            rclcpp_action::ClientGoalHandle<shr_msg::action::ReadScriptRequest>;

    rclcpp_action::Client<shr_msg::action::ReadScriptRequest>::SharedPtr navigation_action_client_;
    std::shared_future<AudioGoalHandle::SharedPtr> future_navigation_goal_handle_;
    AudioGoalHandle::SharedPtr navigation_goal_handle_;

    shr_msg::action::ReadScriptRequest::Goal navigation_goal_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NotifyAutomated>();

    node->set_parameter(rclcpp::Parameter("action_name", "notifyautomatedat"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}