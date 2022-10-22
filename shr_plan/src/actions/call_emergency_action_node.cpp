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

#include "shr_msgs/action/call_request.hpp"

using namespace std::chrono_literals;

class CallEmergency : public plansys2::ActionExecutorClient {
public:
    CallEmergency()
            : plansys2::ActionExecutorClient("callemergency", 500ms) {

    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) {
        send_feedback(0.0, "Begin call");

        action_client_ =
                rclcpp_action::create_client<shr_msgs::action::CallRequest>(shared_from_this(), "make_call");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for /make_call action server...");

            is_action_server_ready =
                    action_client_->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "/make_call action server ready");

        auto person = get_arguments()[1];
        RCLCPP_INFO(get_logger(), "call emergency for [%s]", person.c_str());


        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::CallRequest>::SendGoalOptions();

        send_goal_options.result_callback = [this](auto) {
            finish(true, 1.0, "Message completed");
        };

        if (!person.empty()){ // should be specific to person
            goal_.script_name = "call_msg_911.xml";
        }
        goal_.phone_number = "6038514204";

        future_goal_handle_ = action_client_->async_send_goal(goal_, send_goal_options);


        return ActionExecutorClient::on_activate(previous_state);
    }

private:

    void do_work() {
    }

    using GoalHandle =
            rclcpp_action::ClientGoalHandle<shr_msgs::action::CallRequest>;

    rclcpp_action::Client<shr_msgs::action::CallRequest>::SharedPtr action_client_;
    std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
    GoalHandle::SharedPtr goal_handle_;

    shr_msgs::action::CallRequest::Goal goal_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CallEmergency>();

    node->set_parameter(rclcpp::Parameter("action_name", "callemergency"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}